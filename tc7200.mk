CROSS_COMPILE := mips-linux-gnu-
OBJCOPY := $(CROSS_COMPILE)objcopy
PROGRAM_STORE := ProgramStore

.PHONY: linux.sto

all: linux.sto

linux.sto:
	make ARCH=mips CROSS_COMPILE=$(CROSS_COMPILE) -j3
	$(OBJCOPY) -O binary vmlinux vmlinux.bin
	$(PROGRAM_STORE) -c 4 -s 0xa825 -a 0x80010000 -f vmlinux.bin -o linux.sto

defconfig:
	make ARCH=mips CROSS_COMPILE=$(CROSS_COMPILE) bmips_be_defconfig

menuconfig:
	make ARCH=mips CROSS_COMPILE=$(CROSS_COMPILE) menuconfig
