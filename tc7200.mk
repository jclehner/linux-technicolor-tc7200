CROSS_COMPILE := mips-linux-gnu-
OBJCOPY := $(CROSS_COMPILE)objcopy
PROGRAM_STORE := ProgramStore
LOAD_ADDR := 0x80010000

.PHONY: linux.sto

all: linux.sto

linux.sto:
	make ARCH=mips CROSS_COMPILE=$(CROSS_COMPILE) -j3
	$(OBJCOPY) -O binary vmlinux vmlinux.bin
	$(PROGRAM_STORE) -c 4 -s 0xa825 -a $(LOAD_ADDR) -f vmlinux.bin -o linux.sto

cpio:

uncpio:
	rm -fr rootfs && mkdir rootfs
	cd rootfs; sudo cpio -i -R ${USER} < ../tc7200-rootfs.cpio

cpio:
	cd rootfs; find . | cpio -H newc -o > ../tc7200-rootfs.cpio

defconfig:
	make ARCH=mips CROSS_COMPILE=$(CROSS_COMPILE) bmips_be_defconfig

menuconfig:
	make ARCH=mips CROSS_COMPILE=$(CROSS_COMPILE) menuconfig
