#include <linux/pci.h>

#define BCM3383_PCIE0_BASE 0xb2800000
#define BCM3383_PCIE1_BASE 0xb2a00000

static inline u32 bcm3383_pcie_bus_base(u32 bus)
{
	/* bus 0 is mpi root, bus 1 is pcie0 root, bus 3 is pcie1 root */
	BUG_ON(!bus);
	return bus < 3 ? BCM3383_PCIE0_BASE : BCM3383_PCIE1_BASE;
}

#define PCIE_DEV_OFFSET 0x9000

static inline u32 bcm3383_pcie_dev_addr(u32 bus, int where)
{
	return bcm3383_pcie_bus_base(bus) + 
		(bus & 1 ? 0 : PCIE_DEV_OFFSET) + (where & ~3);
}

#define PCIE_EXT_CFG_REGS 0x8400

static void bcm3383_pcie_cfg_select(u32 bus, int devfn)
{
	u32 *reg = (u32*)(bcm3383_pcie_bus_base(bus) + PCIE_EXT_CFG_REGS);
	*reg = (bus << 20 | PCI_SLOT(devfn) << 15 | PCI_FUNC(devfn) << 12);
}

static int bcm3383_pcie_read(struct pci_bus *bus, unsigned devfn,
		int where, int size, u32 *val)
{
	u32 data;

	bcm3383_pcie_cfg_select(bus->number, devfn);
	data = *(u32*)bcm3383_pcie_dev_addr(bus->number, where);
	if (data == 0xdeaddead) {
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	if (size == 1) {
		*val = (data >> ((where & 3) << 3)) & 0xff;
	} else if (size == 2) {
		*val = (data >> ((where & 3) << 3)) & 0xffff;
	} else {
		*val = data;
	}

	return PCIBIOS_SUCCESSFUL;
}

static int bcm3383_pcie_write(struct pci_bus *bus, unsigned devfn,
		int where, int size, u32 val)
{
	u32 *addr = (u32*)bcm3383_pcie_dev_addr(bus->number, where);
	if (*addr == 0xdeaddead) {
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	if (size == 1) {
		*addr = (*addr & ~(0xff << ((where & 3) << 3))) |
			(val << ((where & 3) << 3));
	} else if (size == 2) {
		*addr = (*addr & ~(0xffff << ((where & 3) << 3))) |
			(val << ((where & 3) << 3));
	} else {
		*addr = val;
	}

	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops bcm3383_pcie_ops = {
	.read = bcm3383_pcie_read,
	.write = bcm3383_pcie_write
};

static struct resource bcm3383_io_resource = {
	.name = "bcm3383 pcie io",
	.start = 0,
	.end = 0,
	.flags = 0
};

static struct resource bcm3383_mem_resource = {
	.name = "bcm3383 pcie mem",
	.start = BCM3383_PCIE0_BASE,
	.end = BCM3383_PCIE0_BASE + 0x10000000 - 1,
	.flags = IORESOURCE_MEM
};

static struct pci_controller bcm3383_pcie_controller = {
	.pci_ops		= &bcm3383_pcie_ops,
	.io_resource 	= &bcm3383_io_resource,
	.mem_resource	= &bcm3383_mem_resource
};

static void bcm3383_init_pcie(int id)
{
	volatile u32 *pcie;
	u32 base;
	int link;

	if (id > 1) {
		return;
	}

	// PCIE0_CLK_EN = 0x10 | PCIE1_CLK_EN = 0x11
	bcm3383_intc->clk_ctrl_low |= (0x10 << id);
	bcm3383_intc->clk_ctrl_ubus |= (0x10 << id);

	// PCIE0 = 0x100, PCIE1 = 0x1000
	bcm3383_intc->soft_resetb_low &= ~(0x4 << id);
	bcm3383_intc->soft_resetb_high &= ~(0x4 << id);
	bcm3383_intc->soft_reset &= ~(0x4 << id);

	mdelay(10);

	// PCIE0 = 0x100, PCIE1 = 0x1000
	bcm3383_intc->soft_resetb_low |= (0x4 << id);
	bcm3383_intc->soft_reset |= (0x4 << id);

	base = !id ? 0xb2800000 : 0xb2a00000;

	pcie = (u32*)(base + 0x4204);
	// &= 0xff7fffff
	*pcie &= ~(1 << 23);

	//u32 v0 = ((id << 3) + id) << 3;

	bcm3383_intc->soft_reset |= (0x4 << id);

	mdelay(200);

	*pcie &= ~(1 << 23);

	pcie = (u32*)(base + 0x4068);

	link = *pcie & 0x20;
	pr_info("%s: link=%d\n", __func__, link);

	if (!link) {
		mdelay(1000);
		link = *pcie & 0x20;
	}

	pr_info("%s: link=%d\n", __func__, link);

	//base += 0x9000;

	bcm3383_pcie_cfg_select(1, 0);
	pr_info("%s: 01:00.0 [%08x]\n", __func__, *(volatile u32*)base);
	bcm3383_pcie_cfg_select(2, 0);
	pr_info("%s: 02:00.0 [%08x]\n", __func__, *(volatile u32*)base);

	pr_info("%s: write=%p\n", __func__, bcm3383_pcie_controller.pci_ops->write);

	//register_pci_controller(&bcm3383_pcie_controller);
}
