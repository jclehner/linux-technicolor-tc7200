#include <linux/pci.h>

#define BCM3383_PCIE0_BASE	0xb2800000
#define BCM3383_PCIE1_BASE	0xb2a00000
#define BCM3383_GPIO_BASE	0xb4e00100

static u32 bcm3383_pcie_base[2] = { 0xb2800000, 0xb2a00000 };

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


typedef struct 
{
	u32 addr;
	u32 wdata;
	u32 rdata;
} bcm3383_mdio;

static u16 bcm3383_pcie_mdio_read(int port, u16 phyad, u16 regad)
{
	int timeout;
	u32 data;
	volatile bcm3383_mdio *mdio = 
		(bcm3383_mdio*)(bcm3383_pcie_base[port] + 0x1100);

	mdio->addr = 0x100000 | ((phyad & 0xf) << 16) | (regad & 0x1f);
	mdelay(1);

	timeout = 2;
	while (timeout-- > 0) {
		data = mdio->rdata;
		if (data & 0x80000000) {
			break;
		}
		mdelay(1);
	}

	if (timeout == 0) {
		return 0xdead;
	} else {
		return data & 0xffff;
	}
}

static int bcm3383_pcie_mdio_write(int port, u16 phyad, u16 regad, u16 wrdata)
{
	int timeout;
	volatile bcm3383_mdio *mdio = 
		(bcm3383_mdio*)(bcm3383_pcie_base[port] + 0x1100);

	mdio->addr = ((phyad & 0xf) << 16) | (regad & 0x1f);
	mdelay(1);

	mdio->wdata = 0x80000000 | (wrdata & 0xffff);
	mdelay(1);

	timeout = 2;
	while (timeout-- > 0) {
		if (!(mdio->wdata & 0x80000000)) {
			return 0;
		}
		mdelay(1);
	}

	return -1;
}

static void bcm3383_pcie_phy_mode_cfg(int port)
{
	u16 data;

	bcm3383_pcie_mdio_write(port, 1, 0x1f, 0x8200);
	data = (bcm3383_pcie_mdio_read(port, 1, 0x18) & 0xfff0) | 0xf;
	bcm3383_pcie_mdio_write(port, 1, 0x18, data);

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

static void bcm3383_gpio_set_out(unsigned gpio)
{
	int out = 1;
	volatile u32 *gpio_dir = (u32*)(BCM3383_GPIO_BASE + (gpio < 32 ? 0 : 4));
	volatile u32 *gpio_unk = (u32*)(BCM3383_GPIO_BASE + (gpio < 32 ? 0x10 : 0x14));

	if (out) {
		*gpio_dir |= (1 << gpio);
		*gpio_unk |= (1 << gpio);
	} else {
		*gpio_dir &= ~(1 << gpio);
	}
}

static void bcm3383_init_pcie(int port)
{
	volatile u32 *pcie;
	u32 base;
	int link;

	if (port > 1) {
		return;
	}

	bcm3383_gpio_set_out(port + 4);

	*(volatile u32*)(0xb4e00364) &= ~(1 << 24);
	*(volatile u32*)(0xb4e00368) &= ~(1 << 8);

	mdelay(20);

	bcm3383_intc->clk_ctrl_high |= (0x80 << port);

	// PCIE0_CLK_EN = 0x10 | PCIE1_CLK_EN = 0x11
	bcm3383_intc->clk_ctrl_low |= (0x10 << port);
	bcm3383_intc->clk_ctrl_ubus |= (0x10 << port);

	// PCIE0 = 0x100, PCIE1 = 0x1000
	bcm3383_intc->soft_resetb_low &= ~(0x4 << port);
	bcm3383_intc->soft_resetb_high &= ~(0x4 << port);
	bcm3383_intc->soft_reset &= ~(0x4 << port);

	mdelay(10);

	// PCIE0 = 0x100, PCIE1 = 0x1000
	bcm3383_intc->soft_resetb_low |= (0x4 << port);
	bcm3383_intc->soft_reset |= (0x4 << port);

	base = !port ? 0xb2800000 : 0xb2a00000;

	pcie = (u32*)(base + 0x4204);
	// &= 0xff7fffff
	*pcie &= ~(1 << 23);

	//u32 v0 = ((port << 3) + port) << 3;
	
	bcm3383_pcie_phy_mode_cfg(port);

	bcm3383_intc->soft_reset |= (0x4 << port);

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

	base += 0x9000;

	pr_info("%s: 00:00.0 [%08x]\n", __func__, *(volatile u32*)base);
	bcm3383_pcie_cfg_select(1, 0);
	pr_info("%s: 01:00.0 [%08x]\n", __func__, *(volatile u32*)base);
	bcm3383_pcie_cfg_select(2, 0);
	pr_info("%s: 02:00.0 [%08x]\n", __func__, *(volatile u32*)base);

	pr_info("%s: write=%p\n", __func__, bcm3383_pcie_controller.pci_ops->write);

	//register_pci_controller(&bcm3383_pcie_controller);
}
