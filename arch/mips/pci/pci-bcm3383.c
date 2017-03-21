/*
 * Copyright (C) Joseph C. Lehner 2017
 * Licensed under the GNU GPL v2.0
 */

#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_pci.h>
#include <linux/platform_device.h>
#include <asm/mach-bmips/bcm3383.h>

/*
 * PCIe setup:
 *
 * 00:00.0 [14e0:3383] Root port 1
 * 00:02.0 [14e0:3383] Root port 2
 * 00:03:0 [----:----] mPCIe slot 2
 */

#define IRQ_PCIE		15
#define PCIE0_BASE		0xb2800000
#define PCIE1_BASE		0xb2a00000

#define PCIE0_ROOT_BUS	0
#define PCIE1_ROOT_BUS	2
#define PCIE0_DEV_BUS	1
#define PCIE1_DEV_BUS	3

typedef struct
{
	u32 status;
	u32 mask_status;
	u32 mask_set;
	u32 mask_clear;
} bcm3383_pcie_cpu_intr_regs;

typedef struct
{
	u32 index;
	u32 data;
	u32 scratch;
} bcm3383_pcie_ext_cfg_regs;

#define BCM3383_PCIE0_BASE	0xb2800000
#define BCM3383_PCIE1_BASE	0xb2a00000
#define BCM3383_GPIO_BASE	0xb4e00100

#define BCM3383_GPIO_REG(reg) (volatile u32*)(0xb4e00100 + (reg))
#define BCM3383_PCIE_REG(port, reg) (volatile u32*)(bcm3383_pcie_base[port] + (reg))

#define BCM3383_PCIE_STATUS_REG(port) BCM3383_PCIE_REG(port, 0x4068)

#define BCM3383_PCIE_EXT_CFG_REGS(port, off) BCM3383_PCIE_REG(port, 0x8400 + off)

#define BCM3383_PCIE_HARD_DEBUG_REG(port) BCM3383_PCIE_REG(port, 0x4204)
#define BCM3383_PCIE_SERDES_IDDQ	(1 << 23)

#define BCM3383_PCIE_INTR_REGS(port) ((volatile bcm3383_pcie_cpu_intr_regs*)\
		BCM3383_PCIE_REG(port, 0x8300))


static u32 bcm3383_pcie_base[2] = { BCM3383_PCIE0_BASE, 0xb2a00000 };

static inline u32 bcm3383_pcie_bus_base(u32 bus)
{
	/* bus 0 is mpi root, bus 1 is pcie0 root, bus 3 is pcie1 root */
	//BUG_ON(!bus);
	return bus < 3 ? BCM3383_PCIE0_BASE : BCM3383_PCIE1_BASE;
}

#define PCIE_DEV_OFFSET 0x9000

static inline int bcm3383_bus_to_port(u8 bus)
{
	return bus < 2 ? 0 : 1;
}

static inline int bcm3383_is_root_bus(u8 bus)
{
	return bus == 0 || bus == 2;
}

static inline u32 bcm3383_pcie_dev_addr(u32 bus, int where)
{
	u32 addr = bcm3383_pcie_bus_base(bus) + 
		(bus & 1 ? 0 : 0) + (where & ~3);
	//pr_info("%s: addr=%08x\n", __func__, addr);
	return addr;
}

#define PCIE_EXT_CFG_REGS 0x8400

static inline void bcm3383_pcie_cfg_select(u8 bus, unsigned devfn)
{
	*BCM3383_PCIE_EXT_CFG_REGS(bcm3383_bus_to_port(bus), 0x0) = bus << 20
		| PCI_SLOT(devfn) << 15 | PCI_FUNC(devfn) << 12;
}

static inline volatile u32 *bcm3383_pcie_get_addr(u8 bus, int where)
{
	if (!bcm3383_is_root_bus(bus)) {
		where |= 0x9000;
	}

	return BCM3383_PCIE_REG(bcm3383_bus_to_port(bus), where & ~3);
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
		pr_info("%s: timeout (%d, %02x, %02x)\n", __func__, port, phyad, regad);
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

	pr_info("%s: timeout (%d, %02x, %02x, %04x)\n", __func__, port, phyad, regad, wrdata);

	return -1;
}

static void bcm3383_pcie_phy_mode_cfg(int port)
{
	u16 data;

	bcm3383_pcie_mdio_write(port, 1, 0x1f, 0x8200);

	data = (bcm3383_pcie_mdio_read(port, 1, 0x18) & 0xfff0) | 0xf;
	bcm3383_pcie_mdio_write(port, 1, 0x18, data);

	data = (bcm3383_pcie_mdio_read(port, 0, 0x15) & 0xfdff) | 0x200;
	bcm3383_pcie_mdio_write(port, 0, 0x15, data);

	bcm3383_pcie_mdio_write(port, 0, 0x1f, 0x8400);
	bcm3383_pcie_mdio_write(port, 0, 0x11, 0x1d40);
	bcm3383_pcie_mdio_write(port, 0, 0x1f, 0x8400);
	bcm3383_pcie_mdio_write(port, 0, 0x11, 0x1f40);

	//data = (bcm3383_pcie_mdio_read(port, 0, 0x18);
}

static int bcm3383_pcie_can_access(u8 bus, unsigned devfn)
{
	u8 port = bcm3383_bus_to_port(bus);

	bcm3383_pcie_cfg_select(bus, devfn);

	if (bcm3383_is_root_bus(bus)) {
		return PCI_SLOT(devfn) == 0;
	} else {
		if (!(*BCM3383_PCIE_REG(port, 0x1048) & 0x2000)) {
			return 0;
		}

		return PCI_SLOT(devfn) == 0;
	}

	return 0;
}

static int bcm3383_pcie_read(struct pci_bus *bus, unsigned devfn,
		int where, int size, u32 *val)
{
	u32 data, status;

	if (!bcm3383_pcie_can_access(bus->number, devfn)) {
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	status = read_c0_config();
	write_c0_config(status & ~1);
	data = *bcm3383_pcie_get_addr(bus->number, where);
	write_c0_config(status);

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
	u32 status;
	volatile u32 *addr;
	int err = PCIBIOS_DEVICE_NOT_FOUND;

	if (!bcm3383_pcie_can_access(bus->number, devfn)) {
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	status = read_c0_config();
	write_c0_config(status & ~1);

	addr = bcm3383_pcie_get_addr(bus->number, where);
	if (*addr == 0xdeaddead) {
		goto out;
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

	err = PCIBIOS_SUCCESSFUL;
out:
	write_c0_config(status);
	return err;
}

static struct pci_ops bcm3383_pcie_ops = {
	.read = bcm3383_pcie_read,
	.write = bcm3383_pcie_write
};

static struct resource bcm3383_io_resource = {
	.name = "bcm3383 pcie io",
	.start = 0,
	.end = 0,
	.flags = IORESOURCE_IO,
};

static struct resource bcm3383_mem0_resource = {
	.name = "bcm3383 pcie0 mem",
	.start = BCM3383_PCIE0_BASE,
	.end = BCM3383_PCIE0_BASE + 0x10000000 - 1,
	.flags = IORESOURCE_MEM
};

static struct resource bcm3383_mem1_resource = {
	.name = "bcm3383 pcie1 mem",
	.start = BCM3383_PCIE1_BASE,
	.end = BCM3383_PCIE1_BASE + 0x10000000 - 1,
	.flags = IORESOURCE_MEM
};

static struct resource bcm3383_busn_resource = {
	.name = "bcm3383 busn",
	.start = 0,
	.end = 255,
	.flags = IORESOURCE_BUS
};

static struct pci_controller bcm3383_pcie0 = {
	.pci_ops		= &bcm3383_pcie_ops,
	.io_resource 	= &bcm3383_io_resource,
	.mem_resource	= &bcm3383_mem0_resource,
	.busn_resource	= &bcm3383_busn_resource,
};

static struct pci_controller bcm3383_pcie1 = {
	.pci_ops		= &bcm3383_pcie_ops,
	.io_resource 	= &bcm3383_io_resource,
	.mem_resource	= &bcm3383_mem1_resource,
	.busn_resource	= &bcm3383_busn_resource,
};

static void bcm3383_gpio_set_out(unsigned gpio)
{
	u32 dir_reg, data_reg, val;

	dir_reg = gpio < 32 ? 0x00 : 0x04;
	data_reg = gpio < 32 ? 0x10 : 0x14;

	if (gpio >= 32) {
		gpio -= 32;
	}

	if (gpio < 32) {
		bmips_gpio_w32(dir_reg, bmips_gpio_r32(dir_reg) | (1 << gpio));
		val = bmips_gpio_r32(data_reg) & ~(1 << gpio);
		val |= (1 << gpio);
		bmips_gpio_w32(data_reg, val);
	}
}

/*
static void bcm3383_pcie_reset(int port, int poweron)
{
	*BCM3383_PCIEH_HARD_DEBUG_REG(port) &= ~BCM3383_PCIE_HARD_DEBUG_SERDES_IDDQ;
	mdelay(10);
	bcm3383_intc->soft_resetb |= BCM3383_SOFT_RESET_PCIE_CORE(port);
	mdelay(10);
	bcm3383_intc->soft_resetb_low &= 

	bcm3383_pcie_phy_mode_cfg(port);

}*/

void bcm3383_pinmux_select(int index)
{
	u32 val, reg;
	struct bcm3383_mux *mux = &bcm3383_mux_data[index];

	if (!mux->mask) {
		return;
	}

	pr_info("%s: setting pin mux sel %d, word %d, val %d\n",
			__func__, index, mux->word, mux->val);

	reg = 0xc0 + (mux->word << 2);

	val = bmips_gpio_r32(reg);
	val &= ~(mux->mask << mux->shift);
	val |= (mux->val << mux->shift);
	bmips_gpio_w32(reg, val);
}

static void bcm3383_pcie_power_up(int port)
{
	bcm3383_pinmux_select(6);
	bcm3383_gpio_set_out(port + 4);

	mdelay(10);

	if (port == 0) {
		*(volatile u32*)(0xb4e00364) &= ~(1 << 24);
	} else {
		*(volatile u32*)(0xb4e00368) &= ~(1 << 8);
	}

	BCM3383_INTC->clk_ctrl_high |= (0x800000 << port);
	BCM3383_INTC->clk_ctrl_low |= (0x10 << port);
	BCM3383_INTC->clk_ctrl_high |= (0x200 << port);
	BCM3383_INTC->clk_ctrl_ubus |= (0x10 << port);

	mdelay(100);

	BCM3383_INTC->soft_resetb_low &= ~(0x4 << port);
	BCM3383_INTC->soft_resetb_high &= ~(0x100 << port);
	BCM3383_INTC->soft_reset &= ~(0x40000 << port);

	mdelay(10);

	BCM3383_INTC->soft_resetb_low |= (0x4 << port);
	BCM3383_INTC->soft_resetb_high |= (0x100 << port);

	*BCM3383_PCIE_HARD_DEBUG_REG(port) &= ~BCM3383_PCIE_SERDES_IDDQ;

	if (0) {
		bcm3383_pcie_phy_mode_cfg(port);
	}

	BCM3383_INTC->soft_reset |= (0x40000 << port);

	mdelay(20);
}

static int bcm3383_pcie_power_down(int port)
{
	

}

static int bcm3383_pcie_init_port(int port)
{
	int link, timeout;

	if (port > 1) {
		return 0;
	}

	bcm3383_pcie_power_up(port);

	BCM3383_PCIE_INTR_REGS(port)->mask_set = 2;

	*BCM3383_PCIE_HARD_DEBUG_REG(port) &= ~BCM3383_PCIE_SERDES_IDDQ;

	link = 0;
	timeout = 10;

	while (timeout-- > 0) {
		if (*BCM3383_PCIE_STATUS_REG(port) & 0x20) {
			link = 1;
			break;
		}
		mdelay(20);
	}

	if (!link) {
		pr_info("%s: link status is down on port %d\n", __func__, port);
#if 0
		pr_info("%s: no link status on port %d: %08x=%08x\n", __func__, port,
				(int)BCM3383_PCIE_STATUS_REG(port), *BCM3383_PCIE_STATUS_REG(port));
#endif
		//bcm3383_pcie_power_down(port);
	} else {
		pr_info("%s: link status is up on port %d\n", __func__, port);
	}

	return link;

#if 0
	//volatile u32 *reg;
	u32 base;
	int link, timeout;

	if (port > 1) {
		return;
	}

	bcm3383_set_pcie_pinmux();

	if ((*((u8*)0xb4e0044f) >> 4) == 1) {
		pr_info("%s: pcie not supported on port %d\n", __func__, port);
		return;
	}

	bcm3383_gpio_set_out(port + 4);

	if (0) {
		bcm3383_pcie_phy_mode_cfg(port);
	}

	BCM3383_INTC->soft_reset |= (0x4 << port);

	mdelay(200);

	BCM3383_PCIE_INTR_REGS(port)->mask_set = 2;

	*BCM3383_PCIE_HARD_DEBUG_REG(port) &= ~BCM3383_PCIE_SERDES_IDDQ;

	link = 0;
	timeout = 10;

	while (timeout-- > 0) {
		if (*BCM3383_PCIE_STATUS_REG(port) & 0x20) {
			link = 1;
			break;
		}
		mdelay(10);
	}

	if (!link) {
		pr_info("%s: no link status on port %d\n", __func__, port);
		//bcm3383_pcie_power_down(port);
		return;
	}

	base = bcm3383_pcie_base[port] + 0x9000;

	bcm3383_pcie_cfg_select(0, 0);
	pr_info("%s: 00:00.0 [%08x]\n", __func__, *(volatile u32*)base);
	bcm3383_pcie_cfg_select(1, 0);
	pr_info("%s: 01:00.0 [%08x]\n", __func__, *(volatile u32*)base);
	bcm3383_pcie_cfg_select(2, 0);
	pr_info("%s: 02:00.0 [%08x]\n", __func__, *(volatile u32*)base);

	pr_info("%s: write=%p\n", __func__, bcm3383_pcie_controller.pci_ops->write);

	//register_pci_controller(&bcm3383_pcie_controller);
#endif
}

static int bcm3383_pci_probe(struct platform_device *pdev)
{
	bcm3383_pcie_init_port(0);
	bcm3383_pcie_init_port(1);
	register_pci_controller(&bcm3383_pcie0);
	register_pci_controller(&bcm3383_pcie1);
	return 0;
}

int __init pcibios_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	return IRQ_PCIE;
}

int pcibios_plat_dev_init(struct pci_dev *dev)
{
	return 0;
}

static u8 pci_read8(struct pci_dev *dev, int where)
{
	u8 data;
	if (pci_read_config_byte(dev, 0x19, &data) == 0) {
		return data;
	} else {
		return 0xff;
	}
}

static void bcm3383_pcie_fixup(struct pci_dev *dev)
{
	u32 dword;

	dev->class = 0x0604 << 8;

	pci_write_config_dword(dev, 0x18, 0x00010100);
	pci_write_config_dword(dev, 0x04, 0x00100006);

	pci_read_config_dword(dev, 0x04, &dword);
	pr_info("%s: 0x04: %08x\n", __func__, dword);

	pci_read_config_dword(dev, 0x18, &dword);
	pr_info("%s: 0x18: %08x\n", __func__, dword);

	/*
	pci_write_config_byte(dev, 0x19, dev->bus->number == 0 ? 1 : 3);
	pci_write_config_byte(dev, 0x20, dev->bus->number == 0 ? 1 : 3);
	pr_info("%s: d->class=%08x\n", __func__, dev->class);
	pr_info("%s: sub bus: %d\n", __func__, pci_read8(dev, 0x19));
	pr_info("%s: sub bus: %d\n", __func__, pci_read8(dev, 0x20));
	*/
}

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_BROADCOM, 0x3383, bcm3383_pcie_fixup);

static const struct of_device_id bcm3383_of_ids[] = {
	{ .compatible = "brcm,bcm3383-pci" },
	{},
};

static struct platform_driver bcm3383_pci_driver = {
	.probe = bcm3383_pci_probe,
	.driver = {
		.name = "bcm3383-pci",
		.of_match_table = of_match_ptr(bcm3383_of_ids)
	},
};

static int __init bcm3383_pci_init(void)
{
	return platform_driver_register(&bcm3383_pci_driver);
}

arch_initcall(bcm3383_pci_init);
