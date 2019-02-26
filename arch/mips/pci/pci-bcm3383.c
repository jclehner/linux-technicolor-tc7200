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


#define BRIDGE 0
#define DEVICE 1

//#define IOREMAP

#ifdef IOREMAP
static void __iomem *base_addr[2];
#else
static u32 base_addr[2];
#endif

static u32 bcm3383_pcie_base[2] = { BCM3383_PCIE0_BASE, 0xb2a00000 };


#define PCIE_DEV_OFFSET 0x9000

static inline int bcm3383_is_root_bus(u8 bus)
{
	return bus == 0 || bus == 2;
}

static inline int bcm3383_bus_to_port(u8 bus)
{
	return bus < 2 ? 0 : 1;
}

#ifdef IOREMAP
static inline void __iomem *pcie_base(struct pci_bus *bus)
{
	return base_addr[bus->number == 0 ? BRIDGE : DEVICE];
}
#else
static inline u32 pcie_base(struct pci_bus *bus)
{
	return base_addr[(!bus || bus->number == 0) ? BRIDGE : DEVICE];
}
#endif

static inline volatile u32 *bcm3383_pcie_get_addr(u8 bus, int where)
{
	if (!bcm3383_is_root_bus(bus)) {
		where |= 0x9000;
	}

	return BCM3383_PCIE_REG(bcm3383_bus_to_port(bus), where & ~3);
}

static void check_reg(u32 reg, const char *where)
{
	if (reg & 3) {
		pr_warn("%s: bad register %04x\n", where, reg);
	}
}

#define CHECK_REG(reg) check_reg(reg, __func__)

static inline int bcm3383_dev_to_port(struct pci_dev *dev)
{
	return bcm3383_bus_to_port(dev->bus->number);
}

static inline void pcie_core_w32(u32 reg, u32 val)
{
#ifdef IOREMAP
	iowrite32be(val, base_addr[BRIDGE] + reg);
#else
	*(volatile u32*)(base_addr[BRIDGE] + reg) = val;
#endif
}

static inline u32 pcie_core_r32(u32 reg)
{
#ifdef IOREMAP
	return ioread32be(base_addr[BRIDGE] + reg);
#else
	return *(volatile u32*)(base_addr[BRIDGE] + reg);
#endif
}

static inline void pcie_bus_w32(struct pci_bus *bus, u32 reg, u32 val)
{
#ifdef IOREMAP
	iowrite32be(val, pcie_base(bus) + reg);
#else
	*(volatile u32*)(pcie_base(bus) + reg) = val;
#endif
}

static inline u32 pcie_bus_r32(struct pci_bus *bus, u32 reg)
{
#ifdef IOREMAP
	return ioread32be(pcie_base(bus) + reg);
#else
	return *(volatile u32*)(pcie_base(bus) + reg);
#endif
}

#define PCIE_EXT_CFG_REGS 0x8400

static inline void bcm3383_pcie_cfg_select(u8 bus, unsigned devfn)
{
	*BCM3383_PCIE_EXT_CFG_REGS(bcm3383_bus_to_port(bus), 0x0) = bus << 20
		| PCI_SLOT(devfn) << 15 | PCI_FUNC(devfn) << 12;
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
	data = pcie_bus_r32(bus, where & ~3);
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
	u32 status, data;
	int err = PCIBIOS_DEVICE_NOT_FOUND;

	if (!bcm3383_pcie_can_access(bus ? bus->number : 0, devfn)) {
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	status = read_c0_config();
	write_c0_config(status & ~1);

	data = pcie_bus_r32(bus, where & ~3);
	if (data == 0xdeaddead) {
		goto out;
	}

	if (size == 1) {
		data = (data & ~(0xff << ((where & 3) << 3))) |
			(val << ((where & 3) << 3));
	} else if (size == 2) {
		data = (data & ~(0xffff << ((where & 3) << 3))) |
			(val << ((where & 3) << 3));
	} else {
		data = val;
	}

	pcie_bus_w32(bus, where & ~3, data);
	err = PCIBIOS_SUCCESSFUL;
out:
	write_c0_config(status);
	return err;
}

static struct pci_ops bcm3383_pcie_ops = {
	.read = bcm3383_pcie_read,
	.write = bcm3383_pcie_write
};

#define OFMEM

static struct resource bcm3383_res_pci_io;
static struct resource bcm3383_res_pci_mem
#ifndef OFMEM
= {
	.start = 0xa0000000,
	.end = 0xa0000000 + 0x10000000 -1,
	.flags = IORESOURCE_MEM,
}
#endif
;

static struct pci_controller bcm3383_controller = {
	.pci_ops		= &bcm3383_pcie_ops,
	.mem_resource	= &bcm3383_res_pci_mem,
	//.mem_offset		= 0xa0000000,
	.io_resource 	= &bcm3383_res_pci_io,
	//.io_offset = 0,
	//.io_map_base = 0
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

	if (1) {
		bcm3383_pcie_phy_mode_cfg(port);
	}

	BCM3383_INTC->soft_reset |= (0x40000 << port);

	mdelay(20);
}

static int bcm3383_pcie_power_down(int port)
{
	return -1;
}


#define BCM_MEM_LIMIT_BASE 0x20
#define BCM_PREF_LIMIT_BASE 0x24
#define BCM_PREF_BASE_UPPER32 0x28
#define BCM_PREF_LIMIT_UPPER32 0x2c

#define PCIE_ID_VAL3				0x043c
#define PCIE_CONFIG2				0x408
#define PCIE_PHY_CTRL_1				0x1804
#define PCIE_MISC_CTRL				0x4008
#define PCIE_MEM_WIN0_LO			0x400c
#define PCIE_MEM_WIN0_BASE_LIMIT	0x4070
#define PCIE_BAR1_CONFIG_LO			0x4030
#define PCIE_BAR2_CONFIG_LO			0x4034
#define PCIE_BAR3_CONFIG_LO			0x403c
#define PCIE_REVISION				0x406c
#define PCIE_UBUS_CTRL				0x4080
#define PCIE_UBUS_TIMEOUT			0x4084
#define PCIE_UBUS_BAR1_CFG_REMAP	0x4088
#define PCIE_UBUS_BAR2_CFG_REMAP	0x408c
#define PCIE_UBUS_BAR3_CFG_REMAP	0x4090
#define PCIE_INTR_MASK_CLEAR		0x830c

#define PCIE_UBUS_BAR_CFG_ACCESS_EN	(1 << 0)
#define PCIE_POWERDOWN_P1_PLL_ENA	(1 << 23)
#define PCIE_BAR_CONFIG_LO_128MB	0xc
#define PCIE_ID_VAL3_CLASS_MASK		0x00ffffff
#define PCIE_ID_VAL3_CLASS_SHIFT	16
#define PCIE_CONFIG2_BAR1_SIZE_MASK	0x0000000f
#define PCIE_INTA					(1 << 1)

static int bcm3383_pcie_init_bridge(int port)
{
	u32 val;
	struct resource *mem = bcm3383_controller.mem_resource;
	if (!mem) {
		pr_info("%s: !mem\n", __func__);
		return -EINVAL;
	}

#if 1
	val = (mem->start & 0xfff00000) >> 16;
	//bcm3383_pcie_write(NULL, 0, PCI_PREF_BASE_UPPER32, 2, 0);
	//bcm3383_pcie_write(NULL, 0, PCI_PREF_MEMORY_BASE, 2, val);
	bcm3383_pcie_write(NULL, 0, PCI_MEMORY_BASE, 2, val);
	bcm3383_pcie_write(NULL, 0, PCI_IO_BASE, 2, 0);

	val = (mem->end & 0xfff00000) >> 16;
	//bcm3383_pcie_write(NULL, 0, PCI_PREF_LIMIT_UPPER32, 2, 0);
	//bcm3383_pcie_write(NULL, 0, PCI_PREF_MEMORY_LIMIT, 2, val);
	bcm3383_pcie_write(NULL, 0, PCI_MEMORY_LIMIT, 2, val);
	bcm3383_pcie_write(NULL, 0, PCI_IO_LIMIT, 2, 0);

#if 0
	pcie_core_w32(BCM_PREF_LIMIT_BASE, 0xfff0);
	pcie_core_w32(BCM_PREF_BASE_UPPER32, 0);
	pcie_core_w32(BCM_PREF_LIMIT_UPPER32, 0);

	val = (mem->start & 0xffff0000) | mem->end;
	pcie_core_w32(BCM_MEM_LIMIT_BASE, val);
	pr_info("%s: limit_base=%08x\n", __func__, val);
#endif

	val = mem->start & 0xfff00000;
	pcie_core_w32(PCIE_MEM_WIN0_LO, mem->start & 0xfff00000);
	pr_info("%s: mem_win0_lo=%08x\n", __func__, val);

	val = (mem->end & 0xfff00000) | (mem->start >> 16);
	pcie_core_w32(PCIE_MEM_WIN0_BASE_LIMIT, val);	
	pr_info("%s: mem_win0_base_limit=%08x\n", __func__, val);

#else
	val = ((mem->end & 0xfff00000) | (mem->start >> 23)) << 4;
	pcie_core_w32(PCIE_MEM_WIN0_BASE_LIMIT, val);

	val = pcie_core_r32(PCIE_MEM_WIN0_LO) | (mem->start & 0xfff00000);
	pcie_core_w32(PCIE_MEM_WIN0_LO, val);
#endif

	pcie_core_w32(PCIE_BAR1_CONFIG_LO, PCIE_BAR_CONFIG_LO_128MB);
	pcie_core_w32(PCIE_UBUS_BAR1_CFG_REMAP, PCIE_UBUS_BAR_CFG_ACCESS_EN);

	pcie_core_w32(PCIE_BAR2_CONFIG_LO, 0x10000000 | PCIE_BAR_CONFIG_LO_128MB);
	pcie_core_w32(PCIE_BAR3_CONFIG_LO, PCIE_BAR_CONFIG_LO_128MB);

	pcie_core_w32(0x188, 0x8);

#if 0
	bcm3383_pcie_cfg_select(1, 0);

	val = pcie_core_r32(PCIE_ID_VAL3) & ~PCIE_ID_VAL3_CLASS_MASK;
	pcie_core_w32(PCIE_ID_VAL3, val | (PCI_CLASS_BRIDGE_PCI << 8));

	val = pcie_core_r32(PCIE_CONFIG2) & ~PCIE_CONFIG2_BAR1_SIZE_MASK;
	pcie_core_w32(PCIE_CONFIG2, val);
#endif

	val = pcie_core_r32(PCIE_REVISION);
	pr_info("%s: PCIe rev %d.%d\n", __func__, (val >> 8) & 0xff, val & 0xff);

	if (val >= 0x0202) {
		pcie_core_w32(PCIE_UBUS_BAR2_CFG_REMAP, PCIE_UBUS_BAR_CFG_ACCESS_EN);
		pcie_core_w32(PCIE_UBUS_BAR3_CFG_REMAP, PCIE_UBUS_BAR_CFG_ACCESS_EN);

		if (true /* FIXME */) {
			val = pcie_core_r32(PCIE_PHY_CTRL_1) | PCIE_POWERDOWN_P1_PLL_ENA;
			pcie_core_w32(PCIE_PHY_CTRL_1, val);
		}
	} else {
		pcie_core_w32(PCIE_MISC_CTRL, 0x60001000);
		pcie_core_w32(PCIE_UBUS_TIMEOUT, 0);
	}

	pcie_core_w32(PCIE_UBUS_CTRL, 0x2220);
	pcie_core_w32(PCIE_INTR_MASK_CLEAR, PCIE_INTA);

	return 0;
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
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		return -EINVAL;
	}

#ifdef IOREMAP
	base_addr[0] = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base_addr[0])) {
		dev_info(&pdev->dev, "ioremap failed for pcie memory\n");
		return PTR_ERR(base_addr[0]);
	}
#else
	base_addr[0] = KSEG1ADDR(res->start);
#endif
	base_addr[1] = base_addr[0] + 0x9000;

	dev_info(&pdev->dev, "base=[%08x %08x]\n", base_addr[0], base_addr[1]);

	iomem_resource.start = 0;
	iomem_resource.end = ~0;
	ioport_resource.start = 0;
	ioport_resource.end = ~0;

	if (bcm3383_pcie_init_port(0)) {
#ifdef OFMEM
		pci_load_of_ranges(&bcm3383_controller, pdev->dev.of_node);
#endif
		//bcm3383_pcie_init_bridge(0);

		dev_info(&pdev->dev, "mem: %08x-%08x\n", bcm3383_controller.mem_resource->start,
				bcm3383_controller.mem_resource->end);
		register_pci_controller(&bcm3383_controller);
	}

	/*
	if (bcm3383_pcie_init_port(1)) {
		register_pci_controller(&bcm3383_pcie1);
	}
	*/

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

static void bcm3383_pcie_fixup_early(struct pci_dev *dev)
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

static void bcm3383_pcie_fixup_final(struct pci_dev *dev)
{
	bcm3383_pcie_init_bridge(0);
	return;

	u32 tmp, base = /*pcie_dev_r32(dev, PCI_BASE_ADDRESS_0) & 0xfffffff0*/ 0xa0000000;

	dev_info(&dev->dev, "base=%08x\n", base);

	//pcie_dev_w32(dev, BCM_MEM_LIMIT_BASE, 0xfff0);
	//pcie_dev_w32(dev, BCM_PREF_BASE_UPPER32, 0);
	//pcie_dev_w32(dev, BCM_PREF_LIMIT_UPPER32, 0);
	
	pcie_core_w32(BCM_PREF_LIMIT_BASE, 0xa0f0fff0);

	tmp = (base & 0xffff0000) | (base + 0xffffff);
	dev_info(&dev->dev, "limit=%08x\n", tmp);
	pcie_core_w32(BCM_MEM_LIMIT_BASE, tmp);

	// PCIE_MISC_CPU2_PCI_MEM_WIN0_LO
	pcie_core_w32(0x400c, base); // 0xa0000000

	tmp = ((base + 0xffffff) & (0xfff00000)) | (base >> 16);
	dev_info(&dev->dev, "base_limit=%08x\n", tmp);

	// PCIE_MISC_CPU_2_PCI_MEM_WIN0_BASE_LIMIT
	pcie_core_w32(0x4070, tmp); // 0xa0f0a000

	// PCIE_MISC_RC_BAR2_CONFIG_LO
	pcie_core_w32(0x4034, 0x0c);

	// PCIE_MISC_RC_BAR3_CONFIG_LO
	pcie_core_w32(0x403c, 0x1000000c);

	pcie_core_w32(0x188, 8);

	tmp = pcie_core_r32(0x406c);

	dev_info(&dev->dev, "PCIe rev %d.%d\n", (tmp >> 8) & 0xff, tmp & 0xff);

	// PCIE_MISC_REVISION
	if (pcie_core_r32(0x406c) >= 0x0202) {
		// PCIE_MISC_UBUS_BAR2_CONFIG = ACCESS_EN
		pcie_core_w32(0x408c, 1);
		// PCIE_MISC_UBUS_BAR3_CONFIG = ACCESS_EN
		pcie_core_w32(0x4090, 1);

		if (true /* unknown */) {
			/* PCIE_PHY_CTRL_1 |= REG_POWERDOWN_P1_PLL_ENA */
			pcie_core_w32(0x1804, pcie_core_r32(0x1804) | (1 << 23));
		}
	} else {
		// PCIE_MISC_CTRL
		pcie_core_w32(0x4008, 0x60001000);
		// PCIE_MISC_UBUS_TIMEOUT
		pcie_core_w32(0x4084, 0);
	}

	// PCIE_MISC_UBUS_CTRL
	pcie_core_w32(0x4080, 0x2220);

	// PCIE_CPU_INTR1_MASK_CLEAR
	pcie_core_w32(0x830c, 2);
}

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_BROADCOM, 0x3383, bcm3383_pcie_fixup_early);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_BROADCOM, 0x3383, bcm3383_pcie_fixup_final);

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
