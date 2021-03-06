/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2008 Maxime Bizon <mbizon@freebox.fr>
 * Copyright (C) 2014 Kevin Cernekee <cernekee@gmail.com>
 */

#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/bootmem.h>
#include <linux/clk-provider.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_platform.h>
#include <linux/libfdt.h>
#include <linux/smp.h>
#include <asm/addrspace.h>
#include <asm/bmips.h>
#include <asm/bootinfo.h>
#include <asm/cpu-type.h>
#include <asm/mipsregs.h>
#include <asm/prom.h>
#include <asm/smp-ops.h>
#include <asm/time.h>
#include <asm/traps.h>

#define RELO_NORMAL_VEC		BIT(18)

#define REG_BCM6328_OTP		((void __iomem *)CKSEG1ADDR(0x1000062c))
#define BCM6328_TP1_DISABLED	BIT(9)

#define BCM3383_USB_UBUS_CLK_EN	(1 << 7)
#define BCM3383_USB_CLK_EN		(1 << 7)
#define BCM3383_NAND_CLK_EN		(1 << 17)
#define BCM3383_SFTRST			(1 << 6)
#define BCM3383_IOC				(1 << 4)
#define BCM3383_GPIO_USB0_PWRON	(1 << 2)
#define BCM3383_GPIO_USB1_PWRON	(1 << 4)

static const unsigned long kbase = VMLINUX_LOAD_ADDRESS & 0xfff00000;

struct bmips_quirk {
	const char		*compatible;
	void			(*quirk_fn)(void);
};

static void kbase_setup(void)
{
	__raw_writel(kbase | RELO_NORMAL_VEC,
		     BMIPS_GET_CBR() + BMIPS_RELO_VECTOR_CONTROL_1);
	ebase = kbase;
}

static void bcm3384_viper_quirks(void)
{
	/*
	 * Some experimental CM boxes are set up to let CM own the Viper TP0
	 * and let Linux own TP1.  This requires moving the kernel
	 * load address to a non-conflicting region (e.g. via
	 * CONFIG_PHYSICAL_START) and supplying an alternate DTB.
	 * If we detect this condition, we need to move the MIPS exception
	 * vectors up to an area that we own.
	 *
	 * This is distinct from the OTHER special case mentioned in
	 * smp-bmips.c (boot on TP1, but enable SMP, then TP0 becomes our
	 * logical CPU#1).  For the Viper TP1 case, SMP is off limits.
	 *
	 * Also note that many BMIPS435x CPUs do not have a
	 * BMIPS_RELO_VECTOR_CONTROL_1 register, so it isn't safe to just
	 * write VMLINUX_LOAD_ADDRESS into that register on every SoC.
	 */
	board_ebase_setup = &kbase_setup;
	bmips_smp_enabled = 0;
}

/* 
 * XXX this should be moved to a separate bcm3383 clock driver
 */

struct bcm3383_interrupt
{
	u32 mask;
	u32 status;
};

volatile struct bcm3383_intc {
	u32 rev_id;
	u32 clk_ctrl_low;
	u32 clk_ctrl_high;
	u32 clk_ctrl_ubus;
	u32 timer_ctrl;
	struct bcm3383_interrupt docsis_irq[3];
	u32 int_periph_irq_status;
	struct bcm3383_interrupt periph_irq[4];
	struct bcm3383_interrupt iop_irq[2];
	u32 docsis_irq_sense;
	u32 periph_irq_sense;
	u32 iop_irq_sense;
	u32 ext0_irq_ctrl;
	u32 diag_ctrl;
	u32 ext1_irq_ctrl;
	u32 irq_out_mask;
	u32 diag_select_ctrl;
	u32 diag_read_back_low;
	u32 diag_read_back_high;
	u32 diag_misc_ctrl;
	u32 soft_resetb_low;
	u32 soft_resetb_high;
	u32 soft_reset;
	u32 ext_irq_mux_select;
} *bcm3383_intc = (struct bcm3383_intc*)0xb4e00000;

extern void bcm3383_pinmux_select(int index);

static void bcm3383_init_usb(void)
{
	volatile u32 *gpio_data_hi = (u32*)0xb4e0018c;

	volatile struct bcm3383_usb {
		u32 ctrl_setup;
		u32 pll_ctrl_1;
		u32 fladj_val;
		u32 swap_ctrl;
	} *usb = (struct bcm3383_usb*)0xb2e00200;

	unsigned i;

	/* enable ubus USB clock */
	bcm3383_intc->clk_ctrl_ubus |= BCM3383_USB_UBUS_CLK_EN;

	/* enable USB clocks */
	bcm3383_intc->clk_ctrl_low |= BCM3383_USB_CLK_EN;

	/* reset USB controller */
	usb->ctrl_setup |= BCM3383_SFTRST;
	for (; i < 1000; ++i);
	usb->ctrl_setup &= ~BCM3383_SFTRST;

	/* set polarity */
	usb->ctrl_setup |= BCM3383_IOC;

	/* ?? */
	usb->swap_ctrl |= 0x9;
	usb->pll_ctrl_1 = 0x512750c0;

	/* power on usb ports */
	*gpio_data_hi |= BCM3383_GPIO_USB0_PWRON | BCM3383_GPIO_USB1_PWRON;
}

static void bcm3383_init_nand(void)
{
	bcm3383_intc->clk_ctrl_ubus |= BCM3383_NAND_CLK_EN;
	bcm3383_intc->clk_ctrl_low |= BCM3383_NAND_CLK_EN;
}

static void bcm3383_init_gmac(void)
{
	bcm3383_intc->soft_resetb_low &= ~((1 << 6) | (1 << 8));
	bcm3383_intc->clk_ctrl_low |= (1 << 6);
	bcm3383_intc->clk_ctrl_high |= (1 << 8);
	mdelay(200);
	bcm3383_intc->soft_resetb_low |= ((1 << 6) | (1 << 8));

#if 0
	bcm3383_intc->clk_ctrl_low |= (1 << 1);
	mdelay(100);
	bcm3383_intc->clk_ctrl_low |= (1 << 11);
	mdelay(100);
	bcm3383_intc->clk_ctrl_ubus |= (1 << 8);
	bcm3383_intc->clk_ctrl_high |= (1 << 8);
	bcm3383_intc->clk_ctrl_low |= (1 << 6) | (1 << 8);
#endif
}

static void bcm63xx_fixup_cpu1(void);

static void bcm3383_quirks(void)
{
	//volatile u32 *reg;

	write_c0_status(IE_IRQ5 | read_c0_status());
	bcm63xx_fixup_cpu1();

	bcm3383_pinmux_select(10);

#if 0
	bcm3383_intc->clk_ctrl_low = 0xf636f04b;
	bcm3383_intc->clk_ctrl_high = 0xff;
	bcm3383_intc->clk_ctrl_ubus = 0x7ffff;

	reg = (u32*)0xb4e001c8;

	*reg = (*reg & 0xe3ffffff) | 0x04000000;
	*reg = (*reg & 0xfc7fffff) | 0x00800000;

	reg = (u32*)0xb4e001c4;

	*reg = (*reg & 0xfffffff1) | 0x0000000a;

	*(u32*)(0xb2000238) = 0x170;
	*(u32*)(0xb20005a0) = 0xfffff;
	*(u8*)(0xb4e00048 + 3) &= 0xf7; 
	*(u32*)(0xb4e00530) = 0;
#endif

	bcm3383_init_usb();
	bcm3383_init_nand();
	bcm3383_init_gmac();
}

static void bcm63xx_fixup_cpu1(void)
{
	/*
	 * The bootloader has set up the CPU1 reset vector at
	 * 0xa000_0200.
	 * This conflicts with the special interrupt vector (IV).
	 * The bootloader has also set up CPU1 to respond to the wrong
	 * IPI interrupt.
	 * Here we will start up CPU1 in the background and ask it to
	 * reconfigure itself then go back to sleep.
	 */
	memcpy((void *)0xa0000200, &bmips_smp_movevec, 0x20);
	__sync();
	set_c0_cause(C_SW0);
	cpumask_set_cpu(1, &bmips_booted_mask);
}

static void bcm6328_quirks(void)
{
	/* Check CPU1 status in OTP (it is usually disabled) */
	if (__raw_readl(REG_BCM6328_OTP) & BCM6328_TP1_DISABLED)
		bmips_smp_enabled = 0;
	else
		bcm63xx_fixup_cpu1();
}

static void bcm6358_quirks(void)
{
	/*
	 * BCM3368/BCM6358 need special handling for their shared TLB, so
	 * disable SMP for now
	 */
	bmips_smp_enabled = 0;
}

static void bcm6368_quirks(void)
{
	bcm63xx_fixup_cpu1();
}

static const struct bmips_quirk bmips_quirk_list[] = {
	{ "brcm,bcm3368",		&bcm6358_quirks			},
	{ "brcm,bcm3383",		&bcm3383_quirks		},
	{ "brcm,bcm3384-viper",		&bcm3384_viper_quirks		},
	{ "brcm,bcm33843-viper",	&bcm3384_viper_quirks		},
	{ "brcm,bcm6328",		&bcm6328_quirks			},
	{ "brcm,bcm6358",		&bcm6358_quirks			},
	{ "brcm,bcm6362",		&bcm6368_quirks			},
	{ "brcm,bcm6368",		&bcm6368_quirks			},
	{ "brcm,bcm63168",		&bcm6368_quirks			},
	{ "brcm,bcm63268",		&bcm6368_quirks			},
	{ },
};

void __init prom_init(void)
{
	bmips_cpu_setup();
	register_bmips_smp_ops();
}

void __init prom_free_prom_memory(void)
{
}

const char *get_system_type(void)
{
	return "Generic BMIPS kernel";
}

void __init plat_time_init(void)
{
	struct device_node *np;
	u32 freq;

	np = of_find_node_by_name(NULL, "cpus");
	if (!np)
		panic("missing 'cpus' DT node");
	if (of_property_read_u32(np, "mips-hpt-frequency", &freq) < 0)
		panic("missing 'mips-hpt-frequency' property");
	of_node_put(np);

	mips_hpt_frequency = freq;
}

extern const char __appended_dtb;

void __init plat_mem_setup(void)
{
	void *dtb;
	const struct bmips_quirk *q;

	set_io_port_base(0);
	ioport_resource.start = 0;
	ioport_resource.end = ~0;

#ifdef CONFIG_MIPS_ELF_APPENDED_DTB
	if (!fdt_check_header(&__appended_dtb))
		dtb = (void *)&__appended_dtb;
	else
#endif
	/* intended to somewhat resemble ARM; see Documentation/arm/Booting */
	if (fw_arg0 == 0 && fw_arg1 == 0xffffffff)
		dtb = phys_to_virt(fw_arg2);
	else if (fw_passed_dtb) /* UHI interface */
		dtb = (void *)fw_passed_dtb;
	else if (__dtb_start != __dtb_end)
		dtb = (void *)__dtb_start;
	else
		panic("no dtb found");

	__dt_setup_arch(dtb);

	for (q = bmips_quirk_list; q->quirk_fn; q++) {
		if (of_flat_dt_is_compatible(of_get_flat_dt_root(),
					     q->compatible)) {
			q->quirk_fn();
		}
	}
}

void __init device_tree_init(void)
{
	struct device_node *np;

	unflatten_and_copy_device_tree();

	/* Disable SMP boot unless both CPUs are listed in DT and !disabled */
	np = of_find_node_by_name(NULL, "cpus");
	if (np && of_get_available_child_count(np) <= 1)
		bmips_smp_enabled = 0;
	of_node_put(np);
}

int __init plat_of_setup(void)
{
	return __dt_register_buses("simple-bus", NULL);
}

arch_initcall(plat_of_setup);

static int __init plat_dev_init(void)
{
	of_clk_init(NULL);
	return 0;
}

device_initcall(plat_dev_init);
