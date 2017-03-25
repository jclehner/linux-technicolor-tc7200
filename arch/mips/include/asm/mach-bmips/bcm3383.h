/*
 * Copyright (C) Joseph C. Lehner 2017
 * Licensed under the GNU GPL v2.0
 */

#ifndef _BCM3383_REGS_H
#define _BCM3383_REGS_H

#define BCM3383_PCIE0_BASE	0xb2800000
#define BCM3383_PCIE1_BASE	0xb28a0000
#define BCM3383_MEMC_BASE	0xb2000000

#define BCM3383_GMAC0_BASE	0xb2200000

#define BCM3383_INTC_BASE	0xb4e00000
#define BCM3383_GPIO_BASE	0xb4e00100
#define BCM3383_I2C_BASE	0xb4e00e00

// UTP
#define BCM3383_4KE_CORE0_BASE	0xb3000000
#define BCM3383_4KE_CORE1_BASE	0xb3200000
// aka FAP
#define BCM3383_4KE_CORE2_BASE	0xb3400000
// aka MSP (MSG_PROC)
#define BCM3383_4KE_CORE3_BASE	0xb3600000
// aka MEP (NATP)
#define BCM3383_4KE_CORE4_BASE	0xb4200000

#define BCM3383_4KE_CORE_RESET_REG	0x108c


struct bcm3383_interrupt_data
{
	u32 mask;
	u32 status;
};

struct bcm3383_intc {
	u32 rev_id;
	u32 clk_ctrl_low;
	u32 clk_ctrl_high;
	u32 clk_ctrl_ubus;
	u32 timer_ctrl;
	struct bcm3383_interrupt_data docsis_irq[3];
	u32 int_periph_irq_status;
	struct bcm3383_interrupt_data periph_irq[4];
	struct bcm3383_interrupt_data iop_irq[2];
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
};

#define BCM3383_INTC ((volatile struct bcm3383_intc*)BCM3383_INTC_BASE)

// 0x04
#define BCM3383_CLK_CTRL_LO_FPM			(1 << 1)
#define BCM3383_CLK_CTRL_LO_DAVIC		(1 << 2) /* XXX NAND? GMAC? */
#define BCM3383_CLK_CTRL_LO_PCIE0		(1 << 4)
#define BCM3383_CLK_CTRL_LO_PCIE1		(1 << 5)
#define BCM3383_CLK_CTRL_LO_GMAC0		(1 << 6)
#define BCM3383_CLK_CTRL_LO_USBH		(1 << 7)
#define BCM3383_CLK_CTRL_LO_GMAC1		(1 << 8) /* XXX maybe */
#define BCM3383_CLK_CTRL_LO_UNK1		(1 << 11) /* XXX GMAC? SWITCH? */
#define BCM3383_CLK_CTRL_LO_NAND		(1 << 17)
#define BCM3383_CLK_CTRL_LO_AVS			(1 << 14)
#define BCM3383_CLK_CTRL_LO_MTSIF		(1 << 19)
#define BCM3383_CLK_CTRL_LO_TESTBUS		(1 << 24)

// 0x08
#define BCM3383_CLK_CTRL_HI_GMAC		(1 << 8)
#define BCM3383_CLK_CTRL_HI_APM			(1 << 12) /* XXX could be LO too */
#define BCM3383_CLK_CTRL_HI_USBD		(1 << 19)
#define BCM3383_CLK_CTRL_HI_PCIE0		(1 << 23)
#define BCM3383_CLK_CTRL_HI_PCIE1		(1 << 24)
#define BCM3383_CLK_CTRL_HI_USBH		(1 << 30)

/* DOCSIS DS channels 1-8 */
#define BCM3383_CLK_CTRL_HI_DS_MASK		0x000000ff


// 0x0c
#define BCM3383_CLK_CTRL_UBUS_DAVIC		(1 << 2)
#define BCM3383_CLK_CTRL_UBUS_PCIE0		(1 << 4)
#define BCM3383_CLK_CTRL_UBUS_PCIE1		(1 << 5)
#define BCM3383_CLK_CTRL_UBUS_GMAC		(1 << 6)
#define BCM3383_CLK_CTRL_UBUS_USBH		(1 << 7)

// 0x8c
#define BCM3383_SOFT_RESET_LO_PCIE(x)	(1 << (2 + ((x) & 2)))

#define BCM3383_SOFT_RESET_LO_PCIE0		(1 << 2)
#define BCM3383_SOFT_RESET_LO_PCIE1		(1 << 3)
#define BCM3383_SOFT_RESET_LO_GMAC0		(1 << 6)
#define BCM3383_SOFT_RESET_LO_NAND		(1 << 7)
#define BCM3383_SOFT_RESET_LO_GMAC1		(1 << 8)
#define BCM3383_SOFT_RESET_LO_USBH		(1 << 14)

#define BCM3383_SOFT_RESET_LO_XDRV0		(1 << 11)
#define BCM3383_SOFT_RESET_LO_XDRV1		(1 << 12)

#define BCM3383_SOFT_RESET_LO_4KE_CORE0		(1 << 20)
#define BCM3383_SOFT_RESET_LO_4KE_CORE1		(1 << 21)
#define BCM3383_SOFT_RESET_LO_4KE_CORE2		(1 << 22)
#define BCM3383_SOFT_RESET_LO_4KE_CORE3		(1 << 23)
#define BCM3383_SOFT_RESET_LO_4KE_CORE4		(1 << 24)
#define BCM3383_SOFT_RESET_LO_4KE_CORE5		(1 << 9)

#define BCM3383_SOFT_RESET_LO_DAVIC_MAC0	(1 << 29)
#define BCM3383_SOFT_RESET_LO_DAVIC_MAC1	(1 << 28)

// 0x90
#define BCM3383_SOFT_RESET_HI_PCIE(x)	(1 << (8 + ((x) & 1)))

#define BCM3383_SOFT_RESET_HI_USBD		(1 << 10)
#define BCM3383_SOFT_RESET_HI_PCIE0		(1 << 8)
#define BCM3383_SOFT_RESET_HI_PCIE1		(1 << 9)


// 0x94
#define BCM3383_SOFT_RESET_PCIE(x)			(1 << (18 + ((x) & 1)))

#define BCM3383_SOFT_RESET_MTSIF			(1 << 1)
#define BCM3383_SOFT_RESET_PCIE0			(1 << 18)
#define BCM3383_SOFT_RESET_PCIE1			(1 << 19)

struct bcm3383_mux {
	uint8_t word;
	uint8_t mask;
	uint8_t shift;
	uint8_t val;
} bcm3383_mux_data[] = {
	{ 0, 0,  0, 0 },
	{ 0, 0,  0, 0 },
	{ 0, 0,  0, 0 },
	{ 0, 0,  0, 0 },
	{ 1, 3, 16, 1 },
	{ 0, 7,  0, 5 },
	{ 0, 7,  3, 5 },
	{ 0, 7,  6, 5 },
	{ 0, 7,  9, 5 },
	{ 0, 7, 12, 5 },
	{ 0, 7, 15, 5 },
	{ 0, 7, 18, 5 },
	{ 0, 7, 21, 5 },
	{ 0, 7, 24, 5 },
	{ 0, 7, 27, 5 },
	{ 0, 7, 30, 5 },
	{ 1, 7,  1, 5 },
	{ 1, 7,  4, 5 },
	{ 1, 7,  7, 5 },
	{ 2, 7,  5, 1 },
	{ 0, 0,  0, 0 },
	{ 0, 7, 15, 1 },
	{ 0, 7, 18, 1 },
	{ 0, 7, 18, 1 },
	{ 0, 7, 12, 1 },
	{ 1, 7, 13, 1 },
	{ 1, 7, 16, 1 },
	{ 1, 7, 19, 1 },
	{ 1, 7, 22, 1 },
	{ 1, 7, 10, 2 },
	{ 1, 7, 10, 4 },
	{ 2, 7,  2, 1 },
	{ 2, 7,  5, 1 },
	{ 2, 7,  5, 5 },
	{ 1, 7, 19, 5 },
	{ 1, 7, 22, 5 },
	{ 2, 7,  2, 5 },
	{ 2, 7, 23, 1 },
};

inline u32 bmips_r32(u32 base, u32 reg)
{
	return *(volatile u32*)(base + reg);
}

inline void bmips_w32(u32 base, u32 reg, u32 val)
{
	*(volatile u32*)(base + reg) = val;
}

#define BMIPS_DEF_RW_REG_FUNCS(name, base) \
	inline u32 bmips_ ## name ## _r32(u32 reg) \
	{ \
		return bmips_r32(base, reg); \
	} \
	\
	inline void bmips_ ## name ##_w32(u32 reg, u32 val) \
	{ \
		bmips_w32(base, reg, val); \
	} \
	inline volatile u32 *bmips_ ## name ##_reg(u32 reg) \
	{ \
		return (volatile u32*)(base + reg); \
	}

BMIPS_DEF_RW_REG_FUNCS(perf, BCM3383_INTC_BASE)
BMIPS_DEF_RW_REG_FUNCS(gpio, BCM3383_GPIO_BASE)

#endif
