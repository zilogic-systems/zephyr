/*
 *
 * Copyright (c) 2018 Zilogic Systems.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <clock_control.h>
#include <misc/util.h>
#include <sys_io.h>
#include <clock_control/lpc17xx_clock_control.h>

#define CCM_BASE_ADDR CONFIG_CLOCK_BASE_ADDRESS

enum clock_control_regs {
	CLKSRCSEL_OFFSET		= 0x10C,
	PLL0CON_OFFSET			= 0x080,
	PLL0CFG_OFFSET			= 0x084,
	PLL0STAT_OFFSET			= 0x088,
	PLL0FEED_OFFSET			= 0x08C,
	PLL1CON_OFFSET			= 0x0A0,
	PLL1CFG_OFFSET			= 0x0A4,
	PLL1STAT_OFFSET			= 0x0A8,
	PLL1FEED_OFFSET			= 0x0AC,
	CCLKCFG_OFFSET			= 0x104,
	USBCLKCFG_OFFSET		= 0x108,
	PCLKSEL0_OFFSET			= 0x1A8,
	PCLKSEL1_OFFSET			= 0x1AC,
	PCON_OFFSET                     = 0x0C0,
	PCONP_OFFSET			= 0x0C4,
	CLKOUTCFG_OFFSET		= 0x1C8
};

#define CCM_REG_ADDR(offset)	(CCM_BASE_ADDR + offset)

static void sys_set_bits(u32_t address, u32_t mask,
			 u32_t shift, u32_t data)
{
	u32_t temp;

	temp = sys_read32(address);
	temp &= ~(mask << shift);
	temp |= (data & mask) << shift;
	sys_write32(temp, address);
}

static u32_t select_pclk_addr(u32_t en)
{
	u32_t addr;

	addr = CCM_BASE_ADDR + PCLKSEL0_OFFSET;
	addr += (en < 16) ? 0 : 4;

	return addr;
}

static inline int lpc17xx_clock_control_on(struct device *dev,
					   clock_control_subsys_t sub_system)
{
	struct lpc17xx_clock_t *pclk = (struct lpc17xx_clock_t *)(sub_system);
	u32_t shift;

	shift = (pclk->en * 2) % 32;
	sys_set_bit(CCM_REG_ADDR(PCONP_OFFSET), pclk->en);
	sys_set_bits(select_pclk_addr(pclk->en), 0x3, shift, pclk->sel);

	return 0;
}


static inline int lpc17xx_clock_control_off(struct device *dev,
					    clock_control_subsys_t sub_system)
{
	struct lpc17xx_clock_t *pclk = (struct lpc17xx_clock_t *)(sub_system);
	u32_t shift;

	shift = (pclk->en * 2) % 32;
	sys_clear_bit(CCM_REG_ADDR(PCONP_OFFSET), pclk->en);
	sys_set_bits(select_pclk_addr(pclk->en), 0x3, shift, 0x0);

	return 0;
}


static int lpc17xx_clock_control_get_subsys_rate(
	struct device *clock, clock_control_subsys_t sub_system, u32_t *rate)
{
	struct lpc17xx_clock_t *pclk = (struct lpc17xx_clock_t *)(sub_system);

	switch (pclk->sel) {
	case 0:
		*rate = (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 4);
		break;
	case 1:
		*rate = (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC);
		break;
	case 2:
		*rate = (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 2);
		break;
	case 3:
		*rate = (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 8);
	}

	return 0;
}

static int lpc17xx_clock_control_init(struct device *dev)
{
	return 0;
}

static struct clock_control_driver_api lpc17xx_clock_control_api = {
	.on = lpc17xx_clock_control_on,
	.off = lpc17xx_clock_control_off,
	.get_rate = lpc17xx_clock_control_get_subsys_rate,
};

DEVICE_AND_API_INIT(clock_lpc17xx, CONFIG_CLOCK_LABEL,
		    &lpc17xx_clock_control_init,
		    NULL, NULL,
		    PRE_KERNEL_1,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &lpc17xx_clock_control_api);
