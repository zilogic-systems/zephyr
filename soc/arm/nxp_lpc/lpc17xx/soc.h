/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Board configuration macros for the nxp_lpc54114 platform
 *
 * This header file is used to specify and describe board-level aspects for the
 * 'nxp_lpc17xx' platform.
 */

#ifndef _SOC__H_
#define _SOC__H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef _ASMLANGUAGE

enum sysconfig_reg {

	CLKSRCSEL = 0x400FC10C,
	PLL0CON = 0x400FC080,
	PLL0CFG = 0x400FC084,
	PLL0FEED = 0x400FC08C,
	CCLKCFG = 0x400FC104,
	PCLKSEL1 = 0x400FC1AC,
	SCS = 0x400FC1A0,
	FLASHCFG = 0x400FC000
};

enum scs_config {
	OSCRANGE = 4,
	OSCEN = 5,
	OSCSTAT = 6
};

/* This enums used for enable clock to peripheral, and
 * here we use PCLKSEL1_PCLK_FIRST enum to make each enum
 * value as unique.
 */
enum pclksel_config {
	PCLKSEL1_PCLK_FIRST             = 100,
	PCLKSEL1_PCLK_GPIOINT_DIV1	= 2 + PCLKSEL1_PCLK_FIRST,
	PCLKSEL1_PCLK_PCB_DIV1		= 4 + PCLKSEL1_PCLK_FIRST,
	PCLKSEL1_PCLK_SYSCON_DIV1	= 28 + PCLKSEL1_PCLK_FIRST,
};

enum flashcfg_config {
	FLASHTIM = 12,
	RESERVED_value = 0x3A
};

enum pll0cfg_config {
	PLL0CFG_MSEL0_0_bit = 0,
	PLL0CFG_NSEL0 = 16,
	PLL0CFG_MSEL0 = (1 << PLL0CFG_MSEL0_0_bit)
};


enum pllofeed_config {

	PLL0FEED_FIRST	= 0xAA,
	PLL0FEED_SECOND	= 0x55
};

enum cclkcfg_config {
	CCLKSEL_bit = 0
};

enum pll0con_config {
	PLLE0 = 0,
	PLLC0 = 1
};

#endif /* !_ASMLANGUAGE */

#define IOCON_PIO_FUNC0		0x00u
#define IOCON_PIO_FUNC1		0x01u
#define IOCON_PIO_FUNC2		0x02u
#define IOCON_PIO_FUNC3         0x03u
#define IOCON_PIO_PULLUP        0x00u
#define IOCON_PIO_REPEATER      0x04u
#define IOCON_PIO_NPUNPD        0x08u
#define IOCON_PIO_PULLDOWN      0x0Cu
#define IOCON_PIO_NOD           0x00u
#define IOCON_PIO_OD            0x10u

#ifdef __cplusplus
}
#endif

#endif /* _SOC__H_ */
