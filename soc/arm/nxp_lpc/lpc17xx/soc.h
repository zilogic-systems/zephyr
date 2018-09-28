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

#ifdef CONFIG_UART_LPC17XX
#ifdef CONFIG_UART_LPC17XX_PORT_0

/* Power */
#define LPC17XX_SYSCON_BASE   	  	0x400fc000 /* -0x400fffff: System control */
#define LPC17XX_SYSCON_PCONP_OFFSET 	0x00c4 /* Power Control for Peripherals Register */
#define LPC17XX_SYSCON_PCONP        	(LPC17XX_SYSCON_BASE+LPC17XX_SYSCON_PCONP_OFFSET)
#define SYSCON_PCONP_PCUART0          	(1 << 3)    /* Bit 3:  UART0 power/clock control */

/* Peripheral clock */
#define LPC17XX_SYSCON_PCLKSEL0_OFFSET  0x01a8  /* Peripheral Clock sel reg. 0 */
#define LPC17XX_SYSCON_PCLKSEL0        	(LPC17XX_SYSCON_BASE+LPC17XX_SYSCON_PCLKSEL0_OFFSET)
#define SYSCON_PCLKSEL0_UART0_SHIFT   	(6)     /* Bits 6-7: Peripheral clock UART0 */
#define SYSCON_PCLKSEL0_UART0_MASK    	(3 << SYSCON_PCLKSEL0_UART0_SHIFT)

/* Pin configuration */
#define SYSCON_PINSEL0_TXD0_SHIFT	(4)
#define SYSCON_PINSEL0_TXD0_MASK	(0x03 << SYSCON_PINSEL0_TXD0_SHIFT)
#define SYSCON_PINSEL0_RXD0_SHIFT	(6)
#define SYSCON_PINSEL0_RXD0_MASK	(0x03 << SYSCON_PINSEL0_RXD0_SHIFT)
#define LPC17XX_PINSEL_BASE 		0x4002C000
#define LPC17XX_PINSEL0 		(LPC17XX_PINSEL_BASE + 0x00)
#define PINMODE0			0x4002C040

#endif /* CONFIG_UART_LPC17XX_PORT_0 */
#endif /* CONFIG_UART_LPC17XX */

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
