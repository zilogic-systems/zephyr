/**
 * @file
 * @brief System/hardware module for nxp_lpc17xx platform
 *
 * This module provides routines to initialize and support board-level
 * hardware for the nxp_lpc17xx platform.
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <soc.h>
#include <uart.h>
#include <linker/sections.h>
#include <arch/cpu.h>
#include <cortex_m/exc.h>
#include <sys_io.h>

/* quartz crystal resonator which is connected to the chip */
#define CRYSTAL	                        12000000ul
/* desired target frequency of the core */
#define FREQUENCY			120000000ul

static void flash_latency(u32_t frequency)
{
	u32_t wait_states;

	wait_states = frequency / 20000000;		/* 1 CLK per 20MHz */

	/* 5 CLKs is the highest reasonable value, works for up to 120MHz */
	if (wait_states > 5)
		wait_states = 5;
	/* set the latency */
	sys_write32(((wait_states - 1) << FLASHTIM) | RESERVED_value, FLASHCFG);
}

/* Power-up System Osc. */
static void osc_sel(void)
{
	u32_t value;

	value = sys_read32(PCLKSEL1);
	value |= PCLKSEL1_PCLK_GPIOINT_DIV1  |
		 PCLKSEL1_PCLK_PCB_DIV1      |
		 PCLKSEL1_PCLK_SYSCON_DIV1;
	sys_write32(value, PCLKSEL1);
	sys_clear_bit(SCS, OSCRANGE); /* Clock source Rannge as 20Mhz */
	sys_set_bit(SCS, OSCEN);      /* Enable Main OSC */
	flash_latency(FREQUENCY);
	 /* Wait for main OSC to start up */
	while (sys_test_bit(SCS, OSCSTAT) == 0);
	/* Clock soure selection as main oscillator */
	sys_set_bit(CLKSRCSEL, 0);
	/* Clock soure selection as main oscillator */
	sys_clear_bit(CLKSRCSEL, 1);

}

static void pll0_feed(void)
{
	sys_write32(PLL0FEED_FIRST, PLL0FEED);
	sys_write32(PLL0FEED_SECOND, PLL0FEED);
}

static void pll_config(u32_t crystal, u32_t frequency)
{
	u32_t prediv;
	u32_t mul;
	u32_t corediv;
	u32_t fcco;
	u32_t core_frequency;
	u32_t best_prediv = 0;
	u32_t best_mul = 0;
	u32_t best_corediv = 0;
	u32_t best_core_frequency = 0;

	/* NSEL0 in [1; 32] */
	for (prediv = 1; prediv <= 32; prediv++)
		/* MSEL0 in [5; 512] */
		for (mul = 6; mul <= 512; mul++) {
			/* calculate PLL output frequency */
			fcco = 2 * mul * crystal / prediv;

			/* skip invalid settings - fcco must be in
			 * [275M; 550M]
			 */
			if ((fcco < 275000000) || (fcco > 550000000))
				continue;

			/*CCLKSEL in [1; 256] */
			for (corediv = 1; corediv <= 256; corediv++) {
				/* calculate core frequency */
				core_frequency = fcco / corediv;

				/* skip frequencies above desired value */
				if (core_frequency > frequency)
					continue;

				/* is this configuration better than
				 * previous one?
				 */
				if (core_frequency > best_core_frequency) {
					/* save values */
					best_core_frequency = core_frequency;
					best_prediv = prediv;
					best_mul = mul;
					best_corediv = corediv;

					/* is this configuration "perfect"? */
					if (core_frequency == frequency)
						break;
				}
			}
		}

	/* set NSEL0 and MSEL0 */
	sys_write32(((best_prediv - 1) << PLL0CFG_NSEL0) |
		    ((best_mul - 1) << PLL0CFG_MSEL0_0_bit), PLL0CFG);
	pll0_feed();			/* validate change in PLL0CFG */
	sys_set_bit(PLL0CON, PLLE0);
	pll0_feed();			/* validate change in PLL0CON */
	/* set core clock divider CCLKSEL */
	sys_write32((best_corediv - 1) << CCLKSEL_bit, CCLKCFG);
	 /* wait for PLL0 lock */
	while (sys_test_bit(SCS, OSCSTAT) == 0);
	/* connect PLL0 as clock source */
	sys_set_bit(PLL0CON, PLLC0);
	pll0_feed();	       /* validate connection */

}


/**
 *
 * @brief Initialize the system clock
 *
 * @return N/A
 *
 */

static ALWAYS_INLINE void clkInit(void)
{
	osc_sel();
	pll_config(CRYSTAL, FREQUENCY);
}

/**
 *
 * @brief Perform basic hardware initialization
 *
 * Initialize the interrupt controller device drivers.
 * Also initialize the timer device driver, if required.
 *
 * @return 0
 */

static int nxp_lpc17xx_init(struct device *arg)
{
	ARG_UNUSED(arg);

	/* old interrupt lock level */
	int oldLevel;

	/* disable interrupts */
	oldLevel = irq_lock();

	_ClearFaults();

	/* Initialize FRO/system clock to 48 MHz */
	clkInit();

	/*
	 * install default handler that simply resets the CPU if configured in
	 * the kernel, NOP otherwise
	 */
	NMI_INIT();

	/* restore interrupt state */
	irq_unlock(oldLevel);

	return 0;
}

SYS_INIT(nxp_lpc17xx_init, PRE_KERNEL_1, 0);
