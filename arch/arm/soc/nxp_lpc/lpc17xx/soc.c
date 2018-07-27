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

/**
 *
 * @brief Initialize the system clock
 *
 * @return N/A
 *
 */

static ALWAYS_INLINE void clkInit(void)
{
	/* fixme: Initialize and configure the clock. */
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
