/*
 * Copyright (c) 2018 Zilogic Systems.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _LPC17XX_CLOCK_CONTROL_H_
#define _LPC17XX_CLOCK_CONTROL_H_

#include <clock_control.h>

struct lpc17xx_clock_t {
	u32_t en;
	u32_t sel;
};

#endif /* _LPC17XX_CLOCK_CONTROL_H_ */
