/*
 * Copyright (c) 2018 Zilogic Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SPI_LPC17XX_H_
#define _SPI_LPC17XX_H_

#include "spi_context.h"

typedef void (*irq_config_func_t)(struct device *port);

/* Device configuration parameters */
struct spi_lpc17xx_config {
        u32_t base;
        u32_t clock;
};

/* Device run time data */
struct spi_lpc17xx_data {
        struct spi_context ctx;
};

#endif	/* _SPI_LPC17XX_H_ */
