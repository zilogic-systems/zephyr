/*
 * Copyright (c) 2018 Zilogic Systems.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <uart.h>

#define DEV_CFG(dev) \
	((struct uart_device_config *)(dev)->config->config_info)

/* Register Mapping */
#define LPC17XX_UART_RBR	((DEV_CFG(dev)->base) + 0x000)
#define LPC17XX_UART_THR	((DEV_CFG(dev)->base) + 0x000)
#define LPC17XX_UART_DLL	((DEV_CFG(dev)->base) + 0x000)
#define LPC17XX_UART_DLM	((DEV_CFG(dev)->base) + 0x004)
#define LPC17XX_UART_IER	((DEV_CFG(dev)->base) + 0x004)
#define LPC17XX_UART_FCR	((DEV_CFG(dev)->base) + 0x008)
#define LPC17XX_UART_LCR	((DEV_CFG(dev)->base) + 0x00c)
#define LPC17XX_UART_LSR	((DEV_CFG(dev)->base) + 0x014)

/* UART LCR Reg Bit fields set value*/
#define BIT_LCR_WL		0x03 /* 8 bit character len */
#define BIT_LCR_DLAB		(1 << 7)  /* Bit 7: Divisor Latch Access bit */

/* UART FCR Reg bit field set vale */
#define BIT_FCR_FIFOEN		(1 << 0)  /* Bit 0:  Enable FIFOs */

/* UART LSR Reg bit fields */
#define BIT_LSR_THRE           (1 << 5)  /* Bit 5:  Transmitter Holding Register empty */

static int uart_lpc17xx_init(struct device *dev)
{
	uint16_t dl;
	uint32_t regval;

	/* Set 8-bits char, DLAB=1 */
	regval = sys_read32(LPC17XX_UART_LCR);
	regval |= BIT_LCR_WL | BIT_LCR_DLAB;
	sys_write32(regval, LPC17XX_UART_LCR);

	/* Write DL value i.e = PCLK / BAUD / 16
	 * 120 MHz / 115200 / 16
	 */
	dl = 65;
	sys_write32(dl >> 8, LPC17XX_UART_DLM);
	sys_write32(dl & 0xff, LPC17XX_UART_DLL);

	/* Clear  DLAB = 0 */
	regval = sys_read32(LPC17XX_UART_LCR);
	regval &= ~BIT_LCR_DLAB;
	sys_write32(regval, LPC17XX_UART_LCR);

	/* Enable FIFO */
	regval = sys_read32(LPC17XX_UART_FCR);
	regval |= BIT_FCR_FIFOEN;
	sys_write32(regval, LPC17XX_UART_FCR);
}

/*
 * FIXME: Receive needs to be implemented.
 */
static int uart_lpc17xx_poll_in(struct device *dev, unsigned char *c)
{
	return -ENOTSUP;
}

static unsigned char uart_lpc17xx_poll_out(struct device *dev, unsigned char c)
{
	while ((sys_read32(LPC17XX_UART_LSR) & BIT_LSR_THRE) == 0)
		;

	/* Send the character */
	sys_write32(c, LPC17XX_UART_THR);
}

static const struct uart_driver_api uart_lpc17xx_driver_api = {
	.poll_in = uart_lpc17xx_poll_in,
	.poll_out = uart_lpc17xx_poll_out,
};

#ifdef CONFIG_UART_LPC17XX_PORT_0

static struct uart_device_config uart0_cfg = {
	.base = (u8_t *)CONFIG_UART0_BASE_ADDR,
};

DEVICE_AND_API_INIT(lpc17xx_uart_0, CONFIG_UART0_LABEL,
		    &uart_lpc17xx_init,
		    NULL, &uart0_cfg,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &uart_lpc17xx_driver_api);

#endif /* CONFIG_UART_LPC17XX_PORT_0 */

/* FIXME: Enable UART 1/2/3 */
