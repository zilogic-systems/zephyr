/*
 * Copyright (c) 2018 Zilogic Systems.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief UART Driver for lpc17XX
 *
 * Driver support UART0. UART0 is connected to console
 * and as such supports printk and debugging.
 * Zephyr STDIO console APIs are supported.
 */

#include <uart.h>
#include <clock_control/lpc17xx_clock_control.h>
#include <clock_control.h>

/* Device config parameters */
struct uart_lpc17xx_config {
	u32_t base;
	struct lpc17xx_clock_t pclk;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_config_func_t irq_config_func;
#endif
};

/* Device data structure */
struct uart_lpc17xx_dev_data {
	u32_t baud_rate;			/*Baud rate */
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t cb;	/* Callback function pointer */
	void *cb_data;				/* Callback function argument */
#endif
};

#define DEV_CFG(dev) \
	((struct uart_lpc17xx_config *)(dev)->config->config_info)

#define DEV_DATA(dev) \
	((struct uart_lpc17xx_dev_data *)(dev)->driver_data)

/* Register Mapping */
#define LPC17XX_UART_RBR	((DEV_CFG(dev)->base) + 0x000)
#define LPC17XX_UART_THR	((DEV_CFG(dev)->base) + 0x000)
#define LPC17XX_UART_DLL	((DEV_CFG(dev)->base) + 0x000)
#define LPC17XX_UART_DLM	((DEV_CFG(dev)->base) + 0x004)
#define LPC17XX_UART_IER	((DEV_CFG(dev)->base) + 0x004)
#define LPC17XX_UART_IIR	((DEV_CFG(dev)->base) + 0x008)
#define LPC17XX_UART_FCR	((DEV_CFG(dev)->base) + 0x008)
#define LPC17XX_UART_LCR	((DEV_CFG(dev)->base) + 0x00c)
#define LPC17XX_UART_LSR	((DEV_CFG(dev)->base) + 0x014)

static void uart_lpc17xx_baudrate_set(struct device *dev)
{
	const struct uart_lpc17xx_dev_data *dev_data = DEV_DATA(dev);
	const struct uart_lpc17xx_config *cfg = DEV_CFG(dev);
	u32_t freq;
	u16_t dl;

	/* Get peripheral clock frequency */
	clock_control_get_rate(device_get_binding(CONFIG_CLOCK_LABEL),
			       (clock_control_subsys_t) &cfg->pclk, &freq);

	/* Set DLAB=1, Write DL value i.e = FREQ / BAUDRATE / 16
	 * Finally set DLAB = 0
	 */
	sys_set_bit(LPC17XX_UART_LCR, 7);
	dl = (freq / dev_data->baud_rate) / 16;
	sys_write8(dl >> 8, LPC17XX_UART_DLM);
	sys_write8(dl & 0xff, LPC17XX_UART_DLL);
	sys_clear_bit(LPC17XX_UART_LCR, 7);
}

static int uart_lpc17xx_init(struct device *dev)
{
	const struct uart_lpc17xx_config *cfg = DEV_CFG(dev);

	/* Enable power to UART0 */
	clock_control_on(device_get_binding(CONFIG_CLOCK_LABEL),
			 (clock_control_subsys_t) &cfg->pclk);

	/* Set word-len = 8, Stop bit = 1,
	 * Disable parity generation and checking,
	 * odd parity.
	 */
	sys_write32(0x03, LPC17XX_UART_LCR);

	/* Set baud rate */
	uart_lpc17xx_baudrate_set(dev);

	/* Enable FIFO,
	 * Rx Trigger level is one character in FIFO
	 */
	sys_set_bit(LPC17XX_UART_FCR, 0);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	DEV_CFG(dev)->irq_config_func(dev);
#endif
	return 0;
}

static int uart_lpc17xx_poll_in(struct device *dev, unsigned char *c)
{
	while (!sys_test_bit(LPC17XX_UART_LSR, 0)) {
		/* FIFO is empty and no character to read */
		return -1;
	}

	/* Read the character */
	*c = sys_read8(LPC17XX_UART_RBR);
	return 0;
}

static unsigned char uart_lpc17xx_poll_out(struct device *dev, unsigned char c)
{
	/* Check if transmitter is empty,
	 * and write a character to data reg when empty.
	 */
	while (!sys_test_bit(LPC17XX_UART_LSR, 5))
		;

	/* Send the character */
	sys_write8(c, LPC17XX_UART_THR);

	return c;

/*FIXME: To send a character when hardware flow control is enabled,
 * The handshake signal CTS must be asserted
 */
}

static int uart_lpc17xx_err_check(struct device *dev)
{
	int err = 0;

	if (sys_test_bit(LPC17XX_UART_LSR, 1)) {
		err |= UART_ERROR_OVERRUN;
	}

	if (sys_test_bit(LPC17XX_UART_LSR, 2)) {
		err |= UART_ERROR_PARITY;
	}

	if (sys_test_bit(LPC17XX_UART_LSR, 3)) {
		err |= UART_ERROR_FRAMING;
	}

	return err;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static inline int uart_lpc17xx_irq_fifo_fill(struct device *dev, u8_t *tx_data,
				  int data_len)
{
	u8_t num_tx = 0;

	while ((data_len - num_tx > 0) && sys_test_bit(LPC17XX_UART_LSR, 5)) {
		sys_write8(tx_data[num_tx++], LPC17XX_UART_THR);
	}

	return num_tx;
}

static inline int uart_lpc17xx_irq_fifo_read(struct device *dev, u8_t *rx_data,
				 const int data_len)
{
	u8_t num_rx = 0;

	while ((data_len - num_rx > 0) && sys_test_bit(LPC17XX_UART_LSR, 0)) {
		rx_data[num_rx++] = sys_read8(LPC17XX_UART_RBR);
	}

	return num_rx;
}

static inline void uart_lpc17xx_irq_tx_enable(struct device *dev)
{
	sys_set_bit(LPC17XX_UART_IER, 1);
}

static inline void uart_lpc17xx_irq_tx_disable(struct device *dev)
{
	sys_clear_bit(LPC17XX_UART_IER, 1);
}

static inline int uart_lpc17xx_irq_tx_ready(struct device *dev)
{
	return sys_test_bit(LPC17XX_UART_LSR, 5);
}

static inline void uart_lpc17xx_irq_rx_enable(struct device *dev)
{
	sys_set_bit(LPC17XX_UART_IER, 0);
}

static inline void uart_lpc17xx_irq_rx_disable(struct device *dev)
{
	sys_clear_bit(LPC17XX_UART_IER, 0);
}

static inline int uart_lpc17xx_irq_rx_ready(struct device *dev)
{
	return sys_test_bit(LPC17XX_UART_LSR, 0);
}

static inline void uart_lpc17xx_irq_err_enable(struct device *dev)
{
	sys_set_bit(LPC17XX_UART_IER, 2);
}

static void uart_lpc17xx_irq_err_disable(struct device *dev)
{
	sys_clear_bit(LPC17XX_UART_IER, 2);
}

static int uart_lpc17xx_irq_is_pending(struct device *dev)
{
	return !sys_test_bit(LPC17XX_UART_IIR, 0);
}

static int uart_lpc17xx_irq_update(struct device *dev)
{
	return 1;
}

static void uart_lpc17xx_irq_callback_set(struct device *dev,
					  uart_irq_callback_user_data_t cb,
					  void *cb_data)
{
	struct uart_lpc17xx_dev_data *dev_data = DEV_DATA(dev);

	dev_data->cb = cb;
	dev_data->cb_data = cb_data;
}

static void uart_lpc17xx_isr(void *arg)
{
	struct device *dev = arg;
	struct uart_lpc17xx_dev_data *dev_data = DEV_DATA(dev);

	if (dev_data->cb) {
		dev_data->cb(dev_data->cb_data);
	}
}

#endif /*CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_lpc17xx_driver_api = {
	.poll_in = uart_lpc17xx_poll_in,
	.poll_out = uart_lpc17xx_poll_out,
	.err_check = uart_lpc17xx_err_check,

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_lpc17xx_irq_fifo_fill,
	.fifo_read = uart_lpc17xx_irq_fifo_read,
	.irq_tx_enable = uart_lpc17xx_irq_tx_enable,
	.irq_tx_disable = uart_lpc17xx_irq_tx_disable,
	.irq_tx_ready = uart_lpc17xx_irq_tx_ready,
	.irq_rx_enable = uart_lpc17xx_irq_rx_enable,
	.irq_rx_disable = uart_lpc17xx_irq_rx_disable,
	.irq_rx_ready = uart_lpc17xx_irq_rx_ready,
	.irq_err_enable = uart_lpc17xx_irq_err_enable,
	.irq_err_disable = uart_lpc17xx_irq_err_disable,
	.irq_is_pending = uart_lpc17xx_irq_is_pending,
	.irq_update = uart_lpc17xx_irq_update,
	.irq_callback_set = uart_lpc17xx_irq_callback_set,
#endif
};

#ifdef CONFIG_UART_LPC17XX_PORT_0

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void irq_config_func_0(struct device *dev);
#endif

static struct uart_lpc17xx_config uart0_cfg = {
	.base = CONFIG_UART0_BASE_ADDR,
	.pclk = {
		.en = CONFIG_UART0_CLOCK_ENABLE,
		.sel = CONFIG_UART0_CLOCK_SELECT
	},
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_config_func = irq_config_func_0,
#endif
};

static struct uart_lpc17xx_dev_data uart_lpc17xx_dev_data_0 = {
	.baud_rate = CONFIG_UART0_BAUD_RATE,
};

DEVICE_AND_API_INIT(lpc17xx_uart_0, CONFIG_UART0_LABEL,
		    &uart_lpc17xx_init,
		    &uart_lpc17xx_dev_data_0, &uart0_cfg,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &uart_lpc17xx_driver_api);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void irq_config_func_0(struct device *dev)
{
	IRQ_CONNECT(CONFIG_UART0_IRQ,
		    CONFIG_UART0_IRQ_PRI,
		    uart_lpc17xx_isr, DEVICE_GET(lpc17xx_uart_0),
		    0);
	irq_enable(CONFIG_UART0_IRQ);
	uart_lpc17xx_irq_rx_enable(dev);

}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#endif /* CONFIG_UART_LPC17XX_PORT_0 */

/* FIXME: Enable UART 1/2 */
