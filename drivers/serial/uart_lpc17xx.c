/*
 * Copyright (c) 2018 Zilogic Systems.
 *
 * SPDX-License-Identifier: Apache-2.0
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
	u8_t iir_cache;
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

/* Bit positions in IER */
#define IER_BIT_RBR 0x00
#define IER_BIT_THRE 0x01
#define IER_BIT_RxLS 0x02

/* Bit positions in IIR and values */
#define IIR_BIT_INTSTATUS 0x00
#define IIR_NIP 0x01		/* No interrupt pending */
#define IIR_ID 0x06		/* Interrupt ID mask without NIP */
#define IIR_THRE 0x02		/* Transmit holding register interrupt empty interrupt */
#define IIR_RDA  0x04		/* Receive Data pending interrupt */

/* Bit position in LCR  */
#define LCR_BIT_WL0 0x00
#define LCR_BIT_WL1 0x01
#define LCR_BIT_STP 0x02
#define LCR_BIT_PARITYEN 0x03
#define LCR_BIT_PARITYSL0 0x04
#define LCR_BIT_PARITYSL1 0x05
#define LCR_BIT_BREAKCTL 0x06
#define LCR_BIT_DLA 0x07

/* Bit positions in LSR */
#define LSR_BIT_RDR 0x00
#define LSR_BIT_OE 0x01
#define LSR_BIT_PE 0x02
#define LSR_BIT_FE 0x03
#define LSR_BIT_THRE 0x05

/* Bit positions in FCR */
#define FCR_BIT_FIFOEN 0x00


static void uart_lpc17xx_baudrate_set(struct device *dev)
{
	u32_t freq;
	u16_t dl;

	/* Get peripheral clock frequency */
	clock_control_get_rate(device_get_binding(CONFIG_CLOCK_LABEL),
				       (clock_control_subsys_t) &(DEV_CFG(dev)->pclk),
				       &freq);

	/* Set DLAB=1, Write DL value i.e = FREQ / BAUDRATE / 16
	 * Finally set DLAB = 0
	 */
	sys_set_bit(LPC17XX_UART_LCR, LCR_BIT_DLA);
	dl = (freq / DEV_DATA(dev)->baud_rate) / 16;
	sys_write8(dl >> 8, LPC17XX_UART_DLM);
	sys_write8(dl & 0xff, LPC17XX_UART_DLL);
	sys_clear_bit(LPC17XX_UART_LCR, LCR_BIT_DLA);
}

static void uart_lpc17xx_lcr_reg_set(struct device *dev)
{
	/* Set word-len = 8, i.e set BIT 0,1 to 1(0x03) */
	sys_set_bit(LPC17XX_UART_LCR, LCR_BIT_WL0);
	sys_set_bit(LPC17XX_UART_LCR, LCR_BIT_WL1);

	/* By default , STOP BIT = 1,
	 * Parity generation and detection is disabled,
	* Odd parity
	*/
}
static int uart_lpc17xx_init(struct device *dev)
{
	u32_t lock_out;

	/* Enable power to UART0 */
	clock_control_on(device_get_binding(CONFIG_CLOCK_LABEL),
			 (clock_control_subsys_t) &(DEV_CFG(dev)->pclk));

	/* Set the format of data character to be transmitted and received */
	uart_lpc17xx_lcr_reg_set(dev);

	/* Set baud rate */
	lock_out = irq_lock(); 
	uart_lpc17xx_baudrate_set(dev);
	irq_unlock(lock_out);

	/* Enable FIFO,
	 * Rx Trigger level is one character in FIFO
	 */
	sys_set_bit(LPC17XX_UART_FCR, FCR_BIT_FIFOEN);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	DEV_CFG(dev)->irq_config_func(dev);
#endif
	return 0;
}

static int uart_lpc17xx_poll_in(struct device *dev, unsigned char *c)
{
	while (!sys_test_bit(LPC17XX_UART_LSR, LSR_BIT_RDR)) {
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
	while (!sys_test_bit(LPC17XX_UART_LSR, LSR_BIT_THRE))
		;

	/* Send the character */
	sys_write8(c, LPC17XX_UART_THR);

	return c;
}

static int uart_lpc17xx_err_check(struct device *dev)
{
	int err = 0;

	if (sys_test_bit(LPC17XX_UART_LSR, LSR_BIT_OE)) {
		err |= UART_ERROR_OVERRUN;
	}

	if (sys_test_bit(LPC17XX_UART_LSR, LSR_BIT_PE)) {
		err |= UART_ERROR_PARITY;
	}

	if (sys_test_bit(LPC17XX_UART_LSR, LSR_BIT_FE)) {
		err |= UART_ERROR_FRAMING;
	}

	return err;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static inline int uart_lpc17xx_irq_fifo_fill(struct device *dev, const u8_t *tx_data,
				  int data_len)
{
	u8_t num_tx = 0;

	while ((data_len - num_tx > 0) &&
	sys_test_bit(LPC17XX_UART_LSR, LSR_BIT_THRE)) {
		sys_write8(tx_data[num_tx++], LPC17XX_UART_THR);
	}

	return num_tx;
}

static inline int uart_lpc17xx_irq_fifo_read(struct device *dev, u8_t *rx_data,
				 const int data_len)
{
	u8_t num_rx = 0;

	while ((data_len - num_rx > 0) &&
	sys_test_bit(LPC17XX_UART_LSR, LSR_BIT_RDR)) {
		rx_data[num_rx++] = sys_read8(LPC17XX_UART_RBR);
	}

	return num_rx;
}

static inline void uart_lpc17xx_irq_tx_enable(struct device *dev)
{
	sys_set_bit(LPC17XX_UART_IER, IER_BIT_THRE);
}

static inline void uart_lpc17xx_irq_tx_disable(struct device *dev)
{
	sys_clear_bit(LPC17XX_UART_IER, IER_BIT_THRE);
}

static inline int uart_lpc17xx_irq_tx_ready(struct device *dev)
{
	return ((DEV_DATA(dev)->iir_cache & IIR_ID) == IIR_THRE);
}

static inline void uart_lpc17xx_irq_rx_enable(struct device *dev)
{
	sys_set_bit(LPC17XX_UART_IER, IER_BIT_RBR);
}

static inline void uart_lpc17xx_irq_rx_disable(struct device *dev)
{
	sys_clear_bit(LPC17XX_UART_IER, IER_BIT_RBR);
}

static inline int uart_lpc17xx_irq_rx_ready(struct device *dev)
{
	return ((DEV_DATA(dev)->iir_cache & IIR_ID) == IIR_RDA);
}

static inline void uart_lpc17xx_irq_err_enable(struct device *dev)
{
	sys_set_bit(LPC17XX_UART_IER, IER_BIT_RxLS);
}

static void uart_lpc17xx_irq_err_disable(struct device *dev)
{
	sys_clear_bit(LPC17XX_UART_IER, IER_BIT_RxLS);
}

static int uart_lpc17xx_irq_is_pending(struct device *dev)
{
	return !sys_test_bit(LPC17XX_UART_IIR, IIR_BIT_INTSTATUS);
}

static int uart_lpc17xx_irq_update(struct device *dev)
{
	DEV_DATA(dev)->iir_cache = sys_read8(LPC17XX_UART_IIR);

	return 1;
}

static void uart_lpc17xx_irq_callback_set(struct device *dev,
					  uart_irq_callback_user_data_t cb,
					  void *cb_data)
{
	DEV_DATA(dev)->cb = cb;
	DEV_DATA(dev)->cb_data = cb_data;
}

static void uart_lpc17xx_isr(struct device *dev)
{
	if (DEV_DATA(dev)->cb) {
		DEV_DATA(dev)->cb(DEV_DATA(dev)->cb_data);
	}
}

#endif /*CONFIG_UART_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_LINE_CTRL

static int uart_lpc17xx_line_ctrl_set(struct device *dev,
				      u32_t ctrl, u32_t val)
{
	switch (ctrl) {
	case LINE_CTRL_BAUD_RATE:
		DEV_DATA(dev)->baud_rate = val;
		uart_lpc17xx_baudrate_set(dev);
		return 0;
	};

	return -ENOTSUP;
}

static int uart_lpc17xx_line_ctrl_get(struct device *dev,
				      u32_t ctrl, u32_t *val)
{
	switch (ctrl) {
	case LINE_CTRL_BAUD_RATE:
		*val = DEV_DATA(dev)->baud_rate;
		uart_lpc17xx_baudrate_set(dev);
		return 0;
	};

	return -ENOTSUP;
}

#endif /* CONFIG_UART_LINE_CTRL */
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

#ifdef CONFIG_UART_LINE_CTRL
	.line_ctrl_set = uart_lpc17xx_line_ctrl_set,
	.line_ctrl_get = uart_lpc17xx_line_ctrl_get,
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

#ifdef CONFIG_UART_LPC17XX_PORT_1

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void irq_config_func_1(struct device *dev);
#endif

static struct uart_lpc17xx_config uart1_cfg = {
	.base = CONFIG_UART1_BASE_ADDR,
	.pclk = {
		.en = CONFIG_UART1_CLOCK_ENABLE,
		.sel = CONFIG_UART1_CLOCK_SELECT
	},
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_config_func = irq_config_func_1,
#endif
};

static struct uart_lpc17xx_dev_data uart_lpc17xx_dev_data_1 = {
	.baud_rate = CONFIG_UART1_BAUD_RATE,
};

DEVICE_AND_API_INIT(lpc17xx_uart_1, CONFIG_UART1_LABEL,
		    &uart_lpc17xx_init,
		    &uart_lpc17xx_dev_data_1, &uart1_cfg,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &uart_lpc17xx_driver_api);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void irq_config_func_1(struct device *dev)
{
	IRQ_CONNECT(CONFIG_UART1_IRQ,
		    CONFIG_UART1_IRQ_PRI,
		    uart_lpc17xx_isr, DEVICE_GET(lpc17xx_uart_1),
		    0);
	irq_enable(CONFIG_UART1_IRQ);
	uart_lpc17xx_irq_rx_enable(dev);

}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
#endif /* CONFIG_UART_LPC17XX_PORT_1 */

