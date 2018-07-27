/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <errno.h>
#include <device.h>
#include <pinmux.h>
#include <sys_io.h>
#include <misc/__assert.h>

struct pinmux_lpc17xx_config {
	u32_t port_no;
};

static const u32_t pin_map[5] = {
	0b01111111111111111000111111111111,
	0b11111111111111111100011100010011,
	0b00000000000000000011111111111111,
	0b00000110000000000000000000000000,
	0b00110000000000000000000000000000
};

#define PINMUX_BASE_ADDR 0x4002C000

#define PINMUX_PINSEL_OFFSET 0x0
#define PINMUX_PINMODE_OFFSET 0x040
#define PINMUX_PINMODE_OD_OFFSET 0x068

static u32_t pinmux_pinsel_addr(u8_t port, u8_t pin)
{
	u32_t addr = PINMUX_BASE_ADDR + PINMUX_PINSEL_OFFSET;
	addr += (8 * port);
	addr += (pin < 16) ? 0 : 4;
	return addr;
}

static u32_t pinmux_pinmode_addr(u8_t port, u8_t pin)
{
	u32_t addr = PINMUX_BASE_ADDR + PINMUX_PINMODE_OFFSET;
	addr += (8 * port);
	addr += (pin < 16) ? 0 : 4;
	return addr;
}

static u32_t pinmux_opendrain_addr(u8_t port)
{
	u32_t addr = PINMUX_BASE_ADDR + PINMUX_PINMODE_OD_OFFSET;
	addr += (4 * port);
	return addr;
}

static bool check_pin_available(u8_t port, u8_t pin)
{
	return (pin_map[port] >> pin) & 0x01;
}

#define GET_PINSEL_VALUE(func) (func >> 0) & (0x3)
#define GET_PINMODE_VALUE(func) (func >> 2) & (0x3)
#define GET_PINMODE_OD_VALUE(func) (func >> 4) & (0x1)

static inline void sys_set_bits(u32_t address, u32_t mask, u32_t shift, u32_t data)
{
	u32_t temp = sys_read32(address);
	temp &= ~(mask << shift);
	temp |= (data & mask) << shift;	
	sys_write32(temp, address);	
}

static int pinmux_lpc17xx_set(struct device *dev, u32_t pin, u32_t func)
{
	const struct pinmux_lpc17xx_config *config = dev->config->config_info;
	u32_t port = config->port_no;
	u8_t pos = 0;
	u8_t pinsel;
	u8_t pinmode;
	bool pinmode_od;	
	
	__ASSERT((check_pin_available(port, pin) == 0), "Pinmux Not Available");

	pinsel = GET_PINSEL_VALUE(func);
	pinmode = GET_PINMODE_VALUE(func);
	pinmode_od = GET_PINMODE_OD_VALUE(func);
	
	if (pin < 16)
		pos = pin * 2;
	else
		pos = (pin - 16) * 2;
	
	sys_set_bits(pinmux_pinsel_addr(port, pin), 3, pos, pinsel);
	sys_set_bits(pinmux_pinmode_addr(port, pin), 3, pos, pinmode);

	if (pinmode_od)
		sys_set_bit(pinmux_opendrain_addr(port), pin);
	else
		sys_clear_bit(pinmux_opendrain_addr(port), pin);

	return 0;
}

static int pinmux_lpc17xx_get(struct device *dev, u32_t pin, u32_t *func)
{
	return -ENOTSUP;
}

static int pinmux_lpc17xx_pullup(struct device *dev, u32_t pin, u8_t func)
{
	return -ENOTSUP;
}

static int pinmux_lpc17xx_input(struct device *dev, u32_t pin, u8_t func)
{
	return -ENOTSUP;
}

static int pinmux_lpc17xx_init(struct device *dev)
{
	return 0;
}

static const struct pinmux_driver_api pinmux_lpc17xx_driver_api = {
	.set = pinmux_lpc17xx_set,
	.get = pinmux_lpc17xx_get,
	.pullup = pinmux_lpc17xx_pullup,
	.input = pinmux_lpc17xx_input,
};

#ifdef CONFIG_PINMUX_LPC17XX_PORT0
static const struct pinmux_lpc17xx_config pinmux_lpc17xx_port0_config = {
	.port_no = 0,
};

DEVICE_AND_API_INIT(pinmux_port0, CONFIG_PINMUX_LPC17XX_PORT0_NAME,
		    &pinmux_lpc17xx_init,
		    NULL, &pinmux_lpc17xx_port0_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &pinmux_lpc17xx_driver_api);
#endif /* CONFIG_PINMUX_LPC17XX_LPC_PORT0 */

#ifdef CONFIG_PINMUX_LPC17XX_PORT1
static const struct pinmux_lpc17xx_config pinmux_lpc17xx_port1_config = {
	.port_no = 1,
};

DEVICE_AND_API_INIT(pinmux_port1, CONFIG_PINMUX_LPC17XX_PORT1_NAME,
		    &pinmux_lpc17xx_init,
		    NULL, &pinmux_lpc17xx_port1_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &pinmux_lpc17xx_driver_api);
#endif /* CONFIG_PINMUX_LPC17XX_LPC_PORT1 */

#ifdef CONFIG_PINMUX_LPC17XX_PORT2
static const struct pinmux_lpc17xx_config pinmux_lpc17xx_port2_config = {
	.port_no = 2,
};

DEVICE_AND_API_INIT(pinmux_port2, CONFIG_PINMUX_LPC17XX_PORT1_NAME,
		    &pinmux_lpc17xx_init,
		    NULL, &pinmux_lpc17xx_port2_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &pinmux_lpc17xx_driver_api);
#endif /* CONFIG_PINMUX_LPC17XX_LPC_PORT2 */

#ifdef CONFIG_PINMUX_LPC17XX_PORT1
static const struct pinmux_lpc17xx_config pinmux_lpc17xx_port3_config = {
	.port_no = 3,
};

DEVICE_AND_API_INIT(pinmux_port3, CONFIG_PINMUX_LPC17XX_PORT1_NAME,
		    &pinmux_lpc17xx_init,
		    NULL, &pinmux_lpc17xx_port3_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &pinmux_lpc17xx_driver_api);
#endif /* CONFIG_PINMUX_LPC17XX_LPC_PORT3 */

#ifdef CONFIG_PINMUX_LPC17XX_PORT1
static const struct pinmux_lpc17xx_config pinmux_lpc17xx_port4_config = {
	.port_no = 4,
};

DEVICE_AND_API_INIT(pinmux_port4, CONFIG_PINMUX_LPC17XX_PORT1_NAME,
		    &pinmux_lpc17xx_init,
		    NULL, &pinmux_lpc17xx_port4_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &pinmux_lpc17xx_driver_api);
#endif /* CONFIG_PINMUX_LPC17XX_LPC_PORT1 */
