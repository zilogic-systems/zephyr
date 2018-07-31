/*
 * Copyright (c) 2018, Zilogic Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <errno.h>
#include <device.h>
#include <pinmux.h>
#include <sys_io.h>
#include <misc/__assert.h>

#define PINMUX_BASE_ADDR CONFIG_PINMUX_BASE_ADDR
#define PINMUX_MAX_PORT 5
#define PINMUX_MAX_PIN 32
#define MAX_PINS (PINMUX_MAX_PORT * PINMUX_MAX_PIN)

#define PINMUX_PINSEL_OFFSET 0x0
#define PINMUX_PINMODE_OFFSET 0x040
#define PINMUX_PINMODE_OD_OFFSET 0x068

#define GET_PORT(pin) ((pin) / 32)
#define GET_PIN(pin) ((pin) % 32)
#define GET_PINSEL_ARG_VALUE(value) (((value) >> 0) & 0x3)
#define GET_PINMODE_ARG_VALUE(value) (((value) >> 2) & 0x3)
#define GET_PINMODE_OD_ARG_VALUE(value) (((value) >> 4) & 0x1)

#define GET_PINSEL_REG_VALUE(reg_value, pos)                          \
				(((reg_value) >> (pos)) & 0x3)
#define GET_PINMODE_REG_VALUE(reg_value, pos)                         \
				((((reg_value) >> (pos)) & 0x3) << 2)
#define GET_PINMODE_OD_REG_VALUE(reg_value, pin)                      \
				((((reg_value) >> (pin)) & 0x1) << 4)

static const u32_t pin_valid_bmp[PINMUX_MAX_PORT] = {
	0x7FFF8FFF,     /* 0b01111111111111111000111111111111 */
	0xFFFFC713,     /* 0b11111111111111111100011100010011 */
	0x3FFF,         /* 0b00000000000000000011111111111111 */
	0x6000000,      /* 0b00000110000000000000000000000000 */
	0x30000000      /* 0b00110000000000000000000000000000 */
};

static u32_t pinmux_pinsel_addr(u8_t port, u8_t pin)
{
	u32_t addr;

	addr = PINMUX_BASE_ADDR + PINMUX_PINSEL_OFFSET;
	addr += (8 * port);
	addr += (pin < 16) ? 0 : 4;
	return addr;
}

static u32_t pinmux_pinmode_addr(u8_t port, u8_t pin)
{
	u32_t addr;

	addr = PINMUX_BASE_ADDR + PINMUX_PINMODE_OFFSET;
	addr += (8 * port);
	addr += (pin < 16) ? 0 : 4;
	return addr;
}

static u32_t pinmux_opendrain_addr(u8_t port)
{
	u32_t addr;

	addr = PINMUX_BASE_ADDR + PINMUX_PINMODE_OD_OFFSET;
	addr += (4 * port);
	return addr;
}

static bool check_pin_valid(u32_t pin)
{
	u8_t cpin;
	u8_t cport;

	if (pin > MAX_PINS)
		return 1;

	cport = GET_PORT(pin);
	cpin = GET_PIN(pin);

	return ~((pin_valid_bmp[cport] >> cpin) & 0x01);
}

static inline void sys_set_bits(u32_t address,
				u32_t mask, u32_t shift, u32_t data)
{
	u32_t temp;

	temp = sys_read32(address);
	temp &= ~(mask << shift);
	temp |= (data & mask) << shift;
	sys_write32(temp, address);
}

static int pinmux_lpc17xx_set(struct device *dev, u32_t pin, u32_t func)
{
	ARG_UNUSED(dev);
	u8_t cport;
	u8_t cpin;
	u8_t pos = 0;
	u8_t pinsel;
	u8_t pinmode;
	bool pinmode_od;

	__ASSERT(check_pin_valid(pin), "Pinmux Not Available");

	cport = GET_PORT(pin);
	cpin = GET_PIN(pin);
	
	pinsel = GET_PINSEL_ARG_VALUE(func);
	pinmode = GET_PINMODE_ARG_VALUE(func);
	pinmode_od = GET_PINMODE_OD_ARG_VALUE(func);

	pos = (cpin * 2) % 32;

	sys_set_bits(pinmux_pinsel_addr(cport, cpin), 3, pos, pinsel);
	sys_set_bits(pinmux_pinmode_addr(cport, cpin), 3, pos, pinmode);

	if (pinmode_od)
		sys_set_bit(pinmux_opendrain_addr(cport), cpin);
	else
		sys_clear_bit(pinmux_opendrain_addr(cport), cpin);

	return 0;
}

static int pinmux_lpc17xx_get(struct device *dev, u32_t pin, u32_t *func)
{
	ARG_UNUSED(dev);
	u8_t cport;
	u8_t cpin;
	u32_t addr;
	u8_t pos = 0;

	__ASSERT(check_pin_valid(pin), "Pinmux Not Available");

	cport = GET_PORT(pin);
	cpin = GET_PIN(pin);
	
	*func = 0;

	pos = (cpin * 2) % 32;

	addr = pinmux_pinsel_addr(cport, cpin);
	*func |= GET_PINSEL_REG_VALUE(sys_read32(addr), pos);

	addr = pinmux_pinmode_addr(cport, cpin);
	*func |= GET_PINMODE_REG_VALUE(sys_read32(addr), pos);

	addr = pinmux_opendrain_addr(cport);
	*func |= GET_PINMODE_OD_REG_VALUE(sys_read32(addr), cpin);

	return 0;
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
	ARG_UNUSED(dev);
	return 0;
}

static const struct pinmux_driver_api pinmux_lpc17xx_driver_api = {
	.set = pinmux_lpc17xx_set,
	.get = pinmux_lpc17xx_get,
	.pullup = pinmux_lpc17xx_pullup,
	.input = pinmux_lpc17xx_input,
};

#ifdef CONFIG_PINMUX_LPC17XX

DEVICE_AND_API_INIT(pinmux_dev, CONFIG_PINMUX_NAME,
		    &pinmux_lpc17xx_init,
		    NULL, NULL,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &pinmux_lpc17xx_driver_api);
#endif /* CONFIG_PINMUX_LPC17XX */
