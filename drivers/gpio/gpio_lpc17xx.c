#include <errno.h>
#include <device.h>
#include <gpio.h>
#include <soc.h>

#include <clock_control/lpc17xx_clock_control.h>
#include <clock_control.h>

#include "gpio_utils.h"

#include <sys_io.h>

enum gpio_regs {
	GPIO_DIR_OFFSET       = 0x0,
	GPIO_OUT_SET_OFFSET   = 0x018,
	GPIO_OUT_CLEAR_OFFSET = 0x01C,
	GPIO_IN_OFFSET        = 0x014,
};

#define GPIO_REG_ADDR(base, offset, n) base + offset + (0x20 * n)

#define GET_PORT(pin) (pin / 32) ? 2 :0
#define GET_PIN(pin) pin % 32

struct gpio_lpc17xx_config {
	u32_t base;
	struct lpc17xx_clock_t pclk;
};

static int gpio_lpc17xx_configure(struct device *dev,
				  int access_op, u32_t pin, int flags)
{
	const struct gpio_lpc17xx_config *cfg = dev->config->config_info;
	u32_t base = cfg->base;

	/* Get Port and Pin number */
	u32_t port = GET_PORT(pin);

	pin = GET_PIN(pin);

	/* Check for an invalid pin configuration */
	if ((flags & GPIO_INT) && (flags & GPIO_DIR_OUT)) {
		return -EINVAL;
	}
	/* Check if GPIO port supports interrupts */
	if (flags & GPIO_INT) {
		return -EINVAL;
	}
	/* supports access by pin now,you can add access by port when needed */
	if (access_op != GPIO_ACCESS_BY_PIN) {
		return -EINVAL;
	}
	/* input-0,output-1 */
	if ((flags & GPIO_DIR_MASK) == GPIO_DIR_IN) {
		sys_clear_bit(GPIO_REG_ADDR(base, GPIO_DIR_OFFSET, port), pin);
	} else {
		sys_set_bit(GPIO_REG_ADDR(base, GPIO_DIR_OFFSET, port), pin);
		sys_set_bit(GPIO_REG_ADDR(base,
					  GPIO_OUT_SET_OFFSET, port), pin);
	}

	return 0;
}

static int gpio_lpc17xx_write(struct device *dev,
			      int access_op, u32_t pin, u32_t value)
{
	const struct gpio_lpc17xx_config *cfg = dev->config->config_info;
	u32_t base = cfg->base;

	/* Get Port and Pin number */
	u32_t port = GET_PORT(pin);

	pin = GET_PIN(pin);

	if (access_op != GPIO_ACCESS_BY_PIN) {
		return -EINVAL;
	}

	/* Set/Clear the data output for the respective pin */
	if (value)
		sys_set_bit(GPIO_REG_ADDR(base,
					  GPIO_OUT_SET_OFFSET, port), pin);
	else
		sys_set_bit(GPIO_REG_ADDR(base,
					  GPIO_OUT_CLEAR_OFFSET, port), pin);

	return 0;
}

static int gpio_lpc17xx_read(struct device *dev,
			     int access_op, u32_t pin, u32_t *value)
{
	const struct gpio_lpc17xx_config *cfg = dev->config->config_info;
	u32_t base = cfg->base;

	/* Get Port and Pin number */
	u32_t port = GET_PORT(pin);

	pin = GET_PIN(pin);

	if (access_op != GPIO_ACCESS_BY_PIN) {
		return -EINVAL;
	}

	*value = sys_test_bit(GPIO_REG_ADDR(base, GPIO_IN_OFFSET, port), pin);
	return 0;
}

static int gpio_lpc17xx_init(struct device *dev)
{
	const struct gpio_lpc17xx_config *cfg = dev->config->config_info;

	/* Enabling Power to GPIO */
	clock_control_on(device_get_binding(CONFIG_CLOCK_LABEL),
			 (clock_control_subsys_t) &cfg->pclk);

	return 0;
}

static const struct gpio_driver_api gpio_lpc17xx_driver_api = {
	.config = gpio_lpc17xx_configure,
	.write = gpio_lpc17xx_write,
	.read = gpio_lpc17xx_read,
};

#ifdef CONFIG_GPIO_LPC17XX

static const struct gpio_lpc17xx_config gpio_lpc17xx_port_config = {
	.base = CONFIG_GPIO_BASE_ADDRESS,
	.pclk = {
		.en = CONFIG_GPIO_CLOCK_ENABLE,
		.sel = CONFIG_GPIO_CLOCK_SELECT
	}
};

DEVICE_AND_API_INIT(gpio_lpc17xx_port, CONFIG_GPIO_LABEL,
		    gpio_lpc17xx_init,
		    NULL, &gpio_lpc17xx_port_config,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &gpio_lpc17xx_driver_api);
#endif /* CONFIG_GPIO_LPC17XX */
