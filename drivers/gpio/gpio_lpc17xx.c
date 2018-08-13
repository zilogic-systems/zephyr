#include <errno.h>
#include <device.h>
#include <gpio.h>
#include <soc.h>
#include "gpio_utils.h"

#include <sys_io.h>

#define GPIO_PCONP_REG 0x400FC0C4

#define GPIO_BASE_ADDR 0x2009C000
#define GPIO_DIR_OFFSET 0x0
#define GPIO_OUT_SET_OFFSET 0x018
#define GPIO_OUT_CLEAR_OFFSET 0x01C
#define GPIO_IN_OFFSET 0x014

#define GPIO_REG_ADDR(offset, n) GPIO_BASE_ADDR + offset + (0x20 * n)

#define GET_PORT(pin) (pin / 32) ? 2 :0
#define GET_PIN(pin) pin % 32

#define CONFIG_GPIO_LPC17XX_PORT_NAME "lpc_gpio"


struct gpio_lpc17xx_config {
	u32_t port_no;
};

struct gpio_lpc17xx_data {
	/* port ISR callback routine address */
	sys_slist_t callbacks;
	/* pin callback routine enable flags, by pin number */
	u32_t pin_callback_enables;
};

static int gpio_lpc17xx_configure(struct device *dev,
				   int access_op, u32_t pin, int flags)
{
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
	if (access_op == GPIO_ACCESS_BY_PIN) {
		/* input-0,output-1 */
		if ((flags & GPIO_DIR_MASK) == GPIO_DIR_IN) {
			sys_clear_bit(GPIO_REG_ADDR(GPIO_DIR_OFFSET, port), pin);
		} else {
			sys_set_bit(GPIO_REG_ADDR(GPIO_DIR_OFFSET, port), pin);
			sys_set_bit(GPIO_REG_ADDR(GPIO_OUT_SET_OFFSET, port), pin);
		}
	} else {

		return -EINVAL;
	}

	return 0;
}

static int gpio_lpc17xx_write(struct device *dev,
			       int access_op, u32_t pin, u32_t value)
{
	/* Get Port and Pin number */
	u32_t port = GET_PORT(pin);
	pin = GET_PIN(pin);

	if (access_op == GPIO_ACCESS_BY_PIN) {
		/* Set/Clear the data output for the respective pin */
		if (value)
			sys_set_bit(GPIO_REG_ADDR(GPIO_OUT_SET_OFFSET, port), pin);
		else
			sys_set_bit(GPIO_REG_ADDR(GPIO_OUT_CLEAR_OFFSET, port), pin);
	} else { /* return an error for all other options */
		return -EINVAL;
	}

	return 0;
}

static int gpio_lpc17xx_read(struct device *dev,
			      int access_op, u32_t pin, u32_t *value)
{
	/* Get Port and Pin number */
	u32_t port = GET_PORT(pin);
	pin = GET_PIN(pin);

	if (access_op == GPIO_ACCESS_BY_PIN) {
	  *value = sys_test_bit(GPIO_REG_ADDR(GPIO_IN_OFFSET, port), pin);
	} else { /* return an error for all other options */
		return -EINVAL;
	}

	return 0;
}

static int gpio_lpc17xx_init(struct device *dev)
{
	sys_set_bit(GPIO_PCONP_REG, 15);
	return 0;
}

static const struct gpio_driver_api gpio_lpc17xx_driver_api = {
	.config = gpio_lpc17xx_configure,
	.write = gpio_lpc17xx_write,
	.read = gpio_lpc17xx_read,
};

#ifdef CONFIG_GPIO_LPC17XX

static struct gpio_lpc17xx_data gpio_lpc17xx_port_data;

DEVICE_AND_API_INIT(gpio_lpc17xx_port, CONFIG_GPIO_LPC17XX_PORT_NAME,
		    gpio_lpc17xx_init,
		    &gpio_lpc17xx_port_data, NULL,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &gpio_lpc17xx_driver_api);
#endif /* CONFIG_GPIO_LPC17XX */

