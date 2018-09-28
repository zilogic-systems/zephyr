/*
 * Copyright (c) 2018 Zilogic Systems.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr.h>
#include <soc.h>
#include <kernel.h>
#include <device.h>
#include <i2c.h>
#include <errno.h>
#include "i2c_lpc17xx.h"

#include <clock_control/lpc17xx_clock_control.h>
#include <clock_control.h>

#define DEV_CFG(dev) \
	((const struct i2c_lpc17xx_conf *const)(dev)->config->config_info)

/* Device constant configuration parameters */
struct i2c_lpc17xx_config {
	u32_t  base;
	struct lpc17xx_clock_t pclk;
};


static void sys_set_bits(uint32_t address, uint32_t mask, uint32_t shift, uint32_t data)
{
	uint32_t temp = sys_read32(address);
	temp &= ~(mask << shift);
	temp |= (data & mask) << shift;	
        sys_write32(temp, address);	
}


/*I2C Master Mode - Reset */
static void i2c_lpc17xx_master_mode(u32_t base)
{
	sys_write32(I2C_CONCLR_AAC, base + LPC17_I2C_CONCLR_OFFSET);  /* Clear AA flag */
	sys_write32(I2C_CONCLR_SIC, base + LPC17_I2C_CONCLR_OFFSET);  /* Clear SI flag */
	sys_write32(I2C_CONCLR_STAC, base + LPC17_I2C_CONCLR_OFFSET); /* Clear STA flag */
	sys_set_bits(base + LPC17_I2C_CONSET_OFFSET,0xF,6,0xF);       /* I2c Enable */
}

/* Perform a I2C transfer stop */
static void i2c_lpc17xx_i2c_stop(void)
{
        sys_write32(I2C_CONSET_STO, LPC17_I2C0_CONSET);
        sys_write32(I2C_CONCLR_SIC, LPC17_I2C0_CONCLR);

	/*
	 * When the bus detects the STOP condition, STO is cleared automatically
	 */
	while (1) {
		if ((sys_read32(LPC17_I2C0_CONSET) & I2C_CONSET_STO) == 0) {
			break;
		}
	}
}


/* Perform a I2C transfer start */
static void i2c_lpc17xx_i2c_start(u32_t base)
{
	sys_write32(I2C_CONCLR_SIC, base + LPC17_I2C_CONCLR_OFFSET);	/* Clear SI flag */
	sys_set_bits(base + LPC17_I2C_CONSET_OFFSET,0x1,5,0x1);      /* Set STA */
	while (1) {
		/* check status code whether start condition transmitted */
		if ((sys_read32(base + LPC17_I2C_STAT_OFFSET) & 0xFF) == STA_START) {	/* 0x08 */
			break;
		}
		/* check status code whether restart condition transmitted */
		if ((sys_read32(base + LPC17_I2C_STAT_OFFSET) & 0xFF) == STA_RESTART) {      /* 0x10 */
			break;
		}
	}
}


/* Perform a I2C Write transfer */
bool i2c_lpc17xx_write(struct device *dev, u8_t data)
{
	u8_t regvalue;
	const struct i2c_lpc17xx_config *const device_config = DEV_CFG(dev);
	u32_t base = device_config->base;
	
	/* Load Data */
	sys_write32(data, base + LPC17_I2C_DAT_OFFSET);

	/* Clear SI and STA flag*/
	sys_write32(I2C_CONCLR_SIC, base + LPC17_I2C_CONCLR_OFFSET);   /* Clear SI flag */
	sys_write32(I2C_CONCLR_STAC, base + LPC17_I2C_CONCLR_OFFSET);  /* Clear STA flag */

	
	while (1) {
		/* Read Status */
		regvalue = (sys_read32(base + LPC17_I2C_STAT_OFFSET)) & 0xFF;

		switch (regvalue) {
		case STA_SLAR_ACK:	/* 0x40 */
		case STA_SLAW_ACK:	/* 0x18 */
		case STA_DATA_ACK:	/* 0x28 */
			return 0;

		case STA_SLAR_NACK:	/* 0x48 */
		case STA_SLAW_NACK:	/* 0x20 */
		case STA_DATA_NACK:	/* 0x30 */
			return 1;
		}
	}
}


/* Perform a I2C Write Address */
static bool i2c_lpc17xx_write_address(struct device *dev, u8_t data, bool direction)
{
	return i2c_lpc17xx_write(dev, data | direction);
}


/* Perform a I2C receive transaction */
u8_t i2c_lpc17xx_read(struct device *dev, bool ack)
{
	u8_t regvalue;
	u32_t recv_data = 0x00;
	const struct i2c_lpc17xx_config *const device_config = DEV_CFG(dev);
	u32_t base = device_config->base;
	
	/* Set AA and Clear SI flag*/
	if (ack)
		sys_write32(I2C_CONSET_AA, base + LPC17_I2C_CONSET_OFFSET); /* Set AA */
	else
		sys_write32(I2C_CONCLR_AAC, base + LPC17_I2C_CONCLR_OFFSET); /* Clear AA */
	
	sys_write32(I2C_CONCLR_SIC, base + LPC17_I2C_CONCLR_OFFSET);	/* Clear SI flag */
	
	while (1) {
		/* Read Status */
		regvalue = (sys_read32(device_config->base + LPC17_I2C_STAT_OFFSET)) & 0xFF;
		
		switch (regvalue) {
		case 0x50:	/* 0x50 */
		case 0x58:	/* 0x58 */
			recv_data = (sys_read32(device_config->base + LPC17_I2C_DAT_OFFSET) & 0xFF);
			return recv_data;
		}
	}
}

/*   I2C Set Speed Mode */
static int set_i2c_speed(u32_t base, u8_t speed)
{
  	u32_t sclh;
	u32_t scll;
	u32_t scl_total;
	u32_t i2c_clock;
	u16_t freq = 100; /* Default 100Khz*/
	
	switch (speed) {
	case I2C_SPEED_STANDARD:
		if(base != LPC17_I2C0_BASE)
			break; /* Nothing to do for channel 1 & 2 */
		freq = 100; /* 100 Khz */
		sys_set_bits(LPC17_I2CPADCFG,0xf,0,0x0);
		break;
		
	case I2C_SPEED_FAST:
		if(base != LPC17_I2C0_BASE)
			break;  /* Nothing to do for channel 1 & 2 */
		freq = 400; /* 400 Khz */
		sys_set_bits(LPC17_I2CPADCFG,0xf,0,0x0);
		break;
		
	case I2C_SPEED_FAST_PLUS:
		if(base != LPC17_I2C0_BASE)
			return -ENOTSUP; /* Speed Not Supported for Channel 1 & 2 */
		freq = 1 * 1000; /* 1 Mhz */
		sys_set_bits(LPC17_I2CPADCFG,0xf,0,0xA);  
		break;
		
	default:
		return -ENOTSUP; /* Other Speeds are not supported */
	}

	clock_control_get_rate(device_get_binding(CONFIG_CLOCK_LABEL),
			       (clock_control_subsys_t) &cfg->pclk, &i2c_clock);
	scl_total = (i2c_clock) / (freq * 1000);
	
	/* Duty Cycle Set to 50% - Default */
	sclh = scl_total / 2;
	scll = scl_total / 2;

        sys_write32(scll, base + LPC17_I2C_SCLL_OFFSET);
	sys_write32(sclh, base + LPC17_I2C_SCLH_OFFSET);
	sys_write32(I2C_CONSET_I2EN, base + LPC17_I2C_CONSET_OFFSET);
	
	return 0;
}


/* Device Configure */
static int i2c_lpc17xx_configure(struct device *dev, u32_t config_raw)
{
	const struct i2c_lpc17xx_config *const device_config = DEV_CFG(dev);
	u32_t base = device_config->base;

	if (!(config_raw & I2C_MODE_MASTER))
		return -ENOTSUP; /* MASTER_MODE NOT CONFIGURED */
        i2c_lpc17xx_master_mode(base);
	
	if (config_raw & I2C_ADDR_10_BITS)
		return -ENOTSUP; /* NOT SUPPORTED */
	
	/* Configure Speed */
        return set_i2c_speed(base, I2C_SPEED_GET(config_raw));

}

/* I2C Init */
static int i2c_lpc17xx_init(struct device *dev)
{
	const struct i2c_lpc17xx_conf *const device_config = DEV_CFG(dev);
	int ret;

        /* Enabling Power to I2C */
	clock_control_on(device_get_binding(CONFIG_CLOCK_LABEL),
			 (clock_control_subsys_t) &device_config->pclk);
		
	ret = i2c_lpc17xx_configure(dev, I2C_MODE_MASTER | I2C_SPEED_SET(I2C_SPEED_STANDARD));
	if (ret < 0) {
		return ret;
	}
	
	return 0;
}

/* I2C Transfer */
static int i2c_lpc17xx_transfer(struct device *dev, struct i2c_msg *msgs,
				u8_t num_msgs, u16_t addr)
{
	const struct i2c_lpc17xx_config *const device_config = DEV_CFG(dev);
	u8_t mode;
	u8_t i,j;
	
	mode = msgs->flags & I2C_MSG_RW_MASK;
	
	for(i=0; i < num_msgs; i++, msgs++) {
	        i2c_lpc17xx_i2c_start(device_config->base);
	        i2c_lpc17xx_write_address(dev, addr, mode);
		
		if (mode == I2C_MSG_WRITE) {
			for(j=0; j < msgs->len; j++) {
			        i2c_lpc17xx_write(dev, msgs->buf[j]);
			}
		}
		else {
			for(j=0; j < msgs->len; j++) {
				if(j == (msgs->len - 1))
					msgs->buf[j] = i2c_lpc17xx_read(dev ,I2C_NACK);  /* NACK */
				else
					msgs->buf[j] = i2c_lpc17xx_read(dev ,I2C_ACK); /* ACK */
			}
		}
		
	        i2c_lpc17xx_i2c_stop();
	}

}

static const struct i2c_driver_api i2c_lpc17xx_driver_api = {
	.configure = i2c_lpc17xx_configure,
	.transfer = i2c_lpc17xx_transfer,
};


#ifdef I2C_0
static const struct i2c_lpc17xx_config i2c0_lpc17xx_config = {
	.base = CONFIG_I2C_0_BASE_ADDRESS,
	.pclk = {
		.en = CONFIG_I2C_0_CLOCK_ENABLE,
		.sel = CONFIG_I2C_0_CLOCK_SELECT
	}
};
DEVICE_AND_API_INIT(i2c0_lpc17xx, CONFIG_I2C_0_LABEL, &i2c_lpc17xx_init,
		    NULL, &i2c0_lpc17xx_config,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &i2c_lpc17xx_driver_api);
#endif /* I2C_0 */


#ifdef I2C_1
static const struct i2c_lpc17xx_config i2c1_lpc17xx_config = {
	.base = CONFIG_I2C_1_BASE_ADDRESS,
	.pclk = {
		.en = CONFIG_I2C_1_CLOCK_ENABLE,
		.sel = CONFIG_I2C_1_CLOCK_SELECT
	}
};
DEVICE_AND_API_INIT(i2c1_lpc17xx, CONFIG_I2C_1_LABEL, &i2c_lpc17xx_init,
		    NULL, &i2c1_lpc17xx_config,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &i2c_lpc17xx_driver_api);
#endif /* I2C_1 */


#ifdef I2C_2
static const struct i2c_lpc17xx_config i2c2_lpc17xx_config = {
	.base = CONFIG_I2C_2_BASE_ADDRESS,
	.pclk = {
		.en = CONFIG_I2C_2_CLOCK_ENABLE,
		.sel = CONFIG_I2C_2_CLOCK_SELECT
	}
};
DEVICE_AND_API_INIT(i2c2_lpc17xx, CONFIG_I2C_2_LABEL, &i2c_lpc17xx_init,
		    NULL, &i2c2_lpc17xx_config,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &i2c_lpc17xx_driver_api);
#endif /* I2C_2 */
