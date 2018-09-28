/*
 * Copyright (c) 2017,  NXP
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <pinmux.h>
#include <soc.h>

static int zkit_arm_1769_pinmux_init(struct device *dev)
{
        ARG_UNUSED(dev);

#ifdef CONFIG_PINMUX_LPC17XX
        struct device *pinmux =
                device_get_binding(CONFIG_PINMUX_NAME);

#ifdef CONFIG_SPI_0
        const u32_t pin15_sck0_config = (
                IOCON_PIO_FUNC2  |
                IOCON_PIO_PULLUP |
                IOCON_PIO_NOD
                );

        const u32_t pin16_ssel0_config = (
                IOCON_PIO_FUNC0  |
                IOCON_PIO_PULLUP |
                IOCON_PIO_NOD
                );

        const u32_t pin17_miso0_config = (
                IOCON_PIO_FUNC2  |
                IOCON_PIO_PULLUP |
                IOCON_PIO_NOD
                );

        const u32_t pin18_mosi0_config = (
                IOCON_PIO_FUNC2  |
                IOCON_PIO_PULLUP |
                IOCON_PIO_NOD
                );

        pinmux_pin_set(pinmux, 15, pin15_sck0_config);
        pinmux_pin_set(pinmux, 16, pin16_ssel0_config);
        pinmux_pin_set(pinmux, 17, pin17_miso0_config);
        pinmux_pin_set(pinmux, 18, pin18_mosi0_config);
#endif

#ifdef CONFIG_SPI_1
        const u32_t pin7_sck1_config = (
                IOCON_PIO_FUNC2  |
                IOCON_PIO_PULLUP |
                IOCON_PIO_NOD
                );

        const u32_t pin6_ssel1_config = (
                IOCON_PIO_FUNC0  |
                IOCON_PIO_PULLUP |
                IOCON_PIO_NOD
                );

        const u32_t pin8_miso1_config = (
                IOCON_PIO_FUNC2  |
                IOCON_PIO_PULLUP |
                IOCON_PIO_NOD
                );

        const u32_t pin9_mosi1_config = (
                IOCON_PIO_FUNC2  |
                IOCON_PIO_PULLUP |
                IOCON_PIO_NOD
                );

        pinmux_pin_set(pinmux, 7, pin7_sck1_config);
        pinmux_pin_set(pinmux, 6, pin6_ssel1_config);
        pinmux_pin_set(pinmux, 8, pin8_miso1_config);
        pinmux_pin_set(pinmux, 9, pin9_mosi1_config);
#endif

#ifdef CONFIG_I2C_0
        const u32_t pin0_27_sda0_config = (
                IOCON_PIO_FUNC1  |
                IOCON_PIO_NPUNPD |
                IOCON_PIO_OD
                );

        const u32_t pin0_28_scl0_config = (
                IOCON_PIO_FUNC1  |
                IOCON_PIO_NPUNPD |
                IOCON_PIO_OD
                );

        pinmux_pin_set(pinmux, 27, pin0_27_sda0_config);
        pinmux_pin_set(pinmux, 28, pin0_28_scl0_config);
#endif	

#ifdef CONFIG_I2C_1
        const u32_t pin0_0_sda1_config = (
                IOCON_PIO_FUNC3  |
                IOCON_PIO_NPUNPD |
                IOCON_PIO_OD
                );

        const u32_t pin0_1_scl1_config = (
                IOCON_PIO_FUNC3  |
                IOCON_PIO_NPUNPD |
                IOCON_PIO_OD
                );

        pinmux_pin_set(pinmux, 0, pin0_0_sda1_config);
        pinmux_pin_set(pinmux, 1, pin0_1_scl1_config);
#endif

#ifdef CONFIG_I2C_2
        const u32_t pin0_10_sda2_config = (
                IOCON_PIO_FUNC2  |
                IOCON_PIO_NPUNPD |
                IOCON_PIO_OD
                );

        const u32_t pin0_11_scl2_config = (
                IOCON_PIO_FUNC2  |
                IOCON_PIO_NPUNPD |
                IOCON_PIO_OD
                );

        pinmux_pin_set(pinmux, 27, pin0_10_sda2_config);
        pinmux_pin_set(pinmux, 28, pin0_11_scl2_config);
#endif
	
#endif

        return 0;
}

SYS_INIT(zkit_arm_1769_pinmux_init,  PRE_KERNEL_1,
         CONFIG_PINMUX_INIT_PRIORITY);
