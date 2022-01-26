#include <init.h>
#include <drivers/pinmux.h>
#include <soc.h>

static int board_pinmux_init(const struct device *dev)
{
    const struct device *muxa = DEVICE_DT_GET(DT_NODELABEL(pinmux_a));
    const struct device *muxb = DEVICE_DT_GET(DT_NODELABEL(pinmux_b));

    ARG_UNUSED(dev);

    if (!device_is_ready(muxa)) {
        return -ENXIO;
    }
    if (!device_is_ready(muxb)) {
        return -ENXIO;
    }

#if (ATMEL_SAM0_DT_SERCOM_CHECK(0, atmel_sam0_spi) && CONFIG_SPI_SAM0)
#warning Pin mapping may not be configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(1, atmel_sam0_spi) && CONFIG_SPI_SAM0)
#warning Pin mapping may not be configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(2, atmel_sam0_spi) && CONFIG_SPI_SAM0)
#warning Pin mapping may not be configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(3, atmel_sam0_spi) && CONFIG_SPI_SAM0)
#warning Pin mapping may not be configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(4, atmel_sam0_spi) && CONFIG_SPI_SAM0)
#warning Pin mapping may not be configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(5, atmel_sam0_spi) && CONFIG_SPI_SAM0)
#warning Pin mapping may not be configured
#endif

#if (ATMEL_SAM0_DT_SERCOM_CHECK(0, atmel_sam0_i2c) && CONFIG_I2C_SAM0)
#warning Pin mapping may not be configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(1, atmel_sam0_i2c) && CONFIG_I2C_SAM0)
#warning Pin mapping may not be configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(2, atmel_sam0_i2c) && CONFIG_I2C_SAM0)
#warning Pin mapping may not be configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(3, atmel_sam0_i2c) && CONFIG_I2C_SAM0)
#warning Pin mapping may not be configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(4, atmel_sam0_i2c) && CONFIG_I2C_SAM0)
    /* SERCOM4 on SDA=PB8, SCL=PB9 */
    pinmux_pin_set(muxb, 8,  PINMUX_FUNC_D);
    pinmux_pin_set(muxb, 9,  PINMUX_FUNC_D);
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(5, atmel_sam0_i2c) && CONFIG_I2C_SAM0)
#warning Pin mapping may not be configured
#endif

#if (ATMEL_SAM0_DT_TCC_CHECK(0, atmel_sam0_tcc_pwm) && CONFIG_PWM_SAM0_TCC)
    /* TCC0 on WO0=PA8, WO1=PA9, WO2=PA10 */
    pinmux_pin_set(muxa, 8,  PINMUX_FUNC_E);
    pinmux_pin_set(muxa, 9,  PINMUX_FUNC_E);
    pinmux_pin_set(muxa, 10, PINMUX_FUNC_F);
#endif

#ifdef CONFIG_USB_DC_SAM0
    /* USB DP on PA25, USB DM on PA24 */
    pinmux_pin_set(muxa, 25, PINMUX_FUNC_G);
    pinmux_pin_set(muxa, 24, PINMUX_FUNC_G);
#endif
    return 0;
}

SYS_INIT(board_pinmux_init, PRE_KERNEL_2, CONFIG_PINMUX_INIT_PRIORITY);
