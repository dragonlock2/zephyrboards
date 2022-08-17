#ifndef ZEPHYRBOARDS_INCLUDE_DT_BINDINGS_SWM_LPC84X_SWM_H_
#define ZEPHYRBOARDS_INCLUDE_DT_BINDINGS_SWM_LPC84X_SWM_H_

/* derived from <fsl_swm_connections.h> */

#define kSWM_PortPin_P0_0   0U
#define kSWM_PortPin_P0_1   1U
#define kSWM_PortPin_P0_2   2U
#define kSWM_PortPin_P0_3   3U
#define kSWM_PortPin_P0_4   4U
#define kSWM_PortPin_P0_5   5U
#define kSWM_PortPin_P0_6   6U
#define kSWM_PortPin_P0_7   7U
#define kSWM_PortPin_P0_8   8U
#define kSWM_PortPin_P0_9   9U
#define kSWM_PortPin_P0_10  10U
#define kSWM_PortPin_P0_11  11U
#define kSWM_PortPin_P0_12  12U
#define kSWM_PortPin_P0_13  13U
#define kSWM_PortPin_P0_14  14U
#define kSWM_PortPin_P0_15  15U
#define kSWM_PortPin_P0_16  16U
#define kSWM_PortPin_P0_17  17U
#define kSWM_PortPin_P0_18  18U
#define kSWM_PortPin_P0_19  19U
#define kSWM_PortPin_P0_20  20U
#define kSWM_PortPin_P0_21  21U
#define kSWM_PortPin_P0_22  22U
#define kSWM_PortPin_P0_23  23U
#define kSWM_PortPin_P0_24  24U
#define kSWM_PortPin_P0_25  25U
#define kSWM_PortPin_P0_26  26U
#define kSWM_PortPin_P0_27  27U
#define kSWM_PortPin_P0_28  28U
#define kSWM_PortPin_P0_29  29U
#define kSWM_PortPin_P0_30  30U
#define kSWM_PortPin_P0_31  31U

#define kSWM_PortPin_P1_0   32U
#define kSWM_PortPin_P1_1   33U
#define kSWM_PortPin_P1_2   34U
#define kSWM_PortPin_P1_3   35U
#define kSWM_PortPin_P1_4   36U
#define kSWM_PortPin_P1_5   37U
#define kSWM_PortPin_P1_6   38U
#define kSWM_PortPin_P1_7   39U
#define kSWM_PortPin_P1_8   40U
#define kSWM_PortPin_P1_9   41U
#define kSWM_PortPin_P1_10  42U
#define kSWM_PortPin_P1_11  43U
#define kSWM_PortPin_P1_12  44U
#define kSWM_PortPin_P1_13  45U
#define kSWM_PortPin_P1_14  46U
#define kSWM_PortPin_P1_15  47U
#define kSWM_PortPin_P1_16  48U
#define kSWM_PortPin_P1_17  49U
#define kSWM_PortPin_P1_18  50U
#define kSWM_PortPin_P1_19  51U
#define kSWM_PortPin_P1_20  52U
#define kSWM_PortPin_P1_21  53U
#define kSWM_PortPin_Reset  0xffU


#define kSWM_USART0_TXD        0U
#define kSWM_USART0_RXD        1U
#define kSWM_USART0_RTS        2U
#define kSWM_USART0_CTS        3U
#define kSWM_USART0_SCLK       4U
#define kSWM_USART1_TXD        5U
#define kSWM_USART1_RXD        6U
#define kSWM_USART1_RTS        7U
#define kSWM_USART1_CTS        8U
#define kSWM_USART1_SCLK       9U
#define kSWM_USART2_TXD        10U
#define kSWM_USART2_RXD        11U
#define kSWM_USART2_RTS        12U
#define kSWM_USART2_CTS        13U
#define kSWM_USART2_SCLK       14U
#define kSWM_SPI0_SCK          15U
#define kSWM_SPI0_MOSI         16U
#define kSWM_SPI0_MISO         17U
#define kSWM_SPI0_SSEL0        18U
#define kSWM_SPI0_SSEL1        19U
#define kSWM_SPI0_SSEL2        20U
#define kSWM_SPI0_SSEL3        21U
#define kSWM_SPI1_SCK          22U
#define kSWM_SPI1_MOSI         23U
#define kSWM_SPI1_MISO         24U
#define kSWM_SPI1_SSEL0        25U
#define kSWM_SPI1_SSEL1        26U
#define kSWM_SCT_PIN0          27U
#define kSWM_SCT_PIN1          28U
#define kSWM_SCT_PIN2          29U
#define kSWM_SCT_PIN3          30U
#define kSWM_SCT_OUT0          31U
#define kSWM_SCT_OUT1          32U
#define kSWM_SCT_OUT2          33U
#define kSWM_SCT_OUT3          34U
#define kSWM_SCT_OUT4          35U
#define kSWM_SCT_OUT5          36U
#define kSWM_SCT_OUT6          37U
#define kSWM_I2C1_SDA          38U
#define kSWM_I2C1_SCL          39U
#define kSWM_I2C2_SDA          40U
#define kSWM_I2C2_SCL          41U
#define kSWM_I2C3_SDA          42U
#define kSWM_I2C3_SCL          43U
#define kSWM_ACMP_OUT          44U
#define kSWM_CLKOUT            45U
#define kSWM_GPIO_INT_BMAT     46U
#define kSWM_USART3_TXD        47U
#define kSWM_USART3_RXD        48U
#define kSWM_USART3_SCLK       49U
#define kSWM_USART4_TXD        50U
#define kSWM_USART4_RXD        51U
#define kSWM_USART4_SCLK       52U
#define kSWM_T0_MAT_CHN0       53U
#define kSWM_T0_MAT_CHN1       54U
#define kSWM_T0_MAT_CHN2       55U
#define kSWM_T0_MAT_CHN3       56U
#define kSWM_T0_CAP_CHN0       57U
#define kSWM_T0_CAP_CHN1       58U
#define kSWM_T0_CAP_CHN2       59U
#define kSWM_MOVABLE_NUM_FUNCS 60U


#define kSWM_ACMP_INPUT1        SWM_PINENABLE0_ACMP_I1_MASK
#define kSWM_ACMP_INPUT2        SWM_PINENABLE0_ACMP_I2_MASK
#define kSWM_ACMP_INPUT3        SWM_PINENABLE0_ACMP_I3_MASK
#define kSWM_ACMP_INPUT4        SWM_PINENABLE0_ACMP_I4_MASK
#define kSWM_ACMP_INPUT5        SWM_PINENABLE0_ACMP_I5_MASK
#define kSWM_SWCLK              SWM_PINENABLE0_SWCLK_MASK
#define kSWM_SWDIO              SWM_PINENABLE0_SWDIO_MASK
#define kSWM_XTALIN             SWM_PINENABLE0_XTALIN_MASK
#define kSWM_XTALOUT            SWM_PINENABLE0_XTALOUT_MASK
#define kSWM_RESETN             SWM_PINENABLE0_RESETN_MASK
#define kSWM_CLKIN              SWM_PINENABLE0_CLKIN_MASK
#define kSWM_VDDCMP             SWM_PINENABLE0_VDDCMP_MASK
#define kSWM_I2C0_SDA           SWM_PINENABLE0_I2C0_SDA_MASK
#define kSWM_I2C0_SCL           SWM_PINENABLE0_I2C0_SCL_MASK
#define kSWM_ADC_CHN0           SWM_PINENABLE0_ADC_0_MASK
#define kSWM_ADC_CHN1           SWM_PINENABLE0_ADC_1_MASK
#define kSWM_ADC_CHN2           SWM_PINENABLE0_ADC_2_MASK
#define kSWM_ADC_CHN3           SWM_PINENABLE0_ADC_3_MASK
#define kSWM_ADC_CHN4           SWM_PINENABLE0_ADC_4_MASK
#define kSWM_ADC_CHN5           SWM_PINENABLE0_ADC_5_MASK
#define kSWM_ADC_CHN6           SWM_PINENABLE0_ADC_6_MASK
#define kSWM_ADC_CHN7           SWM_PINENABLE0_ADC_7_MASK
#define kSWM_ADC_CHN8           SWM_PINENABLE0_ADC_8_MASK
#define kSWM_ADC_CHN9           SWM_PINENABLE0_ADC_9_MASK
#define kSWM_ADC_CHN10          SWM_PINENABLE0_ADC_10_MASK
#define kSWM_ADC_CHN11          SWM_PINENABLE0_ADC_11_MASK
#define kSWM_DAC_OUT0           SWM_PINENABLE0_DACOUT0_MASK
#define kSWM_DAC_OUT1           SWM_PINENABLE0_DACOUT1_MASK
#define kSWM_CAPT_X0            SWM_PINENABLE0_CAPT_X0_MASK
#define kSWM_CAPT_X1            SWM_PINENABLE0_CAPT_X1_MASK
#define kSWM_CAPT_X2            SWM_PINENABLE0_CAPT_X2_MASK
#define kSWM_CAPT_X3            (int)SWM_PINENABLE0_CAPT_X3_MASK
#define kSWM_CAPT_X4            (int)(SWM_PINENABLE1_CAPT_X4_MASK | 0x80000000U)
#define kSWM_CAPT_X5            (int)(SWM_PINENABLE1_CAPT_X5_MASK | 0x80000000U)
#define kSWM_CAPT_X6            (int)(SWM_PINENABLE1_CAPT_X6_MASK | 0x80000000U)
#define kSWM_CAPT_X7            (int)(SWM_PINENABLE1_CAPT_X7_MASK | 0x80000000U)
#define kSWM_CAPT_X8            (int)(SWM_PINENABLE1_CAPT_X8_MASK | 0x80000000U)
#define kSWM_CAPT_YL            (int)(SWM_PINENABLE1_CAPT_YL_MASK | 0x80000000U)
#define kSWM_CAPT_YH            (int)(SWM_PINENABLE1_CAPT_YH_MASK | 0x80000000U)
#define kSWM_FIXEDPIN_NUM_FUNCS (int)0x80000041U

#endif /* ZEPHYRBOARDS_INCLUDE_DT_BINDINGS_SWM_LPC84X_SWM_H_ */
