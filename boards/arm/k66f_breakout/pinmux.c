#include <init.h>
#include <drivers/pinmux.h>
#include <fsl_port.h>

static int k66f_breakout_pinmux_init(const struct device *dev)
{
	ARG_UNUSED(dev);

#if DT_NODE_HAS_STATUS(DT_NODELABEL(porta), okay)
	__unused const struct device *porta =
		DEVICE_DT_GET(DT_NODELABEL(porta));
	__ASSERT_NO_MSG(device_is_ready(porta));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(portb), okay)
	__unused const struct device *portb =
		DEVICE_DT_GET(DT_NODELABEL(portb));
	__ASSERT_NO_MSG(device_is_ready(portb));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(portc), okay)
	__unused const struct device *portc =
		DEVICE_DT_GET(DT_NODELABEL(portc));
	__ASSERT_NO_MSG(device_is_ready(portc));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(portd), okay)
	__unused const struct device *portd =
		DEVICE_DT_GET(DT_NODELABEL(portd));
	__ASSERT_NO_MSG(device_is_ready(portd));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(porte), okay)
	__unused const struct device *porte =
		DEVICE_DT_GET(DT_NODELABEL(porte));
	__ASSERT_NO_MSG(device_is_ready(porte));
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart0), okay) && CONFIG_SERIAL
	pinmux_pin_set(portb, 16, PORT_PCR_MUX(kPORT_MuxAlt3));
	pinmux_pin_set(portb, 17, PORT_PCR_MUX(kPORT_MuxAlt3));
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi0), okay) && CONFIG_SPI
	pinmux_pin_set(portc,  5, PORT_PCR_MUX(kPORT_MuxAlt2));
	pinmux_pin_set(portc,  6, PORT_PCR_MUX(kPORT_MuxAlt2));
	pinmux_pin_set(portc,  7, PORT_PCR_MUX(kPORT_MuxAlt2));
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi1), okay) && CONFIG_SPI
	pinmux_pin_set(porte,  1, PORT_PCR_MUX(kPORT_MuxAlt7));
	pinmux_pin_set(porte,  2, PORT_PCR_MUX(kPORT_MuxAlt2));
	pinmux_pin_set(porte,  3, PORT_PCR_MUX(kPORT_MuxAlt7));
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c0), okay) && CONFIG_I2C
	pinmux_pin_set(portd, 8, PORT_PCR_MUX(kPORT_MuxAlt2)
		| PORT_PCR_ODE_MASK);
	pinmux_pin_set(portd, 9, PORT_PCR_MUX(kPORT_MuxAlt2)
		| PORT_PCR_ODE_MASK);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(flexcan0), okay) && CONFIG_CAN
	pinmux_pin_set(portb, 18, PORT_PCR_MUX(kPORT_MuxAlt2));
	pinmux_pin_set(portb, 19, PORT_PCR_MUX(kPORT_MuxAlt2)
		| PORT_PCR_PE_MASK | PORT_PCR_PS_MASK);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(enet), okay) && CONFIG_NET_L2_ETHERNET
	pinmux_pin_set(porta,  5, PORT_PCR_MUX(kPORT_MuxAlt4));
	pinmux_pin_set(porta, 12, PORT_PCR_MUX(kPORT_MuxAlt4));
	pinmux_pin_set(porta, 13, PORT_PCR_MUX(kPORT_MuxAlt4));
	pinmux_pin_set(porta, 14, PORT_PCR_MUX(kPORT_MuxAlt4));
	pinmux_pin_set(porta, 15, PORT_PCR_MUX(kPORT_MuxAlt4));
	pinmux_pin_set(porta, 16, PORT_PCR_MUX(kPORT_MuxAlt4));
	pinmux_pin_set(porta, 17, PORT_PCR_MUX(kPORT_MuxAlt4));

	pinmux_pin_set(portb,  0, PORT_PCR_MUX(kPORT_MuxAlt4)
		| PORT_PCR_ODE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK);

	pinmux_pin_set(portb,  1, PORT_PCR_MUX(kPORT_MuxAlt4));

	pinmux_pin_set(porte, 26, PORT_PCR_MUX(kPORT_MuxAlt2));
#endif

	return 0;
}

SYS_INIT(k66f_breakout_pinmux_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);
