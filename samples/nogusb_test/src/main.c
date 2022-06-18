#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/adc.h>
#include <drivers/spi.h>
#include <drivers/i2c.h>
#include <drivers/dac.h>
#include <drivers/can.h>
#include <usb/usb_device.h>

const struct gpio_dt_spec led[] = {
	GPIO_DT_SPEC_GET(DT_NODELABEL(led_red), gpios),
	GPIO_DT_SPEC_GET(DT_NODELABEL(led_green), gpios),
	GPIO_DT_SPEC_GET(DT_NODELABEL(led_blue), gpios)
};

const struct gpio_dt_spec btn[] = {
	GPIO_DT_SPEC_GET(DT_NODELABEL(dbg_1), gpios),
	GPIO_DT_SPEC_GET(DT_NODELABEL(dbg_2), gpios),
};
struct gpio_callback btn_cb_data;

const struct device *adc = DEVICE_DT_GET(DT_NODELABEL(adc1));
#define ADC_GAIN       ADC_GAIN_1
#define ADC_RESOLUTION 12
#define VBUS_CHANNEL   1
#define ADC_VREF       3300
#define VBUS_DIV       11 // 10K/100K divider

const struct device* spi = DEVICE_DT_GET(DT_NODELABEL(spi2));

const struct device* i2c = DEVICE_DT_GET(DT_NODELABEL(i2c2));

const struct device* dac = DEVICE_DT_GET(DT_NODELABEL(dac1));

const struct device *can = DEVICE_DT_GET(DT_NODELABEL(can1));

const struct zcan_filter can_filter = {
	.id_type  = CAN_STANDARD_IDENTIFIER,
	.rtr_mask = 0,
	.id_mask  = 0
};

CAN_MSGQ_DEFINE(can_msgq, 16);

// app
void btn_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
	if (pins == BIT(btn[0].pin)) {
		printk("DBG1 pressed!\r\n");
	} else if (pins == BIT(btn[1].pin)) {
		printk("DBG2 pressed!\r\n");
	} else {
		printk("Ghost button 0.0\r\n");
	}
}

int main() {
	// GPIO
	gpio_pin_configure_dt(&led[0], GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led[1], GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led[2], GPIO_OUTPUT_INACTIVE);

	gpio_pin_toggle_dt(&led[0]); // indicate powered up

	gpio_pin_configure_dt(&btn[0], GPIO_INPUT);
	gpio_pin_configure_dt(&btn[1], GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&btn[0], GPIO_INT_EDGE_TO_ACTIVE);
	gpio_pin_interrupt_configure_dt(&btn[1], GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&btn_cb_data, btn_cb, BIT(btn[0].pin) | BIT(btn[1].pin));
	gpio_add_callback(btn[0].port, &btn_cb_data); // both same port

	// USB
	const struct device *cdc_dev = device_get_binding("CDC_ACM_0");
	printk("Init USB: %d\r\n", usb_enable(NULL));
	UNUSED(cdc_dev);

	// ADC
	uint16_t vbus_samples[1];
	struct adc_channel_cfg vbus_cfg = {
		.gain             = ADC_GAIN,
		.reference        = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME_MAX,
		.channel_id       = VBUS_CHANNEL,
		.differential     = 0
	};
	struct adc_sequence vbus_seq = {
		.options      = NULL,
		.channels     = BIT(VBUS_CHANNEL),
		.buffer       = vbus_samples,
		.buffer_size  = sizeof(vbus_samples),
		.resolution   = ADC_RESOLUTION,
		.oversampling = 0,
		.calibrate    = false
	};
	adc_channel_setup(adc, &vbus_cfg);
	// TODO figure out how to run calibration

	// SPI
	struct spi_config spi_cfg = {0};
	spi_cfg.frequency = 250000;
	spi_cfg.operation |= SPI_OP_MODE_MASTER;
	// spi_cfg.operation |= SPI_MODE_CPOL;
	// spi_cfg.operation |= SPI_MODE_CPHA;
	// spi_cfg.operation |= SPI_MODE_LOOP;
	spi_cfg.operation |= SPI_TRANSFER_LSB;
	spi_cfg.operation |= SPI_WORD_SET(8); // bits/word
	spi_cfg.operation |= SPI_LINES_SINGLE;
	// spi_cfg.operation |= SPI_HOLD_ON_CS;
	// spi_cfg.operation |= SPI_LOCK_ON;
	// spi_cfg.operation |= SPI_CS_ACTIVE_HIGH;

	uint8_t spi_tx[4] = {1,2,3,4}, spi_rx[4];
	struct spi_buf spi_tx_buf = {
		.buf = spi_tx,
		.len = sizeof(spi_tx)
	};
	struct spi_buf spi_rx_buf = {
		.buf = spi_rx,
		.len = sizeof(spi_rx)
	};
	struct spi_buf_set spi_tx_buf_set = {
		.buffers = &spi_tx_buf,
		.count   = 1
	};
	struct spi_buf_set spi_rx_buf_set = {
		.buffers = &spi_rx_buf,
		.count   = 1
	};
	spi_transceive(spi, &spi_cfg, &spi_tx_buf_set, &spi_rx_buf_set);
	printk("SPI transceived:\r\n");
	for (int i = 0; i < sizeof(spi_tx); i++) {
		printk("TX: %x\tRX: %x\r\n", spi_tx[i], spi_rx[i]);
	}

	// I2C
	for (uint8_t addr = 0; addr < 128; addr++) {
		if (!i2c_write(i2c, NULL, 0, addr)) {
			printk("I2C device found! %d\r\n", addr);
		}
	}

	// DAC
	struct dac_channel_cfg dac_cfg = {
		.channel_id = 2, // OUT2
		.resolution = 12
	};
	dac_channel_setup(dac, &dac_cfg);
	dac_write_value(dac, 2, 1241); // 1V

	// CAN
	can_set_mode(can, CAN_MODE_LOOPBACK);
	/* TODO test physically */
	can_set_bitrate(can, 1000000);
	can_set_bitrate_data(can, 4000000);
	can_add_rx_filter_msgq(can, &can_msgq, &can_filter);

	struct zcan_frame txmsg = {
		.id      = 0x69,
		.id_type = CAN_STANDARD_IDENTIFIER,
		.rtr     = CAN_DATAFRAME,
		.fd      = true,
		.brs     = true
	};

	int idx = 0;
	while (1) {
		adc_read(adc, &vbus_seq);
		int32_t vbus = vbus_samples[0];
		adc_raw_to_millivolts(VBUS_DIV * ADC_VREF,
			ADC_GAIN, ADC_RESOLUTION, &vbus);
		printk("VBUS: %d\r\n", vbus);

		memcpy(txmsg.data, &vbus, sizeof(vbus));
		txmsg.dlc = can_bytes_to_dlc(sizeof(vbus));
		can_send(can, &txmsg, K_FOREVER, NULL, NULL);

		struct zcan_frame rxmsg;
		k_msgq_get(&can_msgq, &rxmsg, K_FOREVER);
		printk("ID: %d DLC: %d\r\n", rxmsg.id, rxmsg.dlc);

		gpio_pin_toggle_dt(&led[idx++]); idx %= 3;
		k_msleep(500);
	}

	return 0;
}
