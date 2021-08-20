#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <drivers/counter.h>
#include <drivers/i2c.h>
#include <drivers/can.h>
#include <drivers/uart.h>
#include <usb/usb_device.h>
#include <storage/disk_access.h>
#include <fs/fs.h>
#include <ff.h>

const struct gpio_dt_spec led[] = {
	GPIO_DT_SPEC_GET(DT_NODELABEL(led_red), gpios),
	GPIO_DT_SPEC_GET(DT_NODELABEL(led_green), gpios),
	GPIO_DT_SPEC_GET(DT_NODELABEL(led_blue), gpios)
};

const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_NODELABEL(user_button), gpios);
struct gpio_callback btn_cb_data;

const struct device *temp_dev = DEVICE_DT_GET(DT_NODELABEL(temp0));

const struct device *counter_dev = DEVICE_DT_GET(DT_NODELABEL(rtc));

const struct gpio_dt_spec sddet = GPIO_DT_SPEC_GET(DT_NODELABEL(sd_detect), gpios);

const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

const struct device *can_dev = DEVICE_DT_GET(DT_NODELABEL(flexcan0));

const struct device *usbcdc_dev;

const struct gpio_dt_spec usb0_id = GPIO_DT_SPEC_GET(DT_NODELABEL(usbid_0), gpios);
const struct gpio_dt_spec usb1_id = GPIO_DT_SPEC_GET(DT_NODELABEL(usbid_1), gpios);

// app
void btn_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
	printk("Button pressed!\r\n");
}

int main() {
	gpio_pin_configure_dt(&led[0], GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led[1], GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led[2], GPIO_OUTPUT_INACTIVE);

	gpio_pin_configure_dt(&btn, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&btn_cb_data, btn_cb, BIT(btn.pin));
	gpio_add_callback(btn.port, &btn_cb_data);

	counter_start(counter_dev);

	gpio_pin_configure_dt(&sddet, GPIO_INPUT);

	gpio_pin_configure_dt(&usb0_id, GPIO_INPUT);
	gpio_pin_configure_dt(&usb1_id, GPIO_INPUT);

	if (gpio_pin_get_dt(&sddet)) {
		FATFS fat_fs;
		struct fs_mount_t mp = {
			.type = FS_FATFS,
			.fs_data = &fat_fs,
			.mnt_point = "/SD:"
		};
		printk("Initializing SD: %d\r\n", disk_access_init("SD"));
		printk("Mounting FS: %d\r\n", fs_mount(&mp));
		struct fs_dir_t dirp;
		struct fs_dirent entry;
		fs_dir_t_init(&dirp);
		fs_opendir(&dirp, mp.mnt_point);
		while (1) {
			fs_readdir(&dirp, &entry);
			if (entry.name[0] == 0) {
				break;
			}
			printk("type: %d name: %s size: %d\r\n", entry.type, entry.name, entry.size);
		}
		fs_closedir(&dirp);
		printk("Unmounting FS: %d\r\n", fs_unmount(&mp));
	}

	for (uint8_t addr = 0; addr < 128; addr++) {
		if (!i2c_write(i2c_dev, NULL, 0, addr)) {
			printk("I2C device found! %d\r\n", addr);
		}
	}

	can_configure(can_dev, CAN_NORMAL_MODE, 1000000);

	// downclocked to 120MHz for now as workaround to get USB working
	const struct device *cdc_dev = device_get_binding("CDC_ACM_0");
	printk("Init USB: %d\r\n", usb_enable(NULL));

	int idx = 0;
	while (1) {
		gpio_pin_toggle_dt(&led[idx++]); idx %= 3;

		sensor_sample_fetch(temp_dev);
		struct sensor_value temp_val;
		sensor_channel_get(temp_dev, SENSOR_CHAN_DIE_TEMP, &temp_val);

		uint32_t ticks;
		counter_get_value(counter_dev, &ticks);

		// can_write(can_dev, (uint8_t*) &ticks, 4, 0x69, CAN_DATAFRAME, K_FOREVER);

		printk("Hello World! %.3fC %u %d %d %d\r\n",
			sensor_value_to_double(&temp_val),
			ticks,
			gpio_pin_get_dt(&sddet),
			gpio_pin_get_dt(&usb0_id),
			gpio_pin_get_dt(&usb1_id));
		k_msleep(1000);
	}

	return 0;
}