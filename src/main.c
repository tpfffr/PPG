#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/devicetree.h>
#include "ble.h"

#define TPS_EN_PIN 31

struct __packed sensor_packet {
	uint64_t timestamp;
	int32_t  ecg;
	int32_t  resp;
};

#define MAX30101_SENSOR_CHANNEL SENSOR_CHAN_GREEN


static void print_sample_fetch(const struct device *dev, int32_t *green_out)
{
	static struct sensor_value green;

	sensor_sample_fetch(dev);
	sensor_channel_get(dev, MAX30101_SENSOR_CHANNEL, &green);

	/* Print LED data*/
	printf("GREEN = %d\n", green.val1);
	*green_out = green.val1;
}

#if CONFIG_MAX30101_TRIGGER
static struct sensor_trigger trig_drdy;

void sensor_data_ready(const struct device *dev, const struct sensor_trigger *trigger)
{
    int32_t green_value;
	print_sample_fetch(dev, &green_value);

    if (ble_is_ready()) {
        struct sensor_packet pkt;

        pkt.timestamp = (uint64_t)k_uptime_get();
        pkt.ecg = green_value;
        pkt.resp = 0; // Not used, but could be set to red_value if desired

        /* Send the 16-byte packet to the Android app */
        ble_send_sensor_data(&pkt, sizeof(pkt));
    }
}
#endif


int main(void)
{
    const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));

    /* 2. Power on the sensor and set up nRF52 "Ears" (GPIO) */
    gpio_pin_configure(gpio_dev, TPS_EN_PIN, GPIO_OUTPUT_HIGH);
    gpio_pin_set(gpio_dev, TPS_EN_PIN, 1);

    k_msleep(200); // Wait for the sensor to power up

    ble_init();

    const struct device *const dev = DEVICE_DT_GET(DT_ALIAS(max30101));

    if (!device_is_ready(dev)) {
        printk("MAX30101 not ready\n");
        return -1;
    }

    struct sensor_value dummy;
    sensor_sample_fetch(dev);
    sensor_channel_get(dev, SENSOR_CHAN_GREEN, &dummy);

    #if CONFIG_MAX30101_TRIGGER
        printk("Setting up data ready trigger...\n");
        trig_drdy.type = SENSOR_TRIG_DATA_READY;
        // trig_drdy.type = SENSOR_TRIG_FIFO_WATERMARK;
        trig_drdy.chan = MAX30101_SENSOR_CHANNEL;
        sensor_trigger_set(dev, &trig_drdy, sensor_data_ready);
    #endif /* CONFIG_MAX30101_TRIGGER */

    int32_t green_value;

    while (1) {
        #if !CONFIG_MAX30101_TRIGGER
            printk("Polling for new sensor data...\n");
            print_sample_fetch(dev, &green_value);
        #endif /* !CONFIG_MAX30101_TRIGGER */

        k_sleep(K_MSEC(20));

        }
    return 0;
}

//     int32_t green_value, red_value;
//     const int sleep_ms = 10;

//     while (1) {

//         print_sample_fetch(dev, &green_value, &red_value);

//         if (ble_is_ready()) {
//             struct sensor_packet pkt;

//             pkt.timestamp = (uint64_t)k_uptime_get();
//             pkt.ecg = green_value;
//             pkt.resp = 0; // Not used, but could be set to red_value if desired

//             /* Send the 16-byte packet to the Android app */
//             ble_send_sensor_data(&pkt, sizeof(pkt));
//         }

//         k_msleep(sleep_ms);
//     }
//     return 0;
// }
