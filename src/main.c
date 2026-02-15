#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/devicetree.h>
#include <hal/nrf_saadc.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>

#include "ble.h"

#define TPS_EN_PIN 31
#define MAX30101_SENSOR_CHANNEL SENSOR_CHAN_GREEN

struct __packed sensor_packet {
	uint64_t timestamp;
	int32_t  ecg;
	int32_t  resp;
};

const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));


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
    struct sensor_packet burst_buffer[10];
    int32_t val;
    uint32_t now = k_ticks_to_us_near32(k_uptime_ticks());

    /* The hardware told us 10 samples are ready. Let's grab them. */
    for (int i = 0; i < 10; i++) {
        /* We still call this 10 times because the driver fetch handles one FIFO level */
        sensor_sample_fetch(dev);
        sensor_channel_get(dev, MAX30101_SENSOR_CHANNEL, (struct sensor_value *)&val);

        burst_buffer[i].timestamp = now; // Back-calculate 10ms steps
        burst_buffer[i].ecg = val;
        // burst_buffer[i].resp = 0;
    }

    if (ble_is_ready()) {
        /* Send all 160 bytes at once */
        ble_send_sensor_data(burst_buffer, sizeof(burst_buffer));
    }
}
#endif

void on_ble_connect(struct bt_conn *conn, uint8_t err)
{
    if (err) return;
    printk("Device Connected! Powering ON sensor...\n");

    /* Turn ON the TPS62746 voltage regulator */
    gpio_pin_set(gpio_dev, TPS_EN_PIN, 1);

    /* Give the sensor a moment to wake up */
    k_msleep(50);
}

void on_ble_disconnect(struct bt_conn *conn, uint8_t reason)
{
    printk("Device Disconnected. Powering OFF sensor...\n");

    /* Turn OFF the TPS62746 voltage regulator */
    gpio_pin_set(gpio_dev, TPS_EN_PIN, 0);
}

/* Define the callback structure */
struct bt_conn_cb connection_callbacks = {
    .connected = on_ble_connect,
    .disconnected = on_ble_disconnect,
};

int main(void)
{


    /* 2. Power on the sensor and set up nRF52 "Ears" (GPIO) */
    gpio_pin_configure(gpio_dev, TPS_EN_PIN, GPIO_OUTPUT_HIGH);
    gpio_pin_set(gpio_dev, TPS_EN_PIN, 0); // Start with sensor powered off

    k_msleep(200); // Wait for the sensor to power up

    bt_conn_cb_register(&connection_callbacks);

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
        // trig_drdy.type = SENSOR_TRIG_DATA_READY;
        trig_drdy.type = SENSOR_TRIG_FIFO_WATERMARK;
        trig_drdy.chan = MAX30101_SENSOR_CHANNEL;
        sensor_trigger_set(dev, &trig_drdy, sensor_data_ready);
    #endif /* CONFIG_MAX30101_TRIGGER */

    int32_t green_value;

    while (1) {
        #if !CONFIG_MAX30101_TRIGGER
            printk("Polling for new sensor data...\n");
            print_sample_fetch(dev, &green_value);
        #endif /* !CONFIG_MAX30101_TRIGGER */

        k_sleep(K_FOREVER);

        }
    return 0;
}
