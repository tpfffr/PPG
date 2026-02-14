#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/devicetree.h>
#include "ble.h"
#include <zephyr/random/random.h>
#include <zephyr/drivers/i2c.h>

#define TPS_EN_PIN 31
#define INT_PIN 5  // P0.05


static struct k_sem data_ready_sem;
//

struct __packed sensor_packet {
	uint64_t timestamp;
	int32_t  ecg;
	int32_t  resp;
};

static struct gpio_callback int_cb_data;


void sensor_int_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    // Signal the main loop that the sensor has data ready
    k_sem_give(&data_ready_sem);
}

static void print_sample_fetch(const struct device *dev, int32_t *green_out, int32_t *red_out)
{
    static struct sensor_value green;
    // static struct sensor_value red;

    sensor_sample_fetch(dev);
    sensor_channel_get(dev, SENSOR_CHAN_GREEN, &green);
    // sensor_channel_get(dev, SENSOR_CHAN_RED, &red);

    printk("GREEN = %d\n", (int32_t)green.val1);
    // printk("RED = %d\n", (int32_t)red.val1);

    *green_out = green.val1;
    // *red_out = red.val1;
}

int main(void)
{
    const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));

    /* 2. Power on the sensor and set up nRF52 "Ears" (GPIO) */
    gpio_pin_configure(gpio_dev, TPS_EN_PIN, GPIO_OUTPUT_HIGH);
    gpio_pin_set(gpio_dev, TPS_EN_PIN, 1);

    k_msleep(200); // Wait for the sensor to power up

    ble_init();
    k_sem_init(&data_ready_sem, 0, 1); // Initialize early so it's ready

    const struct device *const dev = DEVICE_DT_GET(DT_ALIAS(max30101));

    if (!device_is_ready(dev)) {
        printk("MAX30101 not ready\n");
        return -1;
    }

    const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

    /* 1. Basic Hardware Readiness Checks */
    if (!device_is_ready(i2c_dev) || !device_is_ready(gpio_dev)) {
        return -1;
    }


    gpio_pin_configure(gpio_dev, INT_PIN, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure(gpio_dev, INT_PIN, GPIO_INT_EDGE_FALLING);
    // gpio_pin_interrupt_configure(gpio_dev, INT_PIN, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&int_cb_data, sensor_int_handler, BIT(INT_PIN));
    gpio_add_callback(gpio_dev, &int_cb_data);

    /* 4. CRITICAL: Enable Interrupts AFTER the Driver has finished resetting the chip */
    // 0x57 = I2C Addr, 0x02 = INT_EN1, 0x40 = PPG_RDY_EN (Bit 6)
    i2c_reg_write_byte(i2c_dev, 0x57, 0x02, 0x40);

    uint8_t dummy_status;
    i2c_reg_read_byte(i2c_dev, 0x57, 0x00, &dummy_status);

    int32_t green_value, red_value;
    const int sleep_ms = 10;

    while (1) {

        // k_sem_take(&data_ready_sem, K_FOREVER);

        print_sample_fetch(dev, &green_value, &red_value);

        // uint8_t dummy_status;
        // i2c_reg_read_byte(i2c_dev, 0x57, 0x00, &dummy_status);

        if (ble_is_ready()) {
            struct sensor_packet pkt;

            pkt.timestamp = (uint64_t)k_uptime_get();
            pkt.ecg = green_value;
            pkt.resp = 0; // Not used, but could be set to red_value if desired

            /* Send the 16-byte packet to the Android app */
            ble_send_sensor_data(&pkt, sizeof(pkt));
        }

        k_msleep(sleep_ms);
    }
    return 0;
}
