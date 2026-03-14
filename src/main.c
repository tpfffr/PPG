#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/devicetree.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/adc.h>
#include <hal/nrf_power.h>

/* New Driver and BLE Headers */
#include "maxm86161.h"
#include "ble.h"

#define TPS_EN_PIN 31
#define CTRL_PIN   24

/* Structure for BLE data transmission */
struct __packed sensor_packet {
    uint64_t timestamp;
    uint32_t ecg;
    uint32_t resp;

};

/* Global Variables */
int16_t adc_buf;
struct adc_sequence sequence = {
    .buffer = &adc_buf,
    .buffer_size = sizeof(adc_buf),
};

const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
static const struct i2c_dt_spec ppg_sensor = I2C_DT_SPEC_GET(DT_ALIAS(maxm86161));
static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

/* BLE Connection Callbacks */
void on_ble_connect(struct bt_conn *conn, uint8_t err) {
    if (err) return;
    printk("BLE Connected. Powering on sensor...\n");

    // Enable Power via TPS pin
    gpio_pin_set(gpio_dev, TPS_EN_PIN, 1);
    k_msleep(50); // Wait for power to stabilize

    // Initialize/Wake the sensor
    if (maxm86161_init(&ppg_sensor) == 0) {
        maxm86161_start(&ppg_sensor);
        printk("MAXM86161 Started\n");
    }
}

void on_ble_disconnect(struct bt_conn *conn, uint8_t reason) {
    printk("BLE Disconnected. Sleeping sensor...\n");
    maxm86161_stop(&ppg_sensor);
    gpio_pin_set(gpio_dev, TPS_EN_PIN, 0);
}

struct bt_conn_cb connection_callbacks = {
    .connected = on_ble_connect,
    .disconnected = on_ble_disconnect,
};

/* Battery Reading Helper */
uint32_t read_battery_mv(void) {
    int err = adc_read(adc_channel.dev, &sequence);
    if (err < 0) return 0;

    int32_t val_mv = adc_buf;
    adc_raw_to_millivolts_dt(&adc_channel, &val_mv);
    return (uint32_t)((val_mv * 1050) / 300); // Your specific voltage divider math
}

int main(void) {
    // Initial Power setup
    if (!device_is_ready(gpio_dev)) return -1;
    gpio_pin_configure(gpio_dev, CTRL_PIN, GPIO_OUTPUT_HIGH);
    gpio_pin_configure(gpio_dev, TPS_EN_PIN, GPIO_OUTPUT_LOW); // Start OFF

    // Setup ADC
    if (!adc_is_ready_dt(&adc_channel)) return -1;
    adc_channel_setup_dt(&adc_channel);
    adc_sequence_init_dt(&adc_channel, &sequence);

    // Initialize BLE
    bt_conn_cb_register(&connection_callbacks);
    ble_init();

    printk("System initialized. Waiting for BLE connection...\n");

    maxm86161_sample_t raw_sample;
    struct sensor_packet tx_packet;

    while (1) {
        if (ble_is_ready()) {
            // Read from MAXM86161 FIFO
            if (maxm86161_read_fifo(&ppg_sensor, &raw_sample, 1) > 0) {

                tx_packet.timestamp = k_uptime_get();
                tx_packet.ecg = raw_sample.led1;
                tx_packet.resp = read_battery_mv();
                // tx_packet.led3 = raw_sample.led3;
                // tx_packet.batt_mv = read_battery_mv();

                ble_send_sensor_data(&tx_packet, sizeof(tx_packet));
            }
            k_msleep(10); // Adjust based on your sample rate (100Hz = 10ms)
        } else {
            k_msleep(500); // Low power polling when not connected
        }
    }
    return 0;
}
