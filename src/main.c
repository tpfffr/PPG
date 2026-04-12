#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/poweroff.h>
#include <hal/nrf_gpio.h>
#include <zephyr/pm/device.h>
#include <hal/nrf_power.h>
#include <nrfx.h>
#include <zephyr/pm/device_runtime.h>

#include "max32664c.h"
#include "max32664c_api.h"
#include "ble.h"

/* --- Hardware Pin Definitions --- */
#define MAX32664_RSTN_PIN  13  // Pin 1.13 (Reset)
#define MAX32664_MFIO_PIN  3   // Pin 0.3  (Multi-Function IO)
#define PACKET_LEN 10

/* Structure for BLE data transmission */
struct __packed sensor_packet {
    uint64_t timestamp;
    uint32_t ecg;   // GREEN LED Data
    uint32_t resp;  // Battery Data
};

struct sensor_packet batch[PACKET_LEN];
int count = 0;
bool i2c_suspended = false;

K_SEM_DEFINE(ble_ready_sem, 0, 1);

static int regulators_init(void)
{
    NRF_POWER->DCDCEN0 = 1;  /* REG0 -> DC/DC */
    NRF_POWER->DCDCEN  = 0;  /* REG1 -> LDO  */
    NRF_SPIM3->ENABLE = 0;
    *(volatile uint32_t *)0x4002F004 = 1;

    return 0;
}

SYS_INIT(regulators_init, PRE_KERNEL_1, 0);

/* Global Device Pointers */
const struct device *gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
const struct device *gpio1_dev = DEVICE_DT_GET(DT_NODELABEL(gpio1));
const struct device *max32664_dev;
static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));
const struct i2c_dt_spec max32664_i2c_spec = I2C_DT_SPEC_GET(DT_NODELABEL(max32664));

extern volatile bool sensor_busy_updating;
extern int max32664c_init(const struct device *dev);
extern int max32664c_pm_action(const struct device *dev, enum pm_device_action action);


/******************** Function Prototypes ********************/
uint32_t read_battery_mv(void);
void on_ble_connect(struct bt_conn *conn, uint8_t err);
void on_ble_disconnect(struct bt_conn *conn, uint8_t reason);
extern int max32664c_i2c_transmit(const struct device *dev, uint8_t *tx_buf, uint8_t tx_len,
                                  uint8_t *rx_buf, uint32_t rx_len, uint16_t delay_ms);

int16_t adc_buf;
struct adc_sequence sequence = {
    .buffer = &adc_buf,
    .buffer_size = sizeof(adc_buf),
};

uint32_t read_battery_mv(void) {
    if (adc_read(adc_channel.dev, &sequence) < 0) return 0;
    int32_t val_mv = adc_buf;
    adc_raw_to_millivolts_dt(&adc_channel, &val_mv);
    return (uint32_t)(val_mv * 5); // VDDH/5 scaling
}

void on_ble_connect(struct bt_conn *conn, uint8_t err) {

    gpio_pin_configure(gpio1_dev, MAX32664_RSTN_PIN, GPIO_OUTPUT_HIGH);
    gpio_pin_configure(gpio0_dev, MAX32664_MFIO_PIN, GPIO_OUTPUT_HIGH);

    k_msleep(20);

    err = pm_device_runtime_get(max32664_dev);
    k_msleep(20);

    // err = max32664c_init(max32664_dev);
    // printk("MAX32664 initialization err=%d\n", err);

    // k_msleep(20);

    struct sensor_value off = { .val1 = 0 };
    sensor_attr_set(max32664_dev, SENSOR_CHAN_RED, SENSOR_ATTR_CONFIGURATION, &off);
    sensor_attr_set(max32664_dev, SENSOR_CHAN_IR, SENSOR_ATTR_CONFIGURATION, &off);

    struct sensor_value green_curr = { .val1 = 15};
    sensor_attr_set(max32664_dev, SENSOR_CHAN_GREEN, SENSOR_ATTR_CONFIGURATION, &green_curr);
    max32664c_set_mode_raw(max32664_dev);
}

void on_ble_disconnect(struct bt_conn *conn, uint8_t reason)
{
    int err;

    struct sensor_value val = { .val1 = MAX32664C_OP_MODE_IDLE };
    sensor_attr_set(max32664_dev, SENSOR_CHAN_ALL, SENSOR_ATTR_MAX32664C_OP_MODE, &val);

    k_msleep(20);

    gpio_pin_set(gpio1_dev, MAX32664_RSTN_PIN, 1);
    gpio_pin_set(gpio0_dev, MAX32664_MFIO_PIN, 1);

    err = pm_device_runtime_put(max32664_dev);
    printk("Runtime PM put err=%d\n", err);

    k_msleep(20);

    // err = max32664c_shutdown_write_only(max32664_dev);
    // printk("Shutdown command err=%d\n", err);


}

struct bt_conn_cb connection_callbacks = {
    .connected = on_ble_connect,
    .disconnected = on_ble_disconnect,
};

/* --- Main Application --- */
int main(void) {

    gpio_pin_configure(gpio1_dev, MAX32664_RSTN_PIN, GPIO_OUTPUT_HIGH);
    gpio_pin_configure(gpio0_dev, MAX32664_MFIO_PIN, GPIO_OUTPUT_HIGH);

    max32664_dev = DEVICE_DT_GET(DT_NODELABEL(max32664));
    if (!device_is_ready(max32664_dev)) {
        printk("WARNING: MAX32664 driver not ready yet.\n");
    }

    if (adc_is_ready_dt(&adc_channel)) {
        adc_channel_setup_dt(&adc_channel);
        adc_sequence_init_dt(&adc_channel, &sequence);
    }

    bt_conn_cb_register(&connection_callbacks);
    ble_init();
    printk("Advertising started. Waiting for connection...\n");

    struct sensor_value green;
    struct sensor_value ir;
    uint32_t battery_mv = 0;
    uint64_t now = k_uptime_get();

    static int low_batt_count = 0;

    battery_mv = read_battery_mv();

    while (1) {

        if (k_uptime_get() - now >= 1800000) {

            battery_mv = read_battery_mv();
            now = k_uptime_get();

            if (battery_mv > 0 && battery_mv < 3550) {
                low_batt_count++;
            } else {
                low_batt_count = 0;
            }

            printk("Battery Voltage: %u mV (low_batt_count=%d)\n", battery_mv, low_batt_count);

            // if (low_batt_count >= 3) {
            //     shutdown_everything_low_batt();
            // }
        }

        if (!ble_is_ready()) {
            k_sem_take(&ble_ready_sem, K_FOREVER);
            continue;
        }

        if (sensor_busy_updating) {
            k_msleep(10);
            continue;
        }

        while (ble_is_ready() && !sensor_busy_updating &&
           sensor_sample_fetch(max32664_dev) == 0) {
                sensor_channel_get(max32664_dev, SENSOR_CHAN_GREEN, &green);
                batch[count].timestamp = k_uptime_get();
                batch[count].ecg = green.val1;
                batch[count].resp = battery_mv;
                count++;

                // printk("Fetched sample: ECG=%d RESP=%d\n", green.val1, ir.val1);

                if (count >= PACKET_LEN) {
                    int err = ble_send_sensor_data(batch, sizeof(batch));
                    if (err) {
                        printk("notify err=%d ready=%d busy=%d\n",
                            err, ble_is_ready(), sensor_busy_updating);
                    }
                    count = 0;
                }
            }
            k_msleep(5);
        }
    return 0;
}
