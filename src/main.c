#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/poweroff.h>

#include "max32664c.h"
#include "max32664c_api.h"
#include "ble.h"

/* --- Hardware Pin Definitions --- */
#define MAX32664_RSTN_PIN  13  // Pin 1.13 (Reset)
#define MAX32664_MFIO_PIN  3   // Pin 0.3  (Multi-Function IO)

/* Structure for BLE data transmission */
struct __packed sensor_packet {
    uint64_t timestamp;
    uint32_t ecg;   // GREEN LED Data
    uint32_t resp;  // Battery Data
};

K_SEM_DEFINE(ble_ready_sem, 0, 1);

/* Global Device Pointers */
const struct device *gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
const struct device *gpio1_dev = DEVICE_DT_GET(DT_NODELABEL(gpio1));
const struct device *max32664_dev;
static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));
const struct i2c_dt_spec max32664_i2c_spec = I2C_DT_SPEC_GET(DT_NODELABEL(max32664));

extern volatile bool sensor_busy_updating;

/******************** Function Prototypes ********************/
void max32664_hardware_reset(void);
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

/* --- Battery Logic --- */
uint32_t read_battery_mv(void) {
    if (adc_read(adc_channel.dev, &sequence) < 0) return 0;
    int32_t val_mv = adc_buf;
    adc_raw_to_millivolts_dt(&adc_channel, &val_mv);
    return (uint32_t)(val_mv * 5); // VDDH/5 scaling
}

void max32664_hardware_reset(void)
{
    gpio_pin_set(gpio1_dev, MAX32664_RSTN_PIN, 0);
    k_msleep(20);
    gpio_pin_set(gpio1_dev, MAX32664_RSTN_PIN, 1);
    k_msleep(1600);
}

// void on_ble_connect(struct bt_conn *conn, uint8_t err) {
//     if (err) return;
//     printk("\n--- Connected. Resuming current settings ---\n");

//     /* ONLY do the mandatory mode switch, don't overwrite values */
//     struct sensor_value raw_mode = { .val1 = MAX32664C_OP_MODE_RAW };
//     sensor_attr_set(max32664_dev, SENSOR_CHAN_ALL,
//                     (enum sensor_attribute)SENSOR_ATTR_MAX32664C_OP_MODE, &raw_mode);
// }

/* --- BLE Callbacks --- */
void on_ble_connect(struct bt_conn *conn, uint8_t err) {
    if (err) return;
    printk("\n--- Android Connected. Starting RAW stream ---\n");

    max32664_hardware_reset();

    struct sensor_value off = { .val1 = 0 };
    sensor_attr_set(max32664_dev, SENSOR_CHAN_RED, SENSOR_ATTR_CONFIGURATION, &off);
    sensor_attr_set(max32664_dev, SENSOR_CHAN_IR, SENSOR_ATTR_CONFIGURATION, &off);

    struct sensor_value green_curr = { .val1 = 15};
    sensor_attr_set(max32664_dev, SENSOR_CHAN_GREEN, SENSOR_ATTR_CONFIGURATION, &green_curr);
    max32664c_set_mode_raw(max32664_dev);

    struct sensor_value attr_val;
    attr_val.val1 = 3;
    sensor_attr_set(max32664_dev, SENSOR_CHAN_ALL,
                    (enum sensor_attribute) SENSOR_ATTR_MAX32664C_INTEGRATION_TIME,
                    &attr_val);

    attr_val.val1 = 0x0B; // Max ADC Range (Gain)
    sensor_attr_set(max32664_dev, SENSOR_CHAN_ALL,
    (enum sensor_attribute) SENSOR_ATTR_MAX32664C_PPG_CONFIG_1,
    &attr_val);

}

static void shutdown_everything_low_batt(void)
{
    printk("Low battery: shutting everything down\n");

    if (ble_is_ready()) {
        /* optional: disconnect first if you want */
    }

    struct sensor_value idle = { .val1 = MAX32664C_OP_MODE_IDLE };
    sensor_attr_set(max32664_dev, SENSOR_CHAN_ALL,
                    SENSOR_ATTR_MAX32664C_OP_MODE, &idle);
    k_msleep(50);

    uint8_t tx[3] = {0x01, 0x00, 0x01};
    uint8_t rx;
    max32664c_i2c_transmit(max32664_dev, tx, sizeof(tx), &rx, 1, 10);

    k_msleep(20);

    sys_poweroff();
}

// void on_ble_disconnect(struct bt_conn *conn, uint8_t reason) {
//     printk("\n--- Disconnected ---\n");
// }

void on_ble_disconnect(struct bt_conn *conn, uint8_t reason) {
    printk("\n--- Disconnected. Sensor to IDLE + HUB SHUTDOWN ---\n");

    struct sensor_value idle = { .val1 = MAX32664C_OP_MODE_IDLE };
    sensor_attr_set(max32664_dev, SENSOR_CHAN_ALL,
                    SENSOR_ATTR_MAX32664C_OP_MODE, &idle);

    k_msleep(50);

    uint8_t tx[3] = {0x01, 0x00, 0x01};
    uint8_t rx;
    int err = max32664c_i2c_transmit(max32664_dev, tx, sizeof(tx), &rx, 1, 10);
    printk("MAX32664 shutdown err=%d\n", err);
}

struct bt_conn_cb connection_callbacks = {
    .connected = on_ble_connect,
    .disconnected = on_ble_disconnect,
};

/* --- Main Application --- */
int main(void) {
    printk("\n--- Cold Boot Initialized ---\n");

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

    struct sensor_packet tx_packet;
    struct sensor_value green;
    uint32_t battery_mv = 0;
    uint64_t now = k_uptime_get();

    static int low_batt_count = 0;

    while (1) {

        if (!ble_is_ready()) {
            k_sem_take(&ble_ready_sem, K_FOREVER);
            continue;
        }

        if (sensor_busy_updating) {
            k_msleep(10);
            continue;
        }
        if (k_uptime_get() - now >= 10000) {
            battery_mv = read_battery_mv();
            now = k_uptime_get();

            if (battery_mv > 0 && battery_mv < 3550) {
                low_batt_count++;
            } else {
                low_batt_count = 0;
            }

            printk("Battery Voltage: %u mV (low_batt_count=%d)\n", battery_mv, low_batt_count);

            if (low_batt_count >= 3) {
                shutdown_everything_low_batt();
            }
        }
        while (ble_is_ready() && !sensor_busy_updating &&
           sensor_sample_fetch(max32664_dev) == 0) {
            sensor_channel_get(max32664_dev, SENSOR_CHAN_GREEN, &green);
            tx_packet.timestamp = k_uptime_get();
            tx_packet.ecg = green.val1;
            tx_packet.resp = battery_mv;
            int err = ble_send_sensor_data(&tx_packet, sizeof(tx_packet));
            if (err) {
                printk("notify err=%d ready=%d busy=%d\n",
                    err, ble_is_ready(), sensor_busy_updating);
            }
        }
        k_msleep(5);
    }
    return 0;
}
