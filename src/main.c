#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/devicetree.h>
// #include <hal/nrf_saadc.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/drivers/i2c.h> /* Required for I2C access */
#include <zephyr/drivers/adc.h>

#include "ble.h"

#define TPS_EN_PIN 31
#define MAX30101_SENSOR_CHANNEL SENSOR_CHAN_GREEN
#define MAX30101_REG_MODE_CFG       0x09
#define MAX30101_MODE_CFG_SHDN_MASK 0x80

struct __packed sensor_packet {
	uint64_t timestamp;
	int32_t  ecg;
	int32_t  resp;
};

int32_t err;

int16_t buf;
struct adc_sequence sequence = {
    .buffer = &buf,
    .buffer_size = sizeof(buf),
};


const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
const struct device *dev;

static const struct i2c_dt_spec sensor_i2c = I2C_DT_SPEC_GET(DT_ALIAS(max30101));
static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

static bool is_connected = false;

#if CONFIG_MAX30101_TRIGGER
static struct sensor_trigger trig_drdy;

void max30101_enter_sleep(void) {
    if (!device_is_ready(sensor_i2c.bus)) {
        printk("ERR: I2C bus not ready\n");
        return;
    }

    int ret = i2c_reg_update_byte_dt(&sensor_i2c, MAX30101_REG_MODE_CFG,
                                     MAX30101_MODE_CFG_SHDN_MASK,
                                     MAX30101_MODE_CFG_SHDN_MASK);
    if (ret == 0) {
        printk("CMD: Sensor entered Sleep Mode\n");
    } else {
        printk("ERR: Failed to sleep sensor (%d)\n", ret);
    }
}

void max30101_exit_sleep(void) {
    if (!device_is_ready(sensor_i2c.bus)) return;

    int ret = i2c_reg_update_byte_dt(&sensor_i2c, MAX30101_REG_MODE_CFG,
                                     MAX30101_MODE_CFG_SHDN_MASK,
                                     0);
    if (ret == 0) {
        printk("CMD: Sensor woke up\n");
    } else {
        printk("ERR: Failed to wake sensor (%d)\n", ret);
    }
}
void sensor_data_ready(const struct device *dev, const struct sensor_trigger *trigger)
{
    struct sensor_packet burst_buffer[10];
    int32_t val;
    uint32_t now = k_ticks_to_us_near32(k_uptime_ticks());

    err = adc_read(adc_channel.dev, &sequence);
    if (err < 0) {
        printk("ADC read error: %d\n", err);
    }
    else {
        printk("ADC value: %d\n", buf);
    }

    for (int i = 0; i < 10; i++) {

        sensor_sample_fetch(dev);
        sensor_channel_get(dev, MAX30101_SENSOR_CHANNEL, (struct sensor_value *)&val);

        burst_buffer[i].timestamp = now;
        burst_buffer[i].ecg = val;
        burst_buffer[i].resp = buf;
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

    gpio_pin_set(gpio_dev, TPS_EN_PIN, 1);
    max30101_exit_sleep();

    is_connected = true;
}

void on_ble_disconnect(struct bt_conn *conn, uint8_t reason)
{
    is_connected = false;
    max30101_enter_sleep();

    gpio_pin_set(gpio_dev, TPS_EN_PIN, 0);
}

struct bt_conn_cb connection_callbacks = {
    .connected = on_ble_connect,
    .disconnected = on_ble_disconnect,
};



int main(void)
{
    if (!adc_is_ready_dt(&adc_channel)) {
	    printk("ADC device not ready\n");
        return 0;
    }

    err = adc_channel_setup_dt(&adc_channel);
    if (err < 0) {
        printk("Failed to setup ADC channel (%d)\n", err);
        return 0;
    }

    err = adc_sequence_init_dt(&adc_channel, &sequence);
    if (err < 0) {
        printk("Failed to initialize ADC sequence (%d)\n", err);
        return 0;
    }

    gpio_pin_configure(gpio_dev, TPS_EN_PIN, GPIO_OUTPUT_LOW);
    k_msleep(100);

    bt_conn_cb_register(&connection_callbacks);
    k_msleep(100);

    ble_init();
    k_msleep(100);

    dev = DEVICE_DT_GET(DT_ALIAS(max30101));

    if (!device_is_ready(dev)) {
        printk("MAX30101 not ready\n");
        return -1;
    }

    trig_drdy.type = SENSOR_TRIG_FIFO_WATERMARK;
    trig_drdy.chan = MAX30101_SENSOR_CHANNEL;
    sensor_trigger_set(dev, &trig_drdy, sensor_data_ready);

    max30101_enter_sleep();

    while (1) {
        k_sleep(K_FOREVER);
    }
    return 0;
}
