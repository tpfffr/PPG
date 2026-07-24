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
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/time_units.h>
#include <zephyr/sys/util.h>
#include <limits.h>

#include "max32664c.h"
#include "max32664c_api.h"
#include "ble.h"
#include "event_log.h"

/* --- Hardware Pin Definitions --- */
#define MAX32664_RSTN_PIN  13  // Pin 1.13 (Reset)
#define MAX32664_MFIO_PIN  3   // Pin 0.3  (Multi-Function IO)
#define PACKET_LEN 20
#define SENSOR_RING_CAPACITY 2000
#define MAX_FETCH_PER_PASS 10
#define MAX_NOTIFY_BATCHES_PER_PASS 1
#define BACKLOG_NOTIFY_BOOST_THRESHOLD 50
#define MAX_NOTIFY_BATCHES_PER_PASS_BOOSTED 2
#define DEBUG_STATS_INTERVAL_MS 1000
#define BLE_TX_THREAD_STACK_SIZE 2048
#define BLE_TX_THREAD_PRIORITY 5
// #define AVG_WINDOW 20

// typedef struct {
//     float samples[AVG_WINDOW];
//     float sum;
//     int index;
//     int count;
// } MovingAverage;

// void moving_average_init(MovingAverage *ma) {
//     ma->sum = 0.0f;
//     ma->index = 0;
//     ma->count = 0;
//     for (int i = 0; i < AVG_WINDOW; i++) {
//         ma->samples[i] = 0.0f;
//     }
// }

// float moving_average_update(MovingAverage *ma, float sample) {
//     if (ma->count < AVG_WINDOW) {
//         ma->samples[ma->index] = sample;
//         ma->sum += sample;
//         ma->count++;
//     } else {
//         ma->sum -= ma->samples[ma->index];
//         ma->samples[ma->index] = sample;
//         ma->sum += sample;
//     }

//     ma->index = (ma->index + 1) % AVG_WINDOW;
//     return ma->sum / (float)ma->count;
// }

/* Structure for BLE data transmission */
struct __packed sensor_packet {
    uint32_t timestamp;
    uint32_t ecg;   // GREEN LED Data
    uint32_t resp;  // Battery Data
};

struct sensor_ring_buffer {
    struct sensor_packet packets[SENSOR_RING_CAPACITY];
    size_t write_idx;
    size_t read_idx;
    size_t used;
    uint32_t overwritten_samples;
};

static struct sensor_ring_buffer sample_ring;
static struct sensor_packet notify_batch[PACKET_LEN];
static atomic_t session_active = ATOMIC_INIT(0);
static atomic_t explicit_stop_requested = ATOMIC_INIT(0);
static uint64_t session_start_ms;
static bool sensor_streaming;
K_MUTEX_DEFINE(session_lock);
static K_MUTEX_DEFINE(ring_lock);
static K_THREAD_STACK_DEFINE(ble_tx_thread_stack, BLE_TX_THREAD_STACK_SIZE);
static struct k_thread ble_tx_thread_data;

// MovingAverage battery_avg;

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

extern int max32664c_init(const struct device *dev);
extern int max32664c_pm_action(const struct device *dev, enum pm_device_action action);
extern int max32664c_disable_sensors(const struct device *dev);
extern int max32664c_afe_write_reg(const struct device *dev, uint8_t reg, uint8_t val);


/******************** Function Prototypes ********************/
uint32_t read_battery_mv(void);
void on_ble_connect(struct bt_conn *conn, uint8_t err);
void on_ble_disconnect(struct bt_conn *conn, uint8_t reason);
extern int max32664c_i2c_transmit(const struct device *dev, uint8_t *tx_buf, uint8_t tx_len,
                                  uint8_t *rx_buf, uint32_t rx_len, uint16_t delay_ms);
int measurement_session_start(void);
int measurement_session_stop(bool explicit_stop);


int16_t adc_buf;
struct adc_sequence sequence = {
    .buffer = &adc_buf,
    .buffer_size = sizeof(adc_buf),
};

uint32_t read_battery_mv(void) {
    if (adc_read(adc_channel.dev, &sequence) < 0) return 0;

    int32_t val_uv = adc_buf;
    adc_raw_to_microvolts_dt(&adc_channel, &val_uv);
    return (uint32_t)((val_uv * 5 + 500) / 1000);
}

void on_ble_connect(struct bt_conn *conn, uint8_t err) {
    int start_err = 0;

    ARG_UNUSED(conn);
    event_log_add(EVENT_LOG_BLE_CONNECTED, err);
    event_log_dump();

    if (!err && !atomic_get(&session_active)) {
        start_err = measurement_session_start();
    }

    printk("Application BLE connected err=%u session_active=%ld\n",
           err, (long)atomic_get(&session_active));
    if (start_err) {
        printk("Auto-start session failed err=%d\n", start_err);
    }
}

void on_ble_disconnect(struct bt_conn *conn, uint8_t reason)
{
    event_log_add(EVENT_LOG_BLE_DISCONNECTED, reason);

    measurement_session_stop(true);

    ARG_UNUSED(conn);
    printk("Application BLE disconnected reason=%u explicit_stop=%ld session_active=%ld\n",
           reason,
           (long)atomic_get(&explicit_stop_requested),
           (long)atomic_get(&session_active));
}

struct bt_conn_cb connection_callbacks = {
    .connected = on_ble_connect,
    .disconnected = on_ble_disconnect,
};

static void sample_ring_clear(void)
{
    k_mutex_lock(&ring_lock, K_FOREVER);
    sample_ring.write_idx = 0;
    sample_ring.read_idx = 0;
    sample_ring.used = 0;
    k_mutex_unlock(&ring_lock);
}

static void sample_ring_push(const struct sensor_packet *packet)
{
    k_mutex_lock(&ring_lock, K_FOREVER);
    if (sample_ring.used == SENSOR_RING_CAPACITY) {
        sample_ring.read_idx = (sample_ring.read_idx + 1U) % SENSOR_RING_CAPACITY;
        sample_ring.used--;
        sample_ring.overwritten_samples++;
    }

    sample_ring.packets[sample_ring.write_idx] = *packet;
    sample_ring.write_idx = (sample_ring.write_idx + 1U) % SENSOR_RING_CAPACITY;
    sample_ring.used++;
    k_mutex_unlock(&ring_lock);
}

static size_t sample_ring_peek_batch(struct sensor_packet *dest, size_t max_packets)
{
    size_t to_copy;

    k_mutex_lock(&ring_lock, K_FOREVER);
    to_copy = MIN(sample_ring.used, max_packets);

    for (size_t i = 0; i < to_copy; i++) {
        size_t idx = (sample_ring.read_idx + i) % SENSOR_RING_CAPACITY;
        dest[i] = sample_ring.packets[idx];
    }

    k_mutex_unlock(&ring_lock);
    return to_copy;
}

static void sample_ring_consume(size_t count)
{
    size_t consumed;

    k_mutex_lock(&ring_lock, K_FOREVER);
    consumed = MIN(count, sample_ring.used);
    sample_ring.read_idx = (sample_ring.read_idx + consumed) % SENSOR_RING_CAPACITY;
    sample_ring.used -= consumed;
    k_mutex_unlock(&ring_lock);
}

static size_t sample_ring_used(void)
{
    size_t used;

    k_mutex_lock(&ring_lock, K_FOREVER);
    used = sample_ring.used;
    k_mutex_unlock(&ring_lock);

    return used;
}

static void ble_tx_thread(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    while (1) {
        size_t notify_budget = MAX_NOTIFY_BATCHES_PER_PASS;

        if (sample_ring_used() >= BACKLOG_NOTIFY_BOOST_THRESHOLD) {
            notify_budget = MAX_NOTIFY_BATCHES_PER_PASS_BOOSTED;
        }

        for (size_t i = 0; i < notify_budget; i++) {
            size_t batch_count = sample_ring_peek_batch(notify_batch, PACKET_LEN);
            int err;

            if (!ble_is_ready()) {
                break;
            }

            if (batch_count < PACKET_LEN) {
                break;
            }

            err = ble_send_sensor_data(notify_batch,
                                       batch_count * sizeof(notify_batch[0]));


            sample_ring_consume(batch_count);
        }

        k_msleep(5);
    }
}

static int sensor_stream_start_locked(void)
{
    int err;
    struct sensor_value red_curr = { .val1 = 0 };
    struct sensor_value ir_curr = { .val1 = 0 };
    struct sensor_value green_curr = { .val1 = 5 };

    if (sensor_streaming) {
        return 0;
    }

    gpio_pin_configure(gpio1_dev, MAX32664_RSTN_PIN, GPIO_OUTPUT_HIGH);
    gpio_pin_configure(gpio0_dev, MAX32664_MFIO_PIN, GPIO_OUTPUT_HIGH);
    k_msleep(20);

    sensor_attr_set(max32664_dev, SENSOR_CHAN_RED, SENSOR_ATTR_CONFIGURATION, &red_curr);
    sensor_attr_set(max32664_dev, SENSOR_CHAN_IR, SENSOR_ATTR_CONFIGURATION, &ir_curr);
    sensor_attr_set(max32664_dev, SENSOR_CHAN_GREEN, SENSOR_ATTR_CONFIGURATION, &green_curr);


    err = max32664c_set_mode_raw(max32664_dev);
    if (!err) {
        sensor_streaming = true;
        printk("Measurement session started\n");
    } else {
        printk("Failed to enter raw mode err=%d\n", err);
    }

    return err;
}

static int sensor_stream_stop_locked(void)
{
    int err = 0;
    struct sensor_value idle = { .val1 = MAX32664C_OP_MODE_IDLE };

    if (!sensor_streaming) {
        return 0;
    }

    err = sensor_attr_set(max32664_dev, SENSOR_CHAN_ALL,
                          SENSOR_ATTR_MAX32664C_OP_MODE, &idle);
    if (err) {
        printk("Failed to set sensor idle err=%d\n", err);
    }

    k_msleep(20);
    gpio_pin_set(gpio1_dev, MAX32664_RSTN_PIN, 1);
    gpio_pin_set(gpio0_dev, MAX32664_MFIO_PIN, 1);
    k_msleep(20);

    sensor_streaming = false;
    printk("Measurement session stopped\n");

    return err;
}

int measurement_session_start(void)
{
    int err;

    k_mutex_lock(&session_lock, K_FOREVER);
    atomic_set(&explicit_stop_requested, 0);
    atomic_set(&session_active, 1);

    max32664c_reset_sample_counter_state();

    err = sensor_stream_start_locked();
    if (err) {
        atomic_set(&session_active, 0);
    } else {
        session_start_ms = k_uptime_get();
    }
    event_log_add(EVENT_LOG_SESSION_START, err);
    k_mutex_unlock(&session_lock);

    return err;
}

int measurement_session_stop(bool explicit_stop)
{
    int err;

    k_mutex_lock(&session_lock, K_FOREVER);
    atomic_set(&explicit_stop_requested, explicit_stop ? 1 : 0);
    atomic_set(&session_active, 0);

    err = sensor_stream_stop_locked();
    sample_ring_clear();
    event_log_add(EVENT_LOG_SESSION_STOP, err);

    k_mutex_unlock(&session_lock);

    return err;
}


void shutdown_everything_low_batt(void)
{
    // event_log_add(EVENT_LOG_LOW_BATT_SHUTDOWN, 0);
    printk("Battery critically low. Shutting down everything...\n");

    int err = max32664c_disable_sensors(max32664_dev);
    printk("Disabled sensors with err=%d\n", err);

    measurement_session_stop(true);
    printk("Measurement session stopped.\n");

    err = max32664c_afe_write_reg(max32664_dev, 0x0D, 0x01);
    printk("Set AFE shutdown reg 0x0D = 0x01 err=%d\n", err);
    k_msleep(50);

    // int err = pm_device_action_run(max32664_dev, PM_DEVICE_ACTION_TURN_OFF);
    // printk("Turn off MAX32664 err=%d\n", err);
    // k_msleep(50);

    uint8_t tx[3] = {0x01, 0x00, 0x01};
    uint8_t rx;
    max32664c_i2c_transmit(max32664_dev, tx, sizeof(tx), &rx, 1, 10);
    k_msleep(50);

    // const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    // err = pm_device_action_run(i2c_dev, PM_DEVICE_ACTION_SUSPEND);
    // printk("Suspend I2C device err=%d\n", err);

    k_msleep(50);

    sys_poweroff();
}


/* --- Main Application --- */
int main(void) {
    // event_log_init();
    // event_log_add(EVENT_LOG_BOOT, (int16_t)(event_log_boot_count() & 0x7FFFu));
    // event_log_add(EVENT_LOG_MAIN_ENTER, 0);

    gpio_pin_configure(gpio1_dev, MAX32664_RSTN_PIN, GPIO_OUTPUT_HIGH);
    gpio_pin_configure(gpio0_dev, MAX32664_MFIO_PIN, GPIO_OUTPUT_HIGH);

    max32664_dev = DEVICE_DT_GET(DT_NODELABEL(max32664));
    if (!device_is_ready(max32664_dev)) {
        printk("WARNING: MAX32664 driver not ready yet.\n");
    // } else {
    //     event_log_add(EVENT_LOG_SENSOR_READY, 1);
    }

    if (adc_is_ready_dt(&adc_channel)) {
        adc_channel_setup_dt(&adc_channel);
        adc_sequence_init_dt(&adc_channel, &sequence);
    }

    bt_conn_cb_register(&connection_callbacks);
    // event_log_add(EVENT_LOG_BLE_INIT_START, 0);
    ble_init();
    // event_log_add(EVENT_LOG_BLE_INIT_DONE, 0);
    printk("Advertising started. Waiting for connection...\n");
    // event_log_add(EVENT_LOG_ADV_STARTED, 0);

    k_msleep(100);
    k_thread_create(&ble_tx_thread_data, ble_tx_thread_stack,
                    K_THREAD_STACK_SIZEOF(ble_tx_thread_stack),
                    ble_tx_thread, NULL, NULL, NULL,
                    BLE_TX_THREAD_PRIORITY, 0, K_NO_WAIT);

    struct sensor_value green;
    struct sensor_value counter;
    // static int low_batt_count = 0;
    uint32_t battery_mv = 0;

    // moving_average_init(&battery_avg);

    // uint64_t now = k_uptime_get();
    battery_mv = read_battery_mv();

    while (1) {

        // if (k_uptime_get() - now >= 120000) {

        //     // battery_mv = read_battery_mv();
        //     // event_log_add(EVENT_LOG_LOW_BATT_CHECK, (int16_t)MIN(battery_mv, (uint32_t)INT16_MAX));

        //     now = k_uptime_get();

        //     // send battery via BLE advertisement or notification here if needed

        //     // ble_send_sensor_data(&battery_packet, sizeof(battery_packet));

        //     if (battery_mv > 0 && battery_mv < 3550) {
        //         low_batt_count++;
        //     } else {
        //         low_batt_count = 0;
        //     }

        //     // For safety, if battery voltage is critically low, shut down everything immediately
        //     if (battery_mv < 3400) {
        //         shutdown_everything_low_batt();
        //     }

        //     if (low_batt_count >= 3) {
        //         shutdown_everything_low_batt();
        //     }
        // }

        for (size_t i = 0; i < MAX_FETCH_PER_PASS; i++) {
            struct sensor_packet packet;
            int err;
            // size_t ring_used_now;
            k_mutex_lock(&session_lock, K_FOREVER); // Protect the fetch operation
            if (!atomic_get(&session_active)) {
                k_mutex_unlock(&session_lock);
                break;
            }

            err = sensor_sample_fetch(max32664_dev);
            if (!err) {
                sensor_channel_get(max32664_dev, SENSOR_CHAN_GREEN, &green);
                // sensor_channel_get(max32664_dev, SENSOR_CHAN_RED, &red);
                // sensor_channel_get(max32664_dev, SENSOR_CHAN_IR, &ir);
                sensor_channel_get(max32664_dev, SENSOR_CHAN_MAX32664C_SAMPLE_COUNTER, &counter);
            }
            k_mutex_unlock(&session_lock);

            if (err)  {
                // event_log_add(EVENT_LOG_SENSOR_FETCH_ERR, err);
                break;
            }

            packet.timestamp = (uint32_t)counter.val1;
            // packet.timestamp = (uint32_t)(k_uptime_get() - session_start_ms);
            packet.ecg = green.val1;
            // packet.resp = red.val1;
            packet.resp = read_battery_mv(); //battery_mv;
            // packet.ir = ir.val1;

            sample_ring_push(&packet);

        }
        k_msleep(20);
    }

    return 0;
}
