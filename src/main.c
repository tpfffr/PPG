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

#include "max32664c.h"
#include "max32664c_api.h"
#include "ble.h"

/* --- Hardware Pin Definitions --- */
#define MAX32664_RSTN_PIN  13  // Pin 1.13 (Reset)
#define MAX32664_MFIO_PIN  3   // Pin 0.3  (Multi-Function IO)
#define PACKET_LEN 10
#define SENSOR_RING_CAPACITY 2000
#define MAX_FETCH_PER_PASS 10
#define MAX_NOTIFY_BATCHES_PER_PASS 1
#define BACKLOG_NOTIFY_BOOST_THRESHOLD 50
#define MAX_NOTIFY_BATCHES_PER_PASS_BOOSTED 2
#define DEBUG_STATS_INTERVAL_MS 1000
#define BLE_TX_THREAD_STACK_SIZE 2048
#define BLE_TX_THREAD_PRIORITY 5

/* Structure for BLE data transmission */
struct __packed sensor_packet {
    uint64_t timestamp;
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
static bool sensor_streaming;
static K_MUTEX_DEFINE(session_lock);
static K_MUTEX_DEFINE(ring_lock);
static K_THREAD_STACK_DEFINE(ble_tx_thread_stack, BLE_TX_THREAD_STACK_SIZE);
static struct k_thread ble_tx_thread_data;

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
    int start_err = 0;

    ARG_UNUSED(conn);
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

static uint32_t sample_ring_overwritten(void)
{
    uint32_t overwritten;

    k_mutex_lock(&ring_lock, K_FOREVER);
    overwritten = sample_ring.overwritten_samples;
    k_mutex_unlock(&ring_lock);

    return overwritten;
}

static void ble_tx_thread(void *arg1, void *arg2, void *arg3)
{
    uint64_t stats_t0 = k_uptime_get();
    uint32_t ble_batches_this_sec = 0;
    uint32_t ble_samples_this_sec = 0;
    uint32_t notify_errs_this_sec = 0;
    uint32_t loop_iterations_this_sec = 0;
    uint32_t max_loop_us_this_sec = 0;
    uint32_t max_loop_gap_us_this_sec = 0;
    uint32_t max_notify_us_this_sec = 0;
    uint32_t max_batches_per_loop_this_sec = 0;
    uint32_t prev_loop_start_cyc = 0;
    bool prev_loop_start_valid = false;

    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    while (1) {
        bool did_work = false;
        size_t notify_budget = MAX_NOTIFY_BATCHES_PER_PASS;
        uint32_t loop_start_cyc = k_cycle_get_32();
        uint32_t batches_in_loop = 0;

        loop_iterations_this_sec++;
        if (prev_loop_start_valid) {
            uint32_t loop_gap_us = k_cyc_to_us_floor32(loop_start_cyc - prev_loop_start_cyc);
            if (loop_gap_us > max_loop_gap_us_this_sec) {
                max_loop_gap_us_this_sec = loop_gap_us;
            }
        }
        prev_loop_start_cyc = loop_start_cyc;
        prev_loop_start_valid = true;

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

            uint32_t notify_start_cyc = k_cycle_get_32();
            err = ble_send_sensor_data(notify_batch,
                                       batch_count * sizeof(notify_batch[0]));
            uint32_t notify_us = k_cyc_to_us_floor32(k_cycle_get_32() - notify_start_cyc);
            if (notify_us > max_notify_us_this_sec) {
                max_notify_us_this_sec = notify_us;
            }
            if (err) {
                notify_errs_this_sec++;
                printk("notify err=%d ready=%d busy=%d ring_used=%u overwritten=%u\n",
                       err, ble_is_ready(), sensor_busy_updating,
                       (unsigned int)sample_ring_used(),
                       sample_ring_overwritten());
                break;
            }

            sample_ring_consume(batch_count);
            ble_batches_this_sec++;
            ble_samples_this_sec += batch_count;
            batches_in_loop++;
            did_work = true;
        }

        if (batches_in_loop > max_batches_per_loop_this_sec) {
            max_batches_per_loop_this_sec = batches_in_loop;
        }

        {
            uint32_t loop_us = k_cyc_to_us_floor32(k_cycle_get_32() - loop_start_cyc);
            if (loop_us > max_loop_us_this_sec) {
                max_loop_us_this_sec = loop_us;
            }
        }

        if (k_uptime_get() - stats_t0 >= DEBUG_STATS_INTERVAL_MS) {
            printk("BLETX: ring_used=%u ble_ready=%d session=%ld batches=%u samples=%u notify_errs=%u loops=%u max_loop_us=%u max_gap_us=%u max_notify_us=%u max_batch_loop=%u\n",
                   (unsigned int)sample_ring_used(),
                   ble_is_ready(),
                   (long)atomic_get(&session_active),
                   ble_batches_this_sec,
                   ble_samples_this_sec,
                   notify_errs_this_sec,
                   loop_iterations_this_sec,
                   max_loop_us_this_sec,
                   max_loop_gap_us_this_sec,
                   max_notify_us_this_sec,
                   max_batches_per_loop_this_sec);

            ble_batches_this_sec = 0;
            ble_samples_this_sec = 0;
            notify_errs_this_sec = 0;
            loop_iterations_this_sec = 0;
            max_loop_us_this_sec = 0;
            max_loop_gap_us_this_sec = 0;
            max_notify_us_this_sec = 0;
            max_batches_per_loop_this_sec = 0;
            stats_t0 = k_uptime_get();
        }

        if (!did_work) {
            k_msleep(5);
        }
    }
}

static int sensor_stream_start_locked(void)
{
    int err;
    struct sensor_value off = { .val1 = 0 };
    struct sensor_value green_curr = { .val1 = 15 };

    if (sensor_streaming) {
        return 0;
    }

    gpio_pin_configure(gpio1_dev, MAX32664_RSTN_PIN, GPIO_OUTPUT_HIGH);
    gpio_pin_configure(gpio0_dev, MAX32664_MFIO_PIN, GPIO_OUTPUT_HIGH);
    k_msleep(20);

    sensor_attr_set(max32664_dev, SENSOR_CHAN_RED, SENSOR_ATTR_CONFIGURATION, &off);
    sensor_attr_set(max32664_dev, SENSOR_CHAN_IR, SENSOR_ATTR_CONFIGURATION, &off);
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
    sensor_busy_updating = true;
    atomic_set(&explicit_stop_requested, 0);
    atomic_set(&session_active, 1);

    err = sensor_stream_start_locked();
    if (err) {
        atomic_set(&session_active, 0);
    }
    sensor_busy_updating = false;
    k_mutex_unlock(&session_lock);

    return err;
}

int measurement_session_stop(bool explicit_stop)
{
    int err;

    k_mutex_lock(&session_lock, K_FOREVER);
    sensor_busy_updating = true;
    atomic_set(&explicit_stop_requested, explicit_stop ? 1 : 0);
    atomic_set(&session_active, 0);

    err = sensor_stream_stop_locked();
    sample_ring_clear();

    sensor_busy_updating = false;
    k_mutex_unlock(&session_lock);

    return err;
}

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
    k_msleep(100);
    k_thread_create(&ble_tx_thread_data, ble_tx_thread_stack,
                    K_THREAD_STACK_SIZEOF(ble_tx_thread_stack),
                    ble_tx_thread, NULL, NULL, NULL,
                    BLE_TX_THREAD_PRIORITY, 0, K_NO_WAIT);

    struct sensor_value green;
    // struct sensor_value ir;
    uint32_t battery_mv = 0;
    uint64_t now = k_uptime_get();
    uint64_t stats_t0 = k_uptime_get();

    static int low_batt_count = 0;
    uint32_t fetched_this_sec = 0;
    uint32_t pushed_this_sec = 0;
    uint32_t loop_iterations_this_sec = 0;
    uint32_t max_loop_us_this_sec = 0;
    uint32_t max_loop_gap_us_this_sec = 0;
    uint32_t max_fetched_per_loop_this_sec = 0;
    size_t ring_high_water_this_sec = 0;
    uint32_t last_overwritten_total = sample_ring_overwritten();
    uint32_t prev_loop_start_cyc = 0;
    bool prev_loop_start_valid = false;

    battery_mv = read_battery_mv() + 200;

    while (1) {
        bool did_work = false;
        uint32_t loop_start_cyc = k_cycle_get_32();
        uint32_t fetched_in_loop = 0;

        loop_iterations_this_sec++;
        if (prev_loop_start_valid) {
            uint32_t loop_gap_us = k_cyc_to_us_floor32(loop_start_cyc - prev_loop_start_cyc);
            if (loop_gap_us > max_loop_gap_us_this_sec) {
                max_loop_gap_us_this_sec = loop_gap_us;
            }
        }
        prev_loop_start_cyc = loop_start_cyc;
        prev_loop_start_valid = true;

        if (k_uptime_get() - now >= 1800000) {

            battery_mv = read_battery_mv() + 200;
            now = k_uptime_get();

            if (battery_mv > 0 && battery_mv < 3150) {
                low_batt_count++;
            } else {
                low_batt_count = 0;
            }

            printk("Battery Voltage: %u mV (low_batt_count=%d)\n", battery_mv, low_batt_count);

            // if (low_batt_count >= 3) {
            //     shutdown_everything_low_batt();
            // }
        }

        if (sensor_busy_updating) {
            uint32_t loop_us = k_cyc_to_us_floor32(k_cycle_get_32() - loop_start_cyc);
            if (loop_us > max_loop_us_this_sec) {
                max_loop_us_this_sec = loop_us;
            }
            k_msleep(10);
            continue;
        }

        for (size_t i = 0; i < MAX_FETCH_PER_PASS; i++) {
            struct sensor_packet packet;
            int err;
            size_t ring_used_now;

            if (!atomic_get(&session_active) || sensor_busy_updating) {
                break;
            }

            err = sensor_sample_fetch(max32664_dev);
            if (err) {
                break;
            }

            sensor_channel_get(max32664_dev, SENSOR_CHAN_GREEN, &green);
            packet.timestamp = k_uptime_get();
            packet.ecg = green.val1;
            packet.resp = battery_mv;
            sample_ring_push(&packet);
            fetched_this_sec++;
            pushed_this_sec++;
            fetched_in_loop++;
            ring_used_now = sample_ring_used();
            if (ring_used_now > ring_high_water_this_sec) {
                ring_high_water_this_sec = ring_used_now;
            }
            did_work = true;
        }

        if (fetched_in_loop > max_fetched_per_loop_this_sec) {
            max_fetched_per_loop_this_sec = fetched_in_loop;
        }

        {
            uint32_t loop_us = k_cyc_to_us_floor32(k_cycle_get_32() - loop_start_cyc);
            if (loop_us > max_loop_us_this_sec) {
                max_loop_us_this_sec = loop_us;
            }
        }

        if (k_uptime_get() - stats_t0 >= DEBUG_STATS_INTERVAL_MS) {
            uint32_t overwritten_total = sample_ring_overwritten();
            size_t ring_used_now = sample_ring_used();

            printk("MAIN: fetched=%u pushed=%u ring_used=%u ring_hi=%u overwritten_total=%u overwritten_delta=%u ble_ready=%d session=%ld loops=%u max_loop_us=%u max_gap_us=%u max_fetch_loop=%u\n",
                   fetched_this_sec,
                   pushed_this_sec,
                   (unsigned int)ring_used_now,
                   (unsigned int)ring_high_water_this_sec,
                   overwritten_total,
                   overwritten_total - last_overwritten_total,
                   ble_is_ready(),
                   (long)atomic_get(&session_active),
                   loop_iterations_this_sec,
                   max_loop_us_this_sec,
                   max_loop_gap_us_this_sec,
                   max_fetched_per_loop_this_sec);

            fetched_this_sec = 0;
            pushed_this_sec = 0;
            loop_iterations_this_sec = 0;
            max_loop_us_this_sec = 0;
            max_loop_gap_us_this_sec = 0;
            max_fetched_per_loop_this_sec = 0;
            ring_high_water_this_sec = ring_used_now;
            last_overwritten_total = overwritten_total;
            stats_t0 = k_uptime_get();
        }

        if (!did_work) {
            k_msleep(5);
        }
    }

    return 0;
}
