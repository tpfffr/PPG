#include "ble.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/gatt.h>

#include "max32664c.h"
#include "max32664c_api.h"

#define CTRL_CMD_LED_CURRENT       0U
#define CTRL_CMD_ADC_RANGE         1U
#define CTRL_CMD_INTEGRATION_TIME  2U
#define CTRL_CMD_DELIVERED_RATE    3U
#define CTRL_CMD_AVERAGING         4U
#define CTRL_CMD_SESSION_START     5U
#define CTRL_CMD_SESSION_STOP      6U
#define ADV_FAST_INTERVAL_MS       200U
#define ADV_SLOW_INTERVAL_MS       1000U
#define ADV_RECENT_CONN_WINDOW_MS  (30U * 60U * 1000U)

/* Nordic UART Service (NUS) UUIDs */
#define NUS_SVC_UUID BT_UUID_128_ENCODE(0x6e400001, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e)
#define NUS_RX_UUID  BT_UUID_128_ENCODE(0x6e400002, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e) // Write
#define NUS_TX_UUID  BT_UUID_128_ENCODE(0x6e400003, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e) // Notify

static struct bt_uuid_128 nus_svc = BT_UUID_INIT_128(NUS_SVC_UUID);
static struct bt_uuid_128 nus_tx  = BT_UUID_INIT_128(NUS_TX_UUID);
static struct bt_uuid_128 nus_rx  = BT_UUID_INIT_128(NUS_RX_UUID);

static const struct bt_data adv_data[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		(sizeof(CONFIG_BT_DEVICE_NAME) - 1)),
};

extern struct k_mutex session_lock; // Gain access to main's mutex
extern const struct device *max32664_dev;
extern const struct i2c_dt_spec max32664_i2c_spec;
extern struct k_sem ble_ready_sem;
extern int measurement_session_start(void);
extern int measurement_session_stop(bool explicit_stop);

// volatile bool sensor_busy_updating = false;

/* Function prototypes */
static ssize_t write_ctrl_point(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                 const void *buf, uint16_t len, uint16_t offset, uint8_t flags);

static struct bt_conn *current_conn;
static bool notify_enabled;

extern int max32664c_i2c_transmit(const struct device *dev, uint8_t *tx_buf, uint8_t tx_len,
                                  uint8_t *rx_buf, uint32_t rx_len, uint16_t delay);

static uint8_t current_ppg_sr_code  = 0x04; // 200 sps AFE => ~100 delivered/s in your setup
static uint8_t current_smp_ave_code = 0x00; // no averaging

static struct bt_le_conn_param *conn_param = BT_LE_CONN_PARAM(20, 40, 0, 400);

static int afe_read_reg(const struct device *dev, uint8_t reg, uint8_t *val)
{
    uint8_t tx[3] = {0x41, 0x00, reg};
    uint8_t rx[2];

    int err = max32664c_i2c_transmit(dev, tx, sizeof(tx), rx, sizeof(rx), 10);
    if (err) {
        return err;
    }

    *val = rx[1];
    return 0;
}


static int afe_write_reg(const struct device *dev, uint8_t reg, uint8_t val)
{
    uint8_t tx[4] = {0x40, 0x00, reg, val};
    uint8_t rx;
    return max32664c_i2c_transmit(dev, tx, sizeof(tx), &rx, 1, 10);
}

static uint8_t make_reg12(uint8_t ppg_sr_code, uint8_t smp_ave_code)
{
    return ((ppg_sr_code & 0x1F) << 3) | (smp_ave_code & 0x07);
}

static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    printk("CCCD changed: notify %s\n", notify_enabled ? "ENABLED" : "DISABLED");

    if (notify_enabled) {
        k_sem_give(&ble_ready_sem);
    }
}



BT_GATT_SERVICE_DEFINE(my_service,
    BT_GATT_PRIMARY_SERVICE(&nus_svc),
    BT_GATT_CHARACTERISTIC(&nus_tx.uuid,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           NULL, NULL, NULL),
    BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&nus_rx.uuid,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE,
                           NULL, write_ctrl_point, NULL)
);

struct advertiser_info {
	struct k_work work;
	struct k_work_delayable interval_work;
	struct bt_le_ext_adv *adv;
	uint32_t current_interval_ms;
};

static struct advertiser_info singleton_adv;
static int64_t last_connection_timestamp_ms = -1;

static bool has_recent_connection(void)
{
	if (last_connection_timestamp_ms < 0) {
		return false;
	}

	return (k_uptime_get() - last_connection_timestamp_ms) < ADV_RECENT_CONN_WINDOW_MS;
}

static uint32_t get_target_adv_interval_ms(void)
{
	return has_recent_connection() ? ADV_FAST_INTERVAL_MS : ADV_SLOW_INTERVAL_MS;
}

static struct bt_le_adv_param make_adv_param(uint32_t interval_ms)
{
	return (struct bt_le_adv_param)BT_LE_ADV_PARAM_INIT(
		BT_LE_ADV_OPT_CONN,
		BT_GAP_MS_TO_ADV_INTERVAL(interval_ms),
		BT_GAP_MS_TO_ADV_INTERVAL(interval_ms),
		NULL
	);
}

static void schedule_interval_fallback(struct advertiser_info *info)
{
	int64_t remaining_ms;

	if (!has_recent_connection()) {
		k_work_cancel_delayable(&info->interval_work);
		return;
	}

	remaining_ms = ADV_RECENT_CONN_WINDOW_MS - (k_uptime_get() - last_connection_timestamp_ms);
	if (remaining_ms < 1) {
		remaining_ms = 1;
	}

	k_work_reschedule(&info->interval_work, K_MSEC(remaining_ms));
}

static void start_connectable_advertiser(struct k_work *work)
{
	struct advertiser_info *info = CONTAINER_OF(work, struct advertiser_info, work);
	struct bt_le_adv_param adv_param = make_adv_param(get_target_adv_interval_ms());
	uint32_t target_interval_ms = get_target_adv_interval_ms();
	int err;

	if (info->adv == NULL) {
		err = bt_le_ext_adv_create(&adv_param, NULL, &info->adv);
		if (err) {
			printk("Failed to create advertiser (err %d)\n", err);
			return;
		}

		err = bt_le_ext_adv_set_data(info->adv, adv_data, ARRAY_SIZE(adv_data), NULL, 0);
		if (err) {
			printk("Failed to set adv data (err %d)\n", err);
			return;
		}

		info->current_interval_ms = target_interval_ms;
	} else if (info->current_interval_ms != target_interval_ms) {
		err = bt_le_ext_adv_stop(info->adv);
		if (err) {
			printk("Advertiser stop before update returned %d\n", err);
		}

		err = bt_le_ext_adv_update_param(info->adv, &adv_param);
		if (err) {
			printk("Failed to update advertising interval to %u ms (err %d)\n",
			       target_interval_ms, err);
			return;
		}

		info->current_interval_ms = target_interval_ms;
	}

	err = bt_le_ext_adv_start(info->adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		printk("Failed to start advertising (err %d)\n", err);
	} else {
		printk("Advertiser started successfully (%u ms interval)\n", target_interval_ms);
		schedule_interval_fallback(info);
	}
}

static void advertiser_interval_work_handler(struct k_work *work)
{
	struct advertiser_info *info =
		CONTAINER_OF(k_work_delayable_from_work(work), struct advertiser_info, interval_work);

	k_work_submit(&info->work);
}

static void exchange_func(struct bt_conn *conn, uint8_t err,
			  struct bt_gatt_exchange_params *params)
{
	if (!err) {
		printk("MTU exchange successful! Current MTU: %d\n",
                        bt_gatt_get_mtu(conn));
	} else {
		printk("MTU exchange failed (err %d)\n", err);
	}
}

static struct bt_gatt_exchange_params exchange_params = {
	.func = exchange_func
};

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("Connection failed (0x%02x)\n", err);
        return;
    }


    printk("Connected to client!\n");
    last_connection_timestamp_ms = k_uptime_get();
    k_work_cancel_delayable(&singleton_adv.interval_work);
    current_conn = bt_conn_ref(conn);

	int param_err = bt_conn_le_param_update(conn, conn_param);
    if (param_err) {
        printk("Connection interval update failed (err %d)\n", param_err);
    } else {
        printk("Connection interval update requested\n");
    }

    /* 1. Print the starting MTU (typically 23) */
    printk("Initial MTU: %d\n", bt_gatt_get_mtu(conn));

    /* 2. Start the exchange to expand the 'pipe' to 247 */
    /* This uses the exchange_params variable you already defined */
    int mtu_err = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (mtu_err) {
        printk("MTU exchange failed to start (err %d)\n", mtu_err);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (reason %u). Restarting...\n", reason);

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    notify_enabled = false;
    k_work_submit(&singleton_adv.work);
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

int ble_init(void)
{
    int err;

	printk("Starting Bluetooth Peripheral\n");

	err = bt_enable(NULL);
	if (err) {
		printk("BT init failed (err %d)\n", err);
		return 0;
	}

	bt_conn_cb_register(&conn_callbacks);
	printk("Bluetooth initialized\n");
	k_work_init(&singleton_adv.work, start_connectable_advertiser);
	k_work_init_delayable(&singleton_adv.interval_work, advertiser_interval_work_handler);
	k_work_submit(&singleton_adv.work);

    return 0;
}

int ble_send_sensor_data(const void *data, uint16_t len) {
    /* IMPORTANT: notify on the *characteristic value* attribute (attrs[2]) */
	if (!ble_is_ready()) {
        return -ENOTCONN;
    }
	else {
		return bt_gatt_notify(current_conn, &my_service.attrs[2], data, len);
	}
}

bool ble_is_ready(void) {
    return (current_conn != NULL && notify_enabled);
}

int ble_disconnect_current(void)
{
    if (current_conn == NULL) {
        return -ENOTCONN;
    }

    return bt_conn_disconnect(current_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static ssize_t write_ctrl_point(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    const uint8_t *data = buf;

    if (len < 2) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    uint8_t setting_id = data[0];
    uint8_t value      = data[1];

    switch (setting_id) {
    case CTRL_CMD_LED_CURRENT:
    case CTRL_CMD_ADC_RANGE:
    case CTRL_CMD_INTEGRATION_TIME:
    case CTRL_CMD_DELIVERED_RATE:
    case CTRL_CMD_AVERAGING:
    case CTRL_CMD_SESSION_START:
    case CTRL_CMD_SESSION_STOP:
        break;
    default:
        printk("Ignoring unknown setting ID: %u value=%u\n", setting_id, value);
        return len;
    }

    if (setting_id == CTRL_CMD_SESSION_START) {
        int err = measurement_session_start();
        printk("BLE: start session err=%d\n", err);
        return len;
    }

    if (setting_id == CTRL_CMD_SESSION_STOP) {
        int err = measurement_session_stop(true);
        printk("BLE: stop session err=%d\n", err);

        err = ble_disconnect_current();
        if (err && err != -ENOTCONN) {
            printk("BLE: disconnect after stop err=%d\n", err);
        }
        return len;
    }

    // sensor_busy_updating = true;
    k_msleep(50);

    printk("Received setting change request: ID=%u, Value=%u\n", setting_id, value);

    switch (setting_id) {
    case CTRL_CMD_LED_CURRENT: { // LED current

        k_mutex_lock(&session_lock, K_FOREVER); // Lock out the main thread loop

        struct sensor_value val = { .val1 = value };
        sensor_attr_set(max32664_dev, SENSOR_CHAN_GREEN, SENSOR_ATTR_CONFIGURATION, &val);

        int err = afe_write_reg(max32664_dev, 0x23, value & 0xFF);
        printk("BLE: Set LED1 current reg 0x23 = 0x%02X err=%d\n", value & 0xFF, err);
        k_mutex_unlock(&session_lock);
        break;
    }

    case CTRL_CMD_ADC_RANGE: { // ADC range / effective gain via reg 0x11[3:2]
        k_mutex_lock(&session_lock, K_FOREVER); // Lock out the main thread loop
		uint8_t r11;
		int err = afe_read_reg(max32664_dev, 0x11, &r11);
		if (err) {
			printk("BLE: failed to read reg 0x11 err=%d\n", err);
			k_mutex_unlock(&session_lock);
			break;
		}

		uint8_t adc_range_bits = (value & 0x03) << 2;   // bits [3:2]
		uint8_t reg_val = (r11 & 0x03) | adc_range_bits; // preserve integration-time bits [1:0]

		err = afe_write_reg(max32664_dev, 0x11, reg_val);
		printk("BLE: Set ADC range reg 0x11 = 0x%02X err=%d\n", reg_val, err);
		k_mutex_unlock(&session_lock);
		break;
	}

    case CTRL_CMD_INTEGRATION_TIME: { // integration time
        k_mutex_lock(&session_lock, K_FOREVER); // Lock out the main thread loop
        uint8_t r11;
        int err = afe_read_reg(max32664_dev, 0x11, &r11);
        if (!err) {
            uint8_t reg_val = (r11 & 0x0C) | (value & 0x03);
            err = afe_write_reg(max32664_dev, 0x11, reg_val);
            printk("BLE: Set reg 0x11 = 0x%02X err=%d\n", reg_val, err);
            k_mutex_unlock(&session_lock);
        } else {
            printk("BLE: failed to read reg 0x11 err=%d\n", err);
            k_mutex_unlock(&session_lock);
        }
        break;
    }

	case CTRL_CMD_DELIVERED_RATE: { // delivered rate selector
        k_mutex_lock(&session_lock, K_FOREVER); // Lock out the main thread loop
		static const uint8_t delivered_to_afe_sr_map[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05};

		if (value >= ARRAY_SIZE(delivered_to_afe_sr_map)) {
			printk("BLE: invalid rate idx=%u\n", value);
			k_mutex_unlock(&session_lock);
			break;
		}

		current_ppg_sr_code = delivered_to_afe_sr_map[value];
		uint8_t reg12 = make_reg12(current_ppg_sr_code, current_smp_ave_code);

		// int err = afe_write_reg(max32664_dev, 0x12, 0x20);

		int err = afe_write_reg(max32664_dev, 0x12, reg12);
		printk("BLE: Set delivered-rate idx=%u -> reg 0x12 = 0x%02X err=%d\n",
				value, reg12, err);
		k_mutex_unlock(&session_lock);
		break;
	}

    case CTRL_CMD_AVERAGING: { // averaging
        k_mutex_lock(&session_lock, K_FOREVER); // Lock out the main thread loop
        current_smp_ave_code = value & 0x07;
        uint8_t reg12 = make_reg12(current_ppg_sr_code, current_smp_ave_code);

        int err = afe_write_reg(max32664_dev, 0x12, reg12);
        printk("BLE: Set averaging code=%u -> reg 0x12 = 0x%02X err=%d\n",
               current_smp_ave_code, reg12, err);
        k_mutex_unlock(&session_lock);
        break;
    }
    }

    k_msleep(50);
    // sensor_busy_updating = false;
    return len;
}
