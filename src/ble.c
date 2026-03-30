#include "ble.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/gatt.h>

#include "max32664c.h"
#include "max32664c_api.h"

/* Nordic UART Service (NUS) UUIDs */
#define NUS_SVC_UUID BT_UUID_128_ENCODE(0x6e400001, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e)
#define NUS_RX_UUID  BT_UUID_128_ENCODE(0x6e400002, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e) // Write
#define NUS_TX_UUID  BT_UUID_128_ENCODE(0x6e400003, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e) // Notify

static struct bt_uuid_128 nus_svc = BT_UUID_INIT_128(NUS_SVC_UUID);
static struct bt_uuid_128 nus_tx  = BT_UUID_INIT_128(NUS_TX_UUID);
static struct bt_uuid_128 nus_rx  = BT_UUID_INIT_128(NUS_RX_UUID);

extern const struct device *max32664_dev;
extern const struct i2c_dt_spec max32664_i2c_spec;
extern struct k_sem ble_ready_sem;

volatile bool sensor_busy_updating = false;

/* Function prototypes */
static ssize_t write_ctrl_point(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                 const void *buf, uint16_t len, uint16_t offset, uint8_t flags);

static struct bt_conn *current_conn;
static bool notify_enabled;

extern int max32664c_i2c_transmit(const struct device *dev, uint8_t *tx_buf, uint8_t tx_len,
                                  uint8_t *rx_buf, uint32_t rx_len, uint16_t delay);

static uint8_t current_ppg_sr_code  = 0x04; // 200 sps AFE => ~100 delivered/s in your setup
static uint8_t current_smp_ave_code = 0x00; // no averaging

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
	struct bt_le_ext_adv *adv;
};

static struct advertiser_info singleton_adv;

static void start_connectable_advertiser(struct k_work *work)
{
	struct advertiser_info *info = CONTAINER_OF(work, struct advertiser_info, work);

	int err = bt_le_ext_adv_start(info->adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		printk("Failed to start advertising (err %d)\n", err);
	} else {
		printk("Advertiser started successfully\n");
	}
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
    current_conn = bt_conn_ref(conn);

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

	struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(
		BT_LE_ADV_OPT_CONN,
		BT_GAP_MS_TO_ADV_INTERVAL(2000),
	    BT_GAP_MS_TO_ADV_INTERVAL(2000),
		NULL
	);

    err = bt_le_ext_adv_create(&adv_param, NULL, &singleton_adv.adv);
	if (err) {
		printk("Failed to create advertiser (err %d)\n", err);
		return 0;
	}

	/* Advertising payload (include flags + name) */
	const struct bt_data ad[] = {
		BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
		BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
			(sizeof(CONFIG_BT_DEVICE_NAME) - 1)),
	};

	err = bt_le_ext_adv_set_data(singleton_adv.adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Failed to set adv data (err %d)\n", err);
		return 0;
	}

	k_work_init(&singleton_adv.work, start_connectable_advertiser);
	k_work_submit(&singleton_adv.work);

    return 0;
}

int ble_send_sensor_data(const void *data, uint16_t len) {
    /* IMPORTANT: notify on the *characteristic value* attribute (attrs[2]) */
	if (!ble_is_ready()) {
        return -ENOTCONN;
    }
	else {
		return bt_gatt_notify(current_conn, &my_service.attrs[1], data, len);
	}
}

bool ble_is_ready(void) {
    return (current_conn != NULL && notify_enabled);
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
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
        break;
    default:
        printk("Ignoring unknown setting ID: %u value=%u\n", setting_id, value);
        return len;
    }

    sensor_busy_updating = true;
    k_msleep(20);

    printk("Received setting change request: ID=%u, Value=%u\n", setting_id, value);

    switch (setting_id) {
    case 0: { // LED current
        struct sensor_value val = { .val1 = value };
        sensor_attr_set(max32664_dev, SENSOR_CHAN_GREEN, SENSOR_ATTR_CONFIGURATION, &val);

        int err = afe_write_reg(max32664_dev, 0x23, value & 0xFF);
        printk("BLE: Set LED1 current reg 0x23 = 0x%02X err=%d\n", value & 0xFF, err);
        break;
    }

    case 1: { // ADC range / effective gain via reg 0x11[3:2]
		uint8_t r11;
		int err = afe_read_reg(max32664_dev, 0x11, &r11);
		if (err) {
			printk("BLE: failed to read reg 0x11 err=%d\n", err);
			break;
		}

		uint8_t adc_range_bits = (value & 0x03) << 2;   // bits [3:2]
		uint8_t reg_val = (r11 & 0x03) | adc_range_bits; // preserve integration-time bits [1:0]

		err = afe_write_reg(max32664_dev, 0x11, reg_val);
		printk("BLE: Set ADC range reg 0x11 = 0x%02X err=%d\n", reg_val, err);
		break;
	}

    case 2: { // integration time
        uint8_t r11;
        int err = afe_read_reg(max32664_dev, 0x11, &r11);
        if (!err) {
            uint8_t reg_val = (r11 & 0x0C) | (value & 0x03);
            err = afe_write_reg(max32664_dev, 0x11, reg_val);
            printk("BLE: Set reg 0x11 = 0x%02X err=%d\n", reg_val, err);
        } else {
            printk("BLE: failed to read reg 0x11 err=%d\n", err);
        }
        break;
    }

    case 3: { // delivered rate selector
        static const uint8_t delivered_to_afe_sr_map[] = { 0x01, 0x03, 0x04, 0x05 };

        if (value >= ARRAY_SIZE(delivered_to_afe_sr_map)) {
            printk("BLE: invalid rate idx=%u\n", value);
            break;
        }

        current_ppg_sr_code = delivered_to_afe_sr_map[value];
        uint8_t reg12 = make_reg12(current_ppg_sr_code, current_smp_ave_code);

        int err = afe_write_reg(max32664_dev, 0x12, reg12);
        printk("BLE: Set delivered-rate idx=%u -> reg 0x12 = 0x%02X err=%d\n",
               value, reg12, err);
        break;
    }

    case 4: { // averaging
        current_smp_ave_code = value & 0x07;
        uint8_t reg12 = make_reg12(current_ppg_sr_code, current_smp_ave_code);

        int err = afe_write_reg(max32664_dev, 0x12, reg12);
        printk("BLE: Set averaging code=%u -> reg 0x12 = 0x%02X err=%d\n",
               current_smp_ave_code, reg12, err);
        break;
    }
    }

    k_msleep(20);
    sensor_busy_updating = false;
    return len;
}
