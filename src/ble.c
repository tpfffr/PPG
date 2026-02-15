#include "ble.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/gatt.h>

/* Nordic UART Service (NUS) UUIDs */
#define NUS_SVC_UUID BT_UUID_128_ENCODE(0x6e400001, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e)
#define NUS_RX_UUID  BT_UUID_128_ENCODE(0x6e400002, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e) // Write
#define NUS_TX_UUID  BT_UUID_128_ENCODE(0x6e400003, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e) // Notify

static struct bt_uuid_128 nus_svc = BT_UUID_INIT_128(NUS_SVC_UUID);
static struct bt_uuid_128 nus_tx  = BT_UUID_INIT_128(NUS_TX_UUID);
static struct bt_uuid_128 nus_rx  = BT_UUID_INIT_128(NUS_RX_UUID);

static struct bt_conn *current_conn;
static bool notify_enabled;

static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	notify_enabled = (value == BT_GATT_CCC_NOTIFY);
	printk("CCCD changed: notify %s\n", notify_enabled ? "ENABLED" : "DISABLED");
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
                           NULL, NULL, NULL) // No callback = data is ignored
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
        printk("Connection failed (0x02x)\n", err);
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

	/* Create connectable extended advertiser */
	struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(
		BT_LE_ADV_OPT_CONN,
		BT_GAP_ADV_SLOW_INT_MIN,
		BT_GAP_ADV_SLOW_INT_MAX,
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
