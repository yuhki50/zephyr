#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <settings/settings.h>
#include <zephyr.h>

#include "ble/gatt/bas.h"
#include "ble/gatt/gms.h"

static const struct bt_data bt_ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      0x0a, 0x18,                                                                                                               // DIS
		      0x0f, 0x18),                                                                                                              // BAS
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, 0xbe, 0x22, 0xf8, 0xd1, 0x49, 0x11, 0xd8, 0x8a, 0x4d, 0x43, 0xcf, 0x8d, 0x1f, 0xa5, 0x70, 0xf5),     // GMS
};

static void bt_connected(struct bt_conn *conn, u8_t err)
{
	if (err) {
		printk("Connection failed (err %u)\n", err);
		return;
	}

	printk("Connected\n");
}

static void bt_disconnected(struct bt_conn *conn, u8_t reason)
{
	printk("Disconnected (reason %u)\n", reason);
}

static struct bt_conn_cb bt_conn_callbacks = {
	.connected = bt_connected,
	.disconnected = bt_disconnected,
};

static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	bas_init();
	gms_init();

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, bt_ad, ARRAY_SIZE(bt_ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

void main(void)
{
	int err;

	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	bt_conn_cb_register(&bt_conn_callbacks);

	while (1) {
		k_sleep(MSEC_PER_SEC);

		/* Battery level simulation */
		bas_notify();

		/* Gas level simulation */
		gms_notify();
	}
}
