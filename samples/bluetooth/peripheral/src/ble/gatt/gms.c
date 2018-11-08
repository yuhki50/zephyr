/**
 * @file
 * @brief Gas Meter Service
 */

#include <bluetooth/bluetooth.h>
#include <bluetooth/gatt.h>
#include <zephyr.h>

#define BT_UUID_GMS BT_UUID_DECLARE_128(0x1c, 0xa6, 0x1b, 0x16, 0x20, 0x70, 0xd9, 0xaf, 0x7c, 0x49, 0x1d, 0xc4, 0x84, 0x66, 0x04, 0x24)
#define BT_UUID_GMS_METER_VALUE BT_UUID_DECLARE_128(0x26, 0x6d, 0x94, 0xe6, 0x8e, 0xaf, 0x7d, 0x96, 0xd9, 0x49, 0x22, 0xac, 0xc9, 0xff, 0x31, 0x67)
#define BT_UUID_GMS_METER_READ_INTERVAL BT_UUID_DECLARE_128(0x79, 0xd2, 0xad, 0x55, 0xcb, 0x4d, 0xe8, 0x9b, 0x6d, 0x49, 0xbd, 0x6b, 0xfa, 0xf9, 0xe7, 0x2a)
#define BT_UUID_GMS_GAS_VALVE_CONTROL BT_UUID_DECLARE_128(0xbe, 0x22, 0xf8, 0xd1, 0x49, 0x11, 0xd8, 0x8a, 0x4d, 0x43, 0xcf, 0x8d, 0x1f, 0xa5, 0x70, 0xf5)

static struct bt_gatt_ccc_cfg meter_value_ccc_cfg[BT_GATT_CCC_MAX] = {};
static u8_t simulate_blvl;
static u32_t gas_level = 100;
static u16_t meter_read_interval = 5;   // range: 5-65535 [sec]
static u8_t valve_control = 0;          // range: 0:close 1:open

static void meter_value_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
	simulate_blvl = value == BT_GATT_CCC_NOTIFY;
	printk("meter_value_ccc_cfg_changed: %d\n", value);
}

static ssize_t read_meter_value(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, u16_t len, u16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(*value));
}

static ssize_t read_meter_read_interval(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, u16_t len, u16_t offset)
{
	printk("read_meter_read_interval\n");

	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(*value));
}

static ssize_t write_meter_read_interval(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, u16_t len, u16_t offset, u8_t flags)
{
	printk("write_meter_read_interval\n");
	u8_t *value = attr->user_data;

	if (offset + len > sizeof(meter_read_interval)) {
		printk("error\n");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	printk("ok\n");
	memcpy(value + offset, buf, len);

	return len;
}

static ssize_t write_valve_control(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, u16_t len, u16_t offset, u8_t flags)
{
	printk("write_valve_control\n");
	u8_t *value = attr->user_data;

	if (offset + len > sizeof(valve_control)) {
		printk("error\n");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	printk("ok\n");
	memcpy(value + offset, buf, len);

	return len;
}

/* Service Declaration */
static struct bt_gatt_attr attrs[] = {
	/* Service: GMS */
	BT_GATT_PRIMARY_SERVICE(BT_UUID_GMS),

	/* Characteristic: METER_VALUE */
	BT_GATT_CHARACTERISTIC(BT_UUID_GMS_METER_VALUE,
			       BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ,
			       read_meter_value, NULL, &gas_level),
	BT_GATT_CCC(meter_value_ccc_cfg, meter_value_ccc_cfg_changed),

	/* Characteristic: METER_READ_INTERVAL */
	BT_GATT_CHARACTERISTIC(BT_UUID_GMS_METER_READ_INTERVAL,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_meter_read_interval, write_meter_read_interval, &meter_read_interval),

	/* Characteristic: VALVE_CONTROL */
	BT_GATT_CHARACTERISTIC(BT_UUID_GMS_GAS_VALVE_CONTROL,
			       BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_WRITE,
			       NULL, write_valve_control, &valve_control),
};

static struct bt_gatt_attr *meter_value_chr = &attrs[1];
// static struct bt_gatt_attr *meter_read_interval_chr = &attrs[4];
// static struct bt_gatt_attr *valve_control_chr = &attrs[6];

static struct bt_gatt_service svc = BT_GATT_SERVICE(attrs);

void gms_init(void)
{
	bt_gatt_service_register(&svc);
}

void gms_notify(void)
{
	if (!simulate_blvl) {
		return;
	}

	gas_level--;
	if (!gas_level) {
		gas_level = 100;
	}

	bt_gatt_notify(NULL, meter_value_chr, &gas_level, sizeof(gas_level));
}
