#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#if 0
static struct bt_uuid_128 custom_write_service_uuid = BT_UUID_INIT_128(
    0x12, 0x34, 0x56, 0x78, 0x90, 0xAB, 0xCD, 0xEF,
    0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF
    );

static struct bt_uuid_128 custom_write_char_uuid = BT_UUID_INIT_128(
    0x12, 0x34, 0x56, 0x78, 0x90, 0xAB, 0xCD, 0xEF,
    0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF
    );
#endif

#define BT_LE_ADV_CONN_ONE_TIME BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_ONE_TIME,                        \
                        BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL)

#if 1
static struct bt_uuid_128 custom_Read_service_uuid =BT_UUID_INIT_128(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                                                                   0x00, 0x10, 0x00, 0x00, 0xe0, 0xff, 0x00, 0x00);

static struct bt_uuid_128 custom_Read_char_uuid = BT_UUID_INIT_128(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                                                                   0x00, 0x10, 0x00, 0x00, 0xe1, 0xff, 0x00, 0x00);

static struct bt_uuid_128 custom_a_char_uuid = BT_UUID_INIT_128(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                                                                   0x00, 0x10, 0x00, 0x00, 0xe2, 0xff, 0x00, 0x00);

static struct bt_uuid_128 custom_b_char_uuid = BT_UUID_INIT_128(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                                                                   0x00, 0x10, 0x00, 0x00, 0xe3, 0xff, 0x00, 0x00);
#endif

static uint8_t Rx_data[10];
static uint8_t Tx_data[10];

const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));
struct k_timer *timer;


static ssize_t notify_custom_char(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                void *buf, uint16_t len, uint16_t offset)
{
    char *value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(Tx_data));
}


static ssize_t write_custom_value(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                  const void *buf, uint16_t len, uint16_t offset,
                                  uint8_t flags)
{
    uint8_t *value = attr->user_data;
    uint8_t index = 0;

    if (offset + len > 10) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    memcpy(value + offset, buf, len);
    bt_gatt_notify(NULL, attr, value, sizeof(Rx_data));
    while(index < 10) {
        uart_poll_out(uart_dev, *(value + index));
        index++;
    }
    return len;
}

void CCC_cb(const struct bt_gatt_attr *attr, uint16_t value)
{
    bool data = value ==  BT_GATT_CCC_NOTIFY;
}

#if 1
BT_GATT_SERVICE_DEFINE(custom_service,
    BT_GATT_PRIMARY_SERVICE(&custom_Read_service_uuid),
    BT_GATT_CHARACTERISTIC(&custom_Read_char_uuid.uuid,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_READ,
                           notify_custom_char, NULL, &Tx_data),
    BT_GATT_CCC(CCC_cb, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&custom_a_char_uuid.uuid,
                            BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE,
                           NULL, NULL, Rx_data),
    BT_GATT_CCC(CCC_cb, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&custom_b_char_uuid.uuid,
                            BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE,
                           NULL, NULL, Rx_data),
    BT_GATT_CCC(CCC_cb, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);
#endif

void Receive_data(void)
{
    int ret = 0;
    unsigned int index = 0;

    while(index < 10) {
        ret = uart_poll_in(uart_dev, (Tx_data + index));
        if(!ret && *Tx_data == 0x2A) {
            index++;
        }
    }
    bt_gatt_notify(NULL, &custom_service.attrs[1], Tx_data, sizeof(Tx_data));
}

int main(void)
{
    int err;
    unsigned int index = 0, breath_type = 0;

    static const struct bt_data ad[] = {
	BT_DATA(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR), sizeof((BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR))),
    };

    static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
    };
    
    err = bt_enable(NULL);
    if (err) {
        return 1;
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN_ONE_TIME, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        return 1;
    }
    
    while (1) {
        Receive_data();
        k_sleep(K_MSEC(20));

    }
    return 0;
}
