#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

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
volatile bool notify_flag;

const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));

static ssize_t notify_custom_char(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                void *buf, uint16_t len, uint16_t offset)
{
    char *value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(Tx_data));
}

void CCC_cb(const struct bt_gatt_attr *attr, uint16_t value)
{
    bool data = value ==  BT_GATT_CCC_NOTIFY;
}


BT_GATT_SERVICE_DEFINE(custom_service,
    BT_GATT_PRIMARY_SERVICE(&custom_Read_service_uuid),
    BT_GATT_CHARACTERISTIC(&custom_Read_char_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           notify_custom_char, NULL, Rx_data),
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

void uart_rx_cb(const struct device *uart_dev, void *user_data)
{
    int ret;
    unsigned int index = 0;

    ret = uart_irq_update(uart_dev);
    if(ret != 1) 
        return;

    ret = uart_irq_rx_ready(uart_dev);
    if(ret != 1) 
        return;

    while(index < 10) {
        ret = uart_fifo_read(uart_dev, Tx_data + index, sizeof(Tx_data));
        index += ret;
    }
    uart_irq_rx_disable(uart_dev);
}

void notify_timer_handler(struct k_timer *dummy)
{
    notify_flag = true;
}

void connected(struct bt_conn *conn, uint8_t err)
{
     struct bt_le_conn_param *param = BT_LE_CONN_PARAM(16, 16, 0, 400);
     bt_conn_le_param_update(conn, param);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
};


int main(void)
{
    int err;
    struct uart_config uart_cfg;
    struct k_timer notify_timer;

    static const struct bt_data ad[] = {
        BT_DATA(BT_DATA_FLAGS, (BT_LE_AD_GENERAL), sizeof((BT_LE_AD_GENERAL))),
    };

    static const struct bt_data sd[] = {
        BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
    };

    uart_cfg.baudrate = 57600;
    uart_cfg.parity = UART_CFG_PARITY_NONE;
    uart_cfg.stop_bits = UART_CFG_STOP_BITS_1;
    uart_cfg.flow_ctrl = UART_CFG_FLOW_CTRL_RTS_CTS;
    uart_cfg.data_bits = UART_CFG_DATA_BITS_8;

    err = uart_configure(uart_dev, &uart_cfg);
    if(err){
        return 1;
    }

    err = uart_irq_callback_user_data_set(uart_dev, uart_rx_cb, NULL);
    if(err)
        return 1;

    err = bt_enable(NULL);
    if (err) {
        return 1;
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        return 1;
    } 
    uart_irq_rx_enable(uart_dev);

    k_timer_init(&notify_timer, notify_timer_handler, NULL);
    k_timer_start(&notify_timer, K_MSEC(20), K_MSEC(20));

    while(1) {
        if (notify_flag) {
            notify_flag = false;
            bt_gatt_notify(NULL, &custom_service.attrs[1], Tx_data, sizeof(Tx_data));
	    uart_irq_rx_enable(uart_dev);

        }
    }
    return 0;
}
