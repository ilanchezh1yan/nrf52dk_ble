#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#define MIN_CONNECTION_INTERVAL 16	/* 7.25ms connection interval */
#define MAX_CONNECTION_INTERVAL 16	/* 20ms connection interval */
#define SUPERVISOR_TIMEOUT 400		/* 4s connection timeout */

/* UUID for services and characteristic */
static struct bt_uuid_128 custom_Read_service_uuid =BT_UUID_INIT_128(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                                                                   0x00, 0x10, 0x00, 0x00, 0xe0, 0xff, 0x00, 0x00);

static struct bt_uuid_128 custom_Read_char_uuid = BT_UUID_INIT_128(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                                                                   0x00, 0x10, 0x00, 0x00, 0xe1, 0xff, 0x00, 0x00);

static struct bt_uuid_128 custom_a_char_uuid = BT_UUID_INIT_128(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                                                                   0x00, 0x10, 0x00, 0x00, 0xe2, 0xff, 0x00, 0x00);

static struct bt_uuid_128 custom_b_char_uuid = BT_UUID_INIT_128(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                                                                   0x00, 0x10, 0x00, 0x00, 0xe3, 0xff, 0x00, 0x00);

static uint8_t Rx_data[10];
static uint8_t Tx_data[10];
volatile static uint8_t data_packet[10];
volatile static uint8_t alert_packet[10];
volatile bool notify_data_flag;
volatile bool notify_alert_flag;

const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));

static ssize_t notify_custom_char(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                void *buf, uint16_t len, uint16_t offset)
{
    char *value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, 10);
}

void CCC_cb(const struct bt_gatt_attr *attr, uint16_t value)
{
   // bool data = value ==  BT_GATT_CCC_NOTIFY;
}

BT_GATT_SERVICE_DEFINE(custom_service,
    BT_GATT_PRIMARY_SERVICE(&custom_Read_service_uuid),
    BT_GATT_CHARACTERISTIC(&custom_Read_char_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           notify_custom_char, NULL, data_packet),
    BT_GATT_CCC(CCC_cb, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&custom_a_char_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           notify_custom_char, NULL, alert_packet),
    BT_GATT_CCC(CCC_cb, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&custom_b_char_uuid.uuid,
                            BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE,
                           NULL, NULL, Rx_data),
    BT_GATT_CCC(CCC_cb, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

void connected(struct bt_conn *conn, uint8_t err)
{
	struct bt_le_conn_param *param = BT_LE_CONN_PARAM(MIN_CONNECTION_INTERVAL, MAX_CONNECTION_INTERVAL , 0, SUPERVISOR_TIMEOUT);
	bt_conn_le_param_update(conn, param);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
};

static void uart_rx_callback(const struct device *dev, struct uart_event *evt, void *user_data)
{
	unsigned char *ATT_index = (unsigned char *)user_data;
	switch (evt->type) {
		case UART_RX_RDY:
			if(Rx_data[1] == 0x4C) {
				*ATT_index = 1;
			}
			if(Rx_data[1] == 0x4D) {
				*ATT_index = 3;
			}
			break;
		case UART_RX_DISABLED:
			uart_rx_enable(uart_dev, Rx_data, 10, 100);
			break;
		case UART_RX_BUF_REQUEST:
			uart_rx_buf_rsp(dev, Rx_data, 10);
			break;
		default:
			break;
	}
}

int main(void)
{
    int err;
    struct uart_config uart_cfg;
    unsigned char att_index = 0;

    static const struct bt_data ad[] = {
        BT_DATA(BT_DATA_FLAGS, (BT_LE_AD_GENERAL), sizeof((BT_LE_AD_GENERAL))),
    };

    static const struct bt_data sd[] = {
	    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
    };

    uart_cfg.baudrate = 57600;
    uart_cfg.parity = UART_CFG_PARITY_NONE;
    uart_cfg.stop_bits = UART_CFG_STOP_BITS_1;
    uart_cfg.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
    uart_cfg.data_bits = UART_CFG_DATA_BITS_8;

    err = uart_configure(uart_dev, &uart_cfg);
    if(err){
	    return 1;
    }

    err = uart_callback_set(uart_dev, uart_rx_callback, &att_index);
    if (err != 0) {
	    return;
    }

    err = uart_rx_enable(uart_dev, Rx_data, 10,  SYS_FOREVER_US);
    if (err != 0) {
	    return 1;
    }

    err = bt_enable(NULL);
    if (err) {
	    return 1;
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
	    return 1;
    }
    while(1) {
	    if(att_index == 1 || att_index == 3) {
		    bt_gatt_notify(NULL, &custom_service.attrs[att_index], Rx_data, sizeof(Rx_data));
		    att_index = 0;
	    }
    }

    return 0;
}
