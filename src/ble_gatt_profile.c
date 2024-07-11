#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#define RX_BUF_SIZE 4

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

#if 1
static struct bt_uuid_16 custom_Read_service_uuid = BT_UUID_INIT_16(0x1200);

static struct bt_uuid_16 custom_Read_char_uuid = BT_UUID_INIT_16(0x1210);

static struct bt_uuid_16 custom_write_char_uuid = BT_UUID_INIT_16(0x1220);
#endif

static uint8_t Rx_data[RX_BUF_SIZE];
static uint8_t Tx_data[10];

const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));
struct k_timer *timer;

#if 1
static ssize_t notify_custom_char(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                void *buf, uint16_t len, uint16_t offset)
{
    uint8_t index = 0;
    char *value = attr->user_data;
    for(; index < 10 ; index++)
    {
        uart_poll_in(uart_dev, (value + index));
    }
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(Tx_data));
}
#endif

#if 1
static ssize_t write_custom_value(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                  const void *buf, uint16_t len, uint16_t offset,
                                  uint8_t flags)
{
    uint8_t *value = attr->user_data;
    uint8_t index = 0;

    if (offset + len > sizeof(10)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    memcpy(value + offset, buf, len);
    bt_gatt_notify(NULL, attr, value, sizeof(Rx_data));
    while(index < RX_BUF_SIZE) {
        uart_poll_out(uart_dev, *(value + index));
        index++;
    }
    return len;
}
#endif

#if 0
BT_GATT_SERVICE_DEFINE(Data_write,
    BT_GATT_PRIMARY_SERVICE(&custom_write_service_uuid),
    BT_GATT_CHARACTERISTIC(&custom_write_char_uuid.uuid,
                           BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE,
                           NULL, write_custom_value, Rx_data),
);
#endif
void CCC_cb(const struct bt_gatt_attr *attr, uint16_t value)
{
    bt_gatt_attr_read(NULL, attr, &Tx_data[0], 1, 0, 3, 1);
}

#if 1
BT_GATT_SERVICE_DEFINE(custom_service,
    BT_GATT_PRIMARY_SERVICE(&custom_Read_service_uuid),
    BT_GATT_CHARACTERISTIC(&custom_Read_char_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           notify_custom_char, NULL, Tx_data),
    BT_GATT_CCC(CCC_cb, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&custom_write_char_uuid.uuid,
                           BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE,
                           NULL, write_custom_value, Rx_data)
);
#endif

int configure_uart(void)
{
    int ret;
	struct uart_config uart_cfg;

        uart_cfg.baudrate = 115200;
        uart_cfg.parity = UART_CFG_PARITY_NONE;
        uart_cfg.stop_bits = UART_CFG_STOP_BITS_1;
        uart_cfg.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
        uart_cfg.data_bits = UART_CFG_DATA_BITS_8;

        ret = uart_configure(uart_dev, &uart_cfg);
        if(ret){
            return 1;
        }
        return 0;

}

void timer_exp_cb(struct k_timer *timer)
{
    notify_custom_char(NULL, attr_custom_service, Tx_data, 10, 0);
}

int main(void)
{
    int err;
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

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        return 1;
    }
#if 0 /* configuration for uart irq*/
    err = uart_irq_callback_set(uart_dev, uart_rx_cb);
	if(err)
		return 1;

	uart_irq_rx_enable(uart_dev);
#endif

#if 0/* configuration for timer */
    k_timer_init(timer, timer_exp_cb, NULL);
    k_timer_start(timer, K_MSEC(20), K_MSEC(20));
#endif
    
    while (1) {

    }
    return 0;
}
