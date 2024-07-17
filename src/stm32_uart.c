#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>

#define LED0_NODE DT_ALIAS(led0)
#define RX_BUFFER 4
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
//struct k_timer *timer;
//struct k_work *work;

struct graph_data_packet {
	uint8_t hdr_1;
	uint8_t hdr_2;
	uint8_t len;
	uint8_t pressure;
	int16_t flow;
	uint16_t volume;
	uint8_t status;
	uint8_t checksum;
};

#if 0
struct alert_data_packet {
	uint8_t hdr_1;
	uint8_t hdr_2;
	uint8_t len;
	uint8_t set_one;
	uint8_t set_two;
	uint8_t set_three;
	uint8_t set_four;
	uint16_t nc;
	uint8_t checksum;
};
#endif
 
struct data_array {
	uint8_t pressure[50];
	int16_t flow[50];
	uint16_t volume[50];
};

static uint8_t checksum(const unsigned char *buff, size_t len)
{
    uint8_t sum;
    for ( sum = 0 ; len != 0 ; len-- )
        sum += *(buff++);
    return (uint8_t)sum;
}

static int8_t send_graph_packet(const struct device *uart_dev, struct graph_data_packet *packet, struct data_array *data, uint8_t index)
{
	uint8_t *tx_data = (uint8_t *)packet;
	uint8_t pkt_len = 0;

	packet->len = (uint8_t)0x07;
	packet->pressure = data->pressure[index];
	packet->flow = data->flow[index];
	packet->volume = data->volume[index];
	packet->checksum = checksum(&(packet->pressure), packet->len - 1); 

	while(pkt_len < sizeof(struct graph_data_packet)) {
		uart_poll_out(uart_dev, *(tx_data + pkt_len));
		pkt_len++;
	}
	return 0;
}

#if 0
static void send_alert_packet(struct k_work *work)
{
	alert_packet 


}
#endif

static void uart_rx_cb(const struct device *uart_dev, void *user_data)
{
	int ret;
	char *rx_buf = (char *)user_data;
	unsigned int index = 0;

	ret = uart_irq_update(uart_dev);
	if(ret != 1) 
		return;

	ret = uart_irq_rx_ready(uart_dev);
	if(ret != 1) 
		return;

	while(index < RX_BUFFER) {
		ret = uart_fifo_read(uart_dev, (rx_buf + index), RX_BUFFER);
		index += ret;
	}
	if(rx_buf[0] == 0x01) {
		gpio_pin_toggle_dt(&led);
	}


}

int main(void)
{
	struct uart_config uart_cfg;
	const struct device *uart_dev;
	struct graph_data_packet tx_packet = {
				.hdr_1 = 0x2A,
				.hdr_2 = 0x4C,

	};

	struct data_array arry = {
		.pressure= {10,11,13,25,28,29,30,30,30,30,30,31,31,31,31,31,30,31,31,31,31,31,31,25,15,9,8,8,9,9,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10},
		.flow = {0,0,24,58,70,70,67,67,64,61,38,18,2,1,-1,-2,-45,-55,-40,-30,-20,-10,-6,4,2,4,0,3,0,0,0,0,0,0,0,-1,1,2,0,1,2,0,1,0,2,1,2,0,2,0},
		.volume = {0,1,8,62,145,210,287,308,349,386,412,432,434,434,434,434,434,434,434,434,434,434,435,428,380,280,180,60,10,8,7,6,5,4,5,5,4,5,4,5,4,5,4,5,4,5,4,5,4,5}
	};
	uint8_t index = 0, breath_type  = 0;
	volatile unsigned char rx_data[RX_BUFFER] = {0};
	int ret;

	uart_dev = DEVICE_DT_GET(DT_NODELABEL(usart2));
	uart_cfg.baudrate = (uint32_t)57600;
    uart_cfg.parity = UART_CFG_PARITY_NONE;
    uart_cfg.stop_bits = UART_CFG_STOP_BITS_1;
    uart_cfg.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
    uart_cfg.data_bits = UART_CFG_DATA_BITS_8;

    ret = uart_configure(uart_dev, &uart_cfg);
    if(ret)
            return 1;

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return 0;
	}

	ret = uart_irq_callback_user_data_set(uart_dev, uart_rx_cb, (void *)rx_data);
	if(ret)
		return 1;
	uart_irq_rx_enable(uart_dev);

	//k_work_init(work, send_alert_packet);
	//k_timer_init(timer, );


	
	while (1) {
		ret = send_graph_packet(uart_dev, &tx_packet, &arry, index);
		if(ret)
			continue;
		index = (index < 49) ? (uint8_t)index + 1  : (uint8_t)0;
		breath_type++;
		if(breath_type > 25) {
			tx_packet.status ^= 1;
			breath_type = 0;
		}
		k_sleep(K_MSEC(20));
	}


	return 0;
}

