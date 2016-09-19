/* spi.c - SPI based Bluetooth driver */

/*
 * Copyright (c) 2016 Linaro Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <errno.h>
#include <stddef.h>

#include <nanokernel.h>
#include <arch/cpu.h>
#include <atomic.h>
#include <sections.h>

#include <board.h>
#include <init.h>
#include <spi.h>
#include <gpio.h>
#include <misc/util.h>
#include <misc/byteorder.h>
#include <misc/stack.h>
#include <misc/nano_work.h>
#include <string.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/log.h>
#include <bluetooth/hci.h>
#include <bluetooth/driver.h>

#if !defined(CONFIG_BLUETOOTH_DEBUG_DRIVER)
#undef BT_DBG
#define BT_DBG(fmt, ...)
#endif

#if 0
static BT_STACK_NOINIT(tx_stack, 256);
static BT_STACK_NOINIT(rx_stack, 256);

static struct nano_delayed_work ack_work;
static struct nano_delayed_work retx_work;

#define HCI_3WIRE_ACK_PKT	0x00
#define HCI_COMMAND_PKT		0x01
#define HCI_ACLDATA_PKT		0x02
#define HCI_SCODATA_PKT		0x03
#define HCI_EVENT_PKT		0x04
#define HCI_3WIRE_LINK_PKT	0x0f
#define HCI_VENDOR_PKT		0xff

static bool reliable_packet(uint8_t type)
{
	switch (type) {
	case HCI_COMMAND_PKT:
	case HCI_ACLDATA_PKT:
	case HCI_EVENT_PKT:
		return true;
	default:
		return false;
	}
}

/* FIXME: Correct timeout */
#define SPI_RX_ACK_TIMEOUT	(sys_clock_ticks_per_sec / 4)
#define SPI_TX_ACK_TIMEOUT	(sys_clock_ticks_per_sec / 4)

#define SLIP_DELIMITER	0xc0
#define SLIP_ESC	0xdb
#define SLIP_ESC_DELIM	0xdc
#define SLIP_ESC_ESC	0xdd

#define SPI_RX_ESC	1
#define SPI_TX_ACK_PEND	2

#define SPI_HDR_SEQ(hdr)		((hdr)[0] & 0x07)
#define SPI_HDR_ACK(hdr)		(((hdr)[0] >> 3) & 0x07)
#define SPI_HDR_CRC(hdr)		(((hdr)[0] >> 6) & 0x01)
#define SPI_HDR_RELIABLE(hdr)	(((hdr)[0] >> 7) & 0x01)
#define SPI_HDR_PKT_TYPE(hdr)	((hdr)[1] & 0x0f)
#define SPI_HDR_LEN(hdr)		((((hdr)[1] >> 4) & 0x0f) + ((hdr)[2] << 4))

#define SPI_SET_SEQ(hdr, seq)	((hdr)[0] |= (seq))
#define SPI_SET_ACK(hdr, ack)	((hdr)[0] |= (ack) << 3)
#define SPI_SET_RELIABLE(hdr)	((hdr)[0] |= 1 << 7)
#define SPI_SET_TYPE(hdr, type)	((hdr)[1] |= type)
#define SPI_SET_LEN(hdr, len)	(((hdr)[1] |= ((len) & 0x0f) << 4), \
				 ((hdr)[2] |= (len) >> 4))

static struct spi {
	atomic_t		flags;
	struct net_buf		*rx_buf;

	struct nano_fifo	tx_queue;
	struct nano_fifo	rx_queue;
	struct nano_fifo	unack_queue;

	struct nano_sem		active_state;

	uint8_t			tx_win;
	uint8_t			tx_ack;
	uint8_t			tx_seq;

	uint8_t			rx_ack;

	enum {
		UNINIT,
		INIT,
		ACTIVE,
	}			link_state;

	enum {
		START,
		HEADER,
		PAYLOAD,
		END,
	}			rx_state;
} spi;

static uint8_t unack_queue_len;

static const uint8_t sync_req[] = { 0x01, 0x7e };
static const uint8_t sync_rsp[] = { 0x02, 0x7d };
/* Third byte may change */
static uint8_t conf_req[3] = { 0x03, 0xfc };
static const uint8_t conf_rsp[] = { 0x04, 0x7b };
static const uint8_t wakeup_req[] = { 0x05, 0xfa };
static const uint8_t woken_req[] = { 0x06, 0xf9 };
static const uint8_t sleep_req[] = { 0x07, 0x78 };

/* H5 signal buffers pool */
#define CONFIG_BLUETOOTH_MAX_SIG_LEN	3
#define CONFIG_BLUETOOTH_SIGNAL_COUNT	2
#define SIG_BUF_SIZE (CONFIG_BLUETOOTH_HCI_RECV_RESERVE + \
		      CONFIG_BLUETOOTH_MAX_SIG_LEN)
static struct nano_fifo spi_sig;
static NET_BUF_POOL(signal_pool, CONFIG_BLUETOOTH_SIGNAL_COUNT, SIG_BUF_SIZE,
		    &spi_sig, NULL, 0);

static struct device *spi_dev;

static void spi_reset_rx(void)
{
	if (spi.rx_buf) {
		net_buf_unref(spi.rx_buf);
		spi.rx_buf = NULL;
	}

	spi.rx_state = START;
}

static int spi_unslip_byte(uint8_t *byte)
{
	int count;

	if (*byte != SLIP_ESC) {
		return 0;
	}

	do {
		count = uart_fifo_read(spi_dev, byte, sizeof(*byte));
	} while (!count);

	switch (*byte) {
	case SLIP_ESC_DELIM:
		*byte = SLIP_DELIMITER;
		break;
	case SLIP_ESC_ESC:
		*byte = SLIP_ESC;
		break;
	default:
		BT_ERR("Invalid escape byte %x\n", *byte);
		return -EIO;
	}

	return 0;
}

static void process_unack(void)
{
	uint8_t next_seq = spi.tx_seq;
	uint8_t number_removed = unack_queue_len;

	if (!unack_queue_len) {
		return;
	}

	BT_DBG("rx_ack %u tx_ack %u tx_seq %u unack_queue_len %u",
	       spi.rx_ack, spi.tx_ack, spi.tx_seq, unack_queue_len);

	while (unack_queue_len > 0) {
		if (next_seq == spi.rx_ack) {
			/* Next sequence number is the same as last received
			 * ack number
			 */
			break;
		}

		number_removed--;
		/* Similar to (n - 1) % 8 with unsigned conversion */
		next_seq = (next_seq - 1) & 0x07;
	}

	if (next_seq != spi.rx_ack) {
		BT_ERR("Wrong sequence: rx_ack %u tx_seq %u next_seq %u",
		       spi.rx_ack, spi.tx_seq, next_seq);
	}

	BT_DBG("Need to remove %u packet from the queue", number_removed);

	while (number_removed) {
		struct net_buf *buf = net_buf_get_timeout(&spi.unack_queue, 0,
							  TICKS_NONE);

		if (!buf) {
			BT_ERR("Unack queue is empty");
			break;
		}

		/* TODO: print or do something with packet */
		BT_DBG("Remove buf from the unack_queue");

		net_buf_unref(buf);
		unack_queue_len--;
		number_removed--;
	}
}

static void spi_print_header(const uint8_t *hdr, const char *str)
{
	if (H5_HDR_RELIABLE(hdr)) {
		BT_DBG("%s REL: seq %u ack %u crc %u type %u len %u",
		       str, H5_HDR_SEQ(hdr), H5_HDR_ACK(hdr),
		       H5_HDR_CRC(hdr), H5_HDR_PKT_TYPE(hdr),
		       H5_HDR_LEN(hdr));
	} else {
		BT_DBG("%s UNREL: ack %u crc %u type %u len %u",
		       str, H5_HDR_ACK(hdr), H5_HDR_CRC(hdr),
		       H5_HDR_PKT_TYPE(hdr), H5_HDR_LEN(hdr));
	}
}

static uint8_t spi_slip_byte(uint8_t byte)
{
	switch (byte) {
	case SLIP_DELIMITER:
		uart_poll_out(spi_dev, SLIP_ESC);
		uart_poll_out(spi_dev, SLIP_ESC_DELIM);
		return 2;
	case SLIP_ESC:
		uart_poll_out(spi_dev, SLIP_ESC);
		uart_poll_out(spi_dev, SLIP_ESC_ESC);
		return 2;
	default:
		uart_poll_out(spi_dev, byte);
		return 1;
	}
}

static void spi_send(const uint8_t *payload, uint8_t type, int len)
{
	uint8_t hdr[4];
	int i;

	hexdump("<= ", payload, len);

	memset(hdr, 0, sizeof(hdr));

	/* Set ACK for outgoing packet and stop delayed work */
	H5_SET_ACK(hdr, spi.tx_ack);
	nano_delayed_work_cancel(&ack_work);

	if (reliable_packet(type)) {
		H5_SET_RELIABLE(hdr);
		H5_SET_SEQ(hdr, spi.tx_seq);
		spi.tx_seq = (spi.tx_seq + 1) % 8;
	}

	H5_SET_TYPE(hdr, type);
	H5_SET_LEN(hdr, len);

	/* Calculate CRC */
	hdr[3] = ~((hdr[0] + hdr[1] + hdr[2]) & 0xff);

	spi_print_header(hdr, "TX: <");

	uart_poll_out(spi_dev, SLIP_DELIMITER);

	for (i = 0; i < 4; i++) {
		spi_slip_byte(hdr[i]);
	}

	for (i = 0; i < len; i++) {
		spi_slip_byte(payload[i]);
	}

	uart_poll_out(spi_dev, SLIP_DELIMITER);
}

/* Delayed work taking care about retransmitting packets */
static void retx_timeout(struct nano_work *work)
{
	ARG_UNUSED(work);

	BT_DBG("unack_queue_len %u", unack_queue_len);

	if (unack_queue_len) {
		struct nano_fifo tmp_queue;
		struct net_buf *buf;

		nano_fifo_init(&tmp_queue);

		/* Queue to temperary queue */
		while ((buf = net_buf_get_timeout(&spi.tx_queue, 0,
						  TICKS_NONE))) {
			net_buf_put(&tmp_queue, buf);
		}

		/* Queue unack packets to the beginning of the queue */
		while ((buf = net_buf_get_timeout(&spi.unack_queue, 0,
						  TICKS_NONE))) {
			/* include also packet type */
			net_buf_push(buf, sizeof(uint8_t));
			net_buf_put(&spi.tx_queue, buf);
			spi.tx_seq = (spi.tx_seq - 1) & 0x07;
			unack_queue_len--;
		}

		/* Queue saved packets from temp queue */
		while ((buf = net_buf_get_timeout(&tmp_queue, 0, TICKS_NONE))) {
			net_buf_put(&spi.tx_queue, buf);
		}
	}
}

static void ack_timeout(struct nano_work *work)
{
	ARG_UNUSED(work);

	BT_DBG("");

	spi_send(NULL, HCI_3WIRE_ACK_PKT, 0);

	/* Analyze stacks */
	stack_analyze("tx_stack", tx_stack, sizeof(tx_stack));
	stack_analyze("rx_stack", rx_stack, sizeof(rx_stack));
}

static void spi_process_complete_packet(uint8_t *hdr)
{
	struct net_buf *buf;

	BT_DBG("");

	/* rx_ack should be in every packet */
	spi.rx_ack = SPI_HDR_ACK(hdr);

	if (reliable_packet(H5_HDR_PKT_TYPE(hdr))) {
		/* For reliable packet increment next transmit ack number */
		spi.tx_ack = (spi.tx_ack + 1) % 8;
		/* Submit delayed work to ack the packet */
		nano_delayed_work_submit(&ack_work, H5_RX_ACK_TIMEOUT);
	}

	spi_print_header(hdr, "RX: >");

	process_unack();

	buf = spi.rx_buf;
	spi.rx_buf = NULL;

	switch (H5_HDR_PKT_TYPE(hdr)) {
	case HCI_3WIRE_ACK_PKT:
		net_buf_unref(buf);
		break;
	case HCI_3WIRE_LINK_PKT:
		net_buf_put(&spi.rx_queue, buf);
		break;
	case HCI_EVENT_PKT:
	case HCI_ACLDATA_PKT:
		hexdump("=> ", buf->data, buf->len);
		bt_recv(buf);
		break;
	}
}

static void bt_spi_isr(struct device *unused)
{
	static int remaining;
	uint8_t byte;
	int ret;
	static uint8_t hdr[4];

	ARG_UNUSED(unused);

	while (uart_irq_update(spi_dev) &&
	       uart_irq_is_pending(spi_dev)) {

		if (!uart_irq_rx_ready(spi_dev)) {
			if (uart_irq_tx_ready(spi_dev)) {
				BT_DBG("transmit ready");
			} else {
				BT_DBG("spurious interrupt");
			}
			continue;
		}

		ret = uart_fifo_read(spi_dev, &byte, sizeof(byte));
		if (!ret) {
			continue;
		}

		switch (spi.rx_state) {
		case START:
			if (byte == SLIP_DELIMITER) {
				spi.rx_state = HEADER;
				remaining = sizeof(hdr);
			}
			break;
		case HEADER:
			/* In a case we confuse ending slip delimeter
			 * with starting one.
			 */
			if (byte == SLIP_DELIMITER) {
				remaining = sizeof(hdr);
				continue;
			}

			if (spi_unslip_byte(&byte) < 0) {
				spi_reset_rx();
				continue;
			}

			memcpy(&hdr[sizeof(hdr) - remaining], &byte, 1);
			remaining--;

			if (remaining) {
				break;
			}

			remaining = H5_HDR_LEN(hdr);

			switch (H5_HDR_PKT_TYPE(hdr)) {
			case HCI_EVENT_PKT:
				spi.rx_buf = bt_buf_get_evt(0x00);
				if (!spi.rx_buf) {
					BT_WARN("No available event buffers");
					spi_reset_rx();
					continue;
				}

				spi.rx_state = PAYLOAD;
				break;
			case HCI_ACLDATA_PKT:
				spi.rx_buf = bt_buf_get_acl();
				if (!spi.rx_buf) {
					BT_WARN("No available data buffers");
					spi_reset_rx();
					continue;
				}

				spi.rx_state = PAYLOAD;
				break;
			case HCI_3WIRE_LINK_PKT:
			case HCI_3WIRE_ACK_PKT:
				spi.rx_buf = net_buf_get_timeout(&spi_sig, 0,
								TICKS_NONE);
				if (!spi.rx_buf) {
					BT_WARN("No available signal buffers");
					spi_reset_rx();
					continue;
				}

				spi.rx_state = PAYLOAD;
				break;
			default:
				BT_ERR("Wrong packet type %u",
				       H5_HDR_PKT_TYPE(hdr));
				spi.rx_state = END;
				break;
			}
			break;
		case PAYLOAD:
			if (spi_unslip_byte(&byte) < 0) {
				spi_reset_rx();
				continue;
			}

			memcpy(net_buf_add(spi.rx_buf, sizeof(byte)), &byte,
			       sizeof(byte));
			remaining--;
			if (!remaining) {
				spi.rx_state = END;
			}
			break;
		case END:
			if (byte != SLIP_DELIMITER) {
				BT_ERR("Missing ending SLIP_DELIMITER");
				spi_reset_rx();
				break;
			}

			BT_DBG("Received full packet: type %u",
			       H5_HDR_PKT_TYPE(hdr));

			/* Check when full packet is received, it can be done
			 * when parsing packet header but we need to receive
			 * full packet anyway to clear UART.
			 */
			if (H5_HDR_RELIABLE(hdr) &&
			    H5_HDR_SEQ(hdr) != spi.tx_ack) {
				BT_ERR("Seq expected %u got %u. Drop packet",
				       spi.tx_ack, H5_HDR_SEQ(hdr));
				spi_reset_rx();
				break;
			}

			spi_process_complete_packet(hdr);
			spi.rx_state = START;
			break;
		}
	}
}

static uint8_t spi_get_type(struct net_buf *buf)
{
	return net_buf_pull_u8(buf);
}

static int spi_queue(struct net_buf *buf)
{
	uint8_t type;

	BT_DBG("buf %p type %u len %u", buf, bt_buf_get_type(buf), buf->len);

	switch (bt_buf_get_type(buf)) {
	case BT_BUF_CMD:
		type = HCI_COMMAND_PKT;
		break;
	case BT_BUF_ACL_OUT:
		type = HCI_ACLDATA_PKT;
		break;
	default:
		BT_ERR("Unknown packet type %u", bt_buf_get_type(buf));
		return -1;
	}

	memcpy(net_buf_push(buf, sizeof(type)), &type, sizeof(type));

	net_buf_put(&spi.tx_queue, buf);

	return 0;
}

static void tx_fiber(void)
{
	BT_DBG("");

	/* FIXME: make periodic sending */
	spi_send(sync_req, HCI_3WIRE_LINK_PKT, sizeof(sync_req));

	while (true) {
		struct net_buf *buf;
		uint8_t type;

		BT_DBG("link_state %u", spi.link_state);

		switch (spi.link_state) {
		case UNINIT:
			/* FIXME: send sync */
			fiber_sleep(10);
			break;
		case INIT:
			/* FIXME: send conf */
			fiber_sleep(10);
			break;
		case ACTIVE:
			buf = net_buf_get_timeout(&spi.tx_queue, 0,
						  TICKS_UNLIMITED);
			type = spi_get_type(buf);

			spi_send(buf->data, type, buf->len);

			/* buf is dequeued from tx_queue and queued to unack
			 * queue.
			 */
			net_buf_put(&spi.unack_queue, buf);
			unack_queue_len++;

			nano_delayed_work_submit(&retx_work, H5_TX_ACK_TIMEOUT);

			break;
		}
	}
}

static void spi_set_txwin(uint8_t *conf)
{
	conf[2] = spi.tx_win & 0x07;
}

static void rx_fiber(void)
{
	BT_DBG("");

	while (true) {
		struct net_buf *buf;

		buf = net_buf_get_timeout(&spi.rx_queue, 0, TICKS_UNLIMITED);

		hexdump("=> ", buf->data, buf->len);

		if (!memcmp(buf->data, sync_req, sizeof(sync_req))) {
			if (spi.link_state == ACTIVE) {
				/* TODO Reset H5 */
			}

			spi_send(sync_rsp, HCI_3WIRE_LINK_PKT, sizeof(sync_rsp));
		} else if (!memcmp(buf->data, sync_rsp, sizeof(sync_rsp))) {
			if (spi.link_state == ACTIVE) {
				/* TODO Reset H5 */
			}

			spi.link_state = INIT;
			spi_set_txwin(conf_req);
			spi_send(conf_req, HCI_3WIRE_LINK_PKT, sizeof(conf_req));
		} else if (!memcmp(buf->data, conf_req, 2)) {
			/*
			 * The Host sends Config Response messages without a
			 * Configuration Field.
			 */
			spi_send(conf_rsp, HCI_3WIRE_LINK_PKT, sizeof(conf_rsp));

			/* Then send Config Request with Configuration Field */
			spi_set_txwin(conf_req);
			spi_send(conf_req, HCI_3WIRE_LINK_PKT, sizeof(conf_req));
		} else if (!memcmp(buf->data, conf_rsp, 2)) {
			spi.link_state = ACTIVE;
			if (buf->len > 2) {
				/* Configuration field present */
				spi.tx_win = (buf->data[2] & 0x07);
			}

			BT_DBG("Finished H5 configuration, tx_win %u",
			       spi.tx_win);
		} else {
			BT_ERR("Not handled yet %x %x",
			       buf->data[0], buf->data[1]);
		}

		net_buf_unref(buf);

		/* Make sure we don't hog the CPU if the rx_queue never
		 * gets empty.
		 */
		fiber_yield();
	}
}

static void spi_init(void)
{
	BT_DBG("");

	spi.link_state = UNINIT;
	spi.rx_state = START;
	spi.tx_win = 4;

	/* TX fiber */
	nano_fifo_init(&spi.tx_queue);
	fiber_start(tx_stack, sizeof(tx_stack), (nano_fiber_entry_t)tx_fiber,
		    0, 0, 7, 0);

	/* RX fiber */
	net_buf_pool_init(signal_pool);

	nano_fifo_init(&spi.rx_queue);
	fiber_start(rx_stack, sizeof(rx_stack), (nano_fiber_entry_t)rx_fiber,
		    0, 0, 7, 0);

	/* Unack queue */
	nano_fifo_init(&spi.unack_queue);

	/* Init delayed work */
	nano_delayed_work_init(&ack_work, ack_timeout);
	nano_delayed_work_init(&retx_work, retx_timeout);
}
#endif

#if defined(CONFIG_BLUETOOTH_DEBUG_DRIVER)
static void hexdump(const char *str, const uint8_t *packet, size_t length)
{
	int n = 0;

	if (!length) {
		printf("%s zero-length signal packet\n", str);
		return;
	}

	while (length--) {
		if (n % 16 == 0) {
			printf("%s %08X ", str, n);
		}

		printf("%02X ", *packet++);

		n++;
		if (n % 8 == 0) {
			if (n % 16 == 0) {
				printf("\n");
			} else {
				printf(" ");
			}
		}
	}

	if (n % 16) {
		printf("\n");
	}
}
#else
#define hexdump(str, packet, length)
#endif

#define SPI_RECV_FIBER_STACK_SIZE 1024
static char __stack spi_recv_fiber_stack[SPI_RECV_FIBER_STACK_SIZE];
struct nano_sem nano_sem_req;

static struct device *spi_dev;
static struct device *gpio_dev;

/* Frame format: 0 = Motorola, 1 = TI */
#define SET_FRAME_FMT	0
/* Hard-code size to 8 bits */
#define SET_FRAME_SIZE	SPI_WORD(8)
#define OP_MODE		SOC_MASTER_MODE
#define SPI_SLAVE	0

/* TODO: Make this generic */
#if defined(CONFIG_SPI_STM32)
#include <spi/spi_stm32.h>
#define SPI_MAX_CLK_FREQ_250KHZ	128
#define SOC_MASTER_MODE		SPI_STM32_MASTER_MODE
#define SOC_SLAVE_MODE		SPI_STM32_SLAVE_MODE
#define SPI_DRV_NAME		CONFIG_SPI_0_NAME
/* TODO: Extract values from Kconfig */
#define GPIO_DRV_NAME		"GPIOB"
#define GPIO_REQ_DIR		GPIO_DIR_IN
#define GPIO_REQ_PULL		GPIO_PUD_PULL_DOWN
#define GPIO_REQ_PIN		1
#define GPIO_EDGE		(GPIO_INT_EDGE | GPIO_INT_DOUBLE_EDGE)
#endif

/* 2 bytes are used for the header size */
#define HEADER_SIZE	2

static struct gpio_callback gpio_cb;
static int prev_slave_req = 0;

struct spi_config spi_conf = {
	.config = (SET_FRAME_SIZE | OP_MODE | SPI_STM32_SLAVE_HW_SS_OUTPUT),
	.max_sys_freq = SPI_MAX_CLK_FREQ_250KHZ,
};

static void _spi_show(struct spi_config *spi_conf)
{
	BT_DBG("SPI Configuration: %x", spi_conf->config);
	BT_DBG("\tbits per word: %u",
			SPI_WORD_SIZE_GET(spi_conf->config));
	BT_DBG("\tMode: %u", SPI_MODE(spi_conf->config));
	BT_DBG("\tMax speed Hz: 0x%X", spi_conf->max_sys_freq);
#if defined(CONFIG_SPI_STM32)
	BT_DBG("\tOperating mode: %s",
			SPI_STM32_OP_MODE_GET(spi_conf->config)? "slave": "master");
#endif
}

void gpio_slave_req(struct device *gpio, struct gpio_callback *cb,
		    uint32_t pins)
{
	int slave_req;

	gpio_pin_read(gpio, GPIO_REQ_PIN, &slave_req);
	/*
	* FIXME: multiple interrupts are generated during spi transceive,
	* so for now just store and maintain the previous value
	*/
	if (slave_req != prev_slave_req) {
		prev_slave_req = slave_req;
		if (prev_slave_req == 1) {
			nano_isr_sem_give(&nano_sem_req);
		}
	}
}

static void spi_recv_fiber(void)
{
	BT_DBG("");

	uint8_t spi_tx_buf[2];
	uint8_t spi_rx_buf[255];
	uint16_t spi_buf_len;
	uint8_t bt_buf_type;
	struct net_buf *buf;
	int ret;

	while (1) {
		nano_fiber_sem_take(&nano_sem_req, TICKS_UNLIMITED);

		BT_DBG("SPI slave request to send buf");

		/* Send empty header, announce we are ready to receive data */
		BT_DBG("header - empty, announce ready to receive");
		memset(spi_tx_buf, 0, sizeof(spi_tx_buf));
		ret = spi_transceive(spi_dev, spi_tx_buf, sizeof(spi_tx_buf),
						spi_rx_buf, 2);
		if (ret < 0) {
			BT_ERR("Error in spi_transceive: %d", ret);
			continue;
		}

		/* TODO: change to use the /RDY pin */
		fiber_sleep(MSEC(150));

		/* First read is the header, which contains the buffer size */
		memset(&spi_rx_buf, 0, sizeof(spi_rx_buf));
		ret = spi_transceive(spi_dev, spi_tx_buf, sizeof(spi_tx_buf),
						spi_rx_buf, HEADER_SIZE);
		if (ret < 0) {
			BT_ERR("1Failed to read from SPI, err %d", ret);
			continue;
		}

		/* TODO: handle different buf sizes */
		memcpy(&spi_buf_len, spi_rx_buf, HEADER_SIZE);
		BT_DBG("Header: buf size %d", spi_buf_len);

		/* TODO: change to use the /RDY pin */
		fiber_sleep(MSEC(150));

		memset(&spi_rx_buf, 0, sizeof(spi_rx_buf));
		ret = spi_transceive(spi_dev, spi_tx_buf, sizeof(spi_tx_buf),
				spi_rx_buf, spi_buf_len);
		if (ret < 0) {
			BT_ERR("Failed to read from SPI, err %d", ret);
			continue;
		}

		bt_buf_type = spi_rx_buf[0];
		switch (bt_buf_type) {
		case BT_BUF_EVT:
			BT_DBG("BT rx buf type EVT");
			buf = bt_buf_get_evt(bt_buf_type);
			if (!buf) {
				BT_ERR("No available event buffers!");
				continue;
			}
			break;
		case BT_BUF_ACL_IN:
			BT_DBG("BT rx buf type ACL_IN");
			buf = bt_buf_get_acl();
			if (!buf) {
				BT_ERR("No available ACL buffers!");
				continue;
			}
			break;
		default:
			BT_ERR("UNKNOWN BT BUF TYPE %d", bt_buf_type);
			continue;
		}

		memcpy(net_buf_add(buf, spi_buf_len - 1), spi_rx_buf + 1,
						spi_buf_len - 1);
		hexdump("=>", buf->data, buf->len);
		bt_recv(buf);
		buf = NULL;
	}
}

static int spi_send(struct net_buf *buf)
{
	uint8_t spi_tx_buf[255];
	uint8_t spi_rx_buf[1];
	uint16_t spi_buf_len;
	int ret;

	BT_DBG("buf %p type %u len %u", buf, bt_buf_get_type(buf), buf->len);

	if (buf->len) {
		hexdump("<= ", buf->data, buf->len);
	}

	/* First send the header, which contains the tx buffer size */
	memset(spi_tx_buf, 0, sizeof(spi_tx_buf));
	spi_buf_len = buf->len + 1; /* extra byte for buffer type */
	memcpy(spi_tx_buf, &spi_buf_len, sizeof(spi_buf_len));
	BT_DBG("header - spi buf len: %d", spi_buf_len);
	ret = spi_transceive(spi_dev, spi_tx_buf, 2, spi_rx_buf, 1);
	if (ret < 0) {
		BT_ERR("Error in spi_transceive: %d", ret);
		return -EIO;
	}

	/* FIXME: can't go too fast, otherwise nRF51 will fail to read data.
	 * TODO: change to use the /RDY pin.
	 */
	fiber_sleep(MSEC(150));

	/* Now the data, until everything is transmited */
	/* FIXME: allow buf > spi_tx_buf by transmitting multiple times */
	memset(spi_tx_buf, 0, sizeof(spi_tx_buf));
	spi_tx_buf[0] = (uint8_t) bt_buf_get_type(buf);
	memcpy(spi_tx_buf + 1, buf->data, buf->len);
	BT_DBG("sending command buffer, len: %d", buf->len + 1);
	ret = spi_transceive(spi_dev, spi_tx_buf, buf->len + 1, spi_rx_buf, 1);
	if (ret < 0) {
		BT_ERR("Error in spi_transceive: %d", ret);
		return -EIO;
	}

	net_buf_unref(buf);

	return 0;
}

static int spi_open(void)
{
	BT_DBG("");

	spi_configure(spi_dev, &spi_conf);
	spi_slave_select(spi_dev, SPI_SLAVE);
	_spi_show(&spi_conf);

	/* GPIOs, /REQ and /RDY */
	gpio_dev = device_get_binding(GPIO_DRV_NAME);
	if (gpio_dev == NULL) {
		BT_ERR("Failed to initialize GPIO driver %s",
						GPIO_DRV_NAME);
		return -EIO;
	}
	/* TODO: Add /RDY */
	gpio_pin_configure(gpio_dev, GPIO_REQ_PIN, GPIO_REQ_DIR |
						GPIO_INT | GPIO_EDGE);
	gpio_init_callback(&gpio_cb, gpio_slave_req, BIT(GPIO_REQ_PIN));
	gpio_add_callback(gpio_dev, &gpio_cb);
	gpio_pin_enable_callback(gpio_dev, GPIO_REQ_PIN);

	nano_sem_init(&nano_sem_req);

	fiber_start(spi_recv_fiber_stack, sizeof(spi_recv_fiber_stack),
			(nano_fiber_entry_t) spi_recv_fiber, 0, 0, 7, 0);

	return 0;
}

static struct bt_driver drv = {
	.name		= "SPI_Bus",
	.bus		= BT_DRIVER_BUS_SPI,
	.open		= spi_open,
	.send		= spi_send,
};

static int _bt_spi_init(struct device *unused)
{
	ARG_UNUSED(unused);

	spi_dev = device_get_binding(CONFIG_BLUETOOTH_SPI_ON_DEV_NAME);
	if (spi_dev == NULL) {
		return -EINVAL;
	}

	bt_driver_register(&drv);

	return 0;
}

SYS_INIT(_bt_spi_init, NANOKERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
