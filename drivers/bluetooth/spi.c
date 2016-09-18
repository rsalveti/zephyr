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
#include <misc/util.h>
#include <misc/byteorder.h>
#include <misc/stack.h>
#include <misc/nano_work.h>
#include <string.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/log.h>
#include <bluetooth/hci.h>
#include <bluetooth/driver.h>

#include "util.h"

#if !defined(CONFIG_BLUETOOTH_DEBUG_DRIVER)
#undef BT_DBG
#define BT_DBG(fmt, ...)
#endif

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

static int spi_open(void)
{
	BT_DBG("");

	uart_irq_rx_disable(spi_dev);
	uart_irq_tx_disable(spi_dev);

	bt_uart_drain(spi_dev);

	uart_irq_callback_set(spi_dev, bt_spi_isr);

	spi_init();

	uart_irq_rx_enable(spi_dev);

	return 0;
}

static struct bt_driver drv = {
	.name		= "SPI_Bus",
	.bus		= BT_DRIVER_BUS_SPI,
	.open		= spi_open,
	.send		= spi_queue,
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
