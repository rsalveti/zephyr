/*
 * Copyright (c) 2017 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>
#include <stdio.h>

#include <zephyr.h>
#include <misc/byteorder.h>
#include <logging/sys_log.h>
#include <misc/stack.h>

#include <device.h>
#include <init.h>
#include <gpio.h>
#include <spi.h>

#include <net/buf.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/l2cap.h>
#include <bluetooth/hci.h>
#include <bluetooth/buf.h>
#include <bluetooth/hci_raw.h>
#include <bluetooth/common/log.h>

#define HCI_CMD			0x01
#define HCI_ACL			0x02
#define HCI_SCO			0x03
#define HCI_EVT			0x04

/* Special Values */
#define SPI_WRITE		0x0A
#define SPI_READ		0x0B
#define READY_NOW		0x02
#define SANITY_CHECK		0x02

/* Offsets */
#define STATUS_HEADER_READY	0
#define STATUS_HEADER_TOREAD	3

#define PACKET_TYPE		0
#define EVT_BLUE_INITIALIZED    0x01

#define GPIO_IRQ_DEV_NAME	CONFIG_BLUETOOTH_SPI_TO_HOST_IRQ_DEV_NAME
#define GPIO_IRQ_PIN		CONFIG_BLUETOOTH_SPI_TO_HOST_IRQ_PIN

/* Needs to be aligned with the SPI master buffer size */
#define SPI_MAX_MSG_LEN		255

static u8_t rxmsg[SPI_MAX_MSG_LEN];
static u8_t txmsg[SPI_MAX_MSG_LEN];

/* HCI buffer pools */
#define CMD_BUF_SIZE BT_BUF_RX_SIZE

NET_BUF_POOL_DEFINE(cmd_tx_pool, CONFIG_BLUETOOTH_HCI_CMD_COUNT, CMD_BUF_SIZE,
		    BT_BUF_USER_DATA_MIN, NULL);

#define BT_L2CAP_MTU 65 /* 64-byte public key + opcode */
/* Data size needed for ACL buffers */
#define BT_BUF_ACL_SIZE BT_L2CAP_BUF_SIZE(BT_L2CAP_MTU)

#if defined(CONFIG_BLUETOOTH_CONTROLLER_TX_BUFFERS)
#define TX_BUF_COUNT CONFIG_BLUETOOTH_CONTROLLER_TX_BUFFERS
#else
#define TX_BUF_COUNT 6
#endif

NET_BUF_POOL_DEFINE(acl_tx_pool, TX_BUF_COUNT, BT_BUF_ACL_SIZE,
		    BT_BUF_USER_DATA_MIN, NULL);

static struct device *spi_hci_dev;
static struct device *gpio_dev;
static BT_STACK_NOINIT(bt_tx_thread_stack, CONFIG_BLUETOOTH_HCI_TX_STACK_SIZE);
static struct k_thread bt_tx_thread_data;

static K_SEM_DEFINE(sem_spi_rx, 0, 1);
static K_SEM_DEFINE(sem_spi_tx, 0, 1);

static inline int spi_send(struct net_buf *buf)
{
	u8_t header_master[5] = { 0 };
	u8_t header_slave[5] = { READY_NOW, SANITY_CHECK,
				 0x00, 0x00, 0x00 };
	u8_t ret;

	SYS_LOG_DBG("buf %p type %u len %u", buf, bt_buf_get_type(buf),
		    buf->len);

	switch (bt_buf_get_type(buf)) {
	case BT_BUF_ACL_IN:
		net_buf_push_u8(buf, HCI_ACL);
		break;
	case BT_BUF_EVT:
		net_buf_push_u8(buf, HCI_EVT);
		break;
	default:
		SYS_LOG_ERR("Unknown type %u", bt_buf_get_type(buf));
		net_buf_unref(buf);
		return -EINVAL;
	}

	if (buf->len > SPI_MAX_MSG_LEN) {
		SYS_LOG_ERR("TX message too long");
		net_buf_unref(buf);
		return -EINVAL;
	}
	header_slave[STATUS_HEADER_TOREAD] = buf->len;

	gpio_pin_write(gpio_dev, GPIO_IRQ_PIN, 1);

	/* Coordinate transfer lock with the spi rx thread */
	k_sem_take(&sem_spi_tx, K_FOREVER);
	do {
		ret = spi_transceive(spi_hci_dev, header_slave, 5,
				     header_master, 5);
		if (ret < 0) {
			SYS_LOG_ERR("SPI transceive error: %d", ret);
		}
	} while (header_master[STATUS_HEADER_READY] != SPI_READ);

	ret = spi_transceive(spi_hci_dev, buf->data, buf->len,
			     &rxmsg, buf->len);
	if (ret < 0) {
		SYS_LOG_ERR("SPI transceive error: %d", ret);
	}
	net_buf_unref(buf);

	gpio_pin_write(gpio_dev, GPIO_IRQ_PIN, 0);
	k_sem_give(&sem_spi_rx);

	return 0;
}

static void bt_tx_thread(void *p1, void *p2, void *p3)
{
	u8_t header_master[5];
	u8_t header_slave[5] = { READY_NOW, SANITY_CHECK,
				 0x00, 0x00, 0x00 };
	struct net_buf *buf = NULL;
	struct bt_hci_cmd_hdr cmd_hdr;
	struct bt_hci_acl_hdr acl_hdr;
	u8_t ret;

	memset(&txmsg, 0xFF, SPI_MAX_MSG_LEN);

	while (1) {
		do {
			ret = spi_transceive(spi_hci_dev, header_slave, 5,
					     header_master, 5);
			if (ret < 0) {
				SYS_LOG_ERR("SPI transceive error: %d", ret);
			}
		} while ((header_master[STATUS_HEADER_READY] != SPI_READ) &&
			 (header_master[STATUS_HEADER_READY] != SPI_WRITE));

		if (header_master[STATUS_HEADER_READY] == SPI_READ) {
			/* Unblock the spi tx thread and wait for it */
			k_sem_give(&sem_spi_tx);
			k_sem_take(&sem_spi_rx, K_FOREVER);
			continue;
		}

		/* Receiving data from the SPI Host */
		ret = spi_transceive(spi_hci_dev, &txmsg, SPI_MAX_MSG_LEN,
				     &rxmsg, SPI_MAX_MSG_LEN);
		if (ret < 0) {
			SYS_LOG_ERR("SPI transceive error: %d", ret);
		}

		switch (rxmsg[PACKET_TYPE]) {
		case HCI_CMD:
			memcpy(&cmd_hdr, &rxmsg[1], sizeof(cmd_hdr));

			buf = net_buf_alloc(&cmd_tx_pool, K_NO_WAIT);
			if (buf) {
				bt_buf_set_type(buf, BT_BUF_CMD);
				net_buf_add_mem(buf, &cmd_hdr,
						sizeof(cmd_hdr));
				net_buf_add_mem(buf, &rxmsg[4],
						cmd_hdr.param_len);
			} else {
				SYS_LOG_ERR("No available command buffers!");
				continue;
			}
			break;
		case HCI_ACL:
			memcpy(&acl_hdr, &rxmsg[1], sizeof(acl_hdr));

			buf = net_buf_alloc(&acl_tx_pool, K_NO_WAIT);
			if (buf) {
				bt_buf_set_type(buf, BT_BUF_ACL_OUT);
				net_buf_add_mem(buf, &acl_hdr,
						sizeof(acl_hdr));
				net_buf_add_mem(buf, &rxmsg[5],
						sys_le16_to_cpu(acl_hdr.len));
			} else {
				SYS_LOG_ERR("No available ACL buffers!");
				continue;
			}
			break;
		default:
			SYS_LOG_ERR("Unknown BT HCI buf type");
			continue;
		}

		SYS_LOG_DBG("buf %p type %u len %u",
			    buf, bt_buf_get_type(buf), buf->len);

		bt_send(buf);
		stack_analyze("tx_stack", bt_tx_thread_stack,
			      sizeof(bt_tx_thread_stack));

		/* Make sure other threads get a chance to run */
		k_yield();
	}
}

static int hci_spi_init(struct device *unused)
{
	static struct spi_config btspi_config = {
		.config = SPI_WORD(8),
	};

	SYS_LOG_DBG("");

	spi_hci_dev =
		device_get_binding(CONFIG_BLUETOOTH_SPI_TO_HOST_DEV_NAME);
	if (!spi_hci_dev) {
		return -EINVAL;
	}

	if (spi_configure(spi_hci_dev, &btspi_config) < 0) {
		return -EINVAL;
	}

	gpio_dev = device_get_binding(GPIO_IRQ_DEV_NAME);
	if (!gpio_dev) {
		return -EINVAL;
	}
	gpio_pin_configure(gpio_dev, GPIO_IRQ_PIN,
			   GPIO_DIR_OUT | GPIO_PUD_PULL_DOWN);

	return 0;
}

DEVICE_INIT(hci_spi, "hci_spi", &hci_spi_init, NULL, NULL,
	    APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

void main(void)
{
	static K_FIFO_DEFINE(rx_queue);
	struct bt_hci_evt_hdr *evt_hdr;
	struct net_buf *buf;
	int err;

	SYS_LOG_DBG("Start");

	bt_enable_raw(&rx_queue);

	/* Spawn the TX thread, which feeds cmds and data to the controller */
	k_thread_create(&bt_tx_thread_data, bt_tx_thread_stack,
			sizeof(bt_tx_thread_stack), bt_tx_thread,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);

	/* Send a vendor event to announce that the slave is initialized */
	buf = net_buf_alloc(&cmd_tx_pool, K_FOREVER);
	if (buf) {
		bt_buf_set_type(buf, BT_BUF_EVT);
		evt_hdr = net_buf_add(buf, sizeof(*evt_hdr));
		evt_hdr->evt = BT_HCI_EVT_VENDOR;
		evt_hdr->len = 2;
		u16_t *param = net_buf_add(buf, sizeof(u16_t));
		*param = sys_cpu_to_le16(EVT_BLUE_INITIALIZED);
	}
	spi_send(buf);

	while (1) {
		buf = net_buf_get(&rx_queue, K_FOREVER);
		err = spi_send(buf);
		if (err) {
			SYS_LOG_ERR("Failed to send");
		}
	}
}
