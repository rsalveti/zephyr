/* bt_spi_tunnel.c - Program to grab raw HCI frames to send over SPI */

/*
 * Copyright (c) 2016 Linaro Limited
 * based on btusb.c
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

#include <nanokernel.h>
#include <stdio.h>

#define SYS_LOG_LEVEL SYS_LOG_SPI_LEVEL
#include <misc/sys_log.h>

#include <errno.h>

#include <device.h>
#include <gpio.h>

#include <net/buf.h>
#include <bluetooth/buf.h>
#include <bluetooth/hci_raw.h>

#include <spi.h>
#ifdef CONFIG_SPI_NRF5
#include <spi/spi_nrf5.h>
#endif

/*
 *  \|/
 *   |
 *   |           [   Zephyr   ]           \       \       [   Zephyr   ]
 * [Radio] <---> [ controller ] <--XXXX---/ Break /--XX-> [  BLE HCI   ] <---> [Application]
 *               [   driver   ]           \       \       [   driver   ]
 *                     ^                                        ^
 *                     |                                        |
 *                     |                                        |
 *                     v                                        v
 *               [ Raw HCI driver ]                    [ Zephyr HCI stack ]
 *                     ^                                        ^
 *                     |                                        |
 *                     |                                        |
 *                     v                                        v
 *               [ This program ]                    [ BLE SPI Transport driver ]
 *                     |                                        |
 *                     |-------------- [ SPI Tunnel ] ----------|
 *                        (slave)                      (master)
 *
 *    [Connectivity processor]                       [ Application Processor ]
 *    [      (e.g. nRF51)    ]                       [      (e.g. STM43F4)   ]
 */


#define SPI_RX_FIBER_STACK_SIZE 1024
static char __stack bt_spi_rx_fiber_stack[SPI_RX_FIBER_STACK_SIZE];

static struct device *spi_dev;
static struct device *gpio_dev;

#if 0
#define BTSPI_BUFFER_SIZE 64

/* Max Bluetooth command data size */
#define BTSPI_CLASS_MAX_DATA_SIZE	100

/* Misc. macros */
#define LOW_BYTE(x)	((x) & 0xFF)
#define HIGH_BYTE(x)	((x) >> 8)
#endif

static struct nano_fifo rx_queue;

/* HCI command buffers */
#define CMD_BUF_SIZE (CONFIG_BLUETOOTH_HCI_SEND_RESERVE + \
		      sizeof(struct bt_hci_cmd_hdr) + \
		      CONFIG_BLUETOOTH_MAX_CMD_LEN)

static struct nano_fifo avail_tx;
static NET_BUF_POOL(tx_pool, CONFIG_BLUETOOTH_HCI_CMD_COUNT, CMD_BUF_SIZE,
		    &avail_tx, NULL, sizeof(uint8_t));

#define BT_L2CAP_MTU 64
/** Data size needed for ACL buffers */
#define BT_BUF_ACL_SIZE (CONFIG_BLUETOOTH_HCI_RECV_RESERVE + \
			 sizeof(struct bt_hci_acl_hdr) + \
			 4 /* L2CAP header size */ + \
			 BT_L2CAP_MTU)

static struct nano_fifo avail_acl_tx;
static NET_BUF_POOL(acl_tx_pool, 2, BT_BUF_ACL_SIZE, &avail_acl_tx, NULL,
		    sizeof(uint8_t));

/* FIXME: This make it nRF5 specific, generalise it later */
#define OP_MODE SPI_NRF5_OP_MODE_SLAVE

/* TODO: Move to a proper place */
/* Slave uses RDY and REQ pins to coordinate the messages with master */
#define GPIO_DRV_NAME	CONFIG_GPIO_NRF5_P0_DEV_NAME
#if defined(CONFIG_BOARD_NRF51_CARBON)
#define GPIO_RDY_PIN	29
#define GPIO_REQ_PIN	28
#elif defined(CONFIG_BOARD_NRF51_PCA10028)
#define GPIO_RDY_PIN	22
#define GPIO_REQ_PIN	21
#endif
#define GPIO_RDY_DIR	GPIO_DIR_OUT
#define GPIO_RDY_PULL	GPIO_PUD_PULL_DOWN
#define GPIO_REQ_DIR	GPIO_DIR_OUT
#define GPIO_REQ_PULL	GPIO_PUD_PULL_DOWN

/* 2 bytes are used for the header size */
#define HEADER_SIZE	2

static struct spi_config btspi_config = {
	.config = (SPI_WORD(8) | OP_MODE),
	.max_sys_freq = 128,
};

struct nano_sem nano_sem_fiber;
struct nano_sem nano_sem_task;

/* TODO: move to standard utils */
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

static inline int bt_spi_transceive(const void *tx_buf, uint32_t tx_buf_len,
				    void *rx_buf, uint32_t rx_buf_len)
{
	int ret = 0;

	SYS_LOG_DBG("bt_spi_transceive: /RDY set to 1 -> 0 (notify master)");
	gpio_pin_write(gpio_dev, GPIO_RDY_PIN, 1);
	gpio_pin_write(gpio_dev, GPIO_RDY_PIN, 0);
	ret = spi_transceive(spi_dev, tx_buf, tx_buf_len, rx_buf, rx_buf_len);

	return ret;
}

static int bt_spi_tx(uint8_t bt_buf_type, struct net_buf *buf)
{
	uint8_t spi_tx_buf[255];
	uint8_t spi_rx_buf[1] = { 0 };
	uint32_t spi_buf_len;
	int ret;

	/* To send data we first must notify the master side with /REQ */
	SYS_LOG_DBG("setting /REQ to 1 -> 0");
	gpio_pin_write(gpio_dev, GPIO_REQ_PIN, 1);
	gpio_pin_write(gpio_dev, GPIO_REQ_PIN, 0);

	/* Wait until rx fiber says we're good to go */
	SYS_LOG_DBG("sem take task, wait for rx fiber");
	nano_task_sem_take(&nano_sem_task, TICKS_UNLIMITED);

	/* Send the header, containing the buf size */
	memset(spi_tx_buf, 0, sizeof(spi_tx_buf));
	spi_buf_len = buf->len + 1; /* extra byte for buffer type */
	memcpy(spi_tx_buf, &spi_buf_len, sizeof(spi_buf_len));
	SYS_LOG_DBG("header - spi buf len: %d", spi_buf_len);
	ret = bt_spi_transceive(spi_tx_buf, 2, spi_rx_buf, 1);
	if (ret < 0) {
		SYS_LOG_ERR("SPI transceive error (header): %d", ret);
		nano_task_sem_give(&nano_sem_fiber);
		return -EIO;
	}

	/* Then send the data */
	/* TODO: transmit multiple times when buf > tx_buf */
	/* TODO: should we send buf type together with header, for zero copy? */
	memset(spi_tx_buf, 0, sizeof(spi_tx_buf));
	spi_tx_buf[0] = bt_buf_type;
	memcpy(spi_tx_buf + 1, buf->data, buf->len);
	SYS_LOG_DBG("sending command buffer, len: %d", buf->len + 1);
	ret = bt_spi_transceive(spi_tx_buf, buf->len + 1, spi_rx_buf, 1);
	if (ret < 0) {
		SYS_LOG_ERR("SPI transceive error (data): %d", ret);
		nano_task_sem_give(&nano_sem_fiber);
		return -EIO;
	}

	SYS_LOG_DBG("sem give fiber");
	nano_task_sem_give(&nano_sem_fiber);

	return 0;
}

/* Fiber responsible for receiving data from master */
static void bt_spi_rx_fiber(void)
{
	uint8_t spi_rx_buf[255];
	uint8_t spi_tx_buf[1] = { 0 };
	uint32_t spi_buf_len;
	uint8_t bt_buf_type;
	struct net_buf *buf;
	int ret;

	SYS_LOG_DBG("Starting bt_spi_rx_fiber");

	while (1) {
		/* First read is the header, which contains the buffer size */
		memset(&spi_rx_buf, 0, sizeof(spi_rx_buf));
		ret = bt_spi_transceive(spi_tx_buf, sizeof(spi_tx_buf),
					spi_rx_buf, HEADER_SIZE);
		if (ret != 0) {
			SYS_LOG_ERR("SPI transceive error (header): %d", ret);
			continue;
		}
		/* Check buffer content:
		 * If buffer contains only 0, master ready to accept data
		 * If buffer > 0, read buffer and add to the queue
		 */
		memcpy(&spi_buf_len, spi_rx_buf, HEADER_SIZE);
		SYS_LOG_DBG("Header: buf size %d", spi_buf_len);
		if (spi_buf_len == 0) {
			/* Let the tx task to do its job and wait */
			SYS_LOG_DBG("sem give task");
			nano_fiber_sem_give(&nano_sem_task);
			SYS_LOG_DBG("sem take fiber, wait for tx task");
			nano_fiber_sem_take(&nano_sem_fiber, TICKS_UNLIMITED);
			continue;
		}

		/* TODO: manage multiple read operations (buf > 255) */
		memset(&spi_rx_buf, 0, sizeof(spi_rx_buf));
		ret = bt_spi_transceive(spi_tx_buf, sizeof(spi_tx_buf),
					spi_rx_buf, spi_buf_len);
		if (ret != 0) {
			SYS_LOG_ERR("SPI transceive error (data): %d", ret);
			continue;
		}

		bt_buf_type = spi_rx_buf[0];
		switch (bt_buf_type) {
		case BT_BUF_CMD:
			SYS_LOG_DBG("BT rx buf type BUF_CMD");
			buf = net_buf_get(&avail_tx, 0);
			if (!buf) {
				SYS_LOG_ERR("Cannot get free tx buffer");
				continue;
			}
			break;
		case BT_BUF_ACL_OUT:
			SYS_LOG_DBG("BT rx buf type ACL_OUT");
			buf = net_buf_get(&avail_acl_tx, 0);
			if (!buf) {
				SYS_LOG_ERR("Cannot get free acl tx buffer");
				continue;
			}
			break;
		default:
			SYS_LOG_ERR("Unknown bluetooth buffer type");
			continue;
		}

		bt_buf_set_type(buf, bt_buf_type);
		memcpy(net_buf_add(buf, spi_buf_len - 1), spi_rx_buf + 1,
							spi_buf_len - 1);
		hexdump(">", buf->data, buf->len);
		bt_send(buf);
	}
}

void main(void)
{
	int ret;

	SYS_LOG_DBG("\n\nStart\n");

	spi_dev = device_get_binding(CONFIG_BLUETOOTH_SPI_ON_DEV_NAME);
	if (!spi_dev) {
		SYS_LOG_ERR("Cannot find device %s",
				CONFIG_BLUETOOTH_SPI_ON_DEV_NAME);
		return;
	}

	/* Initialize the SPI driver with the right configuration */
	ret = spi_configure(spi_dev, &btspi_config);
	if (ret < 0) {
		SYS_LOG_ERR("Failed to configure %s",
				CONFIG_BLUETOOTH_SPI_ON_DEV_NAME);
		return;
	}

	/* Initialize the /REQ and /RDY GPIO pins */
	gpio_dev = device_get_binding(GPIO_DRV_NAME);
	gpio_pin_configure(gpio_dev, GPIO_RDY_PIN,
				GPIO_RDY_DIR | GPIO_RDY_PULL);
	gpio_pin_configure(gpio_dev, GPIO_REQ_PIN,
				GPIO_REQ_DIR | GPIO_REQ_PULL);

	/* Initialize the buffer pools */
	net_buf_pool_init(tx_pool);
	net_buf_pool_init(acl_tx_pool);
	nano_fifo_init(&rx_queue);

	nano_sem_init(&nano_sem_fiber);
	nano_sem_init(&nano_sem_task);

	/* Receiver fiber */
	fiber_start(bt_spi_rx_fiber_stack, sizeof(bt_spi_rx_fiber_stack),
			(nano_fiber_entry_t) bt_spi_rx_fiber, 0, 0, 7, 0);

	bt_enable_raw(&rx_queue);

	while (1) {
		struct net_buf *buf;
		uint8_t bt_buf_type;

		/* With TICKS_UNLIMITED we always get a valid buffer */
		buf = net_buf_get_timeout(&rx_queue, 0, TICKS_UNLIMITED);
		bt_buf_type = bt_buf_get_type(buf);

		hexdump("<", buf->data, buf->len);

		bt_spi_tx(bt_buf_type, buf);

		net_buf_unref(buf);
	}
}
