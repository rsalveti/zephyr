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

#define SYS_LOG_LEVEL SYS_LOG_SPI_LEVEL
#include <misc/sys_log.h>
#include <misc/printk.h>
#include <misc/util.h>

#include <errno.h>

#include <device.h>

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


#define BTSPI_BUFFER_SIZE 64

/* Max Bluetooth command data size */
#define BTSPI_CLASS_MAX_DATA_SIZE	100

/* Misc. macros */
#define LOW_BYTE(x)	((x) & 0xFF)
#define HIGH_BYTE(x)	((x) >> 8)

static struct nano_fifo rx_queue;

/* HCI command buffers */
#define CMD_BUF_SIZE (CONFIG_BLUETOOTH_HCI_SEND_RESERVE +	  \
			sizeof(struct bt_hci_cmd_hdr) +	  \
			CONFIG_BLUETOOTH_MAX_CMD_LEN)

static struct nano_fifo avail_tx;
static NET_BUF_POOL(tx_pool, CONFIG_BLUETOOTH_HCI_CMD_COUNT, CMD_BUF_SIZE,
		&avail_tx, NULL, sizeof(uint8_t));

#define BT_L2CAP_MTU 64
/** Data size needed for ACL buffers */
#define BT_BUF_ACL_SIZE (CONFIG_BLUETOOTH_HCI_RECV_RESERVE +		\
				sizeof(struct bt_hci_acl_hdr) +	\
				4 /* L2CAP header size */ +		\
				BT_L2CAP_MTU)

static struct nano_fifo avail_acl_tx;
static NET_BUF_POOL(acl_tx_pool, 2, BT_BUF_ACL_SIZE, &avail_acl_tx, NULL,
		    sizeof(uint8_t));

/* FIXME: This make it nRF5 specific, generalise it later */
#define OP_MODE SPI_NRF5_OP_MODE_SLAVE

enum spi_status_t {
	SPI_UNKNOWN = -1,
	SPI_SEND = 1,
};

/* Tunnel state */
struct spi_tunnel_data_t {
	struct device *spi_dev;
	enum spi_status_t spi_status;
	uint32_t tx_bytes;
	uint32_t rx_bytes;
};

/* TODO: move to standard utils */
static void hexdump(const char *str, const uint8_t *packet, size_t length)
{
	int n = 0;

	if (!length) {
		printk("%s zero-length signal packet\n", str);
		return;
	}

	while (length--) {
		if (n % 16 == 0) {
			printk("%s %08X ", str, n);
		}

		printk("%02X ", *packet++);

		n++;
		if (n % 8 == 0) {
			if (n % 16 == 0) {
				printk("\n");
			} else {
				printk(" ");
			}
		}
	}

	if (n % 16) {
		printk("\n");
	}
}

static int try_write(struct device *dev, struct net_buf *buf)
{
	while (1) {
		int ret = spi_write(dev, buf->data, buf->len);

		switch (ret) {
		case -EAGAIN:
			break;
		/* TODO: Handle other error codes */
		default:
			return ret;
		}
	}
}

static struct spi_config btspi_config = {
	.config = (SPI_WORD(8) | OP_MODE),
	.max_sys_freq = 128,
};

static struct spi_tunnel_data_t btspi_data = {
	.spi_status = SPI_UNKNOWN,
};

void main(void)
{
	struct device *spi_dev;
	int ret;

	SYS_LOG_DBG("Start");

	spi_dev = device_get_binding(CONFIG_BLUETOOTH_SPI_ON_DEV_NAME);
	if (!spi_dev) {
		SYS_LOG_ERR("Cannot find device %s", CONFIG_BLUETOOTH_SPI_ON_DEV_NAME);
	}
	btspi_data.spi_dev = spi_dev;

	/* Initialize the SPI driver with the right configuration */
	ret = spi_configure(spi_dev, &btspi_config);
	if (ret < 0) {
		SYS_LOG_ERR("Failed to configure %s", CONFIG_BLUETOOTH_SPI_ON_DEV_NAME);
		return;
	}

	/* Initialize the buffer pools */
	net_buf_pool_init(tx_pool);
	net_buf_pool_init(acl_tx_pool);
	nano_fifo_init(&rx_queue);

	bt_enable_raw(&rx_queue);

	while (1) {
		struct net_buf *buf;

		buf = net_buf_get_timeout(&rx_queue, 0, TICKS_UNLIMITED);
    if (buf) {
            hexdump("<", buf->data, buf->len);
            ret = try_write(spi_dev, buf);
            net_buf_unref(buf);
    } else {
            printk("No buf\n");
            fiber_yield();
    }
	}
}
