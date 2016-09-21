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
struct nano_sem nano_sem_rdy;

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
/* Pin RDY is used by the slave to announce it is ready to transfer over spi */
#define GPIO_RDY_DIR		GPIO_DIR_IN
#define GPIO_RDY_PULL		GPIO_PUD_PULL_DOWN
#define GPIO_RDY_PIN		0
#define GPIO_RDY_EDGE		(GPIO_INT_EDGE | GPIO_INT_DOUBLE_EDGE)
/* Pin REQ is used by the slave to request permission to send data to master */
#define GPIO_REQ_DIR		GPIO_DIR_IN
#define GPIO_REQ_PULL		GPIO_PUD_PULL_DOWN
#define GPIO_REQ_PIN		1
#define GPIO_REQ_EDGE		(GPIO_INT_EDGE | GPIO_INT_DOUBLE_EDGE)
#endif

/* 2 bytes are used for the header size */
#define HEADER_SIZE	2

#define SPI_RDY_WAIT_TIMEOUT	100

static struct gpio_callback gpio_req_cb;
static struct gpio_callback gpio_rdy_cb;
static int prev_slave_req = 0;
static int prev_slave_rdy = 0;

static struct spi_config spi_conf = {
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

void gpio_slave_rdy(struct device *gpio, struct gpio_callback *cb,
		    uint32_t pins)
{
	int slave_rdy;

	gpio_pin_read(gpio, GPIO_RDY_PIN, &slave_rdy);
	/*
	 * FIXME: multiple interrupts are generated during spi transceive,
	 * so for now just store and maintain the previous value
	 */
	if (slave_rdy != prev_slave_rdy) {
		prev_slave_rdy = slave_rdy;
		if (prev_slave_rdy == 1) {
			nano_isr_sem_give(&nano_sem_rdy);
		}
	}
}

static inline int bt_spi_transceive(const void *tx_buf, uint32_t tx_buf_len,
				    void *rx_buf, uint32_t rx_buf_len)
{
	BT_DBG("");
	int ret = 0;

	if (prev_slave_rdy == 0) {
		/* Wait for the sem release */
		nano_fiber_sem_take(&nano_sem_rdy, SPI_RDY_WAIT_TIMEOUT);
	}
	ret = spi_transceive(spi_dev, tx_buf, tx_buf_len, rx_buf, rx_buf_len);

	return ret;
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
		ret = bt_spi_transceive(spi_tx_buf, sizeof(spi_tx_buf),
					spi_rx_buf, 2);
		if (ret < 0) {
			BT_ERR("SPI transceive error (empty header): %d", ret);
			continue;
		}

		/* First read is the header, which contains the buffer size */
		memset(&spi_rx_buf, 0, sizeof(spi_rx_buf));
		ret = bt_spi_transceive(spi_tx_buf, sizeof(spi_tx_buf),
					spi_rx_buf, HEADER_SIZE);
		if (ret < 0) {
			BT_ERR("SPI transceive error (header): %d", ret);
			continue;
		}

		/* TODO: handle different buf sizes */
		memcpy(&spi_buf_len, spi_rx_buf, HEADER_SIZE);
		BT_DBG("Header: buf size %d", spi_buf_len);

		memset(&spi_rx_buf, 0, sizeof(spi_rx_buf));
		ret = bt_spi_transceive(spi_tx_buf, sizeof(spi_tx_buf),
					spi_rx_buf, spi_buf_len);
		if (ret < 0) {
			BT_ERR("SPI transceive error (data): %d", ret);
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
	ret = bt_spi_transceive(spi_tx_buf, 2, spi_rx_buf, 1);
	if (ret < 0) {
		BT_ERR("SPI transceive error (header): %d", ret);
		return -EIO;
	}

	/* Now the data, until everything is transmited */
	/* FIXME: allow buf > spi_tx_buf by transmitting multiple times */
	memset(spi_tx_buf, 0, sizeof(spi_tx_buf));
	spi_tx_buf[0] = (uint8_t) bt_buf_get_type(buf);
	memcpy(spi_tx_buf + 1, buf->data, buf->len);
	BT_DBG("sending command buffer, len: %d", buf->len + 1);
	ret = bt_spi_transceive(spi_tx_buf, buf->len + 1, spi_rx_buf, 1);
	if (ret < 0) {
		BT_ERR("SPI transceive error (data): %d", ret);
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
	gpio_pin_configure(gpio_dev, GPIO_REQ_PIN, GPIO_REQ_DIR |
					GPIO_INT | GPIO_REQ_EDGE);
	gpio_init_callback(&gpio_req_cb, gpio_slave_req, BIT(GPIO_REQ_PIN));
	gpio_add_callback(gpio_dev, &gpio_req_cb);
	gpio_pin_enable_callback(gpio_dev, GPIO_REQ_PIN);

	gpio_pin_configure(gpio_dev, GPIO_RDY_PIN, GPIO_RDY_DIR |
					GPIO_INT | GPIO_RDY_EDGE);
	gpio_init_callback(&gpio_rdy_cb, gpio_slave_rdy, BIT(GPIO_RDY_PIN));
	gpio_add_callback(gpio_dev, &gpio_rdy_cb);
	gpio_pin_enable_callback(gpio_dev, GPIO_RDY_PIN);

	nano_sem_init(&nano_sem_req);
	nano_sem_init(&nano_sem_rdy);

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
