/* spi_nrf5.c - SPI driver for Nordic nRF5x SoCs */
/*
 * Copyright (c) 2016 Linaro Limited.
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
 *
 */

#define SYS_LOG_LEVEL CONFIG_SYS_LOG_SPI_LEVEL

#include <nanokernel.h>
#include <device.h>
#include <errno.h>

#include <misc/sys_log.h>
#include <sys_io.h>
#include <board.h>
#include <init.h>
#include <gpio.h>
#include <spi.h>

#include "spi_nrf5_priv.h"
#include <spi/spi_nrf5.h>

#define DEV_CFG(dev)	((struct spi_nrf5_config * const)(dev)->config->config_info)
#define SPI_REGS(dev)	((volatile struct spi_slave_nrf5 *)(DEV_CFG(dev))->base_addr)
#define DEV_DATA(dev)	((struct spi_nrf5_data * const)(dev)->driver_data)

static inline bool is_buf_in_ram(const void *buf)
{
	return ((((uintptr_t) buf) & 0xE0000000) == 0x20000000);
}

static void spis_nrf5_print_cfg_registers(struct device *dev)
{
	volatile __attribute__((__unused__)) struct spi_slave_nrf5 *regs = SPI_REGS(dev);

	SYS_LOG_DBG("\n"
		"SHORTS: %x, IRQ: %x, SEMSTAT: %x\n"
		"CONFIG: %x, STATUS: %x, ENABLE: %x\n"
		"SCKPIN: %x, MISOPIN: %x, MOSIPIN: %x, CSNPIN: %x\n"
		"RXD (PTR: %x, MAXCNT: %x, AMOUNT: %x)\n"
		"TXD (PTR: %x, MAXCNT: %x, AMOUNT: %x)",
		regs->SHORTS, regs->INTENSET, regs->SEMSTAT,
		regs->CONFIG, regs->STATUS, regs->ENABLE,
		regs->PSELSCK, regs->PSELMISO, regs->PSELMOSI, regs->PSELCSN,
		regs->RXDPTR, regs->RXDMAXCNT, regs->RXDAMOUNT,
		regs->TXDPTR, regs->TXDMAXCNT, regs->TXDAMOUNT);
}

/**
 * @brief Configure the SPI host controller for operating against slaves
 * @param dev Pointer to the device structure for the driver instance
 * @param config Pointer to the application provided configuration
 *
 * @return 0 if successful, another DEV_* code otherwise.
 */
static int spis_nrf5_configure(struct device *dev, struct spi_config *config)
{
	volatile struct spi_slave_nrf5 *spi_regs = SPI_REGS(dev);
	struct spi_nrf5_data *priv_data = DEV_DATA(dev);
	uint32_t flags;

	/* make sure module is disabled */
	spi_regs->ENABLE = 0;

	/* TODO: Muck with IRQ priority if needed */

	/* Clear any pending events */
	spi_regs->EVENTS_ACQUIRED = 0;
	spi_regs->EVENTS_ENDRX = 0;
	spi_regs->EVENTS_END = 0;
	spi_regs->INTENCLR = 0xFFFFFFFF; /* do we need to clear INT ?*/
	spi_regs->SHORTS = SPI_NRF5_SHORTCUT_END_ACQUIRE;
	spi_regs->INTENSET |= (SPIS_INTENSET_ACQUIRED_Msk | SPIS_INTENSET_END_Msk);

	/* default transmit and over-read characters */
	spi_regs->DEF = 0x00000055;
	spi_regs->ORC = 0x000000AA;

	/* user configuration */
	flags = config->config;

	/* FIXME: init from configuration MSB, (CPOL, CPHA) = (0, 0) */
	spi_regs->CONFIG = (flags & SPI_TRANSFER_LSB) ? (1 << 0) : 0;
	spi_regs->CONFIG |= (flags & SPI_MODE_CPHA) ? (1 << 1) : 0;
	spi_regs->CONFIG |= (flags & SPI_MODE_CPOL) ? (1 << 2) : 0;

	if (flags & SPI_NRF5_OP_MODE_SLAVE) {
		/* Slave */
	} else {
		/* Master */
	}

	/* Initialize driver's Tx/Rx queues and frame size */
	priv_data->tx_buf_len = 0;
	priv_data->rx_buf_len = 0;
	priv_data->frame_sz = 8;
	priv_data->tx_buf = NULL;
	priv_data->rx_buf = NULL;

	/* Enable the SPIS - peripherals sharing same ID will be disabled */
	spi_regs->ENABLE = SPIS_NRF5_ENABLE;

	spis_nrf5_print_cfg_registers(dev);

	SYS_LOG_DBG("SPI Slave Driver configured");

	return 0;
}

/**
 * @brief Read and/or write a defined amount of data through an SPI driver
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param tx_buf Memory buffer that data should be transferred from
 * @param tx_buf_len Size of the memory buffer available for reading from
 * @param rx_buf Memory buffer that data should be transferred to
 * @param rx_buf_len Size of the memory buffer available for writing to
 *
 * @return 0 if successful, another DEV_* code otherwise.
 */
static int spis_nrf5_transceive(struct device *dev, const void *tx_buf,
			 uint32_t tx_buf_len, void *rx_buf, uint32_t rx_buf_len)
{
	volatile struct spi_slave_nrf5 *spi_regs = SPI_REGS(dev);
	struct spi_nrf5_data *priv_data = DEV_DATA(dev);

	__ASSERT(!(tx_buf_len && !tx_buf  || rx_buf_len && !rx_buf),
		"spi_nrf5_transceive: ERROR - NULL buffers");

	/* Buffer needs to be in RAM for EasyDMA to work */
	if (!tx_buf || !is_buf_in_ram(tx_buf)) {
		SYS_LOG_ERR("Invalid TX buf %p", tx_buf);
		return -EINVAL;
	}
	if (!rx_buf || !is_buf_in_ram(rx_buf)) {
		SYS_LOG_ERR("Invalid RX buf %p", rx_buf);
		return -EINVAL;
	}

	/* Set buffers info */
	priv_data->tx_buf_len = tx_buf_len;
	priv_data->rx_buf_len = rx_buf_len;
	priv_data->tx_buf = tx_buf;
	priv_data->rx_buf = rx_buf;

	priv_data->trans_len = max(tx_buf_len, rx_buf_len);
	priv_data->transmitted = 0;
	priv_data->received = 0;
	priv_data->error = 0;

	SYS_LOG_DBG("semstat: 0x%x", spi_regs->SEMSTAT);

	if (spi_regs->EVENTS_ACQUIRED == 1) {
		spi_regs->TXDPTR = (uint32_t) priv_data->tx_buf;
		spi_regs->RXDPTR = (uint32_t) priv_data->rx_buf;
		spi_regs->TXDMAXCNT = priv_data->tx_buf_len;
		spi_regs->RXDMAXCNT = priv_data->rx_buf_len;
		spi_regs->TASKS_RELEASE = 1;
	} else {
		/* Wait for the semaphore to assign buffers */
		spi_regs->TASKS_ACQUIRE = 1;
	}

	/* wait for transfer to complete */
	device_sync_call_wait(&priv_data->sync);

	if (priv_data->error) {
		priv_data->error = 0;
		return -EIO;
	}

	return 0;
}

/**
 * @brief Complete SPI module data transfer operations.
 * @param dev Pointer to the device structure for the driver instance
 * @param error Error condition (0 = no error, otherwise an error occurred)
 * @return None.
 */
static void spis_nrf5_complete(struct device *dev, uint32_t error)
{
	volatile __attribute__((__unused__)) struct spi_slave_nrf5 *spi_regs = SPI_REGS(dev);
	struct spi_nrf5_data *priv_data = DEV_DATA(dev);

	SYS_LOG_DBG("bytes transferred: TX: %u, RX: %u [%s]",
		spi_regs->TXDAMOUNT, spi_regs->RXDAMOUNT,
		error == 0 ? "OK":"KO");

	priv_data->error = error ? 1 : 0;

	device_sync_call_complete(&priv_data->sync);
}

/**
 * @brief SPI module interrupt handler.
 * @param arg Pointer to the device structure for the driver instance
 * @return None.
 */
void spis_nrf5_isr(void *arg)
{
	struct device *dev = arg;
	volatile struct spi_slave_nrf5 *spi_regs = SPI_REGS(dev);
	struct spi_nrf5_data *priv_data = DEV_DATA(dev);
	uint32_t error = 0;
	uint32_t tmp;

	SYS_LOG_DBG("SPI Slave driver ISR");

	/* We get an interrupt for the following reasons:
	 * 1. Semapthore ACQUIRED:
	 *      Allows start of a transceive where the buffers are "handed over"
	 *      to the SPI module
	 * 2. End of Granted SPI transaction:
	 *      So we can swap out buffers if needed
	 */

	/* NOTE:
	 * Section 15.8.1 of nrf52 manual suggests reading back the register
	 * to cause a 4-cycle delay to prevent the interrupt from re-occuring
	 */

	if (spi_regs->EVENTS_ACQUIRED) {
		SYS_LOG_DBG("EVENTS_ACQUIRED: semstat: 0x%x", spi_regs->SEMSTAT);
		spi_regs->EVENTS_ACQUIRED = 0;

		/* force registesr flush (per spec) */
		tmp = spi_regs->EVENTS_ACQUIRED;

		/* update buffers */
		spi_regs->RXDPTR = (uint32_t) priv_data->rx_buf;
		spi_regs->TXDPTR = (uint32_t) priv_data->tx_buf;
		spi_regs->TXDMAXCNT = priv_data->tx_buf_len;
		spi_regs->RXDMAXCNT = priv_data->rx_buf_len;

		spi_regs->TASKS_RELEASE = 1;
	}

	if (spi_regs->EVENTS_END) {
		SYS_LOG_DBG("EVENTS_END: semstat: 0x%x", spi_regs->SEMSTAT);
		spi_regs->EVENTS_END = 0;

		/* force register flush (per spec) */
		tmp = spi_regs->EVENTS_END;

		spis_nrf5_complete(dev, error);
	}

	return;
}

static struct spi_driver_api nrf5_spis_api = {
	.transceive = spis_nrf5_transceive,
	.configure = spis_nrf5_configure,
	.slave_select = NULL,
};

int spis_nrf5_init(struct device *dev)
{
	volatile struct spi_slave_nrf5 *spi_regs = SPI_REGS(dev);
	struct spi_nrf5_data *priv_data = DEV_DATA(dev);
	struct spi_nrf5_config *cfg = DEV_CFG(dev);
	struct device *gpio_dev;

	SYS_LOG_DBG("SPI Slave driver init: %p", dev);

	/* Enable constant latency for faster SPIS response */
	NRF_POWER->TASKS_CONSTLAT = 1;

	spi_regs->ENABLE = 0;

	gpio_dev = device_get_binding(CONFIG_GPIO_NRF5_P0_DEV_NAME);

	(void) gpio_pin_configure(gpio_dev, CONFIG_SPI_0_NRF5_GPIO_MISO,
				  GPIO_DIR_IN | GPIO_PUD_NORMAL);

	(void) gpio_pin_configure(gpio_dev, CONFIG_SPI_0_NRF5_GPIO_MOSI,
				  GPIO_DIR_IN | GPIO_PUD_NORMAL);

	(void) gpio_pin_configure(gpio_dev, CONFIG_SPI_0_NRF5_GPIO_CS,
				  GPIO_DIR_IN | GPIO_PUD_PULL_UP);

	(void) gpio_pin_configure(gpio_dev, CONFIG_SPI_0_NRF5_GPIO_SCK,
				  GPIO_DIR_IN | GPIO_PUD_NORMAL);

	spi_regs->PSELMOSI = CONFIG_SPI_0_NRF5_GPIO_MOSI;
	spi_regs->PSELMISO = CONFIG_SPI_0_NRF5_GPIO_MISO;
	spi_regs->PSELSCK  = CONFIG_SPI_0_NRF5_GPIO_SCK;
	spi_regs->PSELCSN  = CONFIG_SPI_0_NRF5_GPIO_CS;

	cfg->config_func();

	device_sync_call_init(&priv_data->sync);

	SYS_LOG_DBG("SPI Slave driver initialized on device: %p", dev);

	return 0;
}

/* system bindings */
#ifdef CONFIG_SPI_0

#ifdef CONFIG_SOC_SERIES_NRF52X
	#define NRF5_SPIS_0_IRQ NRF52_IRQ_SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn
#elif CONFIG_SOC_SERIES_NRF51X
	#define NRF5_SPIS_0_IRQ NRF51_IRQ_SPI1_TWI1_IRQn
#endif

void spis_config_0_irq(void);

struct spi_nrf5_data spis_nrf5_data_port_0;

struct spi_nrf5_config spis_nrf5_config_0 = {
#ifdef CONFIG_SOC_SERIES_NRF52X
	.base_addr = NRF_SPIS0_BASE,
#elif CONFIG_SOC_SERIES_NRF51X
	.base_addr = NRF_SPIS1_BASE,
#endif
	.config_func = spis_config_0_irq
};

DEVICE_AND_API_INIT(spis_nrf5_port_0, CONFIG_SPI_0_NAME, spis_nrf5_init,
		    &spis_nrf5_data_port_0, &spis_nrf5_config_0, PRIMARY,
		    CONFIG_SPI_INIT_PRIORITY, &nrf5_spis_api);

void spis_config_0_irq(void)
{
	IRQ_CONNECT(NRF5_SPIS_0_IRQ, CONFIG_SPI_0_IRQ_PRI,
		    spis_nrf5_isr, DEVICE_GET(spis_nrf5_port_0), 0);
	irq_enable(NRF5_SPIS_0_IRQ);
}

#endif /* CONFIG_SPI_0 */
