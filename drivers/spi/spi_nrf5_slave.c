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

#include <errno.h>

#include <nanokernel.h>
#include <device.h>

#define SYS_LOG_LEVEL SYS_LOG_SPI_LEVEL
#include <misc/sys_log.h>

#include <board.h>
#include <init.h>
#include <gpio.h>
#include <gpio/gpio_nrf5.h>

#include <sys_io.h>

#include <spi.h>
#include <spi/spi_nrf5.h>
#include "spi_nrf5_priv.h"

/* convenience defines */
#define DEV_CFG(dev)                                            \
  ((struct spi_nrf5_config * const)(dev)->config->config_info)
#define DEV_DATA(dev)                                 \
  ((struct spi_nrf5_data * const)(dev)->driver_data)
#define SPI_REGS(dev)                                       \
  ((volatile struct spi_slave_nrf5 *)(DEV_CFG(dev))->base_addr)

static void spis_nrf5_print_cfg_registers(struct device *dev)
{
  volatile struct spi_slave_nrf5 *spi_regs = SPI_REGS(dev);

  SYS_LOG_DBG("\nSHORTS: %x, IRQ: %x, SEMSTAT: %x\n"
              "CONFIG: %x, STATUS: %x, ENABLE: %x\n"
              "SCKPIN: %x, MISOPIN: %x, MOSIPIN: %x, CSNPIN: %x\n"
              "RXD (PTR: %x, MAXCNT: %x, AMOUNT: %x)\n"
              "TXD (PTR: %x, MAXCNT: %x, AMOUNT: %x)\n",
              spi_regs->SHORTS, spi_regs->INTENSET, spi_regs->SEMSTAT,
              spi_regs->CONFIG, spi_regs->STATUS, spi_regs->ENABLE,
              spi_regs->PSELSCK, spi_regs->PSELMISO, spi_regs->PSELMOSI, spi_regs->PSELCSN,
              spi_regs->RXDPTR, spi_regs->RXDMAXCNT, spi_regs->RXDAMOUNT,
              spi_regs->TXDPTR, spi_regs->TXDMAXCNT, spi_regs->TXDAMOUNT);
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
	struct spi_nrf5_data *priv_data = DEV_DATA(dev);
  volatile struct spi_slave_nrf5 *spi_regs = SPI_REGS(dev);
  uint32_t flags = config->config;
  uint32_t c = 0;

  spi_regs->ENABLE = 0;

  /* Clear any pending events */
  spi_regs->EVENTS_END = 0;
  spi_regs->EVENTS_ACQUIRED = 0;
  spi_regs->EVENTS_ENDRX = 0;

  /* Clear any pending interrupt (is this the same as clearing the pending event?)*/
  spi_regs->INTENCLR = 0xFFFFFFFF;

  /* FIXME: Init from configuration
     MSB, (CPOL, CPHA) = (0, 0) */
  if (flags & SPI_NRF5_OP_MODE_SLAVE) {
    /* Slave */
  } else {
    /* Master */
  }
  if (flags & SPI_TRANSFER_LSB) {
    c |= 1;
  }
  if (flags & SPI_MODE_CPHA) {
    c |= (0x1 << 1);
  }
  if (flags & SPI_MODE_CPOL) {
    c |= (0x1 << 2);
  }
  spi_regs->CONFIG = c;

  /* 1-byte */
  priv_data->frame_sz = 8;

  spi_regs->SHORTS = SPI_NRF5_SHORTCUT_END_ACQUIRE;

  /* Enable interrupts */
  spi_regs->INTENSET |= (SPIS_INTENSET_ACQUIRED_Msk |
                         SPIS_INTENSET_END_Msk);

  /* TODO: Muck with IRQ priority if needed */

  /* Default and Over-read characters */
  spi_regs->DEF = (uint32_t) 0x55;
  spi_regs->ORC = (uint32_t) 0xAA;

  /* Initialize driver's Tx/Rx queue */
  priv_data->tx_buf = priv_data->rx_buf = NULL;
  priv_data->tx_buf_len = priv_data->rx_buf_len = 0;

  /* Enable the SPIS module (Other peripherals sharing same ID disabled automcatically) */
  spi_regs->ENABLE = SPIS_NRF5_ENABLE;

  spis_nrf5_print_cfg_registers(dev);

  SYS_LOG_DBG("nRF5 SPI Slave Driver configured\n");

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
static int spis_nrf5_transceive(struct device *dev,
				const void *tx_buf, uint32_t tx_buf_len,
				void *rx_buf, uint32_t rx_buf_len)
{
	struct spi_nrf5_data *priv_data = DEV_DATA(dev);
  volatile struct spi_slave_nrf5 *spi_regs = SPI_REGS(dev);

  __ASSERT(!((tx_buf_len && (tx_buf == NULL)) ||
             (rx_buf_len && (rx_buf == NULL))),
           "spi_nrf5_transceive: ERROR - NULL buffers");

  /* Set buffers info */
  priv_data->tx_buf = tx_buf;
  priv_data->rx_buf = rx_buf;
  priv_data->tx_buf_len = tx_buf_len;
  priv_data->rx_buf_len = rx_buf_len;
  priv_data->error = 0;
  priv_data->transmitted = 0;
  priv_data->received = 0;
  priv_data->trans_len = max(tx_buf_len, rx_buf_len);

  SYS_LOG_DBG("state 0x%x", spi_regs->SEMSTAT);

  if (spi_regs->EVENTS_ACQUIRED == 1) {
    spi_regs->TXDMAXCNT = priv_data->tx_buf_len;
    spi_regs->TXDPTR = (uint32_t) priv_data->tx_buf;
    spi_regs->RXDMAXCNT = priv_data->rx_buf_len;
    spi_regs->RXDPTR = (uint32_t) priv_data->rx_buf;

    spi_regs->TASKS_RELEASE = 1;
  } else {
    /* Wait for the semaphore to assign buffers */
    spi_regs->TASKS_ACQUIRE = 1;
  }

  /* wait for transfer to complete */
  device_sync_call_wait(&priv_data->sync);

  /* check completion status */
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
	struct spi_nrf5_data *priv_data = DEV_DATA(dev);
  volatile struct spi_slave_nrf5 *spi_regs = SPI_REGS(dev);

  SYS_LOG_DBG("Bytes transferred: TX: %u, RX: %u\n",
              spi_regs->TXDAMOUNT, spi_regs->RXDAMOUNT);

  /* Signal completion */
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
	struct spi_nrf5_data *priv_data = DEV_DATA(dev);
  volatile struct spi_slave_nrf5 *spi_regs = SPI_REGS(dev);
	uint32_t error = 0;
  uint32_t tmp;

  SYS_LOG_DBG("nRF5 SPI Slave driver ISR\n");

  /* We get an interrupt for the following reasons:
   * 1. Semaphore ACQUIRED: Allows start of a transceive where the buffers are
   *    "handed over" to the SPI module
   * 2. End of Granted SPI transaction, so we can swap out buffers if needed
   */

  /* NOTE: Section 15.8.1 of nrf52 manual suggests
   * reading back the register to cause a 4-cycle delay
   * to prevent the interrupt from re-occuring */

  if (spi_regs->EVENTS_ACQUIRED) {
    SYS_LOG_DBG("ACQUIRED: semstat = 0x%x\n", spi_regs->SEMSTAT);
    spi_regs->EVENTS_ACQUIRED = 0;
    tmp = spi_regs->EVENTS_ACQUIRED;

    spi_regs->TXDMAXCNT = priv_data->tx_buf_len;
    spi_regs->TXDPTR = (uint32_t) priv_data->tx_buf;
    spi_regs->RXDMAXCNT = priv_data->rx_buf_len;
    spi_regs->RXDPTR = (uint32_t) priv_data->rx_buf;

    spi_regs->TASKS_RELEASE = 1;
  }

  if (spi_regs->EVENTS_END) {
    SYS_LOG_DBG("EVENTS_END: sem_stat = 0x%x\n", spi_regs->SEMSTAT);
    spi_regs->EVENTS_END = 0;
    tmp = spi_regs->EVENTS_END;

    spis_nrf5_complete(dev, error);
  }

  return;
}

static struct spi_driver_api nrf5_spis_api = {
	.configure = spis_nrf5_configure,
	.slave_select = NULL,
	.transceive = spis_nrf5_transceive,
};


int spis_nrf5_init(struct device *dev)
{
  struct spi_nrf5_config *cfg = DEV_CFG(dev);
	struct spi_nrf5_data *priv_data = DEV_DATA(dev);
  volatile struct spi_slave_nrf5 *spi_regs = SPI_REGS(dev);
  struct device *gpio_dev;

  SYS_LOG_DBG("nRF5 SPI Slave driver init: %p\n", dev);
  spi_regs->ENABLE = 0;

  gpio_dev = device_get_binding(CONFIG_GPIO_NRF5_P0_DEV_NAME);

  (void) gpio_pin_configure(gpio_dev, CONFIG_SPI_0_NRF5_GPIO_MISO, (GPIO_DIR_INPUT |
                                             GPIO_INPUT_CONNECT |
                                             GPIO_PULL_DISABLE |
                                             GPIO_SENSE_DISABLE |
                                             GPIO_DRIVE_S0S1));
  (void) gpio_pin_configure(gpio_dev, CONFIG_SPI_0_NRF5_GPIO_MOSI, (GPIO_DIR_INPUT |
                                             GPIO_INPUT_CONNECT |
                                             GPIO_PULL_DISABLE |
                                             GPIO_SENSE_DISABLE |
                                             GPIO_DRIVE_S0S1));
  (void) gpio_pin_configure(gpio_dev, CONFIG_SPI_0_NRF5_GPIO_CS, (GPIO_DIR_INPUT |
                                            GPIO_INPUT_CONNECT |
                                            GPIO_PULL_DISABLE |
                                            GPIO_SENSE_DISABLE |
                                            GPIO_DRIVE_S0S1));
  (void) gpio_pin_configure(gpio_dev, CONFIG_SPI_0_NRF5_GPIO_SCK, (GPIO_DIR_INPUT |
                                            GPIO_INPUT_CONNECT |
                                            GPIO_PULL_DISABLE |
                                            GPIO_SENSE_DISABLE |
                                            GPIO_DRIVE_S0S1));

  spi_regs->PSELMOSI = CONFIG_SPI_0_NRF5_GPIO_MOSI;
  spi_regs->PSELCSN  = CONFIG_SPI_0_NRF5_GPIO_CS;
  spi_regs->PSELMISO = CONFIG_SPI_0_NRF5_GPIO_MISO;
  spi_regs->PSELSCK  = CONFIG_SPI_0_NRF5_GPIO_SCK;

  cfg->config_func();

  /* Set up the synchronous call mechanism */
	device_sync_call_init(&priv_data->sync);

  SYS_LOG_DBG("nRF5 SPI Slave driver initialized on device: %p\n", dev);

	return 0;
}

/* system bindings */
#ifdef CONFIG_SPI_0

#ifdef CONFIG_SOC_SERIES_NRF52X
#define NRF5_SPIS_0_IRQ NRF52_IRQ_SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn
#elif CONFIG_SOC_SERIES_NRF51X
#define NRF5_SPIS_0_IRQ NRF51_IRQ_SPI0_TWI0_IRQn
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
