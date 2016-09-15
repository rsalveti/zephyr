/* spi_nrf5_priv.h - Nordic Semiconductor NRF5 SPI driver private definitions */

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
 */

#ifndef __SPI_NRF5_PRIV_H__
#define __SPI_NRF5_PRIV_H__

#define ___cat(a,b)		a##b
#define __cat(a,b)		___cat(a,b)
#define __padding(__words)	struct { uint32_t __cat(__pad, __COUNTER__)[__words]; }

typedef void (*spi_nrf5_config_t)(void);

struct spi_nrf5_config {
	uint32_t base_addr;		/* base address of SPI module registers */
	spi_nrf5_config_t config_func;	/* IRQ configuration function pointer */
};

struct spi_nrf5_data {
	device_sync_call_t sync;	/* synchronisationn */
	uint8_t frame_sz;		/* frame/word size, in bits */
	const uint8_t *tx_buf;
	uint8_t *rx_buf;
	uint8_t tx_buf_len;
	uint8_t rx_buf_len;
	uint8_t trans_len;
	uint32_t transmitted;
	uint32_t received;
	uint8_t error;
};

struct spi_slave_nrf5 {
	__padding(9);
	uint32_t  TASKS_ACQUIRE;	/* Acquire SPI semaphore */
	uint32_t  TASKS_RELEASE;	/* Release SPI semaphore, enabling the SPI slave to acquire it */
	__padding(54);
	uint32_t  EVENTS_END;		/* Granted transaction completed */
	__padding(2);
	uint32_t  EVENTS_ENDRX;		/* End of RXD buffer reached */
	__padding(5);
	uint32_t  EVENTS_ACQUIRED;	/* Semaphore acquired */
	__padding(53);
	uint32_t  SHORTS;		/* Shortcut register */
	__padding(64);
	uint32_t  INTENSET;		/* Enable interrupt */
	uint32_t  INTENCLR;		/* Disable interrupt */
	__padding(61);
	uint32_t  SEMSTAT;		/* Semaphore status register */
	__padding(15);
	uint32_t  STATUS;		/* Status from last transaction */
	__padding(47);
	uint32_t  ENABLE;		/* Enable SPI slave */
	__padding(1);
	uint32_t  PSELSCK;		/* Pin select for SCK */
	uint32_t  PSELMISO;		/* Pin select for MISO signal  */
	uint32_t  PSELMOSI;		/* Pin select for MOSI signal  */
	uint32_t  PSELCSN;		/* Pin select for CSN signal*/
	__padding(7);
	uint32_t  RXDPTR;		/* RXD data pointer */
	uint32_t  RXDMAXCNT;		/* Maximum number of bytes in receive buffer  */
	uint32_t  RXDAMOUNT;		/* Number of bytes received in last granted transaction */
	__padding(1);
	uint32_t  TXDPTR;		/* TXD data pointer */
	uint32_t  TXDMAXCNT;		/* Maximum number of bytes in transmit buffer */
	uint32_t  TXDAMOUNT;		/* Number of bytes transmitted in last granted transaction */
	__padding(1);
	uint32_t  CONFIG;		/* Configuration register */
	__padding(1);
	uint32_t  DEF;			/* Default character - clocked out in case of an ignored transaction */
	__padding(24);
	uint32_t  ORC;			/* Over-read character */
};

/* Pin selection macros */
#define PIN_SELECT(_pin_, _en_) ((_pin_) | ((_en_) << 31))

#define SPI_NRF5_SHORTCUT_END_ACQUIRE (1 << 2)

/* Register fields */
#define SPIM_NRF52_ENABLE 7
#define SPIM_NRF51_ENABLE 1
#define SPIS_NRF5_ENABLE 2

#ifdef CONFIG_SOC_SERIES_NRF52X
	#define SPIM_NRF5_ENABLE SPIM_NRF52_ENABLE
#elif CONFIG_SOC_SERIES_NRF51X
	#define SPIM_NRF5_ENABLE SPIM_NRF51_ENABLE
#endif

#define SEM_FREE 0
#define SEM_CPU 1
#define SEM_SPIS 2
#define SEM_CPU_PENDING 3

#endif /* __SPI_NRF5_PRIV_H__ */
