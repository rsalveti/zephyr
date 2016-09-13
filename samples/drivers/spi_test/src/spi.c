/* spi.c - SPI test source file */

/*
 * Copyright (c) 2015 Intel Corporation.
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

#include <zephyr.h>

#define SYS_LOG_LEVEL SYS_LOG_SPI_LEVEL
#include <misc/sys_log.h>
#include <misc/printk.h>

#include <string.h>
#include <spi.h>
#include <sys_clock.h>
//#define SPI_DRV_NAME "SPI_0"

#ifdef CONFIG_SPI_INTEL

#include <spi/spi_intel.h>
#if defined(CONFIG_SPI_1)
#define SPI_DRV_NAME "SPI_1"
#endif
#define SPI_SLAVE 0

#elif defined(CONFIG_SPI_DW)

#define SPI_MAX_CLK_FREQ_250KHZ 128
#define SPI_SLAVE 2

#elif defined(CONFIG_SPI_QMSI)

#define SPI_MAX_CLK_FREQ_250KHZ 128
#define SPI_SLAVE 1

#elif defined(CONFIG_SPI_STM32)

#include <spi/spi_stm32.h>
/* We don't process max freq yet */
#define SPI_MAX_CLK_FREQ_250KHZ 128
#define SPI_SLAVE 0
#define SOC_MASTER_MODE SPI_STM32_MASTER_MODE
#define SOC_SLAVE_MODE SPI_STM32_SLAVE_MODE
#if defined(CONFIG_BOARD_NUCLEO_F401RE)
#define SPI_DRV_NAME CONFIG_SPI_0_NAME
#elif defined(CONFIG_BOARD_CARBON)
#define SPI_DRV_NAME CONFIG_SPI_1_NAME
#endif

#elif defined(CONFIG_SPI_K64)

#define SPI_DRV_NAME "SPI_0"
#define SPI_MAX_CLK_FREQ_250KHZ 250000
#define SPI_SLAVE 0

#elif defined(CONFIG_SPI_NRF5)

#include <spi/spi_nrf5.h>
/* We don't process max freq yet */
#define SPI_MAX_CLK_FREQ_250KHZ 128
#define SPI_SLAVE 0
#define SOC_SLAVE_MODE SPI_NRF5_OP_MODE_SLAVE
#if defined(CONFIG_BOARD_NRF52_NITROGEN)
#define SPI_DRV_NAME CONFIG_SPI_0_NAME
#elif defined(CONFIG_BOARD_NRF51_BLENANO)
#define SPI_DRV_NAME CONFIG_SPI_0_NAME
#endif

#endif

/* Frame format: 0 = Motorola, 1 = TI */
#define SET_FRAME_FMT 0
/* Hard-code size to 8 bits */
#define SET_FRAME_SIZE SPI_WORD(8)

#if CONFIG_SPI_SLAVE == 0
#define OP_MODE SOC_MASTER_MODE
#elif CONFIG_SPI_SLAVE == 1
#define OP_MODE SOC_SLAVE_MODE
#endif

#if SET_FRAME_FMT == 0
#define FRAME_FMT SPI_STM32_FRAME_MOTOROLA
#elif SET_FRAME_FMT == 1
#define FRAME_FMT SPI_STM32_FRAME_TI
#endif

//unsigned char m_wbuf1[] = "Hello, Carbon";
//unsigned char s_wbuf1[] = "Hello, Nucleo";
//unsigned char m_wbuf2[] = "How do you do?";
//unsigned char s_wbuf2[] = "I'm fine, thank you";
unsigned char m_wbuf1[] = "MMMMMM";
unsigned char s_wbuf1[] = "SSSSSS";
unsigned char rbuf[16]    = { [0 ... 15] = 0xFF };

static void print_buf_hex(unsigned char *sent, unsigned char *recv, uint32_t len)
{
  uint32_t i;

  printk("\t\tSent    -->     Received\n");
  printk("\t\t--------------------\n");
	for (i = 0; i < len; i++) {
		printk("%u:\t0x%x [%c]\t--> [%c]\t0x%x\n", i, *sent, *sent, *recv, *recv);
		sent++;
    recv++;
	}

	SYS_LOG_DBG("\n");
}

struct spi_config spi_conf = {
#ifdef CONFIG_SPI_STM32
  //  .config = (FRAME_FMT | SET_FRAME_SIZE | OP_MODE | SPI_MODE_CPOL | SPI_MODE_CPHA),

#ifdef CONFIG_SPI_SLAVE /* Nucleo */
  .config = (SET_FRAME_SIZE | OP_MODE | SPI_STM32_SLAVE_HW_NO_OUTPUT ),
  //  .config = (SET_FRAME_SIZE | OP_MODE | SPI_STM32_SLAVE_HW_NO_OUTPUT | SPI_MODE_CPOL | SPI_MODE_CPHA ),
#else /* MASTER */ /* Carbon */
  .config = (SET_FRAME_SIZE | OP_MODE | SPI_STM32_SLAVE_HW_SS_OUTPUT),
  //  .config = (SET_FRAME_SIZE | OP_MODE | SPI_STM32_SLAVE_HW_SS_OUTPUT | SPI_MODE_CPOL | SPI_MODE_CPHA ),

#endif /* CONFIG_SPI_SLAVE */

#else /* Nitrogen */
  //  .config = (SET_FRAME_SIZE | SPI_MODE_CPOL | SPI_MODE_CPHA | OP_MODE),
  .config = (SET_FRAME_SIZE | OP_MODE),
#endif
	.max_sys_freq = SPI_MAX_CLK_FREQ_250KHZ,
};

static void _spi_show(struct spi_config *spi_conf)
{
	SYS_LOG_DBG("SPI Configuration: %x\n", spi_conf->config);
	SYS_LOG_DBG("\tbits per word: %u\n",
		    SPI_WORD_SIZE_GET(spi_conf->config));
	SYS_LOG_DBG("\tMode: %u\n", SPI_MODE(spi_conf->config));
	SYS_LOG_DBG("\tMax speed Hz: 0x%X\n", spi_conf->max_sys_freq);
#if defined(CONFIG_SPI_STM32)
	SYS_LOG_DBG("\tOperating mode: %s\n",
        SPI_STM32_OP_MODE_GET(spi_conf->config)? "slave": "master");
#endif
}

void main(void)
{
	struct device *spi;
  uint32_t i = 0;
  uint32_t sz_w, sz_r;
  int ret;

	SYS_LOG_DBG("==== SPI Test Application ====\n");

	spi = device_get_binding(SPI_DRV_NAME);
  if (!spi) {
    SYS_LOG_ERR("Cannot find device %s\n", SPI_DRV_NAME);
  }
	SYS_LOG_DBG("Starting...\n");

	spi_configure(spi, &spi_conf);
	spi_slave_select(spi, SPI_SLAVE);

	_spi_show(&spi_conf);

#if CONFIG_SPI_SLAVE == 0
  /* Master */
  while (1) {
    ++i;

    /* Stop after one iteration */
    if (i > 1)
      break;

    SYS_LOG_DBG("------------\n");
    SYS_LOG_DBG("Count: %i\n", i);
    SYS_LOG_DBG("------------\n");

    sz_r = sizeof(rbuf) / sizeof(rbuf[0]);

    sz_w = sizeof(m_wbuf1) / sizeof(m_wbuf1[0]);
    SYS_LOG_INF("spi_transceive: Text: %s\n", m_wbuf1);
    ret = spi_transceive(spi, m_wbuf1, sz_w, rbuf, sz_r);
    if (ret  < 0) {
      SYS_LOG_ERR("Error in spi_transcieve: %i\n", ret);
    }
    print_buf_hex(m_wbuf1, rbuf, (sz_r > sz_w)? sz_r: sz_w);
  }
#elif CONFIG_SPI_SLAVE == 1
  /* Slave */
  while (1) {
    ++i;

    SYS_LOG_DBG("------------\n");
    SYS_LOG_DBG("Count: %i\n", i);
    SYS_LOG_DBG("------------\n");

    sz_r = sizeof(rbuf) /  sizeof(rbuf[0]);

    sz_w = sizeof(s_wbuf1) /  sizeof(s_wbuf1[0]);
    SYS_LOG_INF("spi_transceive: Text: %s\n", s_wbuf1);
    ret = spi_transceive(spi, s_wbuf1, sz_w, rbuf, sz_r);
    if (ret  < 0) {
      SYS_LOG_ERR("Error in spi_transcieve: %i\n", ret);
    }
    print_buf_hex(s_wbuf1, rbuf, (sz_r > sz_w)? sz_r: sz_w);
  }
#endif /* CONFIG_SPI_SLAVE */
}
