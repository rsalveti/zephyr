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
	#else
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

#define TOT_NUM 6
#define DISP_NUM 2

unsigned char rbuf[64]     = { [0 ... 63] = 0xFF };
unsigned char *m_wbuf[TOT_NUM] = {"Hello, nRF5x",
			      "How do you do?",
			      "Doing good, just punting BLE packets your way",
			      "....",
			      ".. Oh. Didn't realise. Sorry!",
			      "You could use your /REQ /RDY lines, you know"};
unsigned char *s_wbuf[TOT_NUM] = {"Hello, STM32F4",
			      "I'm fine, thank you. You?",
			      "Yeah, I noticed",
			      "Mind slowing down?",
			      "That's OK.",
			      "I didn't know, let me find out how"};

//unsigned char m_rbuf[32]    = { [0 ... 31] = 0xFF };
//unsigned char s_rbuf[20]    = { [0 ... 19] = 0xFF };

static void print_buf_hex(unsigned char *tx, unsigned char *rx)
{
	printk("Tx: \"%s\"\n", tx);
	printk("Rx: \"%s\"\n\n", rx);
}

struct spi_config spi_conf = {
#ifdef CONFIG_SPI_STM32
	//  .config = (FRAME_FMT | SET_FRAME_SIZE | OP_MODE | SPI_MODE_CPOL | SPI_MODE_CPHA),

#ifdef CONFIG_SPI_SLAVE 
	.config = (SET_FRAME_SIZE | OP_MODE | SPI_STM32_SLAVE_HW_NO_OUTPUT ),
#else /* MASTER */ /* Carbon STM32F4 */
	.config = (SET_FRAME_SIZE | OP_MODE | SPI_STM32_SLAVE_HW_SS_OUTPUT),
#endif /* CONFIG_SPI_SLAVE */

#else /* nRF5x */
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

	printk("==== SPI Test Application ====\n");

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
	for (i = 0; i < DISP_NUM; i++) {
		SYS_LOG_DBG("------------\n");
		SYS_LOG_DBG("Count: %i\n", i);
		SYS_LOG_DBG("------------\n");

		sz_r = ARRAY_SIZE(rbuf);
		sz_w = strlen(m_wbuf[i]) + 1;

		SYS_LOG_INF("spi_transceive: Text [%s], Tx [%u], Rx [%u]\n",
			    m_wbuf[i], sz_w, sz_r);
		ret = spi_transceive(spi, m_wbuf[i], sz_w, rbuf, sz_r);
		if (ret  < 0) {
			SYS_LOG_ERR("Error in spi_transcieve: %i\n", ret);
		}
		print_buf_hex(m_wbuf[i], rbuf);
	}
#elif CONFIG_SPI_SLAVE == 1
	/* Slave */
	for (i = 0; i < DISP_NUM; i++) {
		SYS_LOG_DBG("------------\n");
		SYS_LOG_DBG("Count: %i\n", i);
		SYS_LOG_DBG("------------\n");

		sz_r = ARRAY_SIZE(rbuf);
		sz_w = strlen(s_wbuf[i]) + 1;

		SYS_LOG_INF("spi_transceive: Text [%s], Tx [%u], Rx [%u]\n",
			    s_wbuf[i], sz_w, sz_r);
		ret = spi_transceive(spi, s_wbuf[i], sz_w, rbuf, sz_r);
		if (ret  < 0) {
			SYS_LOG_ERR("Error in spi_transcieve: %i\n", ret);
		}
		print_buf_hex(s_wbuf[i], rbuf);
	}
#endif /* CONFIG_SPI_SLAVE */
}
