/*
 * Copyright (c) 2016 Pawel Wodnicki 32bitmicro.com
 * Copyright (c) 2016 Intel Corporation.
 * Copyright (c) 2013-2015 Wind River Systems, Inc.
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

/**
 * @file
 * @brief System/hardware module for ST STM32 family processor
 *
 * This module provides routines to initialize and support board-level hardware
 * for the ST STM32 family processor.
 *
 * Clock configuration
 * AN3988 Application note Clock configuration tool for STM32F40xx/41xx/427x/437x microcontrollers
 * DocID 022298 Rev 2
 */

#include <nanokernel.h>
#include <device.h>
#include <init.h>
#include <soc.h>

#ifdef CONFIG_RUNTIME_NMI
extern void _NmiInit(void);
#define NMI_INIT() _NmiInit()
#else
#define NMI_INIT()
#endif

/**
 * @brief Setup various clocks on the SoC
 *
 * By default this configures the system to use the HSE.
 *
 * Assumption:
 * SLCK = 32.768kHz
 */
static void clock_init(void)
{
	uint32_t regtmp = 0;
#if defined(CONFIG_SOC_STM32_CLOCK_HSE)
	RCC->cr |= RCC_CR_HSEON;

	/*
	 * Time to spin until the HSE is ready.
	 */
	while (!(RCC->cr & RCC_CR_HSERDY)) {
		regtmp = RCC->cr & RCC_CR_HSERDY;
	}
#elif defined(CONFIG_SOC_STM32_CLOCK_HSI)
	RCC->cr |= RCC_CR_HSION;
	/*
	 * Time to spin until the HSI is ready.
	 */
	while (!(RCC->cr & RCC_CR_HSIRDY)) {
		regtmp = RCC->cr & RCC_CR_HSIRDY;
	}

#endif


	/*
	 * This will setup the following values:
	 * * HCLK = SYSCLK / 1
	 * * PCLK2 = HCLK  / 2
	 * * PCLK1 = HCLK  / 4
	 */
	RCC->cfgr |= (RCC_CFGR_HPRE_0 | RCC_CFGR_PPRE_2_2 | RCC_CFGR_PPRE_1_4);

	/*
	 * Now that the CLK is setup, configure the main PLL
	 */
#ifdef CONFIG_SOC_STM32F4XX
	RCC->pllcfgr = STM32F4XX_PLL_M |
			(STM32F4XX_PLL_N << 6) |
#if defined(CONFIG_SOC_STM32_CLOCK_HSE)
			(RCC_CFGR_PLLSRC_HSE) |
#elif defined(CONFIG_SOC_STM32_CLOCK_HSI)
			(RCC_CFGR_PLLSRC_HSI) |
#endif
			(((STM32F4XX_PLL_P >> 1) - 1) << 16) |
			(STM32F4XX_PLL_Q << 24) |
			(STM32F4XX_PLL_R << 28);
#endif
	/*
	 * If everything has gone right, at this point we can enable the PLL
	 */
	RCC->cr |= RCC_CR_PLLON;

	/*
	 * Now spin until the PLL is ready
	 */
	while ((RCC->cr & RCC_CR_PLLRDY) == 0) {
		/* do nothing */
	}

	/*
	 * The PLL is now ready! First clear out any possible values, then
	 * select it as the system clock source
	 */
	regtmp = RCC->cfgr;
	regtmp &= ~(RCC_CFGR_SW_MASK);
	regtmp |= RCC_CFGR_SW_PLL;
	RCC->cfgr = regtmp;

	/* Once again, spin until this change is ready */
	while ((RCC->cfgr & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_PLL) {
		/* do nothing */
	}
}

/**
 * @brief Perform basic hardware initialization at boot for STM32.
 *
 * This needs to be run from the very beginning.
 * So the init priority has to be 0 (zero).
 *
 * @return 0
 */
static int st_stm32f4_init(struct device *arg)
{
	uint32_t key;

	ARG_UNUSED(arg);

	key = irq_lock();

	/* Setup the vector table offset register (VTOR),
	 * which is located at the beginning of flash area.
	 */
	_scs_relocate_vector_table((void *)CONFIG_FLASH_BASE_ADDRESS);

	/* Clear all faults */
	_ScbMemFaultAllFaultsReset();
	_ScbBusFaultAllFaultsReset();
	_ScbUsageFaultAllFaultsReset();

	_ScbHardFaultAllFaultsReset();

	/* Setup master clock */
	clock_init();

	/*
	 * Install default handler that simply resets the CPU
	 * if configured in the kernel, NOP otherwise
	 */
	NMI_INIT();

	irq_unlock(key);

	return 0;
}

SYS_INIT(st_stm32f4_init, PRIMARY, 0);
