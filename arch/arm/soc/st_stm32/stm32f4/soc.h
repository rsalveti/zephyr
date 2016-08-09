/*
 * Copyright (c) 2016 Pawel Wodnicki 32bitmicro.com
 * Copyright (c) 2016 Intel Corporation.
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
 * @file SoC configuration macros for the ST STM32F4 family processors.
 *
 * Based on reference manual:
 * XXXX: TODO
 */

#ifndef _STM32F4X_SOC_H_
#define _STM32F4X_SOC_H_

/* peripherals start address */
#define PERIPH_BASE           0x40000000

#define APB1PERIPH_BASE        PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x10000)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x20000)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000)

/* UART */
#define USART1_ADDR           (APB2PERIPH_BASE + 0x1000)
#define USART2_ADDR           (APB1PERIPH_BASE + 0x4400)
#define USART6_ADDR           (APB2PERIPH_BASE + 0x1400)

/* Reset and Clock Control */
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800)

#define GPIO_REG_SIZE         0x400
#define GPIOA_BASE            AHB1PERIPH_BASE
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00)

/* base address for where GPIO registers start */
#define GPIO_PORTS_BASE       (GPIOA_BASE)

/* EXTI */
#define EXTI_BASE            (APB2PERIPH_BASE + 0x3C00)

/* Watchdog */
#define IWDG_BASE            (APB1PERIPH_BASE + 0x3000)
#define WWDG_BASE            (APB1PERIPH_BASE + 0x2C00)

/* RTC */
#define RTC_BASE            (APB1PERIPH_BASE + 0x2800)

/* FLASH */
#define FLASH_BASE           (AHB1PERIPH_BASE + 0x3C00)

/* TIMx timers */
#define TIM5_BASE		         (APB1PERIPH_BASE + 0x0C00)
#define TIM4_BASE		         (APB1PERIPH_BASE + 0x0800)
#define TIM3_BASE		         (APB1PERIPH_BASE + 0x0400)
#define TIM2_BASE		         APB1PERIPH_BASE



#ifndef _ASMLANGUAGE

#include <device.h>
#include <misc/util.h>
#include <drivers/rand32.h>

#if 0
/* IO pin functions */
enum stm32f10x_pin_config_mode {
	STM32F10X_PIN_CONFIG_BIAS_HIGH_IMPEDANCE,
	STM32F10X_PIN_CONFIG_BIAS_PULL_UP,
	STM32F10X_PIN_CONFIG_BIAS_PULL_DOWN,
	STM32F10X_PIN_CONFIG_ANALOG,
	STM32F10X_PIN_CONFIG_DRIVE_OPEN_DRAIN,
	STM32F10X_PIN_CONFIG_DRIVE_PUSH_PULL,
	STM32F10X_PIN_CONFIG_AF_PUSH_PULL,
	STM32F10X_PIN_CONFIG_AF_OPEN_DRAIN,
};
#endif /* 0 */

#include "soc_irq.h"

#endif /* !_ASMLANGUAGE */

#define HSE_VALUE		0x26000000
#define HSE_STARTUP_TIMEOUT	0x0500
#define HSI_VALUE		0x16000000

/*
 * @brief STM32 standard peripherals library version number v1.1.3
 */
#define STM32_STDPERIPH_VERSION (((0x01) << 24) | \
			     ((0x01) << 16) | \
			     ((0x03) << 8)  | \
			     ((0x00)))

/*
 * @brief RCC Clock Control Register (RCC_CR)
 */
#define RCC_CR_HSION			BIT(0)
#define RCC_CR_HSIRDY			BIT(1)
#define RCC_CR_HSITRIM_MASK		0x000000F8
#define RCC_CR_HSITRIM_DEFAULT		0x00000058
#define RCC_CR_HSICAL_MASK		0x0000FF00
#define RCC_CR_HSEON			BIT(16)
#define RCC_CR_HSERDY			BIT(17)
#define RCC_CR_HSEBYP			BIT(18)
#define RCC_CR_CSSON			BIT(19)
#define RCC_CR_PLLON			BIT(24)
#define RCC_CR_PLLRDY			BIT(25)
#define RCC_CR_PLLI2SON			BIT(26)
#define RCC_CR_PLLI2SRDY		BIT(27)

/*
 * @brief RCC PLL configuration register (RCC_PLLCFGR)
 */
#define RCC_PLLCFGR_PLLM0	BIT(0)
#define RCC_PLLCFGR_PLLM1	BIT(1)
#define RCC_PLLCFGR_PLLM2	BIT(2)
#define RCC_PLLCFGR_PLLM3	BIT(3)
#define RCC_PLLCFGR_PLLM4	BIT(4)
#define RCC_PLLCFGR_PLLM5	BIT(5)
#define RCC_PLLCFGR_PLLN0	BIT(6)
#define RCC_PLLCFGR_PLLN1	BIT(7)
#define RCC_PLLCFGR_PLLN2	BIT(8)
#define RCC_PLLCFGR_PLLN3	BIT(9)
#define RCC_PLLCFGR_PLLN4	BIT(10)
#define RCC_PLLCFGR_PLLN5	BIT(11)
#define RCC_PLLCFGR_PLLN6	BIT(12)
#define RCC_PLLCFGR_PLLN7	BIT(13)
#define RCC_PLLCFGR_PLLN8	BIT(14)
#define RCC_PLLCFGR_PLLP0	BIT(16)
#define RCC_PLLCFGR_PLLP1	BIT(17)
#define RCC_PLLCFGR_PLLSRC	BIT(22)
#define RCC_PLLCFGR_PLLQ0	BIT(24)
#define RCC_PLLCFGR_PLLQ1	BIT(25)
#define RCC_PLLCFGR_PLLQ2	BIT(26)
#define RCC_PLLCFGR_PLLQ3	BIT(27)

/*
 * @brief RCC_CFGR System Clock Switch flags
 */

#define RCC_CFGR_SW_MASK	0x00000003
#define RCC_CFGR_SW_HSI		0x00000000  /* set system clock = HSI */
#define RCC_CFGR_SW_HSE		0x00000001  /* set system clock = HSE */
#define RCC_CFGR_SW_PLL		0x00000002  /* set system clock = PLL */

#define RCC_CFGR_SWS_MASK	0x0000000C
#define RCC_CFGR_SWS_HSI	0x00000000  /* system clock = HSI */
#define RCC_CFGR_SWS_HSE	0x00000004  /* system clock = HSE */
#define RCC_CFGR_SWS_PLL	0x00000008  /* system clock = PLL */

#define RCC_CFGR_HPRE_MASK	0x000000F0
#define RCC_CFGR_HPRE_0		0x00000000  /* SYSCLK not divided    */
#define RCC_CFGR_HPRE_2		0x00000080  /* SYSCLK divided by 2   */
#define RCC_CFGR_HPRE_4		0x00000090  /* SYSCLK divided by 4   */
#define RCC_CFGR_HPRE_8		0x000000A0  /* SYSCLK divided by 8   */
#define RCC_CFGR_HPRE_16	0x000000B0  /* SYSCLK divided by 16  */
#define RCC_CFGR_HPRE_64	0x000000C0  /* SYSCLK divided by 64  */
#define RCC_CFGR_HPRE_128	0x000000D0  /* SYSCLK divided by 128 */
#define RCC_CFGR_HPRE_256	0x000000E0  /* SYSCLK divided by 256 */
#define RCC_CFGR_HPRE_512	0x000000F0  /* SYSCLK divided by 512 */

#define RCC_CFGR_PPRE_1_MASK	0x00001C00
#define RCC_CFGR_PPRE_1_0	0x00000000
#define RCC_CFGR_PPRE_1_2	0x00001000
#define RCC_CFGR_PPRE_1_4	0x00001400
#define RCC_CFGR_PPRE_1_8	0x00001800
#define RCC_CFGR_PPRE_1_16	0x00001C00

#define RCC_CFGR_PPRE_2_MASK	0x0000E000
#define RCC_CFGR_PPRE_2_0	0x00000000
#define RCC_CFGR_PPRE_2_2	0x00008000
#define RCC_CFGR_PPRE_2_4	0x0000A000
#define RCC_CFGR_PPRE_2_8	0x0000C000
#define RCC_CFGR_PPRE_2_16	0x0000E000

#define RCC_CFGR_RTCPRE_MASK	0x001F0000

#define RCC_CFGR_MCO_1_MASK	0x00600000
#define RCC_CFGR_MCO_1_HSI	0x00000000
#define RCC_CFGR_MCO_1_LSE	0x00200000
#define RCC_CFGR_MCO_1_HSE	0x00400000
#define RCC_CFGR_MCO_1_PLL	0x00600000

#define RCC_CFGR_I2SSRC		0x00800000
#define RCC_CFGR_PLLSRC_HSI	0x00000000
#define RCC_CFGR_PLLSRC_HSE	0x00800000

#define RCC_CFGR_MCO_1_PRE	0x07000000
#define RCC_CFGR_MCO_1_PRE_0	0x00000000
#define RCC_CFGR_MCO_1_PRE_2	0x04000000
#define RCC_CFGR_MCO_1_PRE_3	0x05000000
#define RCC_CFGR_MCO_1_PRE_4	0x06000000
#define RCC_CFGR_MCO_1_PRE_5	0x07000000

#define RCC_CFGR_MCO_2_PRE	0x38000000
#define RCC_CFGR_MCO_2_PRE_0	0x00000000
#define RCC_CFGR_MCO_2_PRE_2	0x20000000
#define RCC_CFGR_MCO_2_PRE_3	0x28000000
#define RCC_CFGR_MCO_2_PRE_4	0x30000000
#define RCC_CFGR_MCO_2_PRE_5	0x38000000

#define RCC_CFGR_MCO_2_MASK	0xC0000000
#define RCC_CFGR_MCO_2_SYSCLK	0x00000000
#define RCC_CFGR_MCO_2_PLLI2S	0x40000000
#define RCC_CFGR_MCO_2_HSE	0x80000000
#define RCC_CFGR_MCO_2_PLL	0xC0000000

/*
 * @brief RCC AHB1 peripheral reset register (RCC_AHB1RSTR)
 */
#define RCC_AHB1RSTR_GPIOA_RST		BIT(0)
#define RCC_AHB1RSTR_GPIOB_RST		BIT(1)
#define RCC_AHB1RSTR_GPIOC_RST		BIT(2)
#define RCC_AHB1RSTR_GPIOD_RST		BIT(3)
#define RCC_AHB1RSTR_GPIOE_RST		BIT(4)
#define RCC_AHB1RSTR_GPIOF_RST		BIT(5)
#define RCC_AHB1RSTR_GPIOG_RST		BIT(6)
#define RCC_AHB1RSTR_GPIOH_RST		BIT(7)
#define RCC_AHB1RSTR_GPIOI_RST		BIT(8)
#define RCC_AHB1RSTR_CRCRST		BIT(12)
#define RCC_AHB1RSTR_DMA1RST		BIT(21)
#define RCC_AHB1RSTR_DMA2RST		BIT(22)
#define RCC_AHB1RSTR_ETHMACRST		BIT(25)
#define RCC_AHB1RSTR_OTGHSRST		BIT(29)

/*
 * @brief RCC AHB2 peripheral reset register (RCC_AHB2RSTR)
 */
#define RCC_AHB2RSTR_DCMIRST		BIT(0)
#define RCC_AHB2RSTR_CRYPRST		BIT(4)
#define RCC_AHB2RSTR_HASHRST		BIT(5)
#define RCC_AHB2RSTR_RNGRST		BIT(6)
#define RCC_AHB2RSTR_OTGFSRST		BIT(7)

/*
 * @brief RCC AHB3 periperal reset register (RCC_AHB3RSTR)
 */
#define RCC_AHB3RSTR_FSMCRST		BIT(0)

/*
 * @brief RCC APB1 peripheral reset register (RCC)APB1RSTR)
 */
#define RCC_APB1RSTR_TIM2RST		BIT(0)
#define RCC_APB1RSTR_TIM3RST		BIT(1)
#define RCC_APB1RSTR_TIM4RST		BIT(2)
#define RCC_APB1RSTR_TIM5RST		BIT(3)
#define RCC_APB1RSTR_TIM6RST		BIT(4)
#define RCC_APB1RSTR_TIM7RST		BIT(5)
#define RCC_APB1RSTR_TIM12RST		BIT(6)
#define RCC_APB1RSTR_TIM13RST		BIT(7)
#define RCC_APB1RSTR_TIM14RST		BIT(8)
#define RCC_APB1RSTR_WWDGRST		BIT(11)
#define RCC_APB1RSTR_SPI2RST		BIT(14)
#define RCC_APB1RSTR_SPI3RST		BIT(15)
#define RCC_APB1RSTR_UART2RST		BIT(17)
#define RCC_APB1RSTR_UART3RST		BIT(18)
#define RCC_APB1RSTR_UART4RST		BIT(19)
#define RCC_APB1RSTR_UART5RST		BIT(20)
#define RCC_APB1RSTR_I2C1RST		BIT(21)
#define RCC_APB1RSTR_I2C2RST		BIT(22)
#define RCC_APB1RSTR_I2C3RST		BIT(23)
#define RCC_APB1RSTR_CAN1RST		BIT(25)
#define RCC_APB1RSTR_CAN2RST		BIT(26)
#define RCC_APB1RSTR_PWRRST		BIT(28)
#define RCC_APB1RSTR_DACRST		BIT(29)

/*
 * @brief RCC APB2 peripheral reset register (RCC_APB2RSTR)
 */
#define RCC_APB2RSTR_TIM1RST		BIT(0)
#define RCC_APB2RSTR_TIM8RST		BIT(1)
#define RCC_APB2RSTR_USART1RST		BIT(4)
#define RCC_APB2RSTR_USART6RST		BIT(5)
#define RCC_APB2RSTR_ADCRST		BIT(8)
#define RCC_APB2RSTR_SDIORST		BIT(11)
#define RCC_APB2RSTR_SPI1RST		BIT(12)
#define RCC_APB2RSTR_SYSCFGRST		BIT(14)
#define RCC_APB2RSTR_TIM9RST		BIT(16)
#define RCC_APB2RSTR_TIM10RST		BIT(17)
#define RCC_APB2RSTR_TIM11RST		BIT(18)

/*
 * @brief RCC AHB1 peripheral clock register (RCC_AHB1ENR)
 */
#define RCC_AHB1ENR_GPIOAEN		BIT(0)
#define RCC_AHB1ENR_GPIOBEN		BIT(1)
#define RCC_AHB1ENR_GPIOCEN		BIT(2)
#define RCC_AHB1ENR_GPIODEN		BIT(3)
#define RCC_AHB1ENR_GPIOEEN		BIT(4)
#define RCC_AHB1ENR_GPIOFEN		BIT(5)
#define RCC_AHB1ENR_GPIOGEN		BIT(6)
#define RCC_AHB1ENR_GPIOHEN		BIT(7)
#define RCC_AHB1ENR_GPIOIEN		BIT(8)
#define RCC_AHB1ENR_CRCEN		BIT(12)
#define RCC_AHB1ENR_BKPSRAMEN		BIT(18)
#define RCC_AHB1ENR_DMA1EN		BIT(21)
#define RCC_AHB1ENR_DMA2EN		BIT(22)
#define RCC_AHB1ENR_ETHMACEN		BIT(25)
#define RCC_AHB1ENR_ETHMACTXEN		BIT(26)
#define RCC_AHB1ENR_ETHMACRXEN		BIT(27)
#define RCC_AHB1ENR_ETHMACPTPEN		BIT(28)
#define RCC_AHB1ENR_OTGHSEN		BIT(29)
#define RCC_AHB1ENR_OTGHSULPIEN		BIT(30)

/*
 * @brief RCC AHB2 peripheral clock enable register (RCC_AHB2ENR)
 */
#define RCC_AHB2ENR_DCMIEN		BIT(0)
#define RCC_AHB2ENR_CRYPEN		BIT(4)
#define RCC_AHB2ENR_HASHEN		BIT(5)
#define RCC_AHB2ENR_RNGEN		BIT(6)
#define RCC_AHB2ENR_OTGFSEN		BIT(7)

/*
 * @brief RCC AHB3 peripheral clock enable register (RCC_AHB3ENR)
 */
#define RCC_AHB3ENR_FSMCEN		BIT(0)

/*
 * @brief RCC APB1 peripheral clock enable register (RCC_APB1ENR)
 */
#define RCC_APB1ENR_TIM2EN		BIT(0)
#define RCC_APB1ENR_TIM3EN		BIT(1)
#define RCC_APB1ENR_TIM4EN		BIT(2)
#define RCC_APB1ENR_TIM5EN		BIT(3)
#define RCC_APB1ENR_TIM6EN		BIT(4)
#define RCC_APB1ENR_TIM7EN		BIT(5)
#define RCC_APB1ENR_TIM12EN		BIT(6)
#define RCC_APB1ENR_TIM13EN		BIT(7)
#define RCC_APB1ENR_TIM14EN		BIT(8)
#define RCC_APB1ENR_WWDGEN		BIT(11)
#define RCC_APB1ENR_SPI2EN		BIT(14)
#define RCC_APB1ENR_SPI3EN		BIT(15)
#define RCC_APB1ENR_USART2EN		BIT(17)
#define RCC_APB1ENR_USART3EN		BIT(18)
#define RCC_APB1ENR_USART4EN		BIT(19)
#define RCC_APB1ENR_UART5EN		BIT(20)
#define RCC_APB1ENR_I2C1EN		BIT(21)
#define RCC_APB1ENR_I2C2EN		BIT(22)
#define RCC_APB1ENR_I2C3EN		BIT(23)
#define RCC_APB1ENR_CAN1EN		BIT(25)
#define RCC_APB1ENR_CAN2EN		BIT(26)
#define RCC_APB1ENR_PWREN		BIT(28)
#define RCC_APB1ENR_DACEN		BIT(29)


/* IRQ numbers pulled from ST DocID 026976 Rev 2
 * 10.2 External interrupt/event controller (EXTI)
 * Table 38. Vector table for STM32F446xx
 */
#define IRQ_WWDG		0	/* Window WatchDog	0x00000040 */
#define IRQ_PVD			1	/* PVD EXTI line	0x00000044 */
#define IRQ_TAMP_STAMP		2	/* Tamper + TimeStamp	0x00000048 */
#define IRQ_RTC_WKUP		3	/* RTC Wakeup		0x0000004C */
#define IRQ_FLASH		4	/* Flash global		0x00000050 */
#define IRQ_RCC			5	/* RCC global		0x00000054 */
#define IRQ_EXTI0		6	/* EXTI Line 0		0x00000058 */
#define IRQ_EXTI1		7	/* EXTI Line 1		0x0000005C */
#define IRQ_EXTI2		8	/* EXTI Line 2		0x00000060 */
#define IRQ_EXTI3		9	/* EXTI Line 3		0x00000064 */
#define IRQ_EXTI4		10	/* EXTI Line 4		0x00000068 */
#define IRQ_DMA1_S0		11	/* DMA1 Stream 0	0x0000006C */
#define IRQ_DMA1_S1		12	/* DMA1 Stream 1	0x00000070 */
#define IRQ_DMA1_S2		13	/* DMA1 Stream 2	0x00000074 */
#define IRQ_DMA1_S3		14	/* DMA1 Stream 3	0x00000078 */
#define IRQ_DMA1_S4		15	/* DMA1 Stream 4	0x0000007C */
#define IRQ_DMA1_S5		16	/* DMA1 Stream 5	0x00000080 */
#define IRQ_DMA1_S6		17	/* DMA1 Stream 6	0x00000084 */
#define IRQ_ADC			18	/* ADC1, ADC2, and ADC3	0x00000088 */
#define IRQ_CAN1_TX		19	/* CAN1 TX		0x0000008C */
#define IRQ_CAN1_RX0		20	/* CAN1 RX0		0x00000090 */
#define IRQ_CAN1_RX1		21	/* CAN1 RX1		0x00000094 */
#define IRQ_CAN1_SCE		22	/* CAN1 SCE		0x00000098 */
#define IRQ_EXTI9_5		23	/* EXTI Line[9:5]	0x0000009C */
#define IRQ_TIM1_BRK_TIM9	24	/* TIM1 break /TIM9	0x000000A0 */
#define IRQ_TIM1_UP_TIM10	25	/* TIM1 update/TIM10	0x000000A4 */
#define IRQ_TIM1_TRG_COM_TIM11	26	/* TIM1 trigger/TIM11	0x000000A8 */
#define IRQ_TIM1_CC		27	/* TIM1 Capture Compare	0x000000AC */
#define IRQ_TIM2		28	/* TIM2			0x000000B0 */
#define IRQ_TIM3		29	/* TIM3			0x000000B4 */
#define IRQ_TIM4		30	/* TIM4			0x000000B8 */
#define IRQ_I2C1_EV		31	/* I2C1 event		0x000000BC */
#define IRQ_I2C1_ER		32	/* I2C1 error		0x000000C0 */
#define IRQ_I2C2_EV		33	/* I2C2 event		0x000000C4 */
#define IRQ_I2C2_ER		34	/* I2C2 error		0x000000C8 */
#define IRQ_SPI1		35	/* SPI1			0x000000CC */
#define IRQ_SPI2		36	/* SPI2			0x000000D0 */
#define IRQ_USART1		37	/* USART1		0x000000D4 */
#define IRQ_USART2		38	/* USART2		0x000000D8 */
#define IRQ_USART3		39	/* USART3		0x000000DC */
#define IRQ_EXTI15_10		40	/* EXTI Line[15:10]	0x000000E0 */
#define IRQ_RTC_ALARM		41	/* RTC Alarms		0x000000E4 */
#define IRQ_OTG_FS_WKUP		42	/* USB OTG Wakeup	0x000000E8 */
#define IRQ_TIM8_BRK_TIM12	43	/* TIM8 break / TIM12	0x000000EC */
#define IRQ_TIM8_UP_TIM13	44	/* TIM8 update/ TIM13	0x000000F0 */
#define IRQ_TIM8_TRG_COM_TIM14	45	/* TIM8 trigger/ TIM14	0x000000F4 */
#define IRQ_TIM8_CC		46	/* TIM8 Capture Compare	0x000000F8 */
#define IRQ_DMA1_STREAM7	47	/* DMA1 Stream 7	0x000000FC */
#define IRQ_FSMC		48	/* FSMC			0x00000100 */
#define IRQ_SDIO		49	/* SDIO			0x00000104 */
#define IRQ_TIM5		50	/* TIM5			0x00000108 */
#define IRQ_SPI3		51	/* SPI3			0x0000010C */
#define IRQ_UART4		52	/* UART4		0x00000110 */
#define IRQ_UART5		53	/* UART5		0x00000114 */
#define IRQ_TIM6_DAC		54	/* TIM6 / DAC1+DAC2	0x00000118 */
					/* underrun errors		   */
#define IRQ_TIM7		55	/* TIM7			0x0000011C */
#define IRQ_DMA2_S0		56	/* DMA2 Stream 0	0x00000120 */
#define IRQ_DMA2_S1		57	/* DMA2 Stream 1	0x00000124 */
#define IRQ_DMA2_S2		58	/* DMA2 Stream 2	0x00000128 */
#define IRQ_DMA2_S3		59	/* DMA2 Stream 3	0x0000012C */
#define IRQ_DMA2_S4		60	/* DMA2 Stream 4	0x00000130 */
#define IRQ_ETH			61	/* Ethernet		0x00000134 */
#define IRQ_ETH_WKUP		62	/* Ethernet wakeup	0x00000138 */
#define IRQ_CAN2_TX		63	/* CAN2 TX		0x0000013C */
#define IRQ_CAN2_RX0		64	/* CAN2 RX0		0x00000140 */
#define IRQ_CAN2_RX1		65	/* CAN2 RX1		0x00000144 */
#define IRQ_CAN2_SCE		66	/* CAN2 SCE		0x00000148 */
#define IRQ_OTG_FS		67	/* USB OTG FS		0x0000014C */
#define IRQ_DMA2_S5		68	/* DMA2 Stream 5	0x00000150 */
#define IRQ_DMA2_S6		69	/* DMA2 Stream 6	0x00000154 */
#define IRQ_DMA2_S7		70	/* DMA2 Stream 7	0x00000158 */
#define IRQ_USART6		71	/* USART6		0x0000015C */
#define IRQ_I2C3_EV		72	/* I2C3 event		0x00000160 */
#define IRQ_I2C3_ER		73	/* I2C3 error		0x00000164 */
#define IRQ_OTG_HS_EP1_OUT	74	/* USB OTG HS End Pnt	0x00000168 */
#define IRQ_OTG_HS_EP1_IN	75	/* USB OTG HS End Pnt	0x0000016C */
#define IRQ_OTG_HS_WKUP		76	/* USB OTG HS wakeup	0x00000170 */
#define IRQ_OTG_HS		77	/* USB OTG HS		0x00000174 */
#define IRQ_DCMI		78	/* DCMI			0x00000178 */
#define IRQ_CRYP		79	/* Crypto		0x0000017C */
#define IRQ_HASH_RNG	80	/* Hash and RNG		0x00000180 */
#define IRQ_FPU			81	/* FPU				0x00000184 */
#define IRQ_SPI4		84	/* SPI4				0x00000190 */
#define IRQ_SAI1		87	/* SAI1				0x0000019C */
#define IRQ_SAI2		91	/* SAI2				0x000001AC */
#define IRQ_QUADSPI		92	/* QuadSPI			0x000001B0 */
#define IRQ_HDMI_CEC	93	/* HDMI-CEC			0x000001B4 */
#define IRQ_SPDIF_RX	94	/* SPDIF-Rx			0x000001B8 */
#define IRQ_FMPI2C1		95	/* FMPI2C1 event	0x000001BC */
#define IRQ_FMPI2C1_ERR	96	/* FMPI2C1 error	0x000001C0 */
/*
 * Pieces needed for setting up the clock
 */

/* STM32F4X */
/* PLL:VCO = (HSE_VALUE | HSI_VALUE) / PLL_M) * PLL_N */
#define STM32F4X_PLL_M			16
#define STM32F4X_PLL_N			336

/* SYSCLK = PLL:VCO / STM32F4X_PLL_P */
#define STM32F4X_PLL_P			2

/* CLK48 = PLL:VCO / STM32F4X_PLL_Q  */
/*  STM32F4X_PLL_Q */
#define STM32F4X_PLL_Q			7

/*  STM32F4X_PLL_R */
#define STM32F4X_PLL_R			2

/*
 * The following addresses all come from ST Document DocID 025644 Rev 3
 * Section 5. Memory mapping
 *
 */
#if 0
#define FMC_BASE        0xA0000000

#define QUADSPI_BASE    0xA0001000

#define DCMI_BASE		0x50050000

#define USB_OTG_FS_BASE	0x50000000

#define USB_OTG_HS_BASE	0x40040000

#define ETH_BASE		0x40028000

#define DMA2_BASE		0x40026400
#define DMA1_BASE		0x40026000

#define BKPSRAM			0x40024000
#define FLASH_INT_BASE	0x40023C00

#define CRC_BASE		0x40023000

#define PIOI_BASE		0x40022000
#define PIOH_BASE		0x40021C00
#define PIOG_BASE		0x40021800
#define PIOF_BASE		0x40021400
#define PIOE_BASE		0x40021000
#define PIOD_BASE		0x40020C00
#define PIOC_BASE		0x40020800
#define PIOB_BASE		0x40020400
#define PIOA_BASE		0x40020000

#define SAI2_BASE       0x40015C00
#define SAI1_BASE       0x40015800

#define TIM11_BASE		0x40014800
#define TIM10_BASE		0x40014400
#define TIM9_BASE		0x40014000

#define SYSCFG_BASE		0x40013800

#define SPI4_BASE		0x40013400
#define SPI1_BASE		0x40013000

#define SDIO_BASE		0x40012800

#define ADC3_BASE		0x40012200
#define ADC2_BASE		0x40012100
#define ADC1_BASE		0x40012000

#define USART6_BASE		0x40011400
#define USART1_BASE		0x40011000

#define TIM8_BASE		0x40010400
#define TIM1_BASE		0x40010000

#define DAC_BASE		0x40007400
#define PWR_BASE		0x40007000

#define HDMI_CEC_BASE   0x40006C00

#define CAN2_BASE		0x40006800
#define CAN1_BASE		0x40006400

#define I2C3_BASE		0x40005C00
#define I2C2_BASE		0x40005800
#define I2C1_BASE		0x40005400

#define SPDIF_RX_BASE	0x40004000

#define SPI3_BASE		0x40003C00
#define SPI2_BASE		0x40003800

//#define TIM14_BASE		0x40002000
//#define TIM12_BASE		0x40001C00
//#define TIM7_BASE		0x40001800
//#define TIM6_BASE		0x40001400

#endif /* 0 */

#ifndef _ASMLANGUAGE

#include <device.h>
#include <drivers/rand32.h>

#include "soc_registers.h"
#include "soc_irq.h"

/* uart configuration settings */
#define UART_IRQ_FLAGS 0

#define RCC		((volatile struct __rcc *)RCC_BASE)
#define SYSCFG	((volatile struct __syscfg *)SYSCFG_BASE)

/* PIO Registers struct */
#define __PIOA		((volatile struct __gpio *)PIOA_ADDR)
#define __PIOB		((volatile struct __gpio *)PIOB_ADDR)
#define __PIOC		((volatile struct __gpio *)PIOC_ADDR)
#define __PIOD		((volatile struct __gpio *)PIOD_ADDR)
#define __PIOE		((volatile struct __gpio *)PIOE_ADDR)
#define __PIOF		((volatile struct __gpio *)PIOF_ADDR)
#define __PIOG		((volatile struct __gpio *)PIOG_ADDR)
#define __PIOH		((volatile struct __gpio *)PIOH_ADDR)
#define __PIOI		((volatile struct __gpio *)PIOI_ADDR)

#endif /* !_ASMLANGUAGE */

#endif /* _STM32F4X_SOC_H_ */
