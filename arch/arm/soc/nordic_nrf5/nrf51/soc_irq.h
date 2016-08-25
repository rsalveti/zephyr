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

/**
 * @file Interrupt numbers for NRF51 family processors.
 *
 * Based on Nordic MDK included header file: nrf51.h
 */


#ifndef _NRF51_SOC_IRQ_H_
#define _NRF51_SOC_IRQ_H_

#define NRF51_IRQ_POWER_CLOCK_IRQn			 0
#define NRF51_IRQ_RADIO_IRQn				 1
#define NRF51_IRQ_UART0_IRQn				 2
#define NRF51_IRQ_SPI0_TWI0_IRQn			 3
#define NRF51_IRQ_SPI1_TWI1_IRQn			 4
#define NRF51_IRQ_GPIOTE_IRQn				 6
#define NRF51_IRQ_ADC_IRQn				 7
#define NRF51_IRQ_TIMER0_IRQn				 8
#define NRF51_IRQ_TIMER1_IRQn				 9
#define NRF51_IRQ_TIMER2_IRQn				 10
#define NRF51_IRQ_RTC0_IRQn				 11
#define NRF51_IRQ_TEMP_IRQn				 12
#define NRF51_IRQ_RNG_IRQn				 13
#define NRF51_IRQ_ECB_IRQn				 14
#define NRF51_IRQ_CCM_AAR_IRQn				 15
#define NRF51_IRQ_WDT_IRQn				 16
#define NRF51_IRQ_RTC1_IRQn				 17
#define NRF51_IRQ_QDEC_IRQn				 18
#define NRF51_IRQ_LPCOMP_IRQn				 19
#define NRF51_IRQ_SWI0_IRQn				 20
#define NRF51_IRQ_SWI1_IRQn				 21
#define NRF51_IRQ_SWI2_IRQn				 22
#define NRF51_IRQ_SWI3_IRQn				 23
#define NRF51_IRQ_SWI4_IRQn				 24
#define NRF51_IRQ_SWI5_IRQn				 25

#endif /* _NRF51_SOC_IRQ_H_ */
