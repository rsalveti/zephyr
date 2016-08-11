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

#ifndef _STM32F4X_FLASH_REGISTERS_H_
#define _STM32F4X_FLASH_REGISTERS_H_

/**
 * @brief
 *
 * Based on reference manual:
 *
 * Chapter 3.4: Embedded Flash Memory
 */

enum {
	STM32F4X_FLASH_LATENCY_0 = 0x0,
	STM32F4X_FLASH_LATENCY_1 = 0x1,
	STM32F4X_FLASH_LATENCY_2 = 0x2,
	STM32F4X_FLASH_LATENCY_3 = 0x3,
	STM32F4X_FLASH_LATENCY_4 = 0x4,
	STM32F4X_FLASH_LATENCY_5 = 0x5,
};

union __flash_acr {
	uint32_t val;
	struct {
		uint32_t latency :4 __packed;
		uint32_t rsvd__4_7 :4 __packed;
		uint32_t prften :1 __packed;
		uint32_t icen :1 __packed;
		uint32_t dcen :1 __packed;
		uint32_t icrst :1 __packed;
		uint32_t dcrst :1 __packed;
		uint32_t rsvd__13_31 :19 __packed;
	} bit;
};

/* 3.8.7 Embedded flash registers */
struct stm32f4x_flash {
	union __flash_acr acr;
	uint32_t keyr;
	uint32_t optkeyr;
	uint32_t sr;
	uint32_t cr;
	uint32_t optcr;
};

#endif	/* _STM32F4X_FLASHREGISTERS_H_ */
