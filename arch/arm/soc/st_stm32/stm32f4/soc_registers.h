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
 * @file SoC configuration macros for the ST STM32F4xx family processors.
 *
 * Refer to the following ST documentation for definitions of the registers
 *
 *
 * Advanced lines:
 * RM0386 Reference manual STM32F469xx and STM32F479xx advanced ARM®-based 32-bit MCUs
 *
 * Foundation lines:
 * RM0390 Reference manual STM32F446xx advanced ARM® -based 32-bit MCUs
 *
 * Access lines:
 * RM0090 Reference manual STM32F405/415, STM32F407/417, STM32F427/437 and
 * STM32F429/439 advanced ARM® -based 32-bit MCUs
 *
 */

#ifndef _ST_STM32F4XX_SOC_REGS_H_
#define _ST_STM32F4XX_SOC_REGS_H_

struct __syscfg {
	uint32_t	memrmp;		/* offset 0x00 */
	uint32_t	pmc;		/* offset 0x04 */
	uint32_t	exticr[4];	/* offset 0x08 */
	uint32_t	reserved[2];	/* offset 0x18 */
	uint32_t	cmpcr;		/* offset 0x20 */
};

struct __rcc {
	uint32_t	cr;		/* offset 0x00 */
	uint32_t	pllcfgr;	/* offset 0x04 */
	uint32_t	cfgr;		/* offset 0x08 */
	uint32_t	cir;		/* offset 0x0C */
	uint32_t	ahb1rstr;	/* offset 0x10 */
	uint32_t	ahb2rstr;	/* offset 0x14 */
	uint32_t	ahb3rstr;	/* offset 0x18 */
	uint32_t	reserved_0;	/* offset 0x1C */
	uint32_t	apb1rstr;	/* offset 0x20 */
	uint32_t	apb2rstr;	/* offset 0x24 */
	uint32_t	reserved_1[2];	/* offset 0x28+0x2C */
	uint32_t	ahb1enr;	/* offset 0x30 */
	uint32_t	ahb2enr;	/* offset 0x34 */
	uint32_t	ahb3enr;	/* offset 0x38 */
	uint32_t	reserved_2;	/* offset 0x3C */
	uint32_t	apb1enr;	/* offset 0x40 */
	uint32_t	apb2enr;	/* offset 0x44 */
	uint32_t	reserved_3[2];	/* offset 0x48+0x4C */
};

/* GPIOx Controller Registers */
struct __gpio {
	uint32_t	mode;		/* offset 0x00 */
	uint32_t	otype;		/* offset 0x04 */
	uint32_t	ospeed;		/* offset 0x08 */
	uint32_t	pupd;		/* offset 0x0C */
	uint32_t	id;		/* offset 0x10 */
	uint32_t	od;		/* offset 0x14 */
	uint32_t	bsr;		/* offset 0x18 */
	uint32_t	lck;		/* offset 0x1C */
	uint32_t	afr[2];		/* offset 0x20 */
};

/* Advanced-control timers */
struct __tim {
	uint16_t	cr_1;
	uint16_t	reserved_0;
	uint16_t	cr_2;
	uint16_t	reserved_1;
	uint16_t	smcr;
	uint16_t	reserved_2;
	uint16_t	dier;
	uint16_t	reserved_3;
	uint16_t	sr;
	uint16_t	reserved_4;
	uint16_t	egr;
	uint16_t	reserved_5;
	uint16_t	ccmr_1;
	uint16_t	reserved_6;
	uint16_t	ccmr_2;
	uint16_t	reserved_7;
	uint16_t	ccer;
	uint16_t	reserved_8;
	uint16_t	cnt;
	uint16_t	reserved_9;
	uint16_t	psc;
	uint16_t	reserved_10;
	uint16_t	arr;
	uint16_t	reserved_11;
	uint16_t	rcr;
	uint16_t	reserved_12;
	uint32_t	ccr1;
	uint16_t	reserved_13;
	uint32_t	ccr2;
	uint16_t	reserved_14;
	uint32_t	ccr3;
	uint16_t	reserved_15;
	uint32_t	ccr4;
	uint16_t	reserved_16;
	uint16_t	bdtr;
	uint16_t	reserved_17;
	uint16_t	dcr;
	uint16_t	reserved_18;
	uint16_t	dmar;
	uint16_t	reserved_19;
};

#endif /* _ST_STM32F4XX_SOC_REGS_H_ */
