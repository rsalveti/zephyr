.. _nucleo_f446re:

ST NUCLEO-F446RE
################

Overview
********

The nucleo_f446re configuration is used by Zephyr applications
that run on the ST Nucleo open development platform (NUCLEO-F446RE).
It provides support for an ARM Cortex-M4 CPU and the following devices:

* Nested Vectored Interrupt Controller (NVIC)

* System Tick System Clock (SYSTICK)

* Virtual Com Port over USB

Supported Boards
****************

The nucleo_f446re board configuration has been tested to run on the
ST Nucleo open development platform.  The physical characteristics of
this board (including pin names, jumper settings, ...) can be found below.
No claims are made about its suitability for use with any other hardware system.

Pin Names
=========

**LED**

* LD2 = PA5
* LD3 PWR = power

**Push buttons**

* B1 USER = PC13
* B2 RESET = NRST

**Arduino Headers**

***CN6 power***
*1 NC =
*2 IOREF = 3.3V Ref
*3 RESET = NRST RESET
*4 +3V3 = 3.3V input/output
*5 +5V = 5V output
*6 GND = Ground
*7 GND = Ground
*8 VIN = Power input

***CN8 analog***
*1 A0 = PA0 - ADC123_IN0
*2 A1 = PA1 - ADC123_IN1
*3 A2 = PA4 - ADC12_IN4
*4 A3 = PB0 - ADC12_IN8
*5 A4 = PC1 - or PB9 ADC123_IN11 (PC1) or I2C1_SDA (PB9)
*6 A5 = PC0 - or PB8(1) ADC123_IN10 (PC0) or I2C1_SCL (PB8)

***CN5 digital***
*1 D8 = PA9 -
*2 D9 = PC7 - TIM8_CH2
*3 D10 = PB6 - TIM4_CH1 || SPI1_CS
*4 D11 = PA7 - TIM14_CH1 || SPI1_MOSI
*5 D12 = PA6 - SPI1_MISO
*6 D13 = PA5 - SPI1_SCK
*7 GND = Ground
*8 AREF = AVDD
*9 D14 = PB9 - I2C1_SDA
*10 D15 = PB8 - I2C1_SCL


Jumpers & Switches
==================

The Zephyr kernel uses the NUCLEO-F446RE default switch and jumper settings.

The default jumper/switch settings for the ST NUCLEO-F446RE are:

+---------------+------------+---------------+
| Switch Number | Switch     | ON Switch OFF |
+===============+============+===============+
|  JP6 (IDD)    |            |      x        |
+---------------+------------+---------------+
