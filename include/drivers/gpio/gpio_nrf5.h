/* gpio_nrf5.h - Nordic Semiconductor NRF5 GPIO controller driver utilities */

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

#ifndef __GPIO_NRF5_H__
#define __GPIO_NRF5_H__

#define GPIO_SENSE_DISABLE    (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
#define GPIO_SENSE_ENABLE     (GPIO_PIN_CNF_SENSE_Enabled << GPIO_PIN_CNF_SENSE_Pos)
#define GPIO_PULL_DISABLE     (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
#define GPIO_PULL_DOWN        (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos)
#define GPIO_PULL_UP          (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos)
#define GPIO_INPUT_CONNECT    (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
#define GPIO_INPUT_DISCONNECT (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
#define GPIO_DIR_INPUT        (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)
#define GPIO_DIR_OUTPUT       (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos)

#define GPIO_DRIVE_S0S1 (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
#define GPIO_DRIVE_H0S1 (GPIO_PIN_CNF_DRIVE_H0S1 << GPIO_PIN_CNF_DRIVE_Pos)
#define GPIO_DRIVE_S0H1 (GPIO_PIN_CNF_DRIVE_S0H1 << GPIO_PIN_CNF_DRIVE_Pos)
#define GPIO_DRIVE_H0H1 (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos)
#define GPIO_DRIVE_D0S1 (GPIO_PIN_CNF_DRIVE_D0S1 << GPIO_PIN_CNF_DRIVE_Pos)
#define GPIO_DRIVE_D0H1 (GPIO_PIN_CNF_DRIVE_D0H1 << GPIO_PIN_CNF_DRIVE_Pos)
#define GPIO_DRIVE_S0D1 (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos)
#define GPIO_DRIVE_H0D1 (GPIO_PIN_CNF_DRIVE_H0D1 << GPIO_PIN_CNF_DRIVE_Pos)

#endif /* __GPIO_NRF5_H__ */
