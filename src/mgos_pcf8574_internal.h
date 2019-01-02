/*
 * Copyright 2019 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once
#include "mgos_pcf8574.h"

#ifdef __cplusplus
extern "C" {
#endif

// PCF8574 I2C address
#define MGOS_PCF8574_I2C_ADDR      (0x20)
#define MGOS_PCF8574_PINS          (8)

// Registers
#define MGOS_PCF8574_REG_CONFIG    (0x00)

struct mgos_pcf8574 {
  struct mgos_i2c *       i2c;
  uint8_t                 i2caddr;

  uint8_t                 _read;                     // Last read from device
  uint8_t                 _write;                    // Last write to device
  uint8_t                 _io;                       // each bit signals pin in INPUT (1) or OUTPUT (0)
  int                     int_gpio;                  // Interrupt pin from device
  mgos_gpio_int_handler_f int_cb[MGOS_PCF8574_PINS]; // callback for {int,button}_handler()
};

/* Mongoose OS initializer */
bool mgos_pcf8574_i2c_init(void);

#ifdef __cplusplus
}
#endif
