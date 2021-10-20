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
#include "mgos_pcf857x.h"
#include "mgos_timers.h"

#ifdef __cplusplus
extern "C" {
#endif

// PCF857X I2C address
#define MGOS_PCF857X_I2C_ADDR    (0x20)

struct mgos_pcf857x_cb {
  mgos_gpio_int_handler_f fn;
  void *                  fn_arg;
  double                  last;
  int                     debounce_ms;
  bool                    enabled;
  bool                    firing;
  enum mgos_gpio_int_mode mode;
};

struct mgos_pcf857x {
  struct mgos_i2c *      i2c;
  uint8_t                i2caddr;

  uint16_t               _state;                     // Last read from device
  uint16_t               _io;                        // each bit signals pin in INPUT (1) or OUTPUT (0)
  uint8_t                num_gpios;                  // PCF8574: 8; PCF8575: 16
  int                    int_gpio;                   // Interrupt pin from device

  struct mgos_pcf857x_cb cb[16];
};

struct mgos_pcf857x_gpio_blink_state {
  int pin;
  struct mgos_pcf857x *dev;
  unsigned int on_ms;
  unsigned int off_ms;
  mgos_timer_id timer_id;
};

/* Callback used by the blink function */
void mgos_pcf857x_gpio_blink_cb(void *arg);

/* Mongoose OS initializer */
bool mgos_pcf857x_i2c_init(void);

#ifdef __cplusplus
}
#endif
