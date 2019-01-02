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

#include "mgos_pcf8574_internal.h"

// Poor person's detect -- power on state reads back 0xff
static bool mgos_pcf8574_detect(struct mgos_i2c *i2c, uint8_t i2caddr) {
  uint8_t val;

  if (!mgos_i2c_read(i2c, i2caddr, &val, 1, true)) {
    return false;
  }
  return val == 0xff;
}

static void mgos_pcf8574_print(struct mgos_pcf8574 *dev) {
  uint8_t n;
  char    r[9], w[9], i[9];

  if (!dev) {
    return;
  }
  for (n = 0; n < 8; n++) {
    r[n] = (dev->_read & (1 << n)) ? '*' : '.';
    w[n] = (dev->_write & (1 << n)) ? '*' : '.';
    i[n] = (dev->_io & (1 << n)) ? 'I' : 'O';
  }
  r[8] = w[8] = i[8] = 0;
  LOG(LL_DEBUG, ("read=0x%02x %s; write=0x%02x %s; io=0x%02x %s", dev->_read, r, dev->_write, w, dev->_io, i));
}

static bool mgos_pcf8574_read(struct mgos_pcf8574 *dev) {
  uint8_t val;

  if (!dev) {
    return false;
  }
  if (!mgos_i2c_read(dev->i2c, dev->i2caddr, &val, 1, true)) {
    return false;
  }
  dev->_read = val;
  mgos_pcf8574_print(dev);
  return true;
}

static bool mgos_pcf8574_write(struct mgos_pcf8574 *dev) {
  if (!dev) {
    return false;
  }
  if (!mgos_i2c_read(dev->i2c, dev->i2caddr, &dev->_write, 1, true)) {
    return false;
  }

  mgos_pcf8574_print(dev);
  return true;
}

static void mgos_pcf8574_irq(int pin, void *arg) {
  struct mgos_pcf8574 *dev = (struct mgos_pcf8574 *)arg;
  uint8_t prev_read;

  if (!dev) {
    return;
  }
  if (dev->int_gpio != pin) {
    return;
  }
  LOG(LL_INFO, ("Interrupt!"));
  mgos_pcf8574_print(dev);
  prev_read = dev->_read;
  mgos_pcf8574_read(dev);

  // TODO(pim): Look for bits in prev_read that differ from dev->_read and issue callbacks
  if (prev_read == dev->_read) {
    return;
  }
  return;
}

struct mgos_pcf8574 *mgos_pcf8574_create(struct mgos_i2c *i2c, uint8_t i2caddr, int int_gpio) {
  struct mgos_pcf8574 *dev = NULL;

  if (!i2c) {
    return NULL;
  }

  if (!mgos_pcf8574_detect(i2c, i2caddr)) {
    LOG(LL_ERROR, ("I2C 0x%02x is not a PCF8574", i2caddr));
    return NULL;
  }

  dev = calloc(1, sizeof(struct mgos_pcf8574));
  if (!dev) {
    return NULL;
  }

  memset(dev, 0, sizeof(struct mgos_pcf8574));
  dev->i2caddr  = i2caddr;
  dev->i2c      = i2c;
  dev->_write   = 0xff; // Set all pins HIGH
  dev->int_gpio = int_gpio;
  if (!mgos_pcf8574_write(dev)) {
    free(dev);
    return NULL;
  }
  if (dev->int_gpio != -1) {
    mgos_gpio_set_int_handler(dev->int_gpio, MGOS_GPIO_INT_EDGE_NEG, mgos_pcf8574_irq, dev);
    mgos_gpio_enable_int(dev->int_gpio);
  }
  LOG(LL_INFO, ("PCF8574 initialized at I2C 0x%02x", dev->i2caddr));
  return dev;
}

bool mgos_pcf8574_destroy(struct mgos_pcf8574 **dev) {
  if (!*dev) {
    return false;
  }

  free(*dev);
  *dev = NULL;
  return true;
}

bool mgos_pcf8574_gpio_set_mode(struct mgos_pcf8574 *dev, int pin, enum mgos_gpio_mode mode) {
  if (!dev || pin < 0 || pin >= MGOS_PCF8574_PINS) {
    return false;
  }
  if (mode == MGOS_GPIO_MODE_INPUT) {
    dev->_io |= (1 << pin);
  } else {
    dev->_io &= ~(1 << pin);
  }
  return true;
}

bool mgos_pcf8574_gpio_setup_output(struct mgos_pcf8574 *dev, int pin, bool level) {
  if (!dev || pin < 0 || pin >= MGOS_PCF8574_PINS) {
    return false;
  }
  mgos_pcf8574_gpio_set_mode(dev, pin, MGOS_GPIO_MODE_OUTPUT);
  mgos_pcf8574_gpio_write(dev, pin, level);
  return true;
}

bool mgos_pcf8574_gpio_read(struct mgos_pcf8574 *dev, int pin) {
  if (!dev || pin < 0 || pin >= MGOS_PCF8574_PINS) {
    return false;
  }
  mgos_pcf8574_read(dev);
  return (dev->_read & (1 << pin)) > 0;
}

void mgos_pcf8574_gpio_write(struct mgos_pcf8574 *dev, int pin, bool level) {
  if (!dev || pin < 0 || pin >= MGOS_PCF8574_PINS) {
    return;
  }
  if (level) {
    dev->_write |= (1 << pin);
  } else {
    dev->_write &= ~(1 << pin);
  }
  mgos_pcf8574_write(dev);
  return;
}

bool mgos_pcf8574_gpio_toggle(struct mgos_pcf8574 *dev, int pin) {
  if (!dev || pin < 0 || pin >= MGOS_PCF8574_PINS) {
    return false;
  }

  dev->_write ^= (1 << pin);
  mgos_pcf8574_write(dev);
  return (dev->_write & (1 << pin)) > 0;
}

bool mgos_pcf8574_gpio_set_int_handler(struct mgos_pcf8574 *dev, int pin, enum mgos_gpio_int_mode mode, mgos_gpio_int_handler_f cb, void *arg) {
  if (!dev || pin < 0 || pin >= MGOS_PCF8574_PINS) {
    return false;
  }
  LOG(LL_ERROR, ("Not implemented yet"));
  return false;
  (void) mode;
  (void) cb;
  (void) arg;
}

bool mgos_pcf8574_gpio_enable_int(struct mgos_pcf8574 *dev, int pin) {
  if (!dev || pin < 0 || pin >= MGOS_PCF8574_PINS) {
    return false;
  }
  LOG(LL_ERROR, ("Not implemented yet"));
  return false;
}

bool mgos_pcf8574_gpio_disable_int(struct mgos_pcf8574 *dev, int pin) {
  if (!dev || pin < 0 || pin >= MGOS_PCF8574_PINS) {
    return false;
  }
  LOG(LL_ERROR, ("Not implemented yet"));
  return false;
}

void mgos_pcf8574_gpio_clear_int(struct mgos_pcf8574 *dev, int pin) {
  if (!dev || pin < 0 || pin >= MGOS_PCF8574_PINS) {
    return;
  }
  LOG(LL_ERROR, ("Not implemented yet"));
  return;
}

void mgos_pcf8574_gpio_remove_int_handler(struct mgos_pcf8574 *dev, int pin, mgos_gpio_int_handler_f *old_cb, void **old_arg) {
  if (!dev || pin < 0 || pin >= MGOS_PCF8574_PINS) {
    return;
  }
  LOG(LL_ERROR, ("Not implemented yet"));
  return;
  (void) old_cb;
  (void) old_arg;
}

bool mgos_pcf8574_gpio_set_button_handler(struct mgos_pcf8574 *dev, int pin, enum mgos_gpio_pull_type pull_type, enum mgos_gpio_int_mode int_mode, int debounce_ms, mgos_gpio_int_handler_f cb, void *arg) {
  if (!dev || pin < 0 || pin >= MGOS_PCF8574_PINS) {
    return false;
  }
  LOG(LL_ERROR, ("Not implemented yet"));
  return false;
  (void) pull_type;
  (void) int_mode;
  (void) debounce_ms;
  (void) cb;
  (void) arg;
}

// Mongoose OS library initialization
bool mgos_pcf8574_i2c_init(void) {
  return true;
}
