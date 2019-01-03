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
  char    s[9], i[9];

  if (!dev) {
    return;
  }
  for (n = 0; n < 8; n++) {
    s[n] = (dev->_state & (1 << n)) ? '1' : '0';
    i[n] = (dev->_io & (1 << n)) ? 'I' : 'O';
  }
  s[8] = i[8] = 0;
  LOG(LL_DEBUG, ("state=0x%02x %s; io=0x%02x %s", dev->_state, s, dev->_io, i));
}

static bool mgos_pcf8574_read(struct mgos_pcf8574 *dev) {
  uint8_t val;

  if (!dev) {
    return false;
  }
  if (!mgos_i2c_read(dev->i2c, dev->i2caddr, &val, 1, true)) {
    return false;
  }
  dev->_state = val;
  mgos_pcf8574_print(dev);
  return true;
}

static bool mgos_pcf8574_write(struct mgos_pcf8574 *dev) {
  uint8_t val;

  if (!dev) {
    return false;
  }
  val = dev->_state | dev->_io;
  if (!mgos_i2c_write(dev->i2c, dev->i2caddr, &val, 1, true)) {
    return false;
  }

  mgos_pcf8574_print(dev);
  return true;
}

static void mgos_pcf8574_irq(int pin, void *arg) {
  struct mgos_pcf8574 *dev = (struct mgos_pcf8574 *)arg;
  uint8_t prev_state, this_state;

  if (!dev) {
    return;
  }
  if (dev->int_gpio != pin) {
    return;
  }
  LOG(LL_INFO, ("Interrupt!"));
  mgos_pcf8574_print(dev);
  prev_state = dev->_state & dev->_io;
  mgos_pcf8574_read(dev);
  this_state = dev->_state & dev->_io;

  if (prev_state != this_state) {
    uint8_t n;
    char    prev[9], this[9];
    for (n = 0; n < 8; n++) {
      prev[n] = (prev_state & (1 << n)) ? '1' : '0';
      this[n] = (this_state & (1 << n)) ? '1' : '0';
    }
    prev[8] = this[8] = 0;
    LOG(LL_INFO, ("prev=%s this=%s", prev, this));

    for (n = 0; n < 8; n++) {
      bool prev_bit  = prev_state & (1 << n);
      bool this_bit  = this_state & (1 << n);
      bool will_call = false;
      switch (dev->cb[n].mode) {
      case MGOS_GPIO_INT_EDGE_POS:
        if (!prev_bit && this_bit) {
          will_call = true;
        }
        break;

      case MGOS_GPIO_INT_EDGE_NEG:
        if (prev_bit && !this_bit) {
          will_call = true;
        }
        break;

      case MGOS_GPIO_INT_EDGE_ANY:
        if (prev_bit != this_bit) {
          will_call = true;
        }
        break;

      case MGOS_GPIO_INT_LEVEL_HI:
        if (this_bit) {
          will_call = true;
        }
        break;

      case MGOS_GPIO_INT_LEVEL_LO:
        if (!this_bit) {
          will_call = true;
        }
        break;

      default:
        return;
      }
      LOG(LL_INFO, ("GPIO[%u] prev_bit=%u this_bit=%u will_call=%u", n, prev_bit, this_bit, will_call));
      if (will_call && dev->cb[n].enabled) {
        dev->cb[n].firing = true;
        dev->cb[n].last   = mg_time();
        if (dev->cb[n].fn) {
          LOG(LL_INFO, ("GPIO[%u] callback issued", n));
          dev->cb[n].fn(n, dev->cb[n].fn_arg);
        }
        dev->cb[n].firing = false;
      }
    }
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
  dev->int_gpio = int_gpio;

  dev->_io = 0x00;      // Set all pins to OUTPUT
  // Read current IO state, assuming all pins are OUTPUT
  if (!mgos_pcf8574_read(dev)) {
    free(dev);
    return NULL;
  }

  // Install interrupt handler, if GPIO pin was specified.
  if (dev->int_gpio != -1) {
    mgos_gpio_set_int_handler(dev->int_gpio, MGOS_GPIO_INT_EDGE_NEG, mgos_pcf8574_irq, dev);
    mgos_gpio_enable_int(dev->int_gpio);
  }
  LOG(LL_INFO, ("PCF8574 initialized %sat I2C 0x%02x", (dev->int_gpio != -1) ? "with interrupts " : "", dev->i2caddr));
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
    dev->_io    |= (1 << pin);
    dev->_state |= (1 << pin);
  } else {
    dev->_io    &= ~(1 << pin);
    dev->_state |= (1 << pin);
  }
  mgos_pcf8574_write(dev);

  return true;
}

bool mgos_pcf8574_gpio_setup_input(struct mgos_pcf8574 *dev, int pin, enum mgos_gpio_pull_type pull_type) {
  if (!dev || pin < 0 || pin >= MGOS_PCF8574_PINS) {
    return false;
  }
  dev->_io |= (1 << pin);
  mgos_pcf8574_gpio_write(dev, pin, 1);
  return true;

  (void)pull_type;
}

bool mgos_pcf8574_gpio_setup_output(struct mgos_pcf8574 *dev, int pin, bool level) {
  if (!dev || pin < 0 || pin >= MGOS_PCF8574_PINS) {
    return false;
  }
  dev->_io &= ~(1 << pin);
  mgos_pcf8574_gpio_write(dev, pin, level);
  return true;
}

bool mgos_pcf8574_gpio_read(struct mgos_pcf8574 *dev, int pin) {
  if (!dev || pin < 0 || pin >= MGOS_PCF8574_PINS) {
    return false;
  }
  mgos_pcf8574_read(dev);
  return (dev->_state & (1 << pin)) > 0;
}

void mgos_pcf8574_gpio_write(struct mgos_pcf8574 *dev, int pin, bool level) {
  if (!dev || pin < 0 || pin >= MGOS_PCF8574_PINS) {
    return;
  }
  if (level) {
    dev->_state |= (1 << pin);
  } else {
    dev->_state &= ~(1 << pin);
  }
  mgos_pcf8574_write(dev);
  return;
}

bool mgos_pcf8574_gpio_toggle(struct mgos_pcf8574 *dev, int pin) {
  if (!dev || pin < 0 || pin >= MGOS_PCF8574_PINS) {
    return false;
  }

  dev->_state ^= (1 << pin);
  mgos_pcf8574_write(dev);
  return (dev->_state & (1 << pin)) > 0;
}

bool mgos_pcf8574_gpio_set_int_handler(struct mgos_pcf8574 *dev, int pin, enum mgos_gpio_int_mode mode, mgos_gpio_int_handler_f cb, void *arg) {
  if (!dev || pin < 0 || pin >= MGOS_PCF8574_PINS) {
    return false;
  }
  dev->cb[pin].fn     = cb;
  dev->cb[pin].fn_arg = arg;
  dev->cb[pin].mode   = mode;
  return true;

  (void)mode;
}

bool mgos_pcf8574_gpio_enable_int(struct mgos_pcf8574 *dev, int pin) {
  if (!dev || pin < 0 || pin >= MGOS_PCF8574_PINS) {
    return false;
  }
  dev->cb[pin].enabled = true;
  return true;
}

bool mgos_pcf8574_gpio_disable_int(struct mgos_pcf8574 *dev, int pin) {
  if (!dev || pin < 0 || pin >= MGOS_PCF8574_PINS) {
    return false;
  }
  dev->cb[pin].enabled = false;
  return true;
}

void mgos_pcf8574_gpio_clear_int(struct mgos_pcf8574 *dev, int pin) {
  if (!dev || pin < 0 || pin >= MGOS_PCF8574_PINS) {
    return;
  }
  dev->cb[pin].firing = false;
  dev->cb[pin].last   = 0.f;
  return;
}

void mgos_pcf8574_gpio_remove_int_handler(struct mgos_pcf8574 *dev, int pin, mgos_gpio_int_handler_f *old_cb, void **old_arg) {
  if (!dev || pin < 0 || pin >= MGOS_PCF8574_PINS) {
    return;
  }
  dev->cb[pin].fn     = NULL;
  dev->cb[pin].fn_arg = NULL;
  return;

  (void)old_cb;
  (void)old_arg;
}

bool mgos_pcf8574_gpio_set_button_handler(struct mgos_pcf8574 *dev, int pin, enum mgos_gpio_pull_type pull_type, enum mgos_gpio_int_mode int_mode, int debounce_ms, mgos_gpio_int_handler_f cb, void *arg) {
  if (!dev || pin < 0 || pin >= MGOS_PCF8574_PINS) {
    return false;
  }
  dev->cb[pin].fn          = cb;
  dev->cb[pin].fn_arg      = arg;
  dev->cb[pin].debounce_ms = debounce_ms;
  dev->cb[pin].mode        = int_mode;
  return true;

  (void)pull_type;
}

// Mongoose OS library initialization
bool mgos_pcf8574_i2c_init(void) {
  return true;
}
