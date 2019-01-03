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

void mgos_pcf8574_print_state(struct mgos_pcf8574 *dev) {
  uint8_t n;
  char    s[9], i[9];

  if (!dev) {
    return;
  }
  for (n = 0; n < 8; n++) {
    s[7 - n] = (dev->_state & (1 << n)) ? '1' : '0';
    i[7 - n] = (dev->_io & (1 << n)) ? 'I' : 'O';
  }
  s[8] = i[8] = 0;
  LOG(LL_INFO, ("state=0x%02x %s; io=0x%02x %s", dev->_state, s, dev->_io, i));
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
//  mgos_pcf8574_print_state(dev);
  return true;
}

static bool mgos_pcf8574_write(struct mgos_pcf8574 *dev) {
  uint8_t val;

  if (!dev) {
    return false;
  }

  // On PCF8574, all INPUT bits are written as '1' and will return
  // to their GPIO level upon read.
  val = dev->_state | dev->_io;
  if (!mgos_i2c_write(dev->i2c, dev->i2caddr, &val, 1, true)) {
    return false;
  }
  return true;
}

static void mgos_pcf8574_irq(int pin, void *arg) {
  struct mgos_pcf8574 *dev = (struct mgos_pcf8574 *)arg;
  uint8_t n;
  uint8_t prev_state, this_state;

  if (!dev) {
    return;
  }
  if (dev->int_gpio != pin) {
    return;
  }
  // mgos_pcf8574_print_state(dev);
  prev_state = dev->_state & dev->_io;
  mgos_pcf8574_read(dev);
  this_state = dev->_state & dev->_io;

  if (prev_state == this_state) {
    return;
  }

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
      will_call = false;
    }
    // LOG(LL_DEBUG, ("GPIO[%u] prev_bit=%u this_bit=%u will_call=%u", n, prev_bit, this_bit, will_call));
    if (will_call && dev->cb[n].enabled) {
      dev->cb[n].firing = true;
      dev->cb[n].last   = mg_time();
      if (dev->cb[n].fn) {
        // LOG(LL_DEBUG, ("GPIO[%u] callback issued", n));
        dev->cb[n].fn(n, dev->cb[n].fn_arg);
      }
      dev->cb[n].firing = false;
    }
  }

  // Clear the interrupt by reading the state again after T(ir) transpires.
  mgos_usleep(5);
  mgos_pcf8574_read(dev);
  return;
}

struct mgos_pcf8574 *mgos_pcf8574_create(struct mgos_i2c *i2c, uint8_t i2caddr, int int_gpio) {
  struct mgos_pcf8574 *dev = NULL;

  if (!i2c) {
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
    LOG(LL_ERROR, ("Could not read state from PCF8574"));
    free(dev);
    return NULL;
  }

  // Install interrupt handler, if GPIO pin was specified.
  if (dev->int_gpio != -1) {
    LOG(LL_INFO, ("Installing interrupt handler on GPIO %d", dev->int_gpio));
    mgos_gpio_set_mode(dev->int_gpio, MGOS_GPIO_MODE_INPUT);
    mgos_gpio_set_pull(dev->int_gpio, MGOS_GPIO_PULL_UP);
    mgos_gpio_set_int_handler(dev->int_gpio, MGOS_GPIO_INT_EDGE_NEG, mgos_pcf8574_irq, dev);
    mgos_gpio_clear_int(dev->int_gpio);
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
    /* When setting INPUT mode on a pin and the pin is reading low, an interrupt will fire as
     * it transitions from the written 1 (which is mandatory on INPUT pins) to 0. We disable
     * interrupts for two cycles (4us each) to allow for this interrupt to fire without calling
     * the user callback.
     */
    dev->_io    |= (1 << pin);
    dev->_state |= (1 << pin);
    mgos_pcf8574_gpio_disable_int(dev, pin);
    mgos_pcf8574_write(dev);
    mgos_usleep(10);
    mgos_pcf8574_read(dev);
    mgos_pcf8574_gpio_enable_int(dev, pin);
    return true;
  }

  dev->_io    &= ~(1 << pin);
  dev->_state |= (1 << pin);
  mgos_pcf8574_write(dev);

  return true;
}

bool mgos_pcf8574_gpio_setup_input(struct mgos_pcf8574 *dev, int pin, enum mgos_gpio_pull_type pull_type) {
  return mgos_pcf8574_gpio_set_mode(dev, pin, MGOS_GPIO_MODE_INPUT);

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
  dev->cb[pin].fn      = cb;
  dev->cb[pin].fn_arg  = arg;
  dev->cb[pin].mode    = mode;
  dev->cb[pin].enabled = true;
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
  dev->cb[pin].fn      = NULL;
  dev->cb[pin].fn_arg  = NULL;
  dev->cb[pin].firing  = false;
  dev->cb[pin].enabled = false;
  return;

  (void)old_cb;
  (void)old_arg;
}

bool mgos_pcf8574_gpio_set_button_handler(struct mgos_pcf8574 *dev, int pin, enum mgos_gpio_pull_type pull_type, enum mgos_gpio_int_mode int_mode, int debounce_ms, mgos_gpio_int_handler_f cb, void *arg) {
  if (!dev || pin < 0 || pin >= MGOS_PCF8574_PINS) {
    return false;
  }
  dev->cb[pin].debounce_ms = debounce_ms;
  return mgos_pcf8574_gpio_set_int_handler(dev, pin, int_mode, cb, arg);

  (void)pull_type;
}

// Mongoose OS library initialization
bool mgos_pcf8574_i2c_init(void) {
  return true;
}
