# PCF8574 I2C Driver

A Mongoose library for PCF8574, a popular and cheap 8x GPIO extender using I2C.

## Implementation details

The PCF8574 is a pseudo bi-directional GPIO, which means that by default ports
are input and output. Writes to the ports can change the state from high to low.
If the port is high, it is pulled up by a weak current. Each port can be pulled
low by an external device, in which case reads from that port return 0. The
implication is that "input" ports are always pulled high.

## API Description

To start, `mgos_pcf8574_create()` is called with the correct `I2C` bus and
address (by default 0x20), and optionally a GPIO pin on the microcontroller that
serves as an interrupt pin, to detect PCF8574 input state changes.

**NOTE:** When the driver starts, it polls the current state from the chip
without changing any ports. The benefit of this is that the MCU can safely
reboot without loss of the GPIO state in PCF8574.

The API follows `mgos_gpio.h` closely, enabling ports to be set as input (ignoring
the `pull_type`, which is always pullup), or as output. Notably,
`mgos_pcf8574_gpio_set_int_handler()` and `mgos_pcf8574_gpio_set_button_handler()`
work identically to the `mgos_gpio_*()` variants.

## Example application

```
#include "mgos.h"
#include "mgos_config.h"
#include "mgos_pcf8574.h"

static void button_cb(int pin, void *user_data) {
  struct mgos_pcf8574 *d = (struct mgos_pcf8574 *)user_data;
  LOG(LL_INFO, ("GPIO[%d] callback, value=%d", pin, mgos_pcf8574_gpio_read(d, pin)));
  mgos_pcf8574_gpio_toggle(d, pin+4);
}

enum mgos_app_init_result mgos_app_init(void) {
  struct mgos_pcf8574 *d;
  int i;

  if (!(d = mgos_pcf8574_create(mgos_i2c_get_global(), mgos_sys_config_get_pcf8574_i2caddr(),
                                mgos_sys_config_get_pcf8574_int_gpio()))) {
    LOG(LL_ERROR, ("Could not create PCF8574"));
    return MGOS_APP_INIT_ERROR;
  }

  for(i=0; i<4; i++) mgos_pcf8574_gpio_setup_input(d, i, MGOS_GPIO_PULL_UP);
  for(i=4; i<8; i++) mgos_pcf8574_gpio_set_mode(d, i, MGOS_GPIO_MODE_OUTPUT);

  mgos_pcf8574_gpio_set_button_handler(d, 0, MGOS_GPIO_PULL_UP, MGOS_GPIO_INT_EDGE_NEG, 10, button_cb, d);
  mgos_pcf8574_gpio_set_button_handler(d, 1, MGOS_GPIO_PULL_UP, MGOS_GPIO_INT_EDGE_POS, 10, button_cb, d);
  mgos_pcf8574_gpio_set_button_handler(d, 2, MGOS_GPIO_PULL_UP, MGOS_GPIO_INT_EDGE_ANY, 10, button_cb, d);
  mgos_pcf8574_gpio_set_button_handler(d, 3, MGOS_GPIO_PULL_UP, MGOS_GPIO_INT_EDGE_ANY, 10, button_cb, d);

  return MGOS_APP_INIT_SUCCESS;
}
```

# Disclaimer

This project is not an official Google project. It is not supported by Google
and Google specifically disclaims all warranties as to its quality,
merchantability, or fitness for a particular purpose.
