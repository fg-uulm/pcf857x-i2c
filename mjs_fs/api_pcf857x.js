let PCF857X = {
    _crt8: ffi('void* mgos_pcf8574_create_js(int, int)'),
    _crt16: ffi('void* mgos_pcf8575_create_js(int, int)'),
    _destroy: ffi('void mgos_pcf857x_destroy(void *)'),
    _set_mode: ffi('int mgos_pcf857x_gpio_set_mode(void *, int, int)'),
    _setup_output: ffi('int mgos_pcf857x_gpio_setup_output(void *, int, int)'),
    _setup_input: ffi('int mgos_pcf857x_gpio_setup_input(void *, int, int)'),
    _read: ffi('int mgos_pcf857x_gpio_read(void *, int)'),
    _write: ffi('void mgos_pcf857x_gpio_write(void *, int, int)'),
    _toggle: ffi('int mgos_pcf857x_gpio_toggle(void *, int)'),
    _print: ffi('void mgos_pcf857x_print_state(void *)'),
    _set_int_handler: ffi('int mgos_pcf857x_gpio_set_int_handler(void *, int,int,void(*)(int,userdata),userdata)'),
    //set_button_handler: ffi('int mgos_pcf857x_gpio_set_button_handler(void *, int,int,int,int,void(*)(int, userdata), userdata)'), //does not work currently, too many arguments for ffi - use following instead
    _set_debounce: ffi('void mgos_pcf857x_gpio_set_debounce_js(void *, int, int)'),

    INT_NONE: 0,
    INT_EDGE_POS: 1,
    INT_EDGE_NEG: 2,
    INT_EDGE_ANY: 3,
    INT_LEVEL_HI: 4,
    INT_LEVEL_LO: 5,

    create: function (address, intpin, gpionum) {
        let obj = Object.create(PCF857X._proto);
        if(gpionum === 8) {
            obj.pcf = PCF857X._crt8(address, intpin);
        }
        else if(gpionum === 16) { 
            obj.pcf = PCF857X._crt16(address, intpin);
        } else {
            Log.error("Unsupported number of gpios");
        }
        return obj;
    },

    _proto: {
        free: function () {
            PCF857X._destroy(this.pcf);
        },

        print: function () {
            PCF857X._print(this.pcf);
        },

        set_mode: function(pin, mode) {
            return PCF857X._set_mode(this.pcf, pin, mode);
        },

        setup_output: function(pin, level) {
            return PCF857X._setup_output(this.pcf, pin, level);
        },

        setup_input: function(pin, pull_type) {
            return PCF857X._setup_input(this.pcf, pin, pull_type);
        },

        read: function(pin) {
            return PCF857X._read(this.pcf, pin);
        },

        write: function(pin) {
            PCF857X._write(this.pcf, pin, level);
        },

        toggle: function(pin) {
            return PCF857X._toggle(this.pcf, pin);
        },

        set_int_handler: function(pin, mode, handler, args) {
            return PCF857X._set_int_handler(this.pcf, pin, mode, handler, args);
        },

        set_debounce: function(pin, debounce_ms) {
            return PCF857X._set_debounce(this.pcf, pin, debounce_ms);
        }
    },
};