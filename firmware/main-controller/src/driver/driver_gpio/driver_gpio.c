#include "../../../inc/driver/driver_gpio/driver_gpio.h"

void driver_gpio_init(driver_gpio_cfg_t* gpio_cfg, driver_gpio_runtime_t* gpio_runtime)
{
    IfxPort_setPinMode(gpio_cfg->gpio_pin->port, gpio_cfg->gpio_pin->pinIndex, gpio_cfg->gpio_mode);
    IfxPort_setPinPadDriver(gpio_cfg->gpio_pin->port, gpio_cfg->gpio_pin->pinIndex, gpio_cfg->gpio_paddriver);

    if (gpio_cfg->gpio_mode > IfxPort_Mode_inputPullUp)
    {
        IfxPort_setPinState(gpio_cfg->gpio_pin->port, gpio_cfg->gpio_pin->pinIndex, gpio_cfg->gpio_state);
    }

    gpio_runtime->gpio_pin = gpio_cfg->gpio_pin;
}

void driver_gpio_setState(driver_gpio_runtime_t* gpio_runtime, IfxPort_State gpio_state)
{
    IfxPort_setPinState(gpio_runtime->gpio_pin->port, gpio_runtime->gpio_pin->pinIndex, gpio_state);
}

boolean driver_gpio_getState(driver_gpio_runtime_t* gpio_runtime)
{
    return IfxPort_getPinState(gpio_runtime->gpio_pin->port, gpio_runtime->gpio_pin->pinIndex);
}
