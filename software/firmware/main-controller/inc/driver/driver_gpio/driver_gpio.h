#ifndef MAD_CIRCUITS_DRIVER_GPIO_H
#define MAD_CIRCUITS_DRIVER_GPIO_H

#include "IfxPort.h"
#include "IfxPort_PinMap.h"
#include "Ifx_Types.h"

typedef struct
{
    IfxPort_Pin* gpio_pin;
    IfxPort_Mode gpio_mode;
    IfxPort_PadDriver gpio_paddriver;
    IfxPort_State gpio_state;
}driver_gpio_cfg_t;

typedef struct
{
    IfxPort_Pin* gpio_pin;
}driver_gpio_runtime_t;

void driver_gpio_init(driver_gpio_cfg_t* gpio_cfg, driver_gpio_runtime_t* gpio_runtime);
void driver_gpio_setState(driver_gpio_runtime_t* gpio_runtime, IfxPort_State gpio_state);
boolean driver_gpio_getState(driver_gpio_runtime_t* gpio_runtime);

#endif
