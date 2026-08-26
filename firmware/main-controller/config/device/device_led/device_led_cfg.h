#ifndef MAD_CIRCUITS_DEVICE_LED_CFG_H
#define MAD_CIRCUITS_DEVICE_LED_CFG_H

#include "../../../inc/driver/driver_gpio/driver_gpio.h"

#define DEVICE_LED_BLINK_TIME 3000000u

typedef enum
{
    DEVICE_LED_1 = 0,
    DEVICE_LED_2 = 1,
    DEVICE_LED_COUNT = 2,
} device_led_id_t;

typedef enum
{
    DEVICE_LED_KEEP = IfxPort_State_notChanged,
    DEVICE_LED_DARK = IfxPort_State_low,
    DEVICE_LED_LIGHT = IfxPort_State_high,
    DEVICE_LED_TOGGLED = IfxPort_State_toggled,
} device_led_action_t;

typedef struct
{
    device_led_id_t led_id;
    driver_gpio_cfg_t gpio_cfg;
} device_led_cfg_t;

typedef struct
{
    device_led_id_t led_id;
    driver_gpio_runtime_t gpio_runtime;
} device_led_runtime_t;

device_led_cfg_t* device_led_cfg_table_get(void);
device_led_runtime_t* device_led_runtime_table_get(void);

#endif
