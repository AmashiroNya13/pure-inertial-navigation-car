#include "./device_led_cfg.h"

device_led_cfg_t device_led_cfg_table[DEVICE_LED_COUNT] =
{
    {
        .led_id = DEVICE_LED_1,
        .gpio_cfg =
        {
            .gpio_pin = &IfxPort_P02_8,
            .gpio_mode = IfxPort_Mode_outputPushPullGeneral,
            .gpio_paddriver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
            .gpio_state = IfxPort_State_low,
        },
    },
    {
        .led_id = DEVICE_LED_2,
        .gpio_cfg =
        {
            .gpio_pin = &IfxPort_P00_2,
            .gpio_mode = IfxPort_Mode_outputPushPullGeneral,
            .gpio_paddriver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
            .gpio_state = IfxPort_State_low,
        },
    },
};

device_led_runtime_t device_led_runtime_table[DEVICE_LED_COUNT] =
{
    {
        .led_id = DEVICE_LED_1,
    },
    {
        .led_id = DEVICE_LED_2,
    },
};

device_led_cfg_t* device_led_cfg_table_get(void)
{
    return device_led_cfg_table;
}

device_led_runtime_t* device_led_runtime_table_get(void)
{
    return device_led_runtime_table;
}
