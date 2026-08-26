#include "./device_key_cfg.h"

device_key_cfg_t device_key_cfg_table[DEVICE_KEY_COUNT] =
{
    {
        .key_id = DEVICE_KEY_1,
        .gpio_cfg =
        {
            .gpio_pin = &IfxPort_P02_0,
            .gpio_mode = IfxPort_Mode_inputPullUp,
            .gpio_paddriver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
            .gpio_state = IfxPort_State_low,
        },
    },
    {
        .key_id = DEVICE_KEY_2,
        .gpio_cfg =
        {
            .gpio_pin = &IfxPort_P02_1,
            .gpio_mode = IfxPort_Mode_inputPullUp,
            .gpio_paddriver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
            .gpio_state = IfxPort_State_low,
        },
    },
};

device_key_runtime_t device_key_runtime_table[DEVICE_KEY_COUNT] =
{
    {
        .key_id = DEVICE_KEY_1,
        .key_press_time = 0,
        .key_flag = DEVICE_KEY_RELEASE_FLAG,
    },
    {
        .key_id = DEVICE_KEY_2,
        .key_press_time = 0,
        .key_flag = DEVICE_KEY_RELEASE_FLAG,
    },
};

device_key_cfg_t* device_key_cfg_table_get(void)
{
    return device_key_cfg_table;
}

device_key_runtime_t* device_key_runtime_table_get(void)
{
    return device_key_runtime_table;
}
