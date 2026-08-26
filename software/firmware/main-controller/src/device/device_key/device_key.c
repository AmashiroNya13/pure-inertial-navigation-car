#include "../../../inc/device/device_key/device_key.h"

void device_key_init(device_key_id_t key_id)
{
    device_key_cfg_t* key_cfg = device_key_cfg_table_get();
    device_key_runtime_t* key_runtime = device_key_runtime_table_get();

    driver_gpio_init(&key_cfg[key_id].gpio_cfg, &key_runtime[key_id].gpio_runtime);
}

void device_key_init_all(void)
{
    device_key_init(DEVICE_KEY_1);
    device_key_init(DEVICE_KEY_2);
}

device_key_state_t device_key_getState(device_key_id_t key_id)
{
    device_key_runtime_t* key_runtime = device_key_runtime_table_get();
    return (device_key_state_t)driver_gpio_getState(&key_runtime[key_id].gpio_runtime);
}

void device_key_scanner(void)
{
    device_key_runtime_t* key_runtime = device_key_runtime_table_get();

    for (uint8 i = 0; i < DEVICE_KEY_COUNT; i++)
    {
        if (driver_gpio_getState(&key_runtime[i].gpio_runtime) == DEVICE_KEY_ONPRESS)
        {
            key_runtime[i].key_press_time++;

            if (key_runtime[i].key_press_time >= DEVICE_KEY_LONG_PRESS_PERIOD)
            {
                key_runtime[i].key_flag = DEVICE_KEY_LONG_PRESS_FLAG;
            }
        }
        else
        {
            if ((key_runtime[i].key_flag != DEVICE_KEY_LONG_PRESS_FLAG) &&
                (key_runtime[i].key_press_time >= DEVICE_KEY_MAX_SHOCK_PERIOD) &&
                (key_runtime[i].key_press_time < DEVICE_KEY_LONG_PRESS_PERIOD))
            {
                key_runtime[i].key_flag = DEVICE_KEY_SHORT_PRESS_FLAG;
            }
            else
            {
                key_runtime[i].key_flag = DEVICE_KEY_RELEASE_FLAG;
            }

            key_runtime[i].key_press_time = 0;
        }
    }
}

device_key_flag_t device_key_getFlag(device_key_id_t key_id)
{
    device_key_runtime_t* key_runtime = device_key_runtime_table_get();
    return key_runtime[key_id].key_flag;
}

void device_key_clearFlag(device_key_id_t key_id)
{
    device_key_runtime_t* key_runtime = device_key_runtime_table_get();
    key_runtime[key_id].key_flag = DEVICE_KEY_RELEASE_FLAG;
}

void device_key_clearAllFlag(void)
{
    for (uint8 i = 0; i < DEVICE_KEY_COUNT; i++)
    {
        device_key_clearFlag(i);
    }
}
