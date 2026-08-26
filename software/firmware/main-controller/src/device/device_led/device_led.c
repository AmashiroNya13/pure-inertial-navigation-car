#include "../../../inc/device/device_led/device_led.h"

void device_led_init(device_led_id_t led_id)
{
    device_led_cfg_t* led_cfg = device_led_cfg_table_get();
    device_led_runtime_t* led_runtime = device_led_runtime_table_get();

    driver_gpio_init(&led_cfg[led_id].gpio_cfg, &led_runtime[led_id].gpio_runtime);
}

void device_led_init_all(void)
{
    device_led_init(DEVICE_LED_1);
    device_led_init(DEVICE_LED_2);
}

void device_led_set_state(device_led_id_t led_id, device_led_action_t led_action)
{
    device_led_runtime_t* led_runtime = device_led_runtime_table_get();
    driver_gpio_setState(&led_runtime[led_id].gpio_runtime, (IfxPort_State)led_action);
}

void device_led_blink(device_led_id_t led_id)
{
    device_led_set_state(led_id, DEVICE_LED_LIGHT);
    for (uint32 i = 0; i < DEVICE_LED_BLINK_TIME; i++)
    {
    }

    device_led_set_state(led_id, DEVICE_LED_DARK);
    for (uint32 i = 0; i < DEVICE_LED_BLINK_TIME; i++)
    {
    }
}
