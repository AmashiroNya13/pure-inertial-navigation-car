#include "test_device_led.h"
#include "device_led.h"

#define TEST_LED_ON_OFF_DELAY 3000000u

void test_device_led_blink(void)
{
    device_led_blink(DEVICE_LED_1);
}

void test_device_led_on(void)
{
    device_led_set_state(DEVICE_LED_1, DEVICE_LED_LIGHT);
    for (volatile uint32 i = 0; i < TEST_LED_ON_OFF_DELAY; i++) { }
}

void test_device_led_off(void)
{
    device_led_set_state(DEVICE_LED_1, DEVICE_LED_DARK);
    for (volatile uint32 i = 0; i < TEST_LED_ON_OFF_DELAY; i++) { }
}
