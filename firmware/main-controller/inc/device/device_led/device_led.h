#ifndef MAD_CIRCUITS_DEVICE_LED_H
#define MAD_CIRCUITS_DEVICE_LED_H

#include "../../../config/device/device_led/device_led_cfg.h"

void device_led_init(device_led_id_t led_id);
void device_led_init_all(void);

void device_led_set_state(device_led_id_t led_id, device_led_action_t led_action);
void device_led_blink(device_led_id_t led_id);

#endif
