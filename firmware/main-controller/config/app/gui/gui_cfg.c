/**
 * @file gui_cfg.c
 * @brief 应用层按键模式选择与指示灯界面默认配置。
 */

#include "./gui_cfg.h"

static gui_cfg_t gui_cfg =
{
    .sysTick_id = SYSTICK1,
    .mode_led_id = DEVICE_LED_1,
    .status_led_id = DEVICE_LED_2,
    .select_key_id = DEVICE_KEY_1,
    .confirm_key_id = DEVICE_KEY_2,
    .scheduler_period_ms = 10u,
    .short_press_min_ms = 30u,
    .long_press_ms = 1000u,
    .mode_led_on_ms = 120u,
    .mode_led_off_ms = 120u,
    .mode_led_group_gap_ms = 500u,
    .status_led_pending_blink_ms = 250u,
    .default_mode = GUI_MODE_POST_PROCESS,
};

/**
 * @brief 获取 GUI 配置表。
 * @param[in] void 无参数。
 * @return GUI 配置表指针。
 */
gui_cfg_t* gui_cfg_get(void)
{
    return &gui_cfg;
}
