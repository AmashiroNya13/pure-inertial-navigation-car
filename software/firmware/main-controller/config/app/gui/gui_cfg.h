/**
 * @file gui_cfg.h
 * @brief 应用层按键模式选择与指示灯界面配置类型。
 */

#ifndef MAD_CIRCUITS_APP_GUI_CFG_H
#define MAD_CIRCUITS_APP_GUI_CFG_H

#include "Ifx_Types.h"

#include "../../../inc/device/device_key/device_key.h"
#include "../../../inc/device/device_led/device_led.h"
#include "../../../inc/middleware/sysTick/sysTick.h"

typedef enum
{
    GUI_MODE_LEARNING = 0,     /**< 学习记录模式，长按确认后进入，离开模式后退出。 */
    GUI_MODE_POST_PROCESS = 1, /**< 后处理停车模式，长按确认后进入，切换模式后退出。 */
    GUI_MODE_COMPETITION = 2,  /**< 比赛复现模式，长按确认后进入，复现结束或切换模式后退出。 */
    GUI_MODE_CNT = 3,          /**< GUI 模式总数，作为边界使用。 */
} gui_mode_t;

typedef struct
{
    sysTick_id_t sysTick_id;                 /**< GUI 使用的 sysTick 编号。 */
    device_led_id_t mode_led_id;             /**< 候选模式指示灯编号。 */
    device_led_id_t status_led_id;           /**< 确认状态指示灯编号。 */
    device_key_id_t select_key_id;           /**< 模式选择按键编号。 */
    device_key_id_t confirm_key_id;          /**< 模式确认按键编号。 */
    uint32 scheduler_period_ms;              /**< GUI 调度周期，单位：毫秒。 */
    uint32 short_press_min_ms;               /**< 短按最小时间，单位：毫秒。 */
    uint32 long_press_ms;                    /**< 长按确认时间，单位：毫秒。 */
    uint32 mode_led_on_ms;                   /**< 模式指示灯点亮时间，单位：毫秒。 */
    uint32 mode_led_off_ms;                  /**< 模式指示灯熄灭时间，单位：毫秒。 */
    uint32 mode_led_group_gap_ms;            /**< 模式指示灯组间隔时间，单位：毫秒。 */
    uint32 status_led_pending_blink_ms;      /**< 待确认状态闪烁间隔，单位：毫秒。 */
    gui_mode_t default_mode;                 /**< GUI 默认已确认模式。 */
} gui_cfg_t;

/**
 * @brief 获取 GUI 配置表。
 * @param[in] void 无参数。
 * @return GUI 配置表指针。
 */
gui_cfg_t* gui_cfg_get(void);

#endif
