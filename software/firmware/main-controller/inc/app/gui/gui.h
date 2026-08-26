/**
 * @file gui.h
 * @brief 应用层按键模式选择与指示灯界面对外接口。
 */

#ifndef MAD_CIRCUITS_APP_GUI_H
#define MAD_CIRCUITS_APP_GUI_H

#include "../../../config/app/gui/gui_cfg.h"

/**
 * @brief 初始化应用层按键模式选择与指示灯界面。
 * @param[in] void 无参数。
 * @return void
 */
void gui_init_all(void);

/**
 * @brief 执行一次按键模式扫描与指示灯调度。
 * @param[in] void 无参数。
 * @return void
 */
void gui_scheduler_callback(void);

/**
 * @brief 获取当前已确认的运行模式。
 * @param[in] void 无参数。
 * @return 当前已确认的运行模式。
 */
gui_mode_t gui_mode_get(void);

/**
 * @brief 获取当前待确认的候选模式。
 * @param[in] void 无参数。
 * @return 当前待确认的候选模式。
 */
gui_mode_t gui_mode_selected_get(void);

#endif
