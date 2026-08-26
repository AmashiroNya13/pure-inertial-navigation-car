/**
 * @file app_task_vehicle_cfg.c
 * @brief 整车最终业务异步任务默认配置。
 */

#include "./app_task_vehicle_cfg.h"
#include "../../module/module_vehicle_control/vehicle_control_cfg.h"

static const app_task_asynchronous_vehicle_cfg_t app_task_asynchronous_vehicle_cfg =
{
    .learning_speed_mm_s = 300.0f,
    .replay_speed_mm_s = 800.0f,
    .drive_stop_speed_mm_s = 0.0f,
    .record_start_delay_s = 1.0f,
    .replay_suction_delay_s = 1.0f,
    .replay_drive_delay_s = 1.0f,
    .replay_finish_drive_hold_s = 0.5f,
    .replay_finish_suction_hold_s = 1.0f,
    .record_suction_duty = VEHICLE_CONTROL_DEFAULT_RUN_SUCTION_DUTY,
    .suction_run_duty = VEHICLE_CONTROL_DEFAULT_RUN_SUCTION_DUTY,
    .suction_stop_duty = 0u,
};

/**
 * @brief 获取只读的整车最终业务异步任务配置。
 * @param[in] void 无参数。
 * @return 整车最终业务异步任务配置指针。
 */
const app_task_asynchronous_vehicle_cfg_t* app_task_asynchronous_vehicle_cfg_get(void)
{
    return &app_task_asynchronous_vehicle_cfg;
}
