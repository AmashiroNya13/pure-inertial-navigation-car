/**
 * @file app_task_vehicle_cfg.h
 * @brief 整车最终业务异步任务配置类型。
 */

#ifndef MAD_CIRCUITS_APP_TASK_ASYNCHRONOUS_VEHICLE_CFG_H
#define MAD_CIRCUITS_APP_TASK_ASYNCHRONOUS_VEHICLE_CFG_H

#include "Ifx_Types.h"

typedef struct
{
    float32 learning_speed_mm_s;   /**< 学习记录模式下目标速度，单位：毫米/秒。 */
    float32 replay_speed_mm_s;     /**< 复现模式下目标速度，单位：毫米/秒。 */
    float32 drive_stop_speed_mm_s; /**< 停车时目标速度，单位：毫米/秒。 */
    float32 record_start_delay_s;
    float32 replay_suction_delay_s;
    float32 replay_drive_delay_s;
    float32 replay_finish_drive_hold_s;
    float32 replay_finish_suction_hold_s;
    uint32 record_suction_duty;    /**< Path recording suction PWM duty command. */
    uint32 suction_run_duty;       /**< 运行时吸风 PWM 占空命令。 */
    uint32 suction_stop_duty;      /**< 停止时吸风 PWM 占空命令。 */
} app_task_asynchronous_vehicle_cfg_t;

/**
 * @brief 获取只读的整车最终业务异步任务配置。
 * @param[in] void 无参数。
 * @return 整车最终业务异步任务配置指针。
 */
const app_task_asynchronous_vehicle_cfg_t* app_task_asynchronous_vehicle_cfg_get(void);

#endif
