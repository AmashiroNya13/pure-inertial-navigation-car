/**
 * @file algorithm_control.h
 * @brief 通用控制算法接口。
 */

#ifndef MAD_CIRCUITS_MIDDLEWARE_ALGORITHM_CONTROL_H
#define MAD_CIRCUITS_MIDDLEWARE_ALGORITHM_CONTROL_H

#include "Ifx_Types.h"

typedef struct
{
    float32 kp;
    float32 ki;
    float32 kd;
    float32 output_min;
    float32 output_max;
    float32 integral_min;
    float32 integral_max;
} algorithm_control_pid_cfg_t;

typedef struct
{
    float32 target;
    float32 feedback;
    float32 error;
    float32 last_error;
    float32 previous_error;
    float32 integral;
    float32 output;
} algorithm_control_pid_state_t;

/**
 * @brief 清空 PID 运行状态。
 * @param[in] pid PID 运行状态指针。
 * @return void
 */
void algorithm_control_pid_reset(algorithm_control_pid_state_t* pid);

/**
 * @brief 将浮点数限制在指定上下限范围内。
 * @param[in] value 待限幅数值。
 * @param[in] min_value 最小允许值。
 * @param[in] max_value 最大允许值。
 * @return 限幅后的数值。
 */
float32 algorithm_control_clamp_f32(float32 value, float32 min_value, float32 max_value);

/**
 * @brief 按位置式 PID 计算控制输出。
 * @param[in] cfg PID 参数配置指针。
 * @param[in,out] pid PID 运行状态指针。
 * @param[in] target 目标值。
 * @param[in] feedback 反馈值。
 * @param[in] dt_s 控制周期，单位：秒。
 * @param[in] wrap_error 是否将误差折算到正负 PI 范围。
 * @return 本次位置式 PID 输出。
 */
float32 algorithm_control_position_pid_update(const algorithm_control_pid_cfg_t* cfg,
                                              algorithm_control_pid_state_t* pid,
                                              float32 target,
                                              float32 feedback,
                                              float32 dt_s,
                                              boolean wrap_error);

/**
 * @brief 按增量式 PID 推进一次控制输出。
 * @param[in] cfg PID 参数配置指针。
 * @param[in,out] pid PID 运行状态指针。
 * @param[in] target 目标值。
 * @param[in] feedback 反馈值。
 * @param[in] dt_s 控制周期，单位：秒。
 * @return 本次增量式 PID 累积输出。
 */
float32 algorithm_control_incremental_pid_update(const algorithm_control_pid_cfg_t* cfg,
                                                 algorithm_control_pid_state_t* pid,
                                                 float32 target,
                                                 float32 feedback,
                                                 float32 dt_s);

#endif
