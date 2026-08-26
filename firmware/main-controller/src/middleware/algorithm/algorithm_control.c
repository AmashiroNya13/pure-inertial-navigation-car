/**
 * @file algorithm_control.c
 * @brief 通用控制算法实现。
 */

#include "../../../inc/middleware/algorithm/algorithm_control.h"

#include "../../../inc/middleware/algorithm/algorithm_attitude.h"

/**
 * @brief 清空 PID 运行状态。
 * @param[in] pid PID 运行状态指针。
 * @return void
 */
void algorithm_control_pid_reset(algorithm_control_pid_state_t* pid)
{
    if (pid == NULL_PTR)
    {
        return;
    }

    pid->target = 0.0f;
    pid->feedback = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->previous_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

/**
 * @brief 将浮点数限制在指定上下限范围内。
 * @param[in] value 待限幅数值。
 * @param[in] min_value 最小允许值。
 * @param[in] max_value 最大允许值。
 * @return 限幅后的数值。
 */
float32 algorithm_control_clamp_f32(float32 value, float32 min_value, float32 max_value)
{
    if (value < min_value)
    {
        value = min_value;
    }

    if (value > max_value)
    {
        value = max_value;
    }

    return value;
}

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
                                              boolean wrap_error)
{
    float32 error = target - feedback;
    float32 derivative = 0.0f;

    if (wrap_error != FALSE)
    {
        error = algorithm_attitude_wrap_pi(error);
    }

    if (dt_s > 0.0f)
    {
        derivative = (error - pid->error) / dt_s;
        pid->integral += error * dt_s;
        pid->integral = algorithm_control_clamp_f32(pid->integral, cfg->integral_min, cfg->integral_max);
    }

    pid->target = target;
    pid->feedback = feedback;
    pid->previous_error = pid->last_error;
    pid->last_error = pid->error;
    pid->error = error;
    pid->output = (cfg->kp * error) + (cfg->ki * pid->integral) + (cfg->kd * derivative);
    pid->output = algorithm_control_clamp_f32(pid->output, cfg->output_min, cfg->output_max);

    return pid->output;
}

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
                                                 float32 dt_s)
{
    float32 error = target - feedback;
    float32 delta_output = 0.0f;

    if (dt_s > 0.0f)
    {
        pid->integral += error * dt_s;
        pid->integral = algorithm_control_clamp_f32(pid->integral, cfg->integral_min, cfg->integral_max);

        delta_output = (cfg->kp * (error - pid->error))
                     + (cfg->ki * error * dt_s)
                     + (cfg->kd * (error - (2.0f * pid->error) + pid->last_error) / dt_s);
    }

    pid->previous_error = pid->last_error;
    pid->last_error = pid->error;
    pid->error = error;
    pid->target = target;
    pid->feedback = feedback;
    pid->output += delta_output;
    pid->output = algorithm_control_clamp_f32(pid->output, cfg->output_min, cfg->output_max);

    return pid->output;
}
