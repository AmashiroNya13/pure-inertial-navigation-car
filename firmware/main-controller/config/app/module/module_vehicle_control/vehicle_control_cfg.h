/**
 * @file vehicle_control_cfg.h
 * @brief Vehicle PID control configuration types.
 */

#ifndef MAD_CIRCUITS_VEHICLE_CONTROL_CFG_H
#define MAD_CIRCUITS_VEHICLE_CONTROL_CFG_H

#include "Ifx_Types.h"

#include "../../../../inc/middleware/algorithm/algorithm_control.h"

#define VEHICLE_CONTROL_DEFAULT_RUN_SUCTION_DUTY ((uint32)13000u)

typedef algorithm_control_pid_cfg_t vehicle_control_pid_cfg_t;

typedef struct
{
    uint32 replay_delay_cnt;
    sint32 drive_idle_duty;
    uint32 suction_idle_duty;
    float32 target_speed_min_mm_s;
    float32 target_speed_max_mm_s;
    float32 yaw_speed_correction_min_mm_s;
    float32 yaw_speed_correction_max_mm_s;
    float32 angle_kp2;
    float32 target_yaw_rate_feedforward_gain_mm;
    float32 target_yaw_rate_lpf_cutoff_hz;
    float32 target_yaw_rate_limit_rad_s;
    float32 speed_accel_feedforward_gain;
    float32 speed_decel_feedforward_gain;
    float32 speed_accel_feedforward_limit;
    vehicle_control_pid_cfg_t angle_pid;
    vehicle_control_pid_cfg_t left_speed_pid;
    vehicle_control_pid_cfg_t right_speed_pid;
} vehicle_control_cfg_t;

const vehicle_control_cfg_t* vehicle_control_cfg_get(void);

#endif
