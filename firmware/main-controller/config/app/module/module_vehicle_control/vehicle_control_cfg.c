/**
 * @file vehicle_control_cfg.c
 * @brief Vehicle drive PID default configuration.
 */

#include "./vehicle_control_cfg.h"

#define VEHICLE_CONTROL_DEFAULT_ANGLE_KP 1500.0f   /**< Heading loop gain, unit: (mm/s)/rad. */
#define VEHICLE_CONTROL_DEFAULT_ANGLE_KD 40.0f     /**< Heading loop damping, unit: (mm/s)/(rad/s). */
#define VEHICLE_CONTROL_DEFAULT_ANGLE_KP2 10.0f    /**< Large-error gain, unit: (mm/s)/(rad^2). */
#define VEHICLE_CONTROL_DEFAULT_TARGET_YAW_RATE_FF_GAIN_MM 60.0f
#define VEHICLE_CONTROL_DEFAULT_TARGET_YAW_RATE_LPF_HZ 12.0f
#define VEHICLE_CONTROL_DEFAULT_TARGET_YAW_RATE_LIMIT_RAD_S 25.0f
#define VEHICLE_CONTROL_DEFAULT_YAW_CORRECTION_LIMIT_MM_S 1400.0f
#define VEHICLE_CONTROL_DEFAULT_SPEED_ACCEL_FF_GAIN 0.0f
#define VEHICLE_CONTROL_DEFAULT_SPEED_DECEL_FF_GAIN 0.0f
#define VEHICLE_CONTROL_DEFAULT_SPEED_ACCEL_FF_LIMIT 3000.0f
#define VEHICLE_CONTROL_DEFAULT_SPEED_KP 5.0f      /**< Speed PID kp, unit: PWM ticks/(mm/s). */
#define VEHICLE_CONTROL_DEFAULT_SPEED_KI 10.0f     /**< Speed PID ki, unit: PWM ticks/mm. */

static const vehicle_control_cfg_t vehicle_control_cfg =
{
    .replay_delay_cnt = 0u,
    .drive_idle_duty = 0,
    .suction_idle_duty = 0u,
    .target_speed_min_mm_s = -3000.0f,
    .target_speed_max_mm_s = 100000.0f,
    .yaw_speed_correction_min_mm_s = -VEHICLE_CONTROL_DEFAULT_YAW_CORRECTION_LIMIT_MM_S,
    .yaw_speed_correction_max_mm_s = VEHICLE_CONTROL_DEFAULT_YAW_CORRECTION_LIMIT_MM_S,
    .angle_kp2 = VEHICLE_CONTROL_DEFAULT_ANGLE_KP2,
    .target_yaw_rate_feedforward_gain_mm = VEHICLE_CONTROL_DEFAULT_TARGET_YAW_RATE_FF_GAIN_MM,
    .target_yaw_rate_lpf_cutoff_hz = VEHICLE_CONTROL_DEFAULT_TARGET_YAW_RATE_LPF_HZ,
    .target_yaw_rate_limit_rad_s = VEHICLE_CONTROL_DEFAULT_TARGET_YAW_RATE_LIMIT_RAD_S,
    .speed_accel_feedforward_gain = VEHICLE_CONTROL_DEFAULT_SPEED_ACCEL_FF_GAIN,
    .speed_decel_feedforward_gain = VEHICLE_CONTROL_DEFAULT_SPEED_DECEL_FF_GAIN,
    .speed_accel_feedforward_limit = VEHICLE_CONTROL_DEFAULT_SPEED_ACCEL_FF_LIMIT,
    .angle_pid =
    {
        .kp = VEHICLE_CONTROL_DEFAULT_ANGLE_KP,
        .ki = 0.0f,
        .kd = VEHICLE_CONTROL_DEFAULT_ANGLE_KD,
        .output_min = -VEHICLE_CONTROL_DEFAULT_YAW_CORRECTION_LIMIT_MM_S,
        .output_max = VEHICLE_CONTROL_DEFAULT_YAW_CORRECTION_LIMIT_MM_S,
        .integral_min = -1.0f,
        .integral_max = 1.0f,
    },
    .left_speed_pid =
    {
        .kp = VEHICLE_CONTROL_DEFAULT_SPEED_KP,
        .ki = VEHICLE_CONTROL_DEFAULT_SPEED_KI,
        .kd = 0.0f,
        .output_min = -3000.0f,
        .output_max = 8000.0f,
        .integral_min = -800.0f,
        .integral_max = 800.0f,
    },
    .right_speed_pid =
    {
        .kp = VEHICLE_CONTROL_DEFAULT_SPEED_KP,
        .ki = VEHICLE_CONTROL_DEFAULT_SPEED_KI,
        .kd = 0.0f,
        .output_min = -3000.0f,
        .output_max = 8000.0f,
        .integral_min = -800.0f,
        .integral_max = 800.0f,
    },
};

/**
 * @brief Get read-only vehicle drive PID control configuration.
 * @param[in] void No parameter.
 * @return Vehicle drive PID control configuration pointer.
 */
const vehicle_control_cfg_t* vehicle_control_cfg_get(void)
{
    return &vehicle_control_cfg;
}
