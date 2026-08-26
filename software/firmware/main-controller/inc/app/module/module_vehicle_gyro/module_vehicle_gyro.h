/**
 * @file module_vehicle_gyro.h
 * @brief 车体航向观测模块接口。
 */

#ifndef MAD_CIRCUITS_APP_MODULE_MODULE_VEHICLE_GYRO_H
#define MAD_CIRCUITS_APP_MODULE_MODULE_VEHICLE_GYRO_H

#include "../../../../config/app/module/module_vehicle_gyro/vehicle_gyro_cfg.h"
#include "../../../../inc/middleware/algorithm/algorithm_attitude.h"

typedef algorithm_vector3_t module_vehicle_gyro_vector3_t;
typedef algorithm_quaternion_t module_vehicle_gyro_quaternion_t;

typedef struct
{
    module_vehicle_gyro_quaternion_t attitude_q;   /**< 当前积分姿态四元数。 */
    module_vehicle_gyro_vector3_t fused_accel_raw; /**< 融合加速度原始计数，单位：LSB。 */
    module_vehicle_gyro_vector3_t linear_accel_mm_s2; /**< 去除静态重力后的车体系加速度，单位：毫米/秒平方。 */
    module_vehicle_gyro_vector3_t fused_gyro_dps;  /**< 融合角速度，单位：度/秒。 */
    module_vehicle_gyro_vector3_t fused_gyro_rad_s;/**< 融合角速度，单位：弧度/秒。 */
    float32 theta_rad;                             /**< 原始积分航向角，单位：弧度。 */
    float32 theta_smooth_rad;                      /**< 供业务消费的平滑航向角，单位：弧度。 */
    float32 theta_accum_rad;                       /**< 不回绕的累计航向角，单位：弧度。 */
    float32 theta_smooth_accum_rad;                /**< 不回绕的平滑累计航向角，单位：弧度。 */
    float32 gyro_z_raw_rad_s;                      /**< 原始 Z 轴角速度，单位：弧度/秒。 */
    float32 gyro_z_rad_s;                          /**< Yaw rate aligned with theta_accum_rad sign, unit: rad/s. */
    float32 gyro_integral_rad;                     /**< Test-only integrated yaw from gyro Z, unit: rad. */
    float32 imu_dt_s;                              /**< 本次更新使用的积分步长，单位：秒。 */
    uint32 fused_timestamp;                        /**< 本次融合使用的时间戳。 */
    boolean gravity_ready;                         /**< 静态重力参考已建立。 */
} module_vehicle_gyro_observation_t;

typedef struct
{
    sint16 accel_raw[VEHICLE_GYRO_AXIS_COUNT]; /**< IMU acceleration raw counts, unit: LSB. */
    uint32 timestamp;                         /**< 样本时间戳，单位：IMU 计数。 */
    module_vehicle_gyro_vector3_t gyro_dps;  /**< 车体坐标系角速度，单位：度/秒。 */
    boolean valid;                           /**< 样本有效标志。 */
} module_vehicle_gyro_sample_t;

/**
 * @brief 初始化车体航向观测模块。
 * @param[in] void 无参数。
 * @return void
 */
void module_vehicle_gyro_init(void);

/**
 * @brief 将航向观测重置到指定角度。
 * @param[in] heading_rad 目标航向角，单位：弧度。
 * @return void
 */
void module_vehicle_gyro_reset_heading(float32 heading_rad);

/**
 * @brief 使用外部提供的类型化 IMU 样本推进一次航向观测。
 * @param[in] sample 按配置顺序排列的 IMU 样本数组。
 * @param[in] fallback_dt_s 时间戳不可用时的备用步长，单位：秒。
 * @return void
 */
void module_vehicle_gyro_update_from_samples(const module_vehicle_gyro_sample_t sample[VEHICLE_GYRO_IMU_COUNT],
                                             float32 fallback_dt_s);

/**
 * @brief 使用当前设备输入推进一次航向观测。
 * @param[in] fallback_dt_s 时间戳不可用时的备用步长，单位：秒。
 * @return void
 */
void module_vehicle_gyro_update(float32 fallback_dt_s);

/**
 * @brief 获取当前航向观测快照。
 * @param[in] void 无参数。
 * @return 只读航向观测状态指针。
 */
const module_vehicle_gyro_observation_t* module_vehicle_gyro_observation_get(void);

/**
 * @brief 获取当前平滑航向角。
 * @param[in] void 无参数。
 * @return 平滑航向角，单位：弧度。
 */
float32 module_vehicle_gyro_heading_get(void);

/**
 * @brief 获取当前平滑 Z 轴角速度。
 * @param[in] void 无参数。
 * @return 平滑 Z 轴角速度，单位：弧度/秒。
 */
float32 module_vehicle_gyro_yaw_rate_get(void);

/**
 * @brief 获取 IMU 上电启动等待是否完成。
 * @param[in] void 无参数。
 * @return TRUE 表示启动等待完成，IMU 航向观测可用。
 */
boolean module_vehicle_gyro_startup_ready_get(void);

/**
 * @brief Print gyro snapshot from the low-frequency service context.
 * @param[in] void No parameter.
 * @return void
 */
void module_vehicle_gyro_print_process(void);

/**
 * @brief Parse gyro print serial command.
 * @param[in] argc Command argument count.
 * @param[in] argv Command argument string array.
 * @return void
 */
void module_vehicle_gyro_print_command(uint8 argc, uint8* argv[]);

/**
 * @brief Notify gyro filtering logic of current suction ESC duty.
 * @param[in] suction_duty Current suction output duty, unit: timer ticks.
 * @return void
 */
void module_vehicle_gyro_suction_duty_notify(uint32 suction_duty);

/**
 * @brief Parse gyro notch-filter serial command.
 * @param[in] argc Command argument count.
 * @param[in] argv Command argument string array.
 * @return void
 */
void module_vehicle_gyro_notch_command(uint8 argc, uint8* argv[]);

#endif
