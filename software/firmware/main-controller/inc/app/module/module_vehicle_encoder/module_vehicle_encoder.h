/**
 * @file module_vehicle_encoder.h
 * @brief 车体里程计模块接口。
 */

#ifndef MAD_CIRCUITS_APP_MODULE_MODULE_VEHICLE_ENCODER_H
#define MAD_CIRCUITS_APP_MODULE_MODULE_VEHICLE_ENCODER_H

#include "../../../../config/app/module/module_vehicle_encoder/vehicle_encoder_cfg.h"

typedef struct
{
    float32 distance_mm;        /**< 车体中心累计里程，单位：毫米。*/
    float32 left_speed_mm_s;    /**< 左轮速度，单位：毫米/秒。*/
    float32 right_speed_mm_s;   /**< 右轮速度，单位：毫米/秒。*/
    float32 speed_mm_s;         /**< 车体中心速度，单位：毫米/秒。*/
    sint16 left_delta_count;    /**< 左轮最近一次有效计数增量，单位：计数。*/
    sint16 right_delta_count;   /**< 右轮最近一次有效计数增量，单位：计数。*/
    float32 left_distance_mm;   /**< 左轮最近一次有效里程增量，单位：毫米。*/
    float32 right_distance_mm;  /**< 右轮最近一次有效里程增量，单位：毫米。*/
    float32 left_total_distance_mm;  /**< 左轮累计里程，单位：毫米。*/
    float32 right_total_distance_mm; /**< 右轮累计里程，单位：毫米。*/
    sint32 left_total_count;         /**< Left wheel accumulated encoder count, unit: count. */
    sint32 right_total_count;        /**< Right wheel accumulated encoder count, unit: count. */
    float32 left_raw_speed_mm_s;     /**< Left wheel instant speed before low-pass, unit: mm/s. */
    float32 right_raw_speed_mm_s;    /**< Right wheel instant speed before low-pass, unit: mm/s. */
    float32 left_sample_dt_s;        /**< Left wheel latest encoder sample dt, unit: second. */
    float32 right_sample_dt_s;       /**< Right wheel latest encoder sample dt, unit: second. */
    sint16 left_raw_delta_count;     /**< Left wheel raw delta before direction sign, unit: count. */
    sint16 right_raw_delta_count;    /**< Right wheel raw delta before direction sign, unit: count. */
    uint16 left_raw_angle;           /**< Left wheel latest raw magnetic encoder angle. */
    uint16 right_raw_angle;          /**< Right wheel latest raw magnetic encoder angle. */
    float32 wheel_distance_delta_mm; /**< Right total distance minus left total distance, unit: mm. */
    float32 theta_accum_rad;         /**< Encoder differential accumulated heading, unit: rad. */
    float32 theta_rad;               /**< Wrapped encoder differential heading, unit: rad. */
    uint32 left_sample_count;   /**< Left wheel valid sample count, unit: sample. */
    uint32 right_sample_count;  /**< Right wheel valid sample count, unit: sample. */
    uint32 left_raw_sample_count;  /**< Left wheel received sample count before validation. */
    uint32 right_raw_sample_count; /**< Right wheel received sample count before validation. */
    uint32 update_count;        /**< 有效里程更新次数，单位：次。*/
    uint32 encoder_drop_count;  /**< 编码器异常跳变丢弃次数，单位：次。*/
    uint32 left_invalid_sample_count;   /**< Left rejected encoder sample count. */
    uint32 right_invalid_sample_count;  /**< Right rejected encoder sample count. */
    uint32 left_invalid_streak_count;   /**< Left consecutive rejected sample count. */
    uint32 right_invalid_streak_count;  /**< Right consecutive rejected sample count. */
    uint16 left_rejected_raw_angle;     /**< Left latest rejected raw angle. */
    uint16 right_rejected_raw_angle;    /**< Right latest rejected raw angle. */
} module_vehicle_encoder_observation_t;

/**
 * @brief 初始化车体里程计模块。
 * @param[in] void 无参数。
 * @return void
 */
void module_vehicle_encoder_init(void);

/**
 * @brief 重置车体里程观测与轮端运行状态。
 * @param[in] void 无参数。
 * @return void
 */
void module_vehicle_encoder_reset(void);

/**
 * @brief Reset odometry and use the current magnetic encoder raw values as the next delta baseline.
 * @param[in] void No parameter.
 * @return void
 */
void module_vehicle_encoder_reset_baseline(void);

/**
 * @brief Push one magnetic encoder sample into the odometry accumulator.
 * @param[in] encoder_id Magnetic encoder device id.
 * @param[in] fallback_dt_s Fallback sample period, unit: second.
 * @return void
 */
void module_vehicle_encoder_sample(device_magnetic_encoder_id_t encoder_id,
                                   float32 fallback_dt_s);

/**
 * @brief 使用当前磁编码器缓存推进一次里程估计。
 * @param[in] fallback_dt_s 备用更新周期，单位：秒。
 * @return void
 */
void module_vehicle_encoder_update(float32 fallback_dt_s);

/**
 * @brief 获取当前车体里程观测。
 * @param[in] void 无参数。
 * @return 只读车体里程观测指针。
 */
const module_vehicle_encoder_observation_t* module_vehicle_encoder_observation_get(void);

float32 module_vehicle_encoder_wheel_circumference_get(void);

void module_vehicle_encoder_wheel_circumference_set(float32 circumference_mm);

/**
 * @brief Print encoder angle snapshot from the low-frequency service context.
 * @param[in] void No parameter.
 * @return void
 */
void module_vehicle_encoder_angle_print_process(void);

#endif
