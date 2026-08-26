/**
 * @file vehicle_encoder_cfg.h
 * @brief 车体编码器里程估计配置类型和常量。
 */

#ifndef MAD_CIRCUITS_VEHICLE_ENCODER_CFG_H
#define MAD_CIRCUITS_VEHICLE_ENCODER_CFG_H

#include "Ifx_Types.h"
#include "../../../../config/device/device_magnetic_encoder/device_magnetic_encoder_cfg.h"

#define VEHICLE_ENCODER_RAW_COUNT_PER_REV ((uint16)16384u) /**< 磁编码器单圈原始计数，单位：计数/圈。 */
#define VEHICLE_ENCODER_RAW_HALF_COUNT    ((sint32)8192)   /**< 磁编码器半圈计数，用于处理回绕，单位：计数。 */

typedef struct
{
    device_magnetic_encoder_id_t encoder_id; /**< 实际使用的磁编码器设备编号。 */
    sint8 direction;                         /**< 轮向修正符号，取值范围：1 或 -1。 */
} vehicle_encoder_wheel_cfg_t;

typedef struct
{
    vehicle_encoder_wheel_cfg_t left_encoder;  /**< 左轮编码器配置。 */
    vehicle_encoder_wheel_cfg_t right_encoder; /**< 右轮编码器配置。 */
    float32 update_frequency_hz;               /**< 备用更新频率，单位：赫兹。 */
    float32 wheel_circumference_mm;            /**< 车轮周长，单位：毫米。 */
    float32 encoder_counts_per_rev;            /**< 轮端等效编码器计数，单位：计数/圈。 */
    float32 encoder_iir_alpha;                 /**< 速度一阶 IIR 系数，范围：0.0 到 1.0。 */
    sint16 encoder_max_delta_count;            /**< 单周期允许的最大计数跳变，单位：计数。 */
    sint16 encoder_delta_gate_min_count; /**< Minimum continuity-gate allowance, unit: count. */
    float32 encoder_delta_gate_ratio;    /**< Continuity allowance relative to the last valid delta. */
} vehicle_encoder_cfg_t;

/**
 * @brief 获取只读的车体编码器里程估计配置。
 * @param[in] void 无参数。
 * @return 车体编码器里程估计配置指针。
 */
const vehicle_encoder_cfg_t* vehicle_encoder_cfg_get(void);

#endif
