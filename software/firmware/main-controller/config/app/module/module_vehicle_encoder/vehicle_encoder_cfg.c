/**
 * @file vehicle_encoder_cfg.c
 * @brief 车体编码器里程估计默认配置。
 */

#include "./vehicle_encoder_cfg.h"

static const vehicle_encoder_cfg_t vehicle_encoder_cfg =
{
    .left_encoder =
    {
        .encoder_id = DEVICE_MAGNETIC_ENCODER_1,
        .direction = -1,
    },
    .right_encoder =
    {
        .encoder_id = DEVICE_MAGNETIC_ENCODER_2,
        .direction = 1,
    },
    .update_frequency_hz = 1920.0f,
    .wheel_circumference_mm = 93.9f,
    .encoder_counts_per_rev = 81920.0f, /* 16384 counts/rev * 5:1 reduction */
    .encoder_iir_alpha = 0.4f,
    .encoder_max_delta_count = 4500,
    .encoder_delta_gate_min_count = 256,
    .encoder_delta_gate_ratio = 0.35f,
};

/**
 * @brief 获取只读的车体编码器里程估计配置。
 * @param[in] void 无参数。
 * @return 车体编码器里程估计配置指针。
 */
const vehicle_encoder_cfg_t* vehicle_encoder_cfg_get(void)
{
    return &vehicle_encoder_cfg;
}
