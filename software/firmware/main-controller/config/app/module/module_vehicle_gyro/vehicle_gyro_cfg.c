/**
 * @file vehicle_gyro_cfg.c
 * @brief 车体陀螺仪融合默认配置。
 */

#include "./vehicle_gyro_cfg.h"

static const vehicle_gyro_cfg_t vehicle_gyro_cfg =
{
    .imu =
    {
        {
            .imu_id = DEVICE_IMU_1,
            .accel_map =
            {
                { .source_axis = VEHICLE_GYRO_AXIS_X, .sign = 1 },
                { .source_axis = VEHICLE_GYRO_AXIS_Y, .sign = 1 },
                { .source_axis = VEHICLE_GYRO_AXIS_Z, .sign = 1 },
            },
            .gyro_map =
            {
                { .source_axis = VEHICLE_GYRO_AXIS_X, .sign = 1 },
                { .source_axis = VEHICLE_GYRO_AXIS_Y, .sign = 1 },
                { .source_axis = VEHICLE_GYRO_AXIS_Z, .sign = 1 },
            },
            .gyro_bias_dps =
            {
                0.0f,
                0.0f,
                0.299086f,
            },
        },
        {
            .imu_id = DEVICE_IMU_2,
            .accel_map =
            {
                { .source_axis = VEHICLE_GYRO_AXIS_X, .sign = 1 },
                { .source_axis = VEHICLE_GYRO_AXIS_Y, .sign = 1 },
                { .source_axis = VEHICLE_GYRO_AXIS_Z, .sign = 1 },
            },
            .gyro_map =
            {
                { .source_axis = VEHICLE_GYRO_AXIS_X, .sign = 1 },
                { .source_axis = VEHICLE_GYRO_AXIS_Y, .sign = 1 },
                { .source_axis = VEHICLE_GYRO_AXIS_Z, .sign = 1 },
            },
            .gyro_bias_dps =
            {
                0.0f,
                0.0f,
                0.364893f,
            },
        },
    },
    .interp_mode = VEHICLE_GYRO_INTERP_QUADRATIC,
    .gyro_scale_dps_per_lsb = 0.07f,
    .timestamp_period_s = 0.00002175f,
    .gyro_z_iir_alpha = 0.70f,
    .theta_iir_alpha = 0.60f,
    .imu_dt_min_s = 0.0002f,
    .imu_dt_max_s = 0.0004f,
};

/**
 * @brief 获取只读的车体陀螺仪融合配置。
 * @param[in] void 无参数。
 * @return 车体陀螺仪融合配置指针。
 */
const vehicle_gyro_cfg_t* vehicle_gyro_cfg_get(void)
{
    return &vehicle_gyro_cfg;
}
