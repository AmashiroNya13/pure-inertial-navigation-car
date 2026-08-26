/**
 * @file algorithm_odometry.c
 * @brief 里程计基础算法实现。
 */

#include "../../../inc/middleware/algorithm/algorithm_odometry.h"

#include <math.h>

/**
 * @brief 计算一圈内原始编码器计数的带回绕差值。
 * @param[in] current_raw 当前原始计数，单位：计数。
 * @param[in] last_raw 上一次原始计数，单位：计数。
 * @param[in] count_per_rev 单圈总计数，单位：计数/圈。
 * @param[in] half_count 半圈计数阈值，单位：计数。
 * @return 带回绕修正后的计数增量，单位：计数。
 */
sint16 algorithm_odometry_raw_diff_wrap(uint16 current_raw,
                                        uint16 last_raw,
                                        uint16 count_per_rev,
                                        sint32 half_count)
{
    sint32 diff_count = (sint32)current_raw - (sint32)last_raw;

    if (diff_count >= half_count)
    {
        diff_count -= (sint32)count_per_rev;
    }
    else if (diff_count < -half_count)
    {
        diff_count += (sint32)count_per_rev;
    }

    return (sint16)diff_count;
}

/**
 * @brief 将编码器计数增量换算为轮端行驶距离。
 * @param[in] delta_count 编码器计数增量，单位：计数。
 * @param[in] wheel_circumference_mm 轮子周长，单位：毫米。
 * @param[in] encoder_counts_per_rev 轮端一圈对应编码器计数，单位：计数/圈。
 * @return 轮端行驶距离增量，单位：毫米。
 */
float32 algorithm_odometry_distance_from_count(sint16 delta_count,
                                               float32 wheel_circumference_mm,
                                               float32 encoder_counts_per_rev)
{
    return ((float32)delta_count * wheel_circumference_mm) / encoder_counts_per_rev;
}

/**
 * @brief 根据航向角和距离增量积分自身平面坐标。
 * @param[in,out] x_mm X 方向坐标指针，单位：毫米。
 * @param[in,out] y_mm Y 方向坐标指针，单位：毫米。
 * @param[in,out] distance_total_mm 累计里程指针，单位：毫米。
 * @param[in] heading_rad 当前航向角，单位：弧度。
 * @param[in] delta_distance_mm 本次里程增量，单位：毫米。
 * @return void
 */
void algorithm_odometry_integrate_pose(float32* x_mm,
                                       float32* y_mm,
                                       float32* distance_total_mm,
                                       float32 heading_rad,
                                       float32 delta_distance_mm)
{
    *distance_total_mm += delta_distance_mm;
    *x_mm += delta_distance_mm * cosf(heading_rad);
    *y_mm += delta_distance_mm * sinf(heading_rad);
}
