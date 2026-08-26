/**
 * @file vehicle_path_cfg.c
 * @brief 车体路径记录与复现默认配置。
 */

#include "./vehicle_path_cfg.h"

/* External planning owns path geometry; the MCU retains speed planning and replay. */
static const vehicle_path_cfg_t vehicle_path_cfg =
{
    .max_point_cnt = VEHICLE_PATH_DEFAULT_MAX_POINT_CNT,
    .min_distance_step_mm = VEHICLE_PATH_DEFAULT_MIN_DISTANCE_MM,
    .flash_offset = VEHICLE_PATH_DEFAULT_FLASH_OFFSET,
    .flash_point_cnt_max = 65535u,
    .min_valid_point_cnt = VEHICLE_PATH_DEFAULT_MIN_VALID_POINT_CNT,
    .replay_lookahead_cnt = VEHICLE_PATH_DEFAULT_LOOKAHEAD_CNT,
    .replay_tangent_gap_cnt = VEHICLE_PATH_DEFAULT_TANGENT_GAP_CNT,
    .replay_position_gain = VEHICLE_PATH_DEFAULT_POSITION_GAIN,
    .plan_curvature_gap_cnt = VEHICLE_PATH_DEFAULT_PLAN_CURVATURE_GAP_CNT,
    .plan_min_speed_mm_s = VEHICLE_PATH_DEFAULT_PLAN_MIN_SPEED_MM_S,
    .plan_max_speed_mm_s = VEHICLE_PATH_DEFAULT_PLAN_MAX_SPEED_MM_S,
    .plan_lateral_accel_mm_s2 = VEHICLE_PATH_DEFAULT_PLAN_LATERAL_ACCEL_MM_S2,
    .plan_curve_lateral_accel_mm_s2 = VEHICLE_PATH_DEFAULT_PLAN_CURVE_LATERAL_ACCEL_MM_S2,
    .plan_accel_mm_s2 = VEHICLE_PATH_DEFAULT_PLAN_ACCEL_MM_S2,
    .plan_decel_mm_s2 = VEHICLE_PATH_DEFAULT_PLAN_DECEL_MM_S2,
    .plan_curvature_epsilon = VEHICLE_PATH_DEFAULT_PLAN_CURVATURE_EPSILON,
    .plan_straight_angle_rad = VEHICLE_PATH_DEFAULT_PLAN_STRAIGHT_ANGLE_RAD,
    .plan_geometry_enable = VEHICLE_PATH_DEFAULT_PLAN_GEOMETRY_ENABLE,
    .plan_smooth_window_cnt = VEHICLE_PATH_DEFAULT_PLAN_SMOOTH_WINDOW_CNT,
    .plan_curve_min_span_cnt = VEHICLE_PATH_DEFAULT_PLAN_CURVE_MIN_SPAN_CNT,
    .plan_target_radius_mm = VEHICLE_PATH_DEFAULT_PLAN_TARGET_RADIUS_MM,
    .plan_radius_lock_enable = VEHICLE_PATH_DEFAULT_PLAN_RADIUS_LOCK_ENABLE,
    .plan_radius_lock_gain = VEHICLE_PATH_DEFAULT_PLAN_RADIUS_LOCK_GAIN,
    .plan_radius_lock_min_ratio = VEHICLE_PATH_DEFAULT_PLAN_RADIUS_LOCK_MIN_RATIO,
    .plan_curvature_smooth_enable = VEHICLE_PATH_DEFAULT_PLAN_CURVATURE_SMOOTH_ENABLE,
    .plan_friction_circle_enable = VEHICLE_PATH_DEFAULT_PLAN_FRICTION_CIRCLE_ENABLE,
    .plan_bspline_enable = VEHICLE_PATH_DEFAULT_PLAN_BSPLINE_ENABLE,
    .plan_clothoid_enable = VEHICLE_PATH_DEFAULT_PLAN_CLOTHOID_ENABLE,
    .plan_curvature_smooth_window_cnt = VEHICLE_PATH_DEFAULT_PLAN_CURVATURE_SMOOTH_WINDOW_CNT,
    .plan_bspline_max_offset_mm = VEHICLE_PATH_DEFAULT_PLAN_BSPLINE_MAX_OFFSET_MM,
    .plan_clothoid_blend_cnt = VEHICLE_PATH_DEFAULT_PLAN_CLOTHOID_BLEND_CNT,
    .plan_long_accel_mm_s2 = VEHICLE_PATH_DEFAULT_PLAN_LONG_ACCEL_MM_S2,
    .plan_long_decel_mm_s2 = VEHICLE_PATH_DEFAULT_PLAN_LONG_DECEL_MM_S2,
};

/**
 * @brief 获取只读的车体路径记录与复现配置。
 * @param[in] void 无参数。
 * @return 车体路径记录与复现配置指针。
 */
const vehicle_path_cfg_t* vehicle_path_cfg_get(void)
{
    return &vehicle_path_cfg;
}
