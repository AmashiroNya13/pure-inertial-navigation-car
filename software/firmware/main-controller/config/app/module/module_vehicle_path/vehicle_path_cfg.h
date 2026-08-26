/**
 * @file vehicle_path_cfg.h
 * @brief Vehicle path record, replay and speed-plan configuration.
 */

#ifndef MAD_CIRCUITS_VEHICLE_PATH_CFG_H
#define MAD_CIRCUITS_VEHICLE_PATH_CFG_H

#include "Ifx_Types.h"

#define VEHICLE_PATH_MAGIC ((uint32)0x50415448u)
#define VEHICLE_PATH_VERSION ((uint16)4u)
#define VEHICLE_PATH_DEFAULT_RAM_BYTES ((uint32)32768u)
#define VEHICLE_PATH_DEFAULT_MAX_POINT_CNT 2048u
#define VEHICLE_PATH_DEFAULT_PLAN_SAMPLE_STEP_MM 5.0f
#define VEHICLE_PATH_DEFAULT_MIN_DISTANCE_MM 5.0f
#define VEHICLE_PATH_DEFAULT_FLASH_OFFSET 0u
#define VEHICLE_PATH_DEFAULT_MIN_VALID_POINT_CNT 2u
#define VEHICLE_PATH_DEFAULT_LOOKAHEAD_CNT 46u
#define VEHICLE_PATH_DEFAULT_TANGENT_GAP_CNT 10u
#define VEHICLE_PATH_DEFAULT_POSITION_GAIN 1.0f
#define VEHICLE_PATH_DEFAULT_REPLAY_END_BRAKE_DISTANCE_MM 10.0f
#define VEHICLE_PATH_DEFAULT_END_DECEL_DISTANCE_MM 300.0f
#define VEHICLE_PATH_DEFAULT_REPLAY_SPEED_MM_S 800.0f
#define VEHICLE_PATH_DEFAULT_REPLAY_SPEED_PREVIEW_DISTANCE_MM 10.0f
#define VEHICLE_PATH_DEFAULT_PLAN_CURVATURE_GAP_CNT 20u
#define VEHICLE_PATH_DEFAULT_PLAN_MIN_SPEED_MM_S 400.0f
#define VEHICLE_PATH_DEFAULT_PLAN_MAX_SPEED_MM_S 9000.0f
#define VEHICLE_PATH_DEFAULT_PLAN_LATERAL_ACCEL_MM_S2 25000.0f
#define VEHICLE_PATH_DEFAULT_PLAN_CURVE_LATERAL_ACCEL_MM_S2 25000.0f
#define VEHICLE_PATH_DEFAULT_PLAN_ACCEL_MM_S2 30000.0f
#define VEHICLE_PATH_DEFAULT_PLAN_DECEL_MM_S2 30000.0f
#define VEHICLE_PATH_DEFAULT_PLAN_CURVATURE_EPSILON 0.00005f
#define VEHICLE_PATH_DEFAULT_PLAN_STRAIGHT_ANGLE_RAD 0.1745329252f
#define VEHICLE_PATH_DEFAULT_AUTO_PLAN_ENABLE (0u)
#define VEHICLE_PATH_ONBOARD_GEOMETRY_ENABLE (0u)
#define VEHICLE_PATH_DEFAULT_PLAN_GEOMETRY_ENABLE FALSE
#define VEHICLE_PATH_DEFAULT_PLAN_SMOOTH_WINDOW_CNT 10u
#define VEHICLE_PATH_DEFAULT_PLAN_CURVE_MIN_SPAN_CNT 8u
#define VEHICLE_PATH_DEFAULT_PLAN_TARGET_RADIUS_MM 200.0f
#define VEHICLE_PATH_DEFAULT_PLAN_RADIUS_LOCK_ENABLE FALSE
#define VEHICLE_PATH_DEFAULT_PLAN_RADIUS_LOCK_GAIN 1.0f
#define VEHICLE_PATH_DEFAULT_PLAN_RADIUS_LOCK_MIN_RATIO 0.98f
#define VEHICLE_PATH_DEFAULT_PLAN_CURVATURE_SMOOTH_ENABLE VEHICLE_PATH_DEFAULT_PLAN_GEOMETRY_ENABLE
#define VEHICLE_PATH_DEFAULT_PLAN_FRICTION_CIRCLE_ENABLE TRUE
#define VEHICLE_PATH_DEFAULT_PLAN_BSPLINE_ENABLE VEHICLE_PATH_DEFAULT_PLAN_GEOMETRY_ENABLE
#define VEHICLE_PATH_DEFAULT_PLAN_CLOTHOID_ENABLE FALSE
#define VEHICLE_PATH_DEFAULT_PLAN_CURVATURE_SMOOTH_WINDOW_CNT 8u
#define VEHICLE_PATH_DEFAULT_PLAN_BSPLINE_MAX_OFFSET_MM 25.0f
#define VEHICLE_PATH_DEFAULT_PLAN_CLOTHOID_BLEND_CNT 20u
#define VEHICLE_PATH_DEFAULT_PLAN_LONG_ACCEL_MM_S2 30000.0f
#define VEHICLE_PATH_DEFAULT_PLAN_LONG_DECEL_MM_S2 30000.0f
#define VEHICLE_PATH_DEFAULT_PLAN_START_HOLD_DISTANCE_MM 200.0f
#define VEHICLE_PATH_DEFAULT_PLAN_START_HOLD_SPEED_MM_S 500.0f

typedef struct
{
    uint32 max_point_cnt;
    float32 min_distance_step_mm;
    uint32 flash_offset;
    uint16 flash_point_cnt_max;
    uint32 min_valid_point_cnt;
    uint32 replay_lookahead_cnt;
    uint32 replay_tangent_gap_cnt;
    float32 replay_position_gain;
    uint32 plan_curvature_gap_cnt;
    float32 plan_min_speed_mm_s;
    float32 plan_max_speed_mm_s;
    float32 plan_lateral_accel_mm_s2;
    float32 plan_curve_lateral_accel_mm_s2;
    float32 plan_accel_mm_s2;
    float32 plan_decel_mm_s2;
    float32 plan_curvature_epsilon;
    float32 plan_straight_angle_rad;
    boolean plan_geometry_enable;
    uint32 plan_smooth_window_cnt;
    uint32 plan_curve_min_span_cnt;
    float32 plan_target_radius_mm;
    boolean plan_radius_lock_enable;
    float32 plan_radius_lock_gain;
    float32 plan_radius_lock_min_ratio;
    boolean plan_curvature_smooth_enable;
    boolean plan_friction_circle_enable;
    boolean plan_bspline_enable;
    boolean plan_clothoid_enable;
    uint32 plan_curvature_smooth_window_cnt;
    float32 plan_bspline_max_offset_mm;
    uint32 plan_clothoid_blend_cnt;
    float32 plan_long_accel_mm_s2;
    float32 plan_long_decel_mm_s2;
} vehicle_path_cfg_t;

const vehicle_path_cfg_t* vehicle_path_cfg_get(void);

#endif
