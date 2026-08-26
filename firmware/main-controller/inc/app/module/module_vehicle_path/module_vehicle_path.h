/**
 * @file module_vehicle_path.h
 * @brief Vehicle path record, replay and speed-plan public interface.
 */

#ifndef MAD_CIRCUITS_APP_MODULE_MODULE_VEHICLE_PATH_H
#define MAD_CIRCUITS_APP_MODULE_MODULE_VEHICLE_PATH_H

#include "../../../config/app/module/module_vehicle_path/vehicle_path_cfg.h"

typedef enum
{
    VEHICLE_PATH_STATUS_IDLE = 0,
    VEHICLE_PATH_STATUS_RECORDING = 1,
    VEHICLE_PATH_STATUS_FULL = 2,
    VEHICLE_PATH_STATUS_REPLAY = 3,
    VEHICLE_PATH_STATUS_FINISHED = 4,
} vehicle_path_status_t;

typedef struct
{
    float32 x_mm;
    float32 y_mm;
    float32 theta_rad;
    float32 speed_mm_s;
} vehicle_path_point_t;

typedef enum
{
    VEHICLE_PATH_MARKER_KIND_NONE = 0,
    VEHICLE_PATH_MARKER_KIND_TURN_IN = 1,
    VEHICLE_PATH_MARKER_KIND_TURN_OUT = 2,
} vehicle_path_marker_kind_t;

typedef enum
{
    VEHICLE_PATH_REPLAY_TRACK_DENSE = 0,
    VEHICLE_PATH_REPLAY_TRACK_MARKER = 1,
    VEHICLE_PATH_REPLAY_TRACK_BLEND_TO_DENSE = 2,
    VEHICLE_PATH_REPLAY_TRACK_BLEND_TO_MARKER = 3,
} vehicle_path_replay_track_mode_t;

typedef struct
{
    vehicle_path_status_t status;
    boolean recording;
    boolean replaying;
    boolean replay_target_valid;
    uint32 point_cnt;
    uint32 dropped_cnt;
    uint32 replay_cursor_cnt;
    uint32 replay_target_index_cnt;
    float32 last_distance_mm;
    float32 replay_start_distance_mm;
    float32 replay_target_theta_rad;
} vehicle_path_state_t;

typedef struct
{
    vehicle_path_point_t base_point;
    vehicle_path_point_t target_point;
    vehicle_path_point_t tangent_point;
    float32 base_index_float;
    float32 target_index_float;
    float32 tangent_index_float;
    float32 lookahead_distance_mm;
    float32 tangent_distance_mm;
    float32 cross_track_error_mm;
    float32 along_track_error_mm;
    float32 feedforward_theta_rad;
    float32 target_theta_rad;
    float32 target_yaw_rate_rad_s;
    float32 yaw_ff_mm_s;
    float32 target_speed_mm_s;
    float32 raw_plan_speed_mm_s;
    float32 speed_preview_limit_mm_s;
    uint32 target_index_cnt;
    uint32 marker_segment_cnt;
    uint32 marker_next_cnt;
    float32 marker_blend_ratio;
    float32 marker_cross_track_error_mm;
    float32 dense_cross_track_error_mm;
    float32 marker_target_theta_rad;
    float32 dense_target_theta_rad;
    float32 progress_predicted_index_float;
    float32 progress_projected_index_float;
    float32 progress_correction_index_float;
    vehicle_path_replay_track_mode_t track_mode;
    boolean curve_section_active;
    boolean valid;
} vehicle_path_replay_target_t;

typedef struct
{
    vehicle_path_point_t point;
    float32 index_float;
    float32 distance_mm;
    float32 distance_sq_mm;
    uint32 segment_index_cnt;
    boolean valid;
} vehicle_path_projection_t;

void module_vehicle_path_init(void);
void module_vehicle_path_run(void);
boolean module_vehicle_path_start(void);
void module_vehicle_path_stop(void);
void module_vehicle_path_clear(void);
boolean module_vehicle_path_update(void);
boolean module_vehicle_path_save(void);
boolean module_vehicle_path_save_close_request(void);
boolean module_vehicle_path_save_is_busy(void);
boolean module_vehicle_path_save_is_failed(void);
boolean module_vehicle_path_load(void);
boolean module_vehicle_path_load_raw(void);
boolean module_vehicle_path_load_planned(void);
boolean module_vehicle_path_load_is_busy(void);
boolean module_vehicle_path_load_is_failed(void);
boolean module_vehicle_path_job_is_busy(void);
boolean module_vehicle_path_plan_last_result_get(boolean* success);
boolean module_vehicle_path_dump(void);
boolean module_vehicle_path_dump_raw(void);
boolean module_vehicle_path_dump_planned(void);
boolean module_vehicle_path_plan(void);
boolean module_vehicle_path_import_begin(uint32 point_cnt);
boolean module_vehicle_path_import_point(uint32 index_cnt, const vehicle_path_point_t* point);
boolean module_vehicle_path_import_marker(vehicle_path_marker_kind_t kind, uint32 index_cnt);
boolean module_vehicle_path_import_phototube_zone(uint32 start_index_cnt, uint32 end_index_cnt);
boolean module_vehicle_path_import_commit(void);
void module_vehicle_path_import_abort(void);
boolean module_vehicle_path_import_active_get(void);
boolean module_vehicle_path_import_preparing_get(void);
uint32 module_vehicle_path_import_received_count_get(void);
uint32 module_vehicle_path_import_expected_count_get(void);
uint32 module_vehicle_path_import_checksum_get(void);
uint32 module_vehicle_path_flash_point_capacity_get(void);
boolean module_vehicle_path_replay_start(void);
void module_vehicle_path_replay_stop(void);
boolean module_vehicle_path_replay_update(void);
boolean module_vehicle_path_replay_target_get(vehicle_path_replay_target_t* target);
void module_vehicle_path_replay_speed_set(float32 speed_mm_s);
float32 module_vehicle_path_replay_speed_get(void);
boolean module_vehicle_path_replay_planned_speed_active_get(void);
boolean module_vehicle_path_replay_planned_speed_available_get(void);
boolean module_vehicle_path_replay_planned_speed_requested_get(void);
boolean module_vehicle_path_replay_planned_speed_enable(boolean enable);
uint32 module_vehicle_path_replay_planned_speed_point_count_get(void);
uint32 module_vehicle_path_replay_planned_speed_source_point_count_get(void);
float32 module_vehicle_path_replay_planned_speed_source_length_get(void);
float32 module_vehicle_path_replay_planned_speed_step_get(void);
boolean module_vehicle_path_replay_planned_speed_meta_valid_get(void);
void module_vehicle_path_replay_speed_preview_distance_set(float32 distance_mm);
float32 module_vehicle_path_replay_speed_preview_distance_get(void);
void module_vehicle_path_replay_ahead_set(float32 lookahead_mm, float32 tangent_mm);
void module_vehicle_path_replay_lookahead_set(float32 lookahead_mm);
void module_vehicle_path_replay_tangent_set(float32 tangent_mm);
void module_vehicle_path_replay_ahead_reset(void);
void module_vehicle_path_replay_ahead_get(float32* lookahead_mm, float32* tangent_mm);
void module_vehicle_path_replay_lookahead_speed_cap_set(float32 speed_mm_s);
boolean module_vehicle_path_marker_add(vehicle_path_marker_kind_t kind);
boolean module_vehicle_path_marker_set(vehicle_path_marker_kind_t kind, uint32 index_cnt);
void module_vehicle_path_marker_clear(void);
void module_vehicle_path_marker_enable(boolean enable);
boolean module_vehicle_path_marker_enabled_get(void);
void module_vehicle_path_marker_print(void);
boolean module_vehicle_path_point_get(uint32 index_cnt, vehicle_path_point_t* point);
boolean module_vehicle_path_phototube_correction_allowed(uint32 index_cnt);
float32 module_vehicle_path_min_distance_step_get(void);
boolean module_vehicle_path_project_local(float32 x_mm,
                                          float32 y_mm,
                                          float32 center_index_float,
                                          float32 window_mm,
                                          vehicle_path_projection_t* projection);
const vehicle_path_state_t* module_vehicle_path_state_get(void);

#endif
