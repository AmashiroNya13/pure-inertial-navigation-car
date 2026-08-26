/**
 * @file module_vehicle_path.c
 * @brief 车体路径记录与复现实现。
 */

#include "../../../../inc/app/module/module_vehicle_path/module_vehicle_path.h"
#include "../../../../inc/app/service/service_storage/service_storage.h"
#include "../../../../inc/app/module/module_vehicle_encoder/module_vehicle_encoder.h"
#include "../../../../inc/app/module/module_vehicle_gyro/module_vehicle_gyro.h"
#include "../../../../inc/app/module/module_vehicle_pose_fusion/module_vehicle_pose_fusion.h"
#include "../../../../inc/device/device_int_flash/device_int_flash.h"
#include "../../../../inc/driver/driver_flash/driver_flash.h"
#include "../../../../inc/middleware/tools/tools_print/tools_print.h"

#include "IfxFlash.h"
#include <math.h>

#define VEHICLE_PATH_PI (3.14159265358979323846f) /**< 圆周率常量，单位：弧度，范围：正数。 */
#define VEHICLE_PATH_RAD_TO_DEG (57.29577951308232f)
#define VEHICLE_PATH_ENCODER_WHEEL_BASE_MM (140.0f)
#define VEHICLE_PATH_ENCODER_HALF_WHEEL_BASE_MM (0.5f * VEHICLE_PATH_ENCODER_WHEEL_BASE_MM)

#define VEHICLE_PATH_CHECKSUM_SEED 0xFFFFFFFFu /**< Flash 路径点校验初始值，单位：无，范围：32 位无符号数。 */
#define VEHICLE_PATH_FLASH_HEADER_LENGTH IFXFLASH_PFLASH_PAGE_LENGTH
#define VEHICLE_PATH_FLASH_POINT_OFFSET VEHICLE_PATH_FLASH_HEADER_LENGTH
#define VEHICLE_PATH_FLASH_SLOT_BYTES ((uint32)0x00080000u)
#define VEHICLE_PATH_FLASH_RAW_OFFSET ((uint32)0u)
#define VEHICLE_PATH_FLASH_PLANNED_OFFSET (VEHICLE_PATH_FLASH_RAW_OFFSET + VEHICLE_PATH_FLASH_SLOT_BYTES)
#define VEHICLE_PATH_WINDOW_BUFFER_COUNT ((uint32)2u)
#define VEHICLE_PATH_WINDOW_BUFFER_POINT_CNT ((uint32)256u)
#define VEHICLE_PATH_WINDOW_PREFETCH_MARGIN_CNT ((uint32)64u)
#define VEHICLE_PATH_RECORD_QUEUE_USABLE_PAGE_COUNT ((uint32)32u)
#define VEHICLE_PATH_RECORD_QUEUE_SLOT_COUNT (VEHICLE_PATH_RECORD_QUEUE_USABLE_PAGE_COUNT + 1u)
#define VEHICLE_PATH_RECORD_QUEUE_DRAIN_PER_RUN ((uint32)4u)
#define VEHICLE_PATH_RECORD_PRINT_QUEUE_USABLE_COUNT ((uint32)64u)
#define VEHICLE_PATH_RECORD_PRINT_QUEUE_SLOT_COUNT (VEHICLE_PATH_RECORD_PRINT_QUEUE_USABLE_COUNT + 1u)
#define VEHICLE_PATH_RECORD_PRINT_QUEUE_DRAIN_PER_RUN ((uint32)5u)
#define VEHICLE_PATH_DUMP_POINTS_PER_RUN ((uint32)4u)
#define VEHICLE_PATH_PLAN_LOAD_POINTS_PER_RUN ((uint32)32u)
#define VEHICLE_PATH_PLAN_LONG_SCAN_POINTS_PER_RUN ((uint32)16u)
#define VEHICLE_PATH_PLAN_LONG_RESAMPLE_POINTS_PER_RUN ((uint32)16u)
#define VEHICLE_PATH_PLAN_PASS_POINTS_PER_RUN ((uint32)64u)
#define VEHICLE_PATH_PLAN_WRITE_PAGES_PER_RUN ((uint32)4u)
#define VEHICLE_PATH_PLAN_SPEED_READ_CHUNK_CNT 16u
#define VEHICLE_PATH_PLAN_RESAMPLE_EPSILON_MM (0.001f)
#define VEHICLE_PATH_PLAN_CHUNK_CORE_POINT_CNT ((uint32)768u)
#define VEHICLE_PATH_PLAN_CHUNK_OVERLAP_POINT_CNT ((uint32)128u)
#define VEHICLE_PATH_PLAN_CHUNK_MAX_POINT_CNT \
    (VEHICLE_PATH_PLAN_CHUNK_CORE_POINT_CNT + (2u * VEHICLE_PATH_PLAN_CHUNK_OVERLAP_POINT_CNT))
#define VEHICLE_PATH_PLAN_MAX_CHUNK_COUNT \
    ((((uint32)65535u) + VEHICLE_PATH_PLAN_CHUNK_CORE_POINT_CNT - 1u) \
     / VEHICLE_PATH_PLAN_CHUNK_CORE_POINT_CNT)
#define VEHICLE_PATH_PLAN_GEOMETRY_MAX_OFFSET_MM (35.0f)
#define VEHICLE_PATH_RECORD_DEFAULT_SPEED_MM_S (1200.0f)
#define VEHICLE_PATH_RECORD_COORD_SCALE_X (1.0454545f)
#define VEHICLE_PATH_RECORD_COORD_SCALE_Y (1.0272727f)
#define VEHICLE_PATH_REPLAY_PURE_SEARCH_FORWARD_MM (100.0f)
#define VEHICLE_PATH_REPLAY_PURE_SEARCH_BACK_MM (100.0f)
#define VEHICLE_PATH_REPLAY_PROGRESS_TANGENT_MM (20.0f)
#define VEHICLE_PATH_REPLAY_PROGRESS_PROJECT_WINDOW_MM (150.0f)
#define VEHICLE_PATH_REPLAY_CTE_GAIN_LOW (0.65f)
#define VEHICLE_PATH_REPLAY_CTE_GAIN_MID (0.80f)
#define VEHICLE_PATH_REPLAY_CTE_GAIN_HIGH (0.95f)
#define VEHICLE_PATH_REPLAY_CTE_GAIN_FULL (1.10f)
#define VEHICLE_PATH_REPLAY_CTE_SPEED_LOW_MM_S (800.0f)
#define VEHICLE_PATH_REPLAY_CTE_SPEED_MID_MM_S (1600.0f)
#define VEHICLE_PATH_REPLAY_CTE_SPEED_HIGH_MM_S (2300.0f)
#define VEHICLE_PATH_REPLAY_CTE_HEADING_LIMIT_LOW_RAD (12.0f / VEHICLE_PATH_RAD_TO_DEG)
#define VEHICLE_PATH_REPLAY_CTE_HEADING_LIMIT_MID_RAD (14.0f / VEHICLE_PATH_RAD_TO_DEG)
#define VEHICLE_PATH_REPLAY_CTE_HEADING_LIMIT_HIGH_RAD (18.0f / VEHICLE_PATH_RAD_TO_DEG)
#define VEHICLE_PATH_REPLAY_CTE_HEADING_LIMIT_FULL_RAD (20.0f / VEHICLE_PATH_RAD_TO_DEG)
#define VEHICLE_PATH_REPLAY_CTE_ERROR_START_MM (40.0f)
#define VEHICLE_PATH_REPLAY_CTE_ERROR_FULL_MM (200.0f)
#define VEHICLE_PATH_REPLAY_CTE_ERROR_EXTRA_MAX_RAD (10.0f / VEHICLE_PATH_RAD_TO_DEG)
#define VEHICLE_PATH_REPLAY_CTE_LIMIT_ERROR_EXTRA_MAX_RAD (12.0f / VEHICLE_PATH_RAD_TO_DEG)
#define VEHICLE_PATH_REPLAY_CTE_CURVE_FULL_RAD (45.0f / VEHICLE_PATH_RAD_TO_DEG)
#define VEHICLE_PATH_REPLAY_CTE_CURVE_SCALE_MAX (1.25f)
#define VEHICLE_PATH_REPLAY_CTE_HEADING_LIMIT_ABS_MAX_RAD (35.0f / VEHICLE_PATH_RAD_TO_DEG)
#define VEHICLE_PATH_REPLAY_AIM_SMOOTH_WINDOW_MM (30.0f)
#define VEHICLE_PATH_REPLAY_CORRECTION_STRAIGHT_CHECK_MM (30.0f)
#define VEHICLE_PATH_REPLAY_CORRECTION_STRAIGHT_HEADING_RAD (0.03f)
#define VEHICLE_PATH_REPLAY_TERMINAL_HEADING_BACK_CNT ((uint32)10u)
#define VEHICLE_PATH_PLAN_WHEEL_SPEED_EXTRA_MM_S (700.0f)
#define VEHICLE_PATH_PLAN_WHEEL_CORRECTION_RESERVE_MM_S (400.0f)
#define VEHICLE_PATH_PLAN_WHEEL_ACCEL_RESERVE_RATIO (0.85f)
#define VEHICLE_PATH_PLAN_WHEEL_COEFF_MIN (0.05f)
#define VEHICLE_PATH_MARKER_MAGIC ((uint32)0x4D41524Bu)
#define VEHICLE_PATH_MARKER_LEGACY_VERSION ((uint16)1u)
#define VEHICLE_PATH_MARKER_VERSION ((uint16)2u)
#define VEHICLE_PATH_MARKER_LEGACY_MAX_COUNT ((uint32)24u)
#define VEHICLE_PATH_MARKER_MAX_COUNT ((uint32)64u)
#define VEHICLE_PATH_MARKER_BLEND_DISTANCE_MM (100.0f)
#define VEHICLE_PATH_MARKER_MIN_SEGMENT_MM (20.0f)
#define VEHICLE_PATH_MARKER_TRACKING_AUTO ((uint8)0u)
#define VEHICLE_PATH_MARKER_TRACKING_ENABLED ((uint8)1u)
#define VEHICLE_PATH_MARKER_TRACKING_DISABLED ((uint8)2u)
#define VEHICLE_PATH_PLAN_META_MAGIC ((uint32)0x504D4554u)
#define VEHICLE_PATH_PLAN_META_VERSION ((uint16)2u)
#define VEHICLE_PATH_PLAN_META_STANDALONE_UPLOAD ((uint16)0x0001u)
#define VEHICLE_PATH_PHOTOTUBE_ZONE_MAGIC ((uint32)0x50545A4Eu)
#define VEHICLE_PATH_PHOTOTUBE_ZONE_VERSION ((uint16)1u)
#define VEHICLE_PATH_PHOTOTUBE_ZONE_MAX_COUNT ((uint32)128u)
#define VEHICLE_PATH_FLASH_MARKER_V2_LENGTH \
    ((((uint32)sizeof(vehicle_path_flash_marker_block_t) \
       + (uint32)sizeof(vehicle_path_flash_plan_meta_t) \
       + IFXFLASH_PFLASH_PAGE_LENGTH - 1u) \
      / IFXFLASH_PFLASH_PAGE_LENGTH) * IFXFLASH_PFLASH_PAGE_LENGTH)
#define VEHICLE_PATH_FLASH_MARKER_V2_OFFSET \
    (VEHICLE_PATH_FLASH_SLOT_BYTES - VEHICLE_PATH_FLASH_MARKER_V2_LENGTH)
#define VEHICLE_PATH_FLASH_MARKER_LENGTH \
    ((((uint32)sizeof(vehicle_path_flash_marker_block_t) \
       + (uint32)sizeof(vehicle_path_flash_plan_meta_t) \
       + (uint32)sizeof(vehicle_path_flash_phototube_zone_block_t) \
       + IFXFLASH_PFLASH_PAGE_LENGTH - 1u) \
      / IFXFLASH_PFLASH_PAGE_LENGTH) * IFXFLASH_PFLASH_PAGE_LENGTH)
#define VEHICLE_PATH_FLASH_MARKER_OFFSET (VEHICLE_PATH_FLASH_SLOT_BYTES - VEHICLE_PATH_FLASH_MARKER_LENGTH)
#define VEHICLE_PATH_FLASH_MARKER_LEGACY_LENGTH \
    ((((uint32)sizeof(vehicle_path_flash_marker_legacy_block_t) \
       + (uint32)sizeof(vehicle_path_flash_plan_meta_t) \
       + IFXFLASH_PFLASH_PAGE_LENGTH - 1u) \
      / IFXFLASH_PFLASH_PAGE_LENGTH) * IFXFLASH_PFLASH_PAGE_LENGTH)
#define VEHICLE_PATH_FLASH_MARKER_LEGACY_OFFSET \
    (VEHICLE_PATH_FLASH_SLOT_BYTES - VEHICLE_PATH_FLASH_MARKER_LEGACY_LENGTH)

typedef char vehicle_path_ram_size_check_t[
    ((VEHICLE_PATH_DEFAULT_MAX_POINT_CNT * sizeof(vehicle_path_point_t)) <= VEHICLE_PATH_DEFAULT_RAM_BYTES) ? 1 : -1];
typedef char vehicle_path_plan_chunk_size_check_t[
    ((VEHICLE_PATH_PLAN_CHUNK_MAX_POINT_CNT * 2u) <= VEHICLE_PATH_DEFAULT_MAX_POINT_CNT) ? 1 : -1];
typedef char vehicle_path_flash_point_page_alignment_check_t[
    ((IFXFLASH_PFLASH_PAGE_LENGTH % (uint32)sizeof(vehicle_path_point_t)) == 0u) ? 1 : -1];
typedef char vehicle_path_plan_chunk_page_alignment_check_t[
    (((VEHICLE_PATH_PLAN_CHUNK_CORE_POINT_CNT * (uint32)sizeof(vehicle_path_point_t))
      % IFXFLASH_PFLASH_PAGE_LENGTH) == 0u) ? 1 : -1];

typedef struct
{
    uint32 magic;     /**< Flash 路径记录魔术字。 */
    uint16 version;   /**< Flash 路径记录格式版本。 */
    uint16 point_cnt; /**< Flash 中保存的路径点数量，单位：点。 */
    uint32 checksum;  /**< Flash 路径点数据校验值，单位：无。 */
} vehicle_path_flash_header_t;

typedef struct
{
    uint32 index_cnt;
    uint8 kind;
    uint8 reserved[3];
} vehicle_path_flash_marker_record_t;

typedef struct
{
    uint32 magic;
    uint16 version;
    uint16 marker_cnt;
    uint32 checksum;
    vehicle_path_flash_marker_record_t markers[VEHICLE_PATH_MARKER_MAX_COUNT];
} vehicle_path_flash_marker_block_t;

typedef struct
{
    uint32 magic;
    uint16 version;
    uint16 marker_cnt;
    uint32 checksum;
    vehicle_path_flash_marker_record_t markers[VEHICLE_PATH_MARKER_LEGACY_MAX_COUNT];
} vehicle_path_flash_marker_legacy_block_t;

typedef struct
{
    uint32 magic;
    uint16 version;
    uint16 planned_point_cnt;
    uint16 source_point_cnt;
    uint16 reserved;
    uint32 source_checksum;
    float32 source_length_mm;
    float32 planned_step_mm;
} vehicle_path_flash_plan_meta_t;

typedef struct
{
    uint16 start_index_cnt;
    uint16 end_index_cnt;
} vehicle_path_phototube_zone_t;

typedef struct
{
    uint32 magic;
    uint16 version;
    uint16 zone_cnt;
    uint32 checksum;
    vehicle_path_phototube_zone_t zones[VEHICLE_PATH_PHOTOTUBE_ZONE_MAX_COUNT];
} vehicle_path_flash_phototube_zone_block_t;

typedef char vehicle_path_marker_slot_size_check_t[
    ((VEHICLE_PATH_FLASH_MARKER_OFFSET > VEHICLE_PATH_FLASH_POINT_OFFSET)
     && ((VEHICLE_PATH_FLASH_MARKER_OFFSET + VEHICLE_PATH_FLASH_MARKER_LENGTH)
         <= VEHICLE_PATH_FLASH_SLOT_BYTES)) ? 1 : -1];

typedef char vehicle_path_plan_meta_slot_size_check_t[
    (((uint32)sizeof(vehicle_path_flash_marker_block_t)
      + (uint32)sizeof(vehicle_path_flash_plan_meta_t))
     <= VEHICLE_PATH_FLASH_MARKER_LENGTH) ? 1 : -1];

typedef char vehicle_path_phototube_zone_slot_size_check_t[
    (((uint32)sizeof(vehicle_path_flash_marker_block_t)
      + (uint32)sizeof(vehicle_path_flash_plan_meta_t)
      + (uint32)sizeof(vehicle_path_flash_phototube_zone_block_t))
     <= VEHICLE_PATH_FLASH_MARKER_LENGTH) ? 1 : -1];

typedef char vehicle_path_plan_meta_legacy_address_check_t[
    (((VEHICLE_PATH_FLASH_MARKER_V2_OFFSET + (uint32)sizeof(vehicle_path_flash_marker_block_t))
      == (VEHICLE_PATH_FLASH_MARKER_LEGACY_OFFSET
          + (uint32)sizeof(vehicle_path_flash_marker_legacy_block_t)))) ? 1 : -1];

typedef enum
{
    VEHICLE_PATH_LOAD_STATE_IDLE = 0,   /**< 加载空闲状态，调用加载接口后退出。 */
    VEHICLE_PATH_LOAD_STATE_HEADER = 1, /**< 正在读取路径头状态，头部读取完成或失败后退出。 */
    VEHICLE_PATH_LOAD_STATE_POINTS = 2, /**< 正在读取路径点状态，路径点读取完成或失败后退出。 */
    VEHICLE_PATH_LOAD_STATE_FAILED = 3, /**< 加载失败状态，重新调用加载接口后退出。 */
} vehicle_path_load_state_t;

typedef enum
{
    VEHICLE_PATH_JOB_IDLE = 0,
    VEHICLE_PATH_JOB_DUMP_HEADER = 1,
    VEHICLE_PATH_JOB_DUMP_POINTS = 2,
    VEHICLE_PATH_JOB_PLAN_HEADER = 3,
    VEHICLE_PATH_JOB_PLAN_LONG_SCAN = 4,
    VEHICLE_PATH_JOB_PLAN_LONG_RESAMPLE = 5,
    VEHICLE_PATH_JOB_PLAN_LOAD_POINTS = 6,
    VEHICLE_PATH_JOB_PLAN_GEOMETRY = 7,
    VEHICLE_PATH_JOB_PLAN_FORWARD = 8,
    VEHICLE_PATH_JOB_PLAN_BACKWARD = 9,
    VEHICLE_PATH_JOB_PLAN_SUMMARY = 10,
    VEHICLE_PATH_JOB_PLAN_PREPARE_WRITE = 11,
    VEHICLE_PATH_JOB_PLAN_WRITE_POINTS = 12,
    VEHICLE_PATH_JOB_PLAN_WRITE_HEADER = 13,
    VEHICLE_PATH_JOB_PLAN_FIXED_CHUNK_LOAD = 14,
    VEHICLE_PATH_JOB_PLAN_FIXED_CHUNK_APPLY = 15,
    VEHICLE_PATH_JOB_PLAN_FIXED_FORWARD = 16,
    VEHICLE_PATH_JOB_PLAN_FIXED_BACKWARD = 17,
    VEHICLE_PATH_JOB_PLAN_FIXED_WRITE_LOAD = 18,
    VEHICLE_PATH_JOB_PLAN_FIXED_WRITE_APPLY = 19,
    VEHICLE_PATH_JOB_PLAN_FIXED_WRITE_PAGES = 20,
    VEHICLE_PATH_JOB_PLAN_FIXED_REVERSE_LOAD = 21,
    VEHICLE_PATH_JOB_PLAN_FIXED_REVERSE_APPLY = 22,
} vehicle_path_job_state_t;

typedef enum
{
    VEHICLE_PATH_PLAN_CHUNK_PASS_FORWARD = 0,
    VEHICLE_PATH_PLAN_CHUNK_PASS_REVERSE = 1,
    VEHICLE_PATH_PLAN_CHUNK_PASS_WRITE = 2,
} vehicle_path_plan_chunk_pass_t;

typedef struct
{
    uint32 index_cnt;
    vehicle_path_marker_kind_t kind;
    vehicle_path_point_t point;
    boolean valid;
} vehicle_path_marker_t;

typedef struct
{
    vehicle_path_point_t base_point;
    vehicle_path_point_t target_point;
    vehicle_path_point_t tangent_point;
    float32 base_index_float;
    float32 target_index_float;
    float32 tangent_index_float;
    float32 cross_track_error_mm;
    float32 along_track_error_mm;
    float32 target_theta_rad;
    float32 feedforward_theta_rad;
    uint32 segment_cnt;
    uint32 next_cnt;
    float32 blend_ratio;
    vehicle_path_replay_track_mode_t mode;
} vehicle_path_marker_target_t;

typedef struct
{
    vehicle_path_point_t point;
    float32 record_distance_mm;
    float32 left_total_distance_mm;
    float32 right_total_distance_mm;
    sint32 left_total_count;
    sint32 right_total_count;
    uint32 left_sample_count;
    uint32 right_sample_count;
    uint32 point_index_cnt;
    uint32 pose_update_count;
    uint32 encoder_update_count;
} vehicle_path_record_print_item_t;

typedef struct
{
    vehicle_path_state_t state;                                    /**< 对外路径记录与复现状态。 */
    vehicle_path_replay_target_t replay_target;                    /**< 当前路径复现目标。 */
    volatile boolean save_busy;                                    /**< TRUE while CPU0 drains queued record pages. */
    vehicle_path_load_state_t load_state;                          /**< 最近一次同步加载状态。 */
    uint32 load_point_length;                                      /**< 最近一次加载点数据长度，单位：字节。 */
    uint16 load_point_cnt;                                         /**< 最近一次加载点数量，单位：点。 */
    uint32 load_checksum;                                          /**< 最近一次加载校验值。 */
    uint32 window_start_index_cnt;                                 /**< RAM window first path point index. */
    uint32 window_valid_cnt;                                       /**< RAM window valid point count. */
    uint32 window_active_buffer_cnt;                               /**< Active half-buffer used by Flash window reads. */
    uint32 window_buffer_start_index_cnt[VEHICLE_PATH_WINDOW_BUFFER_COUNT]; /**< First global point in each half-buffer. */
    uint32 window_buffer_valid_cnt[VEHICLE_PATH_WINDOW_BUFFER_COUNT];       /**< Valid point count in each half-buffer. */
    boolean window_buffer_loaded[VEHICLE_PATH_WINDOW_BUFFER_COUNT];         /**< TRUE when half-buffer metadata is valid. */
    uint32 window_prefetch_start_index_cnt;                        /**< Prefetched window first path point index. */
    uint32 window_prefetch_valid_cnt;                              /**< Prefetched window valid point count. */
    uint32 window_prefetch_buffer_cnt;                             /**< Half-buffer used by the prefetched window. */
    uint32 window_prefetch_request_start_index_cnt;                 /**< Pending prefetch first path point index. */
    boolean window_from_flash;                                     /**< TRUE when RAM is a partial Flash window. */
    boolean window_prefetch_valid;                                 /**< TRUE when next Flash window is already in RAM. */
    boolean window_prefetch_pending;                               /**< TRUE when background prefetch should run. */
    uint32 active_slot_offset;                                      /**< Flash slot used by replay/window reads. */
    uint8 record_page[IFXFLASH_PFLASH_PAGE_LENGTH];                /**< Record streaming page buffer. */
    uint32 record_page_start_offset;                               /**< Record page offset from path point area. */
    uint32 record_page_used_length;                                /**< Valid bytes in record page buffer. */
    uint32 record_checksum;                                        /**< Streaming checksum for recorded path points. */
    uint32 record_stream_slot_offset;
    uint32 import_expected_point_cnt;
    uint32 import_received_point_cnt;
    boolean import_active;
    boolean import_commit_pending;
    boolean import_erase_active;
    boolean import_erase_delay;
    uint32 import_erase_data_sector_count;
    uint32 import_erase_sector_count;
    uint32 import_erase_sector_index;
    volatile boolean record_stream_active;                         /**< TRUE while record points are streamed to Flash. */
    volatile boolean record_stream_failed;                         /**< TRUE when streaming write failed. */
    volatile boolean record_stream_close_requested;                /**< TRUE after async write close is requested. */
    volatile boolean record_save_failed;
    uint8 record_queue[VEHICLE_PATH_RECORD_QUEUE_SLOT_COUNT][IFXFLASH_PFLASH_PAGE_LENGTH];
    uint32 record_queue_offset[VEHICLE_PATH_RECORD_QUEUE_SLOT_COUNT];
    volatile uint32 record_queue_head;
    volatile uint32 record_queue_tail;
    vehicle_path_record_print_item_t record_print_queue[VEHICLE_PATH_RECORD_PRINT_QUEUE_SLOT_COUNT];
    volatile uint32 record_print_queue_head;
    volatile uint32 record_print_queue_tail;
    volatile uint32 record_print_dropped_cnt;
    uint32 record_last_pose_update_count;
    vehicle_path_point_t record_last_point;
    float32 record_path_length_mm;
    vehicle_path_job_state_t job_state;
    vehicle_path_flash_header_t job_header;
    uint32 job_index_cnt;
    uint32 job_point_cnt;
    uint32 job_source_point_cnt;
    uint32 job_checksum;
    uint32 job_write_offset;
    uint32 job_read_slot_offset;
    uint32 job_write_slot_offset;
    uint32 job_planned_cnt;
    float32 job_min_speed_mm_s;
    float32 job_max_speed_mm_s;
    float32 job_path_length_mm;
    vehicle_path_point_t job_last_point;
    boolean job_has_last_point;
    boolean job_downsampled;
    uint32 job_resample_next_index_cnt;
    uint32 job_resample_target_index_cnt;
    float32 job_resample_step_mm;
    float32 job_resample_segment_start_mm;
    float32 job_resample_segment_end_mm;
    vehicle_path_point_t job_resample_prev_point;
    vehicle_path_point_t job_resample_next_point;
    uint32 job_chunk_core_start_cnt;
    uint32 job_chunk_core_count;
    uint32 job_chunk_input_start_cnt;
    uint32 job_chunk_input_count;
    uint32 job_chunk_loaded_count;
    uint32 job_chunk_core_offset_cnt;
    uint32 job_chunk_write_offset_cnt;
    vehicle_path_plan_chunk_pass_t job_chunk_pass;
    float32 job_chunk_forward_start_speed_mm_s[VEHICLE_PATH_PLAN_MAX_CHUNK_COUNT];
    float32 job_chunk_backward_next_speed_mm_s[VEHICLE_PATH_PLAN_MAX_CHUNK_COUNT];
    boolean job_chunk_backward_next_valid[VEHICLE_PATH_PLAN_MAX_CHUNK_COUNT];
    uint32 job_chunk_source_next_index_cnt[VEHICLE_PATH_PLAN_MAX_CHUNK_COUNT];
    float32 job_chunk_source_segment_start_mm[VEHICLE_PATH_PLAN_MAX_CHUNK_COUNT];
    float32 job_chunk_source_segment_end_mm[VEHICLE_PATH_PLAN_MAX_CHUNK_COUNT];
    boolean job_chunk_source_valid[VEHICLE_PATH_PLAN_MAX_CHUNK_COUNT];
    float32 job_speed_pass_carry_mm_s;
    boolean job_speed_pass_carry_valid;
    float32 job_marker_source_distance_mm[VEHICLE_PATH_MARKER_MAX_COUNT];
    boolean job_marker_source_distance_valid[VEHICLE_PATH_MARKER_MAX_COUNT];
    boolean job_result_valid;
    boolean job_result_success;
    float32 replay_last_target_theta_rad;
    boolean replay_target_theta_initialized;
    float32 replay_progress_index_float;
    float32 replay_progress_traveled_mm;
    float32 replay_progress_predicted_index_float;
    float32 replay_progress_projected_index_float;
    float32 replay_progress_correction_index_float;
    boolean replay_progress_valid;
    vehicle_path_marker_t markers[VEHICLE_PATH_MARKER_MAX_COUNT];
    uint32 marker_cnt;
    vehicle_path_phototube_zone_t phototube_zones[VEHICLE_PATH_PHOTOTUBE_ZONE_MAX_COUNT];
    uint32 phototube_zone_cnt;
    uint8 marker_tracking_mode;
    vehicle_path_point_t replay_first_point;
    vehicle_path_point_t replay_last_point;
    boolean replay_edge_points_valid;
    boolean planned_speed_overlay_active;
    boolean planned_speed_enabled;
    uint32 planned_speed_point_cnt;
    uint32 planned_speed_source_point_cnt;
    float32 planned_speed_source_length_mm;
    float32 planned_speed_step_mm;
    boolean planned_speed_meta_valid;
    uint32 source_distance_cache_index_cnt;
    float32 source_distance_cache_mm;
    boolean source_distance_cache_valid;
    uint16 planned_speed_table_mm_s[VEHICLE_PATH_DEFAULT_MAX_POINT_CNT];
#if VEHICLE_PATH_ONBOARD_GEOMETRY_ENABLE
    float32 plan_source_x_mm[VEHICLE_PATH_PLAN_CHUNK_MAX_POINT_CNT];
    float32 plan_source_y_mm[VEHICLE_PATH_PLAN_CHUNK_MAX_POINT_CNT];
#endif
    uint8 job_page[IFXFLASH_PFLASH_PAGE_LENGTH];
    vehicle_path_point_t points[VEHICLE_PATH_DEFAULT_MAX_POINT_CNT]; /**< Path-point workspace. */
} vehicle_path_runtime_t;

static vehicle_path_runtime_t vehicle_path_runtime;
static uint32 vehicle_path_marker_page_words[
    VEHICLE_PATH_FLASH_MARKER_LENGTH / (uint32)sizeof(uint32)];
static float32 vehicle_path_replay_speed_mm_s = VEHICLE_PATH_DEFAULT_REPLAY_SPEED_MM_S;
static float32 vehicle_path_replay_lookahead_speed_cap_mm_s = 0.0f;
static float32 vehicle_path_replay_speed_preview_distance_mm =
    VEHICLE_PATH_DEFAULT_REPLAY_SPEED_PREVIEW_DISTANCE_MM;
static uint32 vehicle_path_replay_lookahead_cnt_runtime = VEHICLE_PATH_DEFAULT_LOOKAHEAD_CNT;
static uint32 vehicle_path_replay_tangent_gap_cnt_runtime = VEHICLE_PATH_DEFAULT_TANGENT_GAP_CNT;

/**
 * @brief 获取有效最大路径点数量。
 * @param[in] void 无参数。
 * @return 最大路径点数量，单位：点。
 */
static uint32 vehicle_path_max_point_cnt_get(void);
static uint32 vehicle_path_flash_slot_point_capacity_get(void);
static uint32 vehicle_path_effective_flash_point_cnt_max_get(void);
static boolean vehicle_path_flash_slot_range_valid(uint32 slot_offset, uint32 point_cnt);
static boolean vehicle_path_flash_slot_read_range_valid(uint32 slot_offset, uint32 point_cnt);
static void vehicle_path_planned_slot_erase_begin(uint32 point_cnt);
static void vehicle_path_planned_slot_erase_run(void);
static void vehicle_path_planned_slot_erase_reset(void);

/**
 * @brief 获取有效最小路径记录距离间隔。
 * @param[in] void 无参数。
 * @return 最小路径记录距离间隔，单位：毫米。
 */
static float32 vehicle_path_min_distance_step_get(void);

/**
 * @brief 获取有效路径复现前瞻点数量。
 * @param[in] void 无参数。
 * @return 路径复现前瞻点数量，单位：点。
 */
static uint32 vehicle_path_replay_lookahead_cnt_get(void);
static uint32 vehicle_path_replay_tangent_gap_cnt_get(void);
static void vehicle_path_replay_target_reset(void);

/**
 * @brief 获取有效路径复现切线计算间隔。
 * @param[in] void 无参数。
 * @return 路径复现切线计算间隔，单位：点。
 */

/**
 * @brief 重置路径记录与复现运行状态。
 * @param[in] status 目标路径模块状态。
 * @return void
 */
static void vehicle_path_state_reset(vehicle_path_status_t status);
static void vehicle_path_runtime_origin_reset(float32 x_mm, float32 y_mm, float32 theta_rad);

/**
 * @brief 将当前编码器位姿记录到路径缓存。
 * @param[in] void 无参数。
 * @return 成功记录路径点返回 TRUE，否则返回 FALSE。
 */
static boolean vehicle_path_record_current_pose(void);
static void vehicle_path_record_pose_point_make(const module_vehicle_pose_fusion_observation_t* pose_observation,
                                                vehicle_path_point_t* point);
static boolean vehicle_path_record_point_append(const vehicle_path_point_t* point,
                                                float32 distance_mm,
                                                uint32 pose_update_count);

/**
 * @brief 计算路径点数据校验值。
 * @param[in] data 路径点原始字节数据指针。
 * @param[in] length 数据长度，单位：字节。
 * @return 路径点数据校验值。
 */
static uint32 vehicle_path_checksum_calculate(const uint8* data, uint32 length);
static uint32 vehicle_path_checksum_update(uint32 checksum, const uint8* data, uint32 length);
static void vehicle_path_record_stream_reset(void);
static void vehicle_path_record_stream_start(uint32 slot_offset);
static boolean vehicle_path_record_stream_append(const vehicle_path_point_t* point);
static boolean vehicle_path_record_stream_flush(boolean force);
static void vehicle_path_record_queue_reset(void);
static boolean vehicle_path_record_queue_push(uint32 offset, const uint8* page);
static boolean vehicle_path_record_queue_is_empty(void);
static void vehicle_path_record_queue_drain(uint32 max_page_count);
static void vehicle_path_record_save_finish_try(void);
static void vehicle_path_record_print_queue_reset(void);
static boolean vehicle_path_record_print_queue_push(const vehicle_path_point_t* point,
                                                    float32 record_distance_mm,
                                                    uint32 point_index_cnt,
                                                    uint32 pose_update_count);
static boolean vehicle_path_record_print_queue_is_empty(void);
static void vehicle_path_record_print_queue_drain(uint32 max_point_count);
static void vehicle_path_job_reset(void);
static boolean vehicle_path_job_busy(void);
static boolean vehicle_path_operation_busy(void);
static boolean vehicle_path_load_from_slot(uint32 slot_offset, const vehicle_path_flash_header_t* header);
static boolean vehicle_path_flash_header_read_from(uint32 slot_offset, vehicle_path_flash_header_t* header);
static boolean vehicle_path_flash_header_valid_at(uint32 slot_offset, vehicle_path_flash_header_t* header);
static boolean vehicle_path_planned_header_matches_raw(const vehicle_path_flash_header_t* planned_header,
                                                       uint16* raw_point_cnt);
static boolean vehicle_path_planned_standalone_valid(const vehicle_path_flash_header_t* planned_header);
static void vehicle_path_planned_speed_overlay_reset(void);
static void vehicle_path_planned_speed_meta_reset(void);
static void vehicle_path_planned_speed_meta_set(uint32 source_point_cnt,
                                                float32 source_length_mm,
                                                uint32 planned_point_cnt,
                                                boolean meta_valid);
static void vehicle_path_planned_speed_meta_set_from_job(uint32 planned_point_cnt);
static boolean vehicle_path_planned_speed_distance_map_valid(void);
static uint16 vehicle_path_speed_mm_s_to_u16(float32 speed_mm_s);
static boolean vehicle_path_planned_speed_table_load(const vehicle_path_flash_header_t* planned_header);
static void vehicle_path_planned_speed_table_store_from_points(uint32 point_cnt);
static void vehicle_path_plan_speed_overlay_activate_failed_reset(void);
static boolean vehicle_path_plan_speed_overlay_activate_from_job(void);
static float32 vehicle_path_planned_speed_at_index_get(float32 raw_index_float, float32 fallback_speed_mm_s);
static float32 vehicle_path_planned_speed_at_scaled_index_get(float32 raw_index_float,
                                                             float32 fallback_speed_mm_s);
static float32 vehicle_path_planned_speed_preview_limit_get(float32 base_index_float,
                                                            float32 base_source_distance_mm,
                                                            boolean base_source_distance_valid,
                                                            float32 fallback_speed_mm_s,
                                                            float32* preview_index_float);
static void vehicle_path_header_page_prepare(uint8* header_page, const vehicle_path_flash_header_t* header);
static boolean vehicle_path_marker_write(uint32 slot_offset, uint32 point_cnt);
static uint32 vehicle_path_marker_checksum_calculate(
    const vehicle_path_flash_marker_record_t* markers,
    uint32 marker_cnt);
static void vehicle_path_plan_meta_page_append(uint32 slot_offset, uint32 point_cnt, uint8* marker_page);
static boolean vehicle_path_plan_meta_read(vehicle_path_flash_plan_meta_t* meta);
static void vehicle_path_marker_reset(void);
static boolean vehicle_path_marker_tracking_enabled_get(void);
static boolean vehicle_path_marker_kind_valid(vehicle_path_marker_kind_t kind);
static const char* vehicle_path_marker_kind_name(vehicle_path_marker_kind_t kind);
static void vehicle_path_marker_sort(void);
static boolean vehicle_path_marker_refresh_points(void);
static boolean vehicle_path_marker_load_from_slot(uint32 slot_offset);
static void vehicle_path_phototube_zone_reset(void);
static boolean vehicle_path_phototube_zone_load_from_slot(uint32 slot_offset);
static boolean vehicle_path_replay_edge_points_refresh(void);
static boolean vehicle_path_dump_from_slot(uint32 slot_offset);
static void vehicle_path_background_job_run(void);
static void vehicle_path_dump_job_run(void);
static void vehicle_path_plan_job_run(void);
static boolean vehicle_path_plan_load_points_step(uint32 max_point_count);
static boolean vehicle_path_plan_source_point_read(uint32 index_cnt, vehicle_path_point_t* point);
static boolean vehicle_path_plan_long_scan_step(uint32 max_point_count);
static boolean vehicle_path_plan_long_resample_begin(void);
static boolean vehicle_path_plan_long_resample_step(uint32 max_point_count);
static boolean vehicle_path_plan_fixed_begin(void);
static void vehicle_path_plan_chunk_begin(vehicle_path_plan_chunk_pass_t pass);
static boolean vehicle_path_plan_chunk_resample_begin(uint32 chunk_cnt);
static boolean vehicle_path_plan_chunk_load_step(uint32 max_point_count);
static void vehicle_path_plan_chunk_apply(void);
static boolean vehicle_path_plan_chunk_write_step(uint32 max_page_count);
static boolean vehicle_path_plan_fixed_finish(void);
static void vehicle_path_plan_markers_remap_to_planned(void);
static void vehicle_path_plan_geometry_compute(uint32 point_cnt, boolean apply_start_speed);
static boolean vehicle_path_plan_forward_step(uint32 max_point_count);
static boolean vehicle_path_plan_backward_step(uint32 max_point_count);
static boolean vehicle_path_plan_summary_step(uint32 max_point_count);
static boolean vehicle_path_plan_write_points_step(uint32 max_page_count);
static boolean vehicle_path_header_write(uint32 slot_offset, uint32 point_cnt, uint32 checksum);
#if VEHICLE_PATH_ONBOARD_GEOMETRY_ENABLE
static void vehicle_path_recalculate_theta(uint32 point_cnt);
#endif
static vehicle_path_point_t* vehicle_path_window_buffer_get(uint32 buffer_index_cnt);
static uint32 vehicle_path_window_inactive_buffer_get(void);
static void vehicle_path_window_buffers_reset(void);
static void vehicle_path_window_buffer_invalidate(uint32 buffer_index_cnt);
static void vehicle_path_window_buffer_mark(uint32 buffer_index_cnt,
                                            uint32 start_index_cnt,
                                            uint32 valid_cnt);
static boolean vehicle_path_window_buffer_contains(uint32 buffer_index_cnt, uint32 index_cnt);
static boolean vehicle_path_window_buffer_read(uint32 buffer_index_cnt,
                                               uint32 index_cnt,
                                               vehicle_path_point_t* point);
static void vehicle_path_window_active_set(uint32 buffer_index_cnt);
static void vehicle_path_window_prefetch_reset(void);
static boolean vehicle_path_window_read_to_buffer(uint32 start_index_cnt,
                                                  uint32 buffer_index_cnt,
                                                  uint32* read_count);
static void vehicle_path_window_prefetch_schedule(uint32 start_index_cnt);
static void vehicle_path_window_prefetch_update(uint32 high_index_cnt);
static void vehicle_path_window_prefetch_run(void);
static boolean vehicle_path_window_prefetch_contains(uint32 index_cnt);
static boolean vehicle_path_window_prefetch_promote(uint32 index_cnt);
static boolean vehicle_path_window_load(uint32 start_index_cnt);
static boolean vehicle_path_window_contains(uint32 index_cnt);
static uint32 vehicle_path_window_start_align(uint32 index_cnt);
static boolean vehicle_path_point_read_cached(uint32 index_cnt, vehicle_path_point_t* point);
static boolean vehicle_path_point_read_preserve_window(uint32 index_cnt, vehicle_path_point_t* point);
static boolean vehicle_path_point_read(uint32 index_cnt, vehicle_path_point_t* point);
static void vehicle_path_source_distance_cache_reset(void);
static boolean vehicle_path_distance_point_read(uint32 index_cnt, vehicle_path_point_t* point);
static boolean vehicle_path_distance_at_index_get(float32 index_float, float32* distance_mm);
static float32 vehicle_path_points_length_calculate(uint32 point_cnt);
static boolean vehicle_path_replay_pure_progress_find(float32 x_mm, float32 y_mm, float32* base_index_float);
static boolean vehicle_path_replay_nearest_point_find(uint32 start_index_cnt,
                                                      uint32 end_index_cnt,
                                                      float32 x_mm,
                                                      float32 y_mm,
                                                      uint32* nearest_index_cnt);
static boolean vehicle_path_replay_tangent_target_find(float32 base_index_float,
                                                       float32 lookahead_mm,
                                                       float32 tangent_distance_mm,
                                                       float32* target_index_float,
                                                       float32* tangent_index_float,
                                                       vehicle_path_point_t* target_point,
                                                       vehicle_path_point_t* tangent_point,
                                                       float32* target_theta_rad);
static boolean vehicle_path_replay_smoothed_point_get(float32 center_index_float,
                                                      float32 window_mm,
                                                      vehicle_path_point_t* smooth_point);
static float32 vehicle_path_replay_point_heading_get(const vehicle_path_point_t* from_point,
                                                     const vehicle_path_point_t* to_point,
                                                     float32 fallback_heading_rad);
static boolean vehicle_path_replay_heading_to_point_get(
    const module_vehicle_pose_fusion_observation_t* pose_observation,
    const vehicle_path_point_t* target_point,
    float32 fallback_heading_rad,
    float32* target_theta_rad,
    float32* distance_mm);
static float32 vehicle_path_replay_cross_track_heading_gain_get(float32 target_speed_mm_s);
static float32 vehicle_path_replay_cross_track_error_ratio_get(float32 cross_track_error_mm);
static float32 vehicle_path_replay_cross_track_curve_scale_get(float32 path_curve_delta_rad);
static float32 vehicle_path_replay_cross_track_heading_limit_get(float32 target_speed_mm_s,
                                                                 float32 cross_track_error_mm,
                                                                 float32 path_curve_delta_rad);
static float32 vehicle_path_replay_cross_track_heading_correction_get(float32 cross_track_error_mm,
                                                                      float32 lookahead_mm,
                                                                      float32 target_speed_mm_s,
                                                                      float32 path_curve_delta_rad);
static float32 vehicle_path_replay_target_heading_get(float32 path_heading_rad,
                                                      float32 cross_track_error_mm,
                                                      float32 lookahead_mm,
                                                      float32 target_speed_mm_s,
                                                      float32 path_curve_delta_rad);
static boolean vehicle_path_marker_tracking_target_get(float32 dense_base_index_float,
                                                       float32 dense_target_index_float,
                                                       float32 lookahead_mm,
                                                       float32 tangent_distance_mm,
                                                       float32 target_speed_mm_s,
                                                       vehicle_path_marker_target_t* marker_target);
static boolean vehicle_path_marker_segment_target_get(uint32 start_index_cnt,
                                                      const vehicle_path_point_t* start_point,
                                                      uint32 end_index_cnt,
                                                      const vehicle_path_point_t* end_point,
                                                      float32 lookahead_mm,
                                                      float32 tangent_distance_mm,
                                                      float32 target_speed_mm_s,
                                                      vehicle_path_marker_target_t* marker_target);
static float32 vehicle_path_smoothstep(float32 ratio);
static float32 vehicle_path_angle_blend(float32 start_rad, float32 end_rad, float32 ratio);
static void vehicle_path_point_blend(const vehicle_path_point_t* start_point,
                                     const vehicle_path_point_t* end_point,
                                     float32 ratio,
                                     vehicle_path_point_t* output_point);
static float32 vehicle_path_segment_length_mm(const vehicle_path_point_t* point_a,
                                              const vehicle_path_point_t* point_b);
static boolean vehicle_path_replay_terminal_heading_get(float32* heading_rad, float32* step_mm);
static boolean vehicle_path_point_at_index_float(float32 index_float, vehicle_path_point_t* point);
static boolean vehicle_path_index_advance_by_distance(float32 start_index_float,
                                                      float32 distance_mm,
                                                      float32* target_index_float);
static float32 vehicle_path_replay_lookahead_distance_mm_get(void);
static float32 vehicle_path_replay_fixed_lookahead_distance_mm_get(void);
static uint32 vehicle_path_replay_distance_to_cnt(float32 distance_mm);
static float32 vehicle_path_replay_traveled_mm_get(void);
static float32 vehicle_path_replay_remaining_mm_get(float32 base_index_float);
static float32 vehicle_path_end_speed_scale_get(float32 remaining_mm);
static float32 vehicle_path_replay_target_theta_continuous(float32 raw_theta_rad);
static boolean vehicle_path_replay_planned_speed_active(void);
static float32 vehicle_path_wrap_pi(float32 angle_rad);
static float32 vehicle_path_plan_curvature_get(uint32 index_cnt, float32* chord_length_mm);
static float32 vehicle_path_plan_effective_curvature_get(uint32 index_cnt);
#if VEHICLE_PATH_ONBOARD_GEOMETRY_ENABLE
static boolean vehicle_path_plan_geometry_index_locked(uint32 index_cnt, uint32 point_cnt);
static void vehicle_path_plan_smooth_points(uint32 point_cnt);
static void vehicle_path_plan_bspline_apply(uint32 point_cnt);
static void vehicle_path_plan_clothoid_apply(uint32 point_cnt);
static void vehicle_path_plan_curvature_smooth_apply(uint32 point_cnt);
static void vehicle_path_plan_radius_lock_apply(uint32 point_cnt);
#endif
static void vehicle_path_plan_curve_speed_apply(uint32 point_cnt);
static void vehicle_path_plan_start_speed_apply(uint32 point_cnt);
static void vehicle_path_plan_end_speed_apply(uint32 point_cnt);
static void vehicle_path_plan_chunk_end_speed_apply(void);
static float32 vehicle_path_plan_base_speed_get(float32 curvature, float32 chord_length_mm);
static float32 vehicle_path_plan_available_long_accel_get(float32 speed_mm_s, float32 curvature, boolean decel);
static float32 vehicle_path_plan_abs_f32(float32 value);
static float32 vehicle_path_plan_wheel_coeff_get(float32 curvature, boolean right_wheel);
static float32 vehicle_path_plan_wheel_speed_limit_get(void);
static float32 vehicle_path_plan_wheel_static_speed_cap_get(float32 curvature);
static float32 vehicle_path_plan_wheel_transition_speed_cap_get(float32 current_cap_mm_s,
                                                               float32 source_speed_mm_s,
                                                               float32 source_curvature,
                                                               float32 target_curvature,
                                                               float32 distance_mm,
                                                               float32 accel_limit_mm_s2);
static float32 vehicle_path_plan_speed_clamp(float32 speed_mm_s);
static float32 vehicle_path_plan_stored_speed_clamp(float32 speed_mm_s);

/**
 * @brief Find replay progress using the old pure-pursuit nearest-point rule.
 * @return TRUE when a nearest path point is found.
 */
static boolean vehicle_path_replay_pure_progress_find(float32 x_mm, float32 y_mm, float32* base_index_float)
{
    uint32 point_cnt = vehicle_path_runtime.state.point_cnt;
    uint32 cursor = vehicle_path_runtime.state.replay_cursor_cnt;
    float32 step_mm = vehicle_path_min_distance_step_get();
    uint32 search_back_cnt;
    uint32 search_forward_cnt;
    uint32 search_start;
    uint32 search_end;
    uint32 nearest_index_cnt;

    if ((base_index_float == NULL_PTR) || (point_cnt < 2u))
    {
        return FALSE;
    }

    if (cursor >= point_cnt)
    {
        cursor = point_cnt - 1u;
    }

    if (step_mm < 1.0f)
    {
        step_mm = 1.0f;
    }

    search_back_cnt = (uint32)((VEHICLE_PATH_REPLAY_PURE_SEARCH_BACK_MM / step_mm) + 0.5f);
    if (search_back_cnt < 1u)
    {
        search_back_cnt = 1u;
    }

    search_forward_cnt = (uint32)((VEHICLE_PATH_REPLAY_PURE_SEARCH_FORWARD_MM / step_mm) + 0.5f);
    if (search_forward_cnt < 1u)
    {
        search_forward_cnt = 1u;
    }

    search_start = (cursor > search_back_cnt) ? (cursor - search_back_cnt) : 0u;
    search_end = cursor + search_forward_cnt;
    if (search_end >= point_cnt)
    {
        search_end = point_cnt - 1u;
    }

    if (vehicle_path_replay_nearest_point_find(search_start,
                                               search_end,
                                               x_mm,
                                               y_mm,
                                               &nearest_index_cnt) == FALSE)
    {
        return FALSE;
    }

    *base_index_float = (float32)nearest_index_cnt;
    return TRUE;
}

static boolean vehicle_path_replay_nearest_point_find(uint32 start_index_cnt,
                                                      uint32 end_index_cnt,
                                                      float32 x_mm,
                                                      float32 y_mm,
                                                      uint32* nearest_index_cnt)
{
    uint32 point_cnt = vehicle_path_runtime.state.point_cnt;
    uint32 index_cnt;
    float32 best_distance_sq = 0.0f;
    boolean found = FALSE;

    if ((nearest_index_cnt == NULL_PTR) || (point_cnt == 0u))
    {
        return FALSE;
    }

    if (start_index_cnt >= point_cnt)
    {
        return FALSE;
    }

    if (end_index_cnt >= point_cnt)
    {
        end_index_cnt = point_cnt - 1u;
    }

    if (start_index_cnt > end_index_cnt)
    {
        return FALSE;
    }

    for (index_cnt = start_index_cnt; index_cnt <= end_index_cnt; index_cnt++)
    {
        vehicle_path_point_t point;
        float32 dx;
        float32 dy;
        float32 distance_sq;

        if (vehicle_path_point_read(index_cnt, &point) == FALSE)
        {
            continue;
        }

        dx = x_mm - point.x_mm;
        dy = y_mm - point.y_mm;
        distance_sq = (dx * dx) + (dy * dy);

        if ((found == FALSE) || (distance_sq < best_distance_sq))
        {
            best_distance_sq = distance_sq;
            *nearest_index_cnt = index_cnt;
            found = TRUE;
        }
    }

    return found;
}

static boolean vehicle_path_replay_tangent_target_find(float32 base_index_float,
                                                       float32 lookahead_mm,
                                                       float32 tangent_distance_mm,
                                                       float32* target_index_float,
                                                       float32* tangent_index_float,
                                                       vehicle_path_point_t* target_point,
                                                       vehicle_path_point_t* tangent_point,
                                                       float32* target_theta_rad)
{
    float32 dx_mm;
    float32 dy_mm;
    float32 tangent_length_sq_mm;
    float32 terminal_heading_rad;
    float32 terminal_step_mm;

    if ((target_index_float == NULL_PTR) || (tangent_index_float == NULL_PTR)
        || (target_point == NULL_PTR) || (tangent_point == NULL_PTR) || (target_theta_rad == NULL_PTR))
    {
        return FALSE;
    }

    if (lookahead_mm < 1.0f)
    {
        lookahead_mm = 1.0f;
    }
    if (tangent_distance_mm < 1.0f)
    {
        tangent_distance_mm = vehicle_path_min_distance_step_get();
        if (tangent_distance_mm < 1.0f)
        {
            tangent_distance_mm = 1.0f;
        }
    }

    if ((vehicle_path_index_advance_by_distance(base_index_float, lookahead_mm, target_index_float) == FALSE)
        || (vehicle_path_point_at_index_float(*target_index_float, target_point) == FALSE))
    {
        return FALSE;
    }

    if ((vehicle_path_index_advance_by_distance(*target_index_float, tangent_distance_mm, tangent_index_float) == FALSE)
        || (vehicle_path_point_at_index_float(*tangent_index_float, tangent_point) == FALSE))
    {
        return FALSE;
    }

    dx_mm = tangent_point->x_mm - target_point->x_mm;
    dy_mm = tangent_point->y_mm - target_point->y_mm;
    tangent_length_sq_mm = (dx_mm * dx_mm) + (dy_mm * dy_mm);

    if (tangent_length_sq_mm < 0.001f)
    {
        if (vehicle_path_replay_terminal_heading_get(&terminal_heading_rad, &terminal_step_mm) != FALSE)
        {
            *target_theta_rad = terminal_heading_rad;
        }
        else
        {
            *target_theta_rad = target_point->theta_rad;
        }
    }
    else
    {
        *target_theta_rad = atan2f(dy_mm, dx_mm);
    }

    *target_theta_rad = vehicle_path_wrap_pi(*target_theta_rad);
    target_point->theta_rad = *target_theta_rad;
    return TRUE;
}

static boolean vehicle_path_replay_smoothed_point_get(float32 center_index_float,
                                                      float32 window_mm,
                                                      vehicle_path_point_t* smooth_point)
{
    uint32 point_cnt = vehicle_path_runtime.state.point_cnt;
    uint32 center_index_cnt;
    uint32 window_cnt;
    uint32 start_index_cnt;
    uint32 end_index_cnt;
    uint32 index_cnt;
    float32 step_mm = vehicle_path_min_distance_step_get();
    float32 weight_sum = 0.0f;
    float32 x_sum_mm = 0.0f;
    float32 y_sum_mm = 0.0f;
    float32 speed_sum_mm_s = 0.0f;
    float32 theta_sin_sum = 0.0f;
    float32 theta_cos_sum = 0.0f;

    if ((smooth_point == NULL_PTR) || (point_cnt == 0u))
    {
        return FALSE;
    }

    if (step_mm < 0.001f)
    {
        step_mm = 1.0f;
    }
    if (window_mm < step_mm)
    {
        window_mm = step_mm;
    }

    if (center_index_float < 0.0f)
    {
        center_index_float = 0.0f;
    }
    if (center_index_float > (float32)(point_cnt - 1u))
    {
        center_index_float = (float32)(point_cnt - 1u);
    }

    center_index_cnt = (uint32)(center_index_float + 0.5f);
    if (center_index_cnt >= point_cnt)
    {
        center_index_cnt = point_cnt - 1u;
    }

    window_cnt = vehicle_path_replay_distance_to_cnt(window_mm);
    start_index_cnt = (center_index_cnt > window_cnt) ? (center_index_cnt - window_cnt) : 0u;
    end_index_cnt = center_index_cnt + window_cnt;
    if (end_index_cnt >= point_cnt)
    {
        end_index_cnt = point_cnt - 1u;
    }

    for (index_cnt = start_index_cnt; index_cnt <= end_index_cnt; index_cnt++)
    {
        vehicle_path_point_t point;
        float32 index_delta = (float32)index_cnt - center_index_float;
        float32 distance_mm;
        float32 weight;

        if (index_delta < 0.0f)
        {
            index_delta = -index_delta;
        }

        distance_mm = index_delta * step_mm;
        weight = 1.0f - (distance_mm / (window_mm + step_mm));
        if (weight <= 0.0f)
        {
            continue;
        }

        if (vehicle_path_point_read(index_cnt, &point) == FALSE)
        {
            continue;
        }

        weight_sum += weight;
        x_sum_mm += point.x_mm * weight;
        y_sum_mm += point.y_mm * weight;
        speed_sum_mm_s += point.speed_mm_s * weight;
        theta_sin_sum += sinf(point.theta_rad) * weight;
        theta_cos_sum += cosf(point.theta_rad) * weight;
    }

    if (weight_sum < 0.001f)
    {
        return vehicle_path_point_at_index_float(center_index_float, smooth_point);
    }

    smooth_point->x_mm = x_sum_mm / weight_sum;
    smooth_point->y_mm = y_sum_mm / weight_sum;
    smooth_point->theta_rad = atan2f(theta_sin_sum, theta_cos_sum);
    smooth_point->speed_mm_s = speed_sum_mm_s / weight_sum;
    return TRUE;
}

static float32 vehicle_path_replay_point_heading_get(const vehicle_path_point_t* from_point,
                                                     const vehicle_path_point_t* to_point,
                                                     float32 fallback_heading_rad)
{
    float32 dx_mm;
    float32 dy_mm;

    if ((from_point == NULL_PTR) || (to_point == NULL_PTR))
    {
        return vehicle_path_wrap_pi(fallback_heading_rad);
    }

    dx_mm = to_point->x_mm - from_point->x_mm;
    dy_mm = to_point->y_mm - from_point->y_mm;
    if (((dx_mm * dx_mm) + (dy_mm * dy_mm)) < 0.001f)
    {
        return vehicle_path_wrap_pi(fallback_heading_rad);
    }

    return vehicle_path_wrap_pi(atan2f(dy_mm, dx_mm));
}

static boolean vehicle_path_replay_heading_to_point_get(
    const module_vehicle_pose_fusion_observation_t* pose_observation,
    const vehicle_path_point_t* target_point,
    float32 fallback_heading_rad,
    float32* target_theta_rad,
    float32* distance_mm)
{
    float32 dx_mm;
    float32 dy_mm;
    float32 distance_sq_mm;

    if ((pose_observation == NULL_PTR) || (target_point == NULL_PTR) || (target_theta_rad == NULL_PTR))
    {
        return FALSE;
    }

    dx_mm = target_point->x_mm - pose_observation->x_mm;
    dy_mm = target_point->y_mm - pose_observation->y_mm;
    distance_sq_mm = (dx_mm * dx_mm) + (dy_mm * dy_mm);
    if (distance_mm != NULL_PTR)
    {
        *distance_mm = sqrtf(distance_sq_mm);
    }

    if (distance_sq_mm < 1.0f)
    {
        *target_theta_rad = vehicle_path_wrap_pi(fallback_heading_rad);
    }
    else
    {
        *target_theta_rad = vehicle_path_wrap_pi(atan2f(dy_mm, dx_mm));
    }

    return TRUE;
}

static float32 vehicle_path_replay_cross_track_heading_gain_get(float32 target_speed_mm_s)
{
    if (target_speed_mm_s < 0.0f)
    {
        target_speed_mm_s = -target_speed_mm_s;
    }

    if (target_speed_mm_s < VEHICLE_PATH_REPLAY_CTE_SPEED_LOW_MM_S)
    {
        return VEHICLE_PATH_REPLAY_CTE_GAIN_LOW;
    }
    if (target_speed_mm_s < VEHICLE_PATH_REPLAY_CTE_SPEED_MID_MM_S)
    {
        return VEHICLE_PATH_REPLAY_CTE_GAIN_MID;
    }
    if (target_speed_mm_s < VEHICLE_PATH_REPLAY_CTE_SPEED_HIGH_MM_S)
    {
        return VEHICLE_PATH_REPLAY_CTE_GAIN_HIGH;
    }

    return VEHICLE_PATH_REPLAY_CTE_GAIN_FULL;
}

static float32 vehicle_path_replay_cross_track_error_ratio_get(float32 cross_track_error_mm)
{
    float32 error_abs_mm = cross_track_error_mm;

    if (error_abs_mm < 0.0f)
    {
        error_abs_mm = -error_abs_mm;
    }

    if (error_abs_mm <= VEHICLE_PATH_REPLAY_CTE_ERROR_START_MM)
    {
        return 0.0f;
    }
    if (error_abs_mm >= VEHICLE_PATH_REPLAY_CTE_ERROR_FULL_MM)
    {
        return 1.0f;
    }

    return (error_abs_mm - VEHICLE_PATH_REPLAY_CTE_ERROR_START_MM)
           / (VEHICLE_PATH_REPLAY_CTE_ERROR_FULL_MM - VEHICLE_PATH_REPLAY_CTE_ERROR_START_MM);
}

static float32 vehicle_path_replay_cross_track_curve_scale_get(float32 path_curve_delta_rad)
{
    float32 curve_abs_rad = path_curve_delta_rad;
    float32 ratio;

    if (curve_abs_rad < 0.0f)
    {
        curve_abs_rad = -curve_abs_rad;
    }

    if (curve_abs_rad <= 0.0f)
    {
        return 1.0f;
    }

    ratio = curve_abs_rad / VEHICLE_PATH_REPLAY_CTE_CURVE_FULL_RAD;
    if (ratio > 1.0f)
    {
        ratio = 1.0f;
    }

    return 1.0f + ((VEHICLE_PATH_REPLAY_CTE_CURVE_SCALE_MAX - 1.0f) * ratio);
}

static float32 vehicle_path_replay_cross_track_heading_limit_get(float32 target_speed_mm_s,
                                                                 float32 cross_track_error_mm,
                                                                 float32 path_curve_delta_rad)
{
    float32 heading_limit_rad;
    float32 error_ratio;
    float32 curve_scale;

    if (target_speed_mm_s < 0.0f)
    {
        target_speed_mm_s = -target_speed_mm_s;
    }

    if (target_speed_mm_s < VEHICLE_PATH_REPLAY_CTE_SPEED_LOW_MM_S)
    {
        heading_limit_rad = VEHICLE_PATH_REPLAY_CTE_HEADING_LIMIT_LOW_RAD;
    }
    else if (target_speed_mm_s < VEHICLE_PATH_REPLAY_CTE_SPEED_MID_MM_S)
    {
        heading_limit_rad = VEHICLE_PATH_REPLAY_CTE_HEADING_LIMIT_MID_RAD;
    }
    else if (target_speed_mm_s < VEHICLE_PATH_REPLAY_CTE_SPEED_HIGH_MM_S)
    {
        heading_limit_rad = VEHICLE_PATH_REPLAY_CTE_HEADING_LIMIT_HIGH_RAD;
    }
    else
    {
        heading_limit_rad = VEHICLE_PATH_REPLAY_CTE_HEADING_LIMIT_FULL_RAD;
    }

    error_ratio = vehicle_path_replay_cross_track_error_ratio_get(cross_track_error_mm);
    curve_scale = vehicle_path_replay_cross_track_curve_scale_get(path_curve_delta_rad);
    heading_limit_rad =
        (heading_limit_rad + (VEHICLE_PATH_REPLAY_CTE_LIMIT_ERROR_EXTRA_MAX_RAD * error_ratio)) * curve_scale;
    if (heading_limit_rad > VEHICLE_PATH_REPLAY_CTE_HEADING_LIMIT_ABS_MAX_RAD)
    {
        heading_limit_rad = VEHICLE_PATH_REPLAY_CTE_HEADING_LIMIT_ABS_MAX_RAD;
    }

    return heading_limit_rad;
}

static float32 vehicle_path_replay_cross_track_heading_correction_get(float32 cross_track_error_mm,
                                                                      float32 lookahead_mm,
                                                                      float32 target_speed_mm_s,
                                                                      float32 path_curve_delta_rad)
{
    float32 heading_correction_rad;
    float32 heading_limit_rad;
    float32 speed_gain;
    float32 error_ratio;
    float32 error_extra_rad;
    float32 curve_scale;

    if (lookahead_mm < 1.0f)
    {
        lookahead_mm = 1.0f;
    }

    speed_gain = vehicle_path_replay_cross_track_heading_gain_get(target_speed_mm_s);
    error_ratio = vehicle_path_replay_cross_track_error_ratio_get(cross_track_error_mm);
    error_extra_rad = VEHICLE_PATH_REPLAY_CTE_ERROR_EXTRA_MAX_RAD * error_ratio;
    if (cross_track_error_mm > 0.0f)
    {
        error_extra_rad = -error_extra_rad;
    }
    else if (cross_track_error_mm == 0.0f)
    {
        error_extra_rad = 0.0f;
    }

    curve_scale = vehicle_path_replay_cross_track_curve_scale_get(path_curve_delta_rad);
    heading_correction_rad =
        (atan2f(-(cross_track_error_mm * speed_gain), lookahead_mm) + error_extra_rad) * curve_scale;
    heading_limit_rad =
        vehicle_path_replay_cross_track_heading_limit_get(target_speed_mm_s,
                                                          cross_track_error_mm,
                                                          path_curve_delta_rad);
    if (heading_correction_rad > heading_limit_rad)
    {
        heading_correction_rad = heading_limit_rad;
    }
    else if (heading_correction_rad < -heading_limit_rad)
    {
        heading_correction_rad = -heading_limit_rad;
    }

    return heading_correction_rad;
}

static float32 vehicle_path_replay_target_heading_get(float32 path_heading_rad,
                                                      float32 cross_track_error_mm,
                                                      float32 lookahead_mm,
                                                      float32 target_speed_mm_s,
                                                      float32 path_curve_delta_rad)
{
    return vehicle_path_wrap_pi(
        path_heading_rad
        + vehicle_path_replay_cross_track_heading_correction_get(cross_track_error_mm,
                                                                 lookahead_mm,
                                                                 target_speed_mm_s,
                                                                 path_curve_delta_rad));
}

static float32 vehicle_path_segment_length_mm(const vehicle_path_point_t* point_a,
                                              const vehicle_path_point_t* point_b)
{
    float32 dx;
    float32 dy;

    if ((point_a == NULL_PTR) || (point_b == NULL_PTR))
    {
        return 0.0f;
    }

    dx = point_b->x_mm - point_a->x_mm;
    dy = point_b->y_mm - point_a->y_mm;
    return sqrtf((dx * dx) + (dy * dy));
}

static boolean vehicle_path_replay_terminal_heading_get(float32* heading_rad, float32* step_mm)
{
    uint32 point_cnt = vehicle_path_runtime.state.point_cnt;
    uint32 start_index_cnt;
    uint32 span_cnt;
    vehicle_path_point_t start_point;
    vehicle_path_point_t end_point;
    float32 dx;
    float32 dy;
    float32 length_mm;

    if ((heading_rad == NULL_PTR) || (step_mm == NULL_PTR) || (point_cnt < 2u))
    {
        return FALSE;
    }

    span_cnt = point_cnt - 1u;
    if (span_cnt > VEHICLE_PATH_REPLAY_TERMINAL_HEADING_BACK_CNT)
    {
        span_cnt = VEHICLE_PATH_REPLAY_TERMINAL_HEADING_BACK_CNT;
    }
    if (span_cnt < 1u)
    {
        span_cnt = 1u;
    }

    start_index_cnt = (point_cnt - 1u) - span_cnt;
    if ((vehicle_path_point_read(start_index_cnt, &start_point) == FALSE)
        || (vehicle_path_point_read(point_cnt - 1u, &end_point) == FALSE))
    {
        return FALSE;
    }

    dx = end_point.x_mm - start_point.x_mm;
    dy = end_point.y_mm - start_point.y_mm;
    length_mm = sqrtf((dx * dx) + (dy * dy));
    if (length_mm < 0.001f)
    {
        return FALSE;
    }

    *heading_rad = atan2f(dy, dx);
    *step_mm = length_mm / (float32)span_cnt;
    if (*step_mm < 0.001f)
    {
        *step_mm = vehicle_path_min_distance_step_get();
    }
    if (*step_mm < 0.001f)
    {
        *step_mm = 1.0f;
    }

    return TRUE;
}

static boolean vehicle_path_point_at_index_float(float32 index_float, vehicle_path_point_t* point)
{
    uint32 point_cnt = vehicle_path_runtime.state.point_cnt;
    uint32 index_cnt;
    uint32 next_index_cnt;
    float32 ratio;
    vehicle_path_point_t point_a;
    vehicle_path_point_t point_b;

    if ((point == NULL_PTR) || (point_cnt == 0u))
    {
        return FALSE;
    }

    if (index_float <= 0.0f)
    {
        return vehicle_path_point_read(0u, point);
    }

    index_cnt = (uint32)index_float;
    if (index_float >= (float32)(point_cnt - 1u))
    {
        float32 heading_rad;
        float32 step_mm;

        if (vehicle_path_point_read(point_cnt - 1u, point) == FALSE)
        {
            return FALSE;
        }

        if (vehicle_path_replay_terminal_heading_get(&heading_rad, &step_mm) != FALSE)
        {
            point->theta_rad = heading_rad;
        }

        return TRUE;
    }

    next_index_cnt = index_cnt + 1u;
    ratio = index_float - (float32)index_cnt;

    if ((vehicle_path_point_read(index_cnt, &point_a) == FALSE)
        || (vehicle_path_point_read(next_index_cnt, &point_b) == FALSE))
    {
        return FALSE;
    }

    point->x_mm = point_a.x_mm + ((point_b.x_mm - point_a.x_mm) * ratio);
    point->y_mm = point_a.y_mm + ((point_b.y_mm - point_a.y_mm) * ratio);
    point->theta_rad = point_a.theta_rad + (vehicle_path_wrap_pi(point_b.theta_rad - point_a.theta_rad) * ratio);
    point->speed_mm_s = point_a.speed_mm_s + ((point_b.speed_mm_s - point_a.speed_mm_s) * ratio);
    return TRUE;
}

static boolean vehicle_path_index_advance_by_distance(float32 start_index_float,
                                                      float32 distance_mm,
                                                      float32* target_index_float)
{
    uint32 point_cnt = vehicle_path_runtime.state.point_cnt;
    uint32 index_cnt;
    float32 ratio;
    float32 remain_mm;

    if ((target_index_float == NULL_PTR) || (point_cnt < 2u))
    {
        return FALSE;
    }

    if (start_index_float < 0.0f)
    {
        start_index_float = 0.0f;
    }

    if (start_index_float >= (float32)(point_cnt - 1u))
    {
        *target_index_float = (float32)(point_cnt - 1u);
        return TRUE;
    }

    if (distance_mm <= 0.0f)
    {
        *target_index_float = start_index_float;
        return TRUE;
    }

    index_cnt = (uint32)start_index_float;
    ratio = start_index_float - (float32)index_cnt;
    remain_mm = distance_mm;

    while (index_cnt < (point_cnt - 1u))
    {
        vehicle_path_point_t point_a;
        vehicle_path_point_t point_b;
        float32 segment_length_mm;
        float32 segment_remain_mm;

        if ((vehicle_path_point_read(index_cnt, &point_a) == FALSE)
            || (vehicle_path_point_read(index_cnt + 1u, &point_b) == FALSE))
        {
            return FALSE;
        }

        segment_length_mm = vehicle_path_segment_length_mm(&point_a, &point_b);
        if (segment_length_mm < 0.001f)
        {
            index_cnt++;
            ratio = 0.0f;
            continue;
        }

        segment_remain_mm = segment_length_mm * (1.0f - ratio);
        if (remain_mm <= segment_remain_mm)
        {
            *target_index_float = (float32)index_cnt + ratio + (remain_mm / segment_length_mm);
            return TRUE;
        }

        remain_mm -= segment_remain_mm;
        index_cnt++;
        ratio = 0.0f;
    }

    *target_index_float = (float32)(point_cnt - 1u);
    return TRUE;
}

static float32 vehicle_path_replay_lookahead_distance_mm_get(void)
{
    return vehicle_path_replay_fixed_lookahead_distance_mm_get();
}

static float32 vehicle_path_replay_fixed_lookahead_distance_mm_get(void)
{
    float32 distance_mm = (float32)vehicle_path_replay_lookahead_cnt_get()
                        * vehicle_path_min_distance_step_get();

    if (distance_mm < 40.0f)
    {
        distance_mm = 40.0f;
    }

    return distance_mm;
}

static float32 vehicle_path_replay_remaining_mm_get(float32 base_index_float)
{
    uint32 point_cnt = vehicle_path_runtime.state.point_cnt;
    float32 path_length_mm = vehicle_path_runtime.state.last_distance_mm;
    float32 progress_distance_mm;
    float32 progress_ratio;

    if (point_cnt < 2u)
    {
        return 0.0f;
    }

    if ((vehicle_path_planned_speed_distance_map_valid() != FALSE)
        && (vehicle_path_distance_at_index_get(base_index_float, &progress_distance_mm) != FALSE))
    {
        path_length_mm = vehicle_path_runtime.planned_speed_source_length_mm;
        if (progress_distance_mm >= path_length_mm)
        {
            return 0.0f;
        }

        return path_length_mm - progress_distance_mm;
    }

    if (path_length_mm <= 0.0f)
    {
        path_length_mm = (float32)(point_cnt - 1u) * vehicle_path_min_distance_step_get();
    }

    if (base_index_float <= 0.0f)
    {
        return path_length_mm;
    }
    if (base_index_float >= (float32)(point_cnt - 1u))
    {
        return 0.0f;
    }

    progress_ratio = base_index_float / (float32)(point_cnt - 1u);
    return path_length_mm * (1.0f - progress_ratio);
}

static float32 vehicle_path_end_speed_scale_get(float32 remaining_mm)
{
    float32 decel_span_mm = VEHICLE_PATH_DEFAULT_END_DECEL_DISTANCE_MM
                          - VEHICLE_PATH_DEFAULT_REPLAY_END_BRAKE_DISTANCE_MM;

    if (remaining_mm <= VEHICLE_PATH_DEFAULT_REPLAY_END_BRAKE_DISTANCE_MM)
    {
        return 0.0f;
    }
    if ((decel_span_mm <= 0.0f)
        || (remaining_mm >= VEHICLE_PATH_DEFAULT_END_DECEL_DISTANCE_MM))
    {
        return 1.0f;
    }

    return sqrtf((remaining_mm - VEHICLE_PATH_DEFAULT_REPLAY_END_BRAKE_DISTANCE_MM)
                 / decel_span_mm);
}

static float32 vehicle_path_replay_traveled_mm_get(void)
{
    const module_vehicle_pose_fusion_observation_t* pose_observation =
        module_vehicle_pose_fusion_observation_get();
    float32 traveled_mm = pose_observation->distance_mm
                        - vehicle_path_runtime.state.replay_start_distance_mm;

    if (traveled_mm < 0.0f)
    {
        traveled_mm = -traveled_mm;
    }

    return traveled_mm;
}

static float32 vehicle_path_replay_target_theta_continuous(float32 raw_theta_rad)
{
    float32 target_theta_rad = vehicle_path_wrap_pi(raw_theta_rad);

    if (vehicle_path_runtime.replay_target_theta_initialized == FALSE)
    {
        vehicle_path_runtime.replay_last_target_theta_rad = target_theta_rad;
        vehicle_path_runtime.replay_target_theta_initialized = TRUE;
        return target_theta_rad;
    }

    while ((target_theta_rad - vehicle_path_runtime.replay_last_target_theta_rad) > VEHICLE_PATH_PI)
    {
        target_theta_rad -= (2.0f * VEHICLE_PATH_PI);
    }
    while ((target_theta_rad - vehicle_path_runtime.replay_last_target_theta_rad) < -VEHICLE_PATH_PI)
    {
        target_theta_rad += (2.0f * VEHICLE_PATH_PI);
    }

    vehicle_path_runtime.replay_last_target_theta_rad = target_theta_rad;
    return target_theta_rad;
}

static boolean vehicle_path_replay_planned_speed_active(void)
{
    return (boolean)((vehicle_path_runtime.planned_speed_overlay_active != FALSE)
                     && (vehicle_path_runtime.planned_speed_enabled != FALSE));
}

static float32 vehicle_path_plan_curvature_get(uint32 index_cnt, float32* chord_length_mm)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();
    uint32 point_cnt = vehicle_path_runtime.state.point_cnt;
    uint32 gap_cnt = cfg->plan_curvature_gap_cnt;
    uint32 prev_index_cnt;
    uint32 next_index_cnt;
    float32 a_len;
    float32 b_len;
    float32 c_len;
    float32 cross;
    vehicle_path_point_t prev_point;
    vehicle_path_point_t current_point;
    vehicle_path_point_t next_point;

    if (chord_length_mm != NULL_PTR)
    {
        *chord_length_mm = 0.0f;
    }

    if (point_cnt < 3u)
    {
        return 0.0f;
    }

    if (gap_cnt == 0u)
    {
        gap_cnt = 1u;
    }

    prev_index_cnt = (index_cnt > gap_cnt) ? (index_cnt - gap_cnt) : 0u;
    next_index_cnt = index_cnt + gap_cnt;
    if (next_index_cnt >= point_cnt)
    {
        next_index_cnt = point_cnt - 1u;
    }

    if ((prev_index_cnt == index_cnt) || (next_index_cnt == index_cnt) || (prev_index_cnt == next_index_cnt))
    {
        return 0.0f;
    }

    if ((vehicle_path_point_read(prev_index_cnt, &prev_point) == FALSE)
        || (vehicle_path_point_read(index_cnt, &current_point) == FALSE)
        || (vehicle_path_point_read(next_index_cnt, &next_point) == FALSE))
    {
        return 0.0f;
    }

    a_len = sqrtf(((current_point.x_mm - prev_point.x_mm) * (current_point.x_mm - prev_point.x_mm))
                + ((current_point.y_mm - prev_point.y_mm) * (current_point.y_mm - prev_point.y_mm)));
    b_len = sqrtf(((next_point.x_mm - current_point.x_mm) * (next_point.x_mm - current_point.x_mm))
                + ((next_point.y_mm - current_point.y_mm) * (next_point.y_mm - current_point.y_mm)));
    c_len = sqrtf(((next_point.x_mm - prev_point.x_mm) * (next_point.x_mm - prev_point.x_mm))
                + ((next_point.y_mm - prev_point.y_mm) * (next_point.y_mm - prev_point.y_mm)));

    if (chord_length_mm != NULL_PTR)
    {
        *chord_length_mm = c_len;
    }

    if ((a_len < 0.001f) || (b_len < 0.001f) || (c_len < 0.001f))
    {
        return 0.0f;
    }

    cross = ((current_point.x_mm - prev_point.x_mm) * (next_point.y_mm - prev_point.y_mm))
          - ((current_point.y_mm - prev_point.y_mm) * (next_point.x_mm - prev_point.x_mm));
    return (2.0f * cross) / (a_len * b_len * c_len);
}

static float32 vehicle_path_plan_effective_curvature_get(uint32 index_cnt)
{
    float32 chord_length_mm;

    return vehicle_path_plan_curvature_get(index_cnt, &chord_length_mm);
}

#if VEHICLE_PATH_ONBOARD_GEOMETRY_ENABLE
static boolean vehicle_path_plan_geometry_index_locked(uint32 index_cnt, uint32 point_cnt)
{
    uint32 lock_cnt = vehicle_path_cfg_get()->plan_smooth_window_cnt;

    if (point_cnt == 0u)
    {
        return TRUE;
    }

    if (lock_cnt == 0u)
    {
        lock_cnt = 1u;
    }

    if (index_cnt < lock_cnt)
    {
        return TRUE;
    }

    return ((index_cnt + lock_cnt) >= point_cnt) ? TRUE : FALSE;
}

static void vehicle_path_plan_smooth_points(uint32 point_cnt)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();
    vehicle_path_point_t* temp_points = &vehicle_path_runtime.points[point_cnt];
    uint32 index_cnt;
    uint32 window_cnt = cfg->plan_smooth_window_cnt;

    if ((point_cnt < 3u) || (window_cnt == 0u) || ((point_cnt * 2u) > vehicle_path_max_point_cnt_get()))
    {
        return;
    }

    for (index_cnt = 0u; index_cnt < point_cnt; index_cnt++)
    {
        uint32 start_cnt = 0u;
        uint32 end_cnt = index_cnt + window_cnt;
        uint32 sample_cnt;
        uint32 count_cnt = 0u;
        float32 sum_x_mm = 0.0f;
        float32 sum_y_mm = 0.0f;

        temp_points[index_cnt] = vehicle_path_runtime.points[index_cnt];
        if (vehicle_path_plan_geometry_index_locked(index_cnt, point_cnt) != FALSE)
        {
            continue;
        }

        start_cnt = index_cnt - window_cnt;
        for (sample_cnt = start_cnt; sample_cnt <= end_cnt; sample_cnt++)
        {
            sum_x_mm += vehicle_path_runtime.points[sample_cnt].x_mm;
            sum_y_mm += vehicle_path_runtime.points[sample_cnt].y_mm;
            count_cnt++;
        }

        if (count_cnt > 0u)
        {
            temp_points[index_cnt].x_mm = sum_x_mm / (float32)count_cnt;
            temp_points[index_cnt].y_mm = sum_y_mm / (float32)count_cnt;
        }
    }

    for (index_cnt = 0u; index_cnt < point_cnt; index_cnt++)
    {
        vehicle_path_runtime.points[index_cnt].x_mm = temp_points[index_cnt].x_mm;
        vehicle_path_runtime.points[index_cnt].y_mm = temp_points[index_cnt].y_mm;
    }
}

static void vehicle_path_plan_bspline_apply(uint32 point_cnt)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();
    vehicle_path_point_t* temp_points = &vehicle_path_runtime.points[point_cnt];
    uint32 index_cnt;

    if ((point_cnt < 4u) || ((point_cnt * 2u) > vehicle_path_max_point_cnt_get()))
    {
        return;
    }

    temp_points[0u] = vehicle_path_runtime.points[0u];
    temp_points[point_cnt - 1u] = vehicle_path_runtime.points[point_cnt - 1u];

    for (index_cnt = 1u; index_cnt < (point_cnt - 1u); index_cnt++)
    {
        uint32 p0 = (index_cnt > 1u) ? (index_cnt - 1u) : 0u;
        uint32 p1 = index_cnt;
        uint32 p2 = index_cnt + 1u;
        float32 smooth_x_mm;
        float32 smooth_y_mm;
        float32 dx;
        float32 dy;
        float32 offset_mm;
        float32 limit_mm = cfg->plan_bspline_max_offset_mm;

        temp_points[index_cnt] = vehicle_path_runtime.points[index_cnt];
        if (vehicle_path_plan_geometry_index_locked(index_cnt, point_cnt) != FALSE)
        {
            continue;
        }

        smooth_x_mm = (vehicle_path_runtime.points[p0].x_mm
                     + (4.0f * vehicle_path_runtime.points[p1].x_mm)
                     + vehicle_path_runtime.points[p2].x_mm) / 6.0f;
        smooth_y_mm = (vehicle_path_runtime.points[p0].y_mm
                     + (4.0f * vehicle_path_runtime.points[p1].y_mm)
                     + vehicle_path_runtime.points[p2].y_mm) / 6.0f;
        dx = smooth_x_mm - vehicle_path_runtime.points[index_cnt].x_mm;
        dy = smooth_y_mm - vehicle_path_runtime.points[index_cnt].y_mm;
        offset_mm = sqrtf((dx * dx) + (dy * dy));

        if ((limit_mm > 0.0f) && (offset_mm > limit_mm))
        {
            dx *= limit_mm / offset_mm;
            dy *= limit_mm / offset_mm;
        }

        temp_points[index_cnt].x_mm = vehicle_path_runtime.points[index_cnt].x_mm + dx;
        temp_points[index_cnt].y_mm = vehicle_path_runtime.points[index_cnt].y_mm + dy;
    }

    for (index_cnt = 0u; index_cnt < point_cnt; index_cnt++)
    {
        vehicle_path_runtime.points[index_cnt].x_mm = temp_points[index_cnt].x_mm;
        vehicle_path_runtime.points[index_cnt].y_mm = temp_points[index_cnt].y_mm;
    }
}

static void vehicle_path_plan_clothoid_apply(uint32 point_cnt)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();
    uint32 blend_cnt = cfg->plan_clothoid_blend_cnt;
    uint32 index_cnt;

    if ((point_cnt < 5u) || (blend_cnt == 0u))
    {
        return;
    }

    for (index_cnt = 1u; index_cnt < (point_cnt - 1u); index_cnt++)
    {
        float32 chord_length_mm;
        float32 curvature = vehicle_path_plan_curvature_get(index_cnt, &chord_length_mm);
        float32 abs_curvature = (curvature < 0.0f) ? -curvature : curvature;
        float32 turn_angle_rad = abs_curvature * chord_length_mm;

        if ((abs_curvature > cfg->plan_curvature_epsilon) && (turn_angle_rad > cfg->plan_straight_angle_rad))
        {
            uint32 local_cnt;
            uint32 max_cnt = blend_cnt;
            if (index_cnt < max_cnt)
            {
                max_cnt = index_cnt;
            }
            if ((point_cnt - 1u - index_cnt) < max_cnt)
            {
                max_cnt = point_cnt - 1u - index_cnt;
            }

            for (local_cnt = 1u; local_cnt <= max_cnt; local_cnt++)
            {
                float32 ratio = (float32)local_cnt / (float32)(max_cnt + 1u);
                float32 gain = ratio * ratio * (3.0f - (2.0f * ratio));
                uint32 before_cnt = index_cnt - local_cnt;
                uint32 after_cnt = index_cnt + local_cnt;
                float32 mid_x_mm = (vehicle_path_runtime.points[before_cnt].x_mm
                                  + vehicle_path_runtime.points[after_cnt].x_mm) * 0.5f;
                float32 mid_y_mm = (vehicle_path_runtime.points[before_cnt].y_mm
                                  + vehicle_path_runtime.points[after_cnt].y_mm) * 0.5f;

                if (vehicle_path_plan_geometry_index_locked(before_cnt, point_cnt) == FALSE)
                {
                    vehicle_path_runtime.points[before_cnt].x_mm =
                        vehicle_path_runtime.points[before_cnt].x_mm
                        + ((mid_x_mm - vehicle_path_runtime.points[before_cnt].x_mm) * gain * 0.25f);
                    vehicle_path_runtime.points[before_cnt].y_mm =
                        vehicle_path_runtime.points[before_cnt].y_mm
                        + ((mid_y_mm - vehicle_path_runtime.points[before_cnt].y_mm) * gain * 0.25f);
                }
                if (vehicle_path_plan_geometry_index_locked(after_cnt, point_cnt) == FALSE)
                {
                    vehicle_path_runtime.points[after_cnt].x_mm =
                        vehicle_path_runtime.points[after_cnt].x_mm
                        + ((mid_x_mm - vehicle_path_runtime.points[after_cnt].x_mm) * gain * 0.25f);
                    vehicle_path_runtime.points[after_cnt].y_mm =
                        vehicle_path_runtime.points[after_cnt].y_mm
                        + ((mid_y_mm - vehicle_path_runtime.points[after_cnt].y_mm) * gain * 0.25f);
                }
            }
        }
    }
}

static void vehicle_path_plan_curvature_smooth_apply(uint32 point_cnt)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();
    vehicle_path_point_t* temp_points = &vehicle_path_runtime.points[point_cnt];
    uint32 window_cnt = cfg->plan_curvature_smooth_window_cnt;
    uint32 index_cnt;

    if ((point_cnt < 3u) || (window_cnt == 0u) || ((point_cnt * 2u) > vehicle_path_max_point_cnt_get()))
    {
        return;
    }

    for (index_cnt = 0u; index_cnt < point_cnt; index_cnt++)
    {
        temp_points[index_cnt] = vehicle_path_runtime.points[index_cnt];
    }

    for (index_cnt = window_cnt; index_cnt < (point_cnt - window_cnt); index_cnt++)
    {
        float32 prev_chord_mm;
        float32 next_chord_mm;
        float32 prev_curvature = vehicle_path_plan_curvature_get(index_cnt - window_cnt, &prev_chord_mm);
        float32 next_curvature = vehicle_path_plan_curvature_get(index_cnt + window_cnt, &next_chord_mm);
        float32 current_chord_mm;
        float32 current_curvature = vehicle_path_plan_curvature_get(index_cnt, &current_chord_mm);
        float32 avg_curvature = (prev_curvature + current_curvature + next_curvature) / 3.0f;
        float32 delta = current_curvature - avg_curvature;
        float32 abs_delta = (delta < 0.0f) ? -delta : delta;

        if (vehicle_path_plan_geometry_index_locked(index_cnt, point_cnt) != FALSE)
        {
            continue;
        }

        if (abs_delta > cfg->plan_curvature_epsilon)
        {
            uint32 prev_index_cnt = index_cnt - 1u;
            uint32 next_index_cnt = index_cnt + 1u;
            float32 tx = vehicle_path_runtime.points[next_index_cnt].x_mm - vehicle_path_runtime.points[prev_index_cnt].x_mm;
            float32 ty = vehicle_path_runtime.points[next_index_cnt].y_mm - vehicle_path_runtime.points[prev_index_cnt].y_mm;
            float32 len = sqrtf((tx * tx) + (ty * ty));
            float32 gain = 0.15f;

            if (len > 0.001f)
            {
                tx /= len;
                ty /= len;
                temp_points[index_cnt].x_mm += ty * delta * current_chord_mm * current_chord_mm * gain;
                temp_points[index_cnt].y_mm -= tx * delta * current_chord_mm * current_chord_mm * gain;
            }
        }
    }

    for (index_cnt = 0u; index_cnt < point_cnt; index_cnt++)
    {
        vehicle_path_runtime.points[index_cnt].x_mm = temp_points[index_cnt].x_mm;
        vehicle_path_runtime.points[index_cnt].y_mm = temp_points[index_cnt].y_mm;
    }
}

static void vehicle_path_plan_radius_lock_apply(uint32 point_cnt)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();
    uint32 index_cnt = 0u;
    float32 target_radius_mm = cfg->plan_target_radius_mm;
    float32 target_curvature;

    if ((point_cnt < 3u) || (target_radius_mm <= 1.0f))
    {
        return;
    }

    target_curvature = 1.0f / target_radius_mm;

    while (index_cnt < point_cnt)
    {
        float32 chord_length_mm;
        float32 curvature;
        float32 abs_curvature;
        float32 turn_angle_rad;
        uint32 segment_start_cnt;
        uint32 segment_end_cnt;
        uint32 segment_cnt;
        uint32 segment_index_cnt;

        curvature = vehicle_path_plan_curvature_get(index_cnt, &chord_length_mm);
        abs_curvature = (curvature < 0.0f) ? -curvature : curvature;
        turn_angle_rad = abs_curvature * chord_length_mm;

        if ((abs_curvature <= cfg->plan_curvature_epsilon)
            || (turn_angle_rad <= cfg->plan_straight_angle_rad))
        {
            index_cnt++;
            continue;
        }

        segment_start_cnt = index_cnt;
        segment_end_cnt = index_cnt;
        index_cnt++;

        while (index_cnt < point_cnt)
        {
            float32 next_chord_length_mm;
            float32 next_curvature = vehicle_path_plan_curvature_get(index_cnt, &next_chord_length_mm);
            float32 next_abs_curvature = (next_curvature < 0.0f) ? -next_curvature : next_curvature;
            float32 next_turn_angle_rad = next_abs_curvature * next_chord_length_mm;

            if ((next_abs_curvature <= cfg->plan_curvature_epsilon)
                || (next_turn_angle_rad <= cfg->plan_straight_angle_rad))
            {
                break;
            }

            segment_end_cnt = index_cnt;
            index_cnt++;
        }

        segment_cnt = segment_end_cnt - segment_start_cnt + 1u;
        if (segment_cnt < cfg->plan_curve_min_span_cnt)
        {
            continue;
        }

        for (segment_index_cnt = segment_start_cnt; segment_index_cnt <= segment_end_cnt; segment_index_cnt++)
        {
            float32 local_chord_length_mm;
            float32 local_curvature = vehicle_path_plan_curvature_get(segment_index_cnt, &local_chord_length_mm);
            float32 local_abs_curvature = (local_curvature < 0.0f) ? -local_curvature : local_curvature;
            float32 radius_mm;
            uint32 prev_index_cnt;
            uint32 next_index_cnt;
            float32 tx;
            float32 ty;
            float32 tangent_len;
            float32 nx;
            float32 ny;
            float32 push_mm;
            float32 side_sign;

            if (vehicle_path_plan_geometry_index_locked(segment_index_cnt, point_cnt) != FALSE)
            {
                continue;
            }

            if (local_abs_curvature <= target_curvature)
            {
                continue;
            }

            radius_mm = 1.0f / local_abs_curvature;
            if (radius_mm >= (target_radius_mm * cfg->plan_radius_lock_min_ratio))
            {
                continue;
            }

            prev_index_cnt = (segment_index_cnt > 0u) ? (segment_index_cnt - 1u) : segment_index_cnt;
            next_index_cnt = segment_index_cnt + 1u;
            if (next_index_cnt >= point_cnt)
            {
                next_index_cnt = segment_index_cnt;
            }

            tx = vehicle_path_runtime.points[next_index_cnt].x_mm - vehicle_path_runtime.points[prev_index_cnt].x_mm;
            ty = vehicle_path_runtime.points[next_index_cnt].y_mm - vehicle_path_runtime.points[prev_index_cnt].y_mm;
            tangent_len = sqrtf((tx * tx) + (ty * ty));
            if (tangent_len < 0.001f)
            {
                continue;
            }

            tx /= tangent_len;
            ty /= tangent_len;
            nx = -ty;
            ny = tx;
            side_sign = (local_curvature >= 0.0f) ? -1.0f : 1.0f;
            push_mm = (target_radius_mm - radius_mm) * cfg->plan_radius_lock_gain;

            vehicle_path_runtime.points[segment_index_cnt].x_mm += nx * side_sign * push_mm;
            vehicle_path_runtime.points[segment_index_cnt].y_mm += ny * side_sign * push_mm;
        }
    }
}
#endif

static void vehicle_path_plan_curve_speed_apply(uint32 point_cnt)
{
    uint32 index_cnt;

    if (point_cnt < 3u)
    {
        return;
    }

    for (index_cnt = 1u; index_cnt < (point_cnt - 1u); index_cnt++)
    {
        float32 curvature = vehicle_path_plan_effective_curvature_get(index_cnt);
        float32 speed_mm_s = vehicle_path_plan_base_speed_get(curvature, 0.0f);
        float32 wheel_speed_cap_mm_s = vehicle_path_plan_wheel_static_speed_cap_get(curvature);

        if (speed_mm_s > wheel_speed_cap_mm_s)
        {
            speed_mm_s = wheel_speed_cap_mm_s;
        }

        if (vehicle_path_runtime.points[index_cnt].speed_mm_s > speed_mm_s)
        {
            vehicle_path_runtime.points[index_cnt].speed_mm_s = speed_mm_s;
        }
    }
}

static void vehicle_path_plan_start_speed_apply(uint32 point_cnt)
{
    float32 hold_speed_mm_s =
        vehicle_path_plan_speed_clamp(VEHICLE_PATH_DEFAULT_PLAN_START_HOLD_SPEED_MM_S);
    float32 progress_mm = 0.0f;
    uint32 index_cnt;

    if (point_cnt == 0u)
    {
        return;
    }

    if (vehicle_path_runtime.points[0u].speed_mm_s > hold_speed_mm_s)
    {
        vehicle_path_runtime.points[0u].speed_mm_s = hold_speed_mm_s;
    }

    for (index_cnt = 1u; index_cnt < point_cnt; index_cnt++)
    {
        float32 dx = vehicle_path_runtime.points[index_cnt].x_mm
                   - vehicle_path_runtime.points[index_cnt - 1u].x_mm;
        float32 dy = vehicle_path_runtime.points[index_cnt].y_mm
                   - vehicle_path_runtime.points[index_cnt - 1u].y_mm;

        progress_mm += sqrtf((dx * dx) + (dy * dy));
        if (progress_mm > VEHICLE_PATH_DEFAULT_PLAN_START_HOLD_DISTANCE_MM)
        {
            break;
        }

        if (vehicle_path_runtime.points[index_cnt].speed_mm_s > hold_speed_mm_s)
        {
            vehicle_path_runtime.points[index_cnt].speed_mm_s = hold_speed_mm_s;
        }
    }
}

static void vehicle_path_plan_end_speed_apply(uint32 point_cnt)
{
    float32 remaining_mm = 0.0f;
    uint32 index_cnt;

    if (point_cnt == 0u)
    {
        return;
    }

    vehicle_path_runtime.points[point_cnt - 1u].speed_mm_s = 0.0f;
    index_cnt = point_cnt - 1u;
    while (index_cnt > 0u)
    {
        float32 dx = vehicle_path_runtime.points[index_cnt].x_mm
                   - vehicle_path_runtime.points[index_cnt - 1u].x_mm;
        float32 dy = vehicle_path_runtime.points[index_cnt].y_mm
                   - vehicle_path_runtime.points[index_cnt - 1u].y_mm;
        float32 speed_scale;

        remaining_mm += sqrtf((dx * dx) + (dy * dy));
        if (remaining_mm >= VEHICLE_PATH_DEFAULT_END_DECEL_DISTANCE_MM)
        {
            break;
        }

        speed_scale = vehicle_path_end_speed_scale_get(remaining_mm);
        vehicle_path_runtime.points[index_cnt - 1u].speed_mm_s *= speed_scale;
        index_cnt--;
    }
}

static void vehicle_path_plan_chunk_end_speed_apply(void)
{
    uint32 local_cnt;

    for (local_cnt = 0u; local_cnt < vehicle_path_runtime.job_chunk_input_count; local_cnt++)
    {
        uint32 global_index_cnt = vehicle_path_runtime.job_chunk_input_start_cnt + local_cnt;
        float32 progress_mm = (float32)global_index_cnt
                            * VEHICLE_PATH_DEFAULT_PLAN_SAMPLE_STEP_MM;
        float32 remaining_mm;

        if ((global_index_cnt + 1u) >= vehicle_path_runtime.job_point_cnt)
        {
            remaining_mm = 0.0f;
        }
        else if (progress_mm >= vehicle_path_runtime.job_path_length_mm)
        {
            remaining_mm = 0.0f;
        }
        else
        {
            remaining_mm = vehicle_path_runtime.job_path_length_mm - progress_mm;
        }

        if (remaining_mm < VEHICLE_PATH_DEFAULT_END_DECEL_DISTANCE_MM)
        {
            vehicle_path_runtime.points[local_cnt].speed_mm_s *=
                vehicle_path_end_speed_scale_get(remaining_mm);
        }
    }
}

static float32 vehicle_path_plan_base_speed_get(float32 curvature, float32 chord_length_mm)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();
    float32 abs_curvature = (curvature < 0.0f) ? -curvature : curvature;
    float32 curve_lateral_accel_mm_s2;
    float32 speed_mm_s;

    (void)chord_length_mm;

    if (abs_curvature <= cfg->plan_curvature_epsilon)
    {
        return vehicle_path_plan_speed_clamp(cfg->plan_max_speed_mm_s);
    }

    curve_lateral_accel_mm_s2 = cfg->plan_curve_lateral_accel_mm_s2;
    if (curve_lateral_accel_mm_s2 <= 0.0f)
    {
        curve_lateral_accel_mm_s2 = cfg->plan_lateral_accel_mm_s2;
    }
    if ((cfg->plan_lateral_accel_mm_s2 > 0.0f)
        && (curve_lateral_accel_mm_s2 > cfg->plan_lateral_accel_mm_s2))
    {
        curve_lateral_accel_mm_s2 = cfg->plan_lateral_accel_mm_s2;
    }
    if (curve_lateral_accel_mm_s2 <= 0.0f)
    {
        return vehicle_path_plan_speed_clamp(cfg->plan_max_speed_mm_s);
    }

    speed_mm_s = sqrtf(curve_lateral_accel_mm_s2 / abs_curvature);
    return vehicle_path_plan_speed_clamp(speed_mm_s);
}

static float32 vehicle_path_plan_available_long_accel_get(float32 speed_mm_s, float32 curvature, boolean decel)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();
    float32 base_accel_mm_s2 = (decel != FALSE) ? cfg->plan_long_decel_mm_s2 : cfg->plan_long_accel_mm_s2;
    float32 abs_curvature = (curvature < 0.0f) ? -curvature : curvature;
    float32 lateral_accel_mm_s2;
    float32 ratio;
    float32 remain;

    if (cfg->plan_friction_circle_enable == FALSE)
    {
        return (decel != FALSE) ? cfg->plan_decel_mm_s2 : cfg->plan_accel_mm_s2;
    }

    lateral_accel_mm_s2 = speed_mm_s * speed_mm_s * abs_curvature;
    if ((cfg->plan_lateral_accel_mm_s2 <= 0.0f) || (lateral_accel_mm_s2 <= 0.0f))
    {
        return base_accel_mm_s2;
    }

    ratio = lateral_accel_mm_s2 / cfg->plan_lateral_accel_mm_s2;
    if (ratio >= 1.0f)
    {
        return 0.0f;
    }

    remain = 1.0f - (ratio * ratio);
    if (remain < 0.0f)
    {
        remain = 0.0f;
    }

    return base_accel_mm_s2 * sqrtf(remain);
}

static float32 vehicle_path_plan_abs_f32(float32 value)
{
    return (value < 0.0f) ? -value : value;
}

static float32 vehicle_path_plan_wheel_coeff_get(float32 curvature, boolean right_wheel)
{
    float32 yaw_coeff = VEHICLE_PATH_ENCODER_HALF_WHEEL_BASE_MM * curvature;

    return (right_wheel != FALSE) ? (1.0f + yaw_coeff) : (1.0f - yaw_coeff);
}

static float32 vehicle_path_plan_wheel_speed_limit_get(void)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();
    float32 speed_limit_mm_s =
        cfg->plan_max_speed_mm_s + VEHICLE_PATH_PLAN_WHEEL_SPEED_EXTRA_MM_S
        - VEHICLE_PATH_PLAN_WHEEL_CORRECTION_RESERVE_MM_S;

    if (speed_limit_mm_s < cfg->plan_min_speed_mm_s)
    {
        speed_limit_mm_s = cfg->plan_min_speed_mm_s;
    }

    return speed_limit_mm_s;
}

static float32 vehicle_path_plan_wheel_static_speed_cap_get(float32 curvature)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();
    float32 outer_coeff =
        1.0f + (VEHICLE_PATH_ENCODER_HALF_WHEEL_BASE_MM * vehicle_path_plan_abs_f32(curvature));
    float32 inner_coeff =
        1.0f - (VEHICLE_PATH_ENCODER_HALF_WHEEL_BASE_MM * vehicle_path_plan_abs_f32(curvature));
    float32 speed_cap_mm_s;

    if (inner_coeff <= VEHICLE_PATH_PLAN_WHEEL_COEFF_MIN)
    {
        return cfg->plan_min_speed_mm_s;
    }

    speed_cap_mm_s = vehicle_path_plan_wheel_speed_limit_get() / outer_coeff;
    return vehicle_path_plan_speed_clamp(speed_cap_mm_s);
}

static float32 vehicle_path_plan_wheel_transition_speed_cap_get(float32 current_cap_mm_s,
                                                               float32 source_speed_mm_s,
                                                               float32 source_curvature,
                                                               float32 target_curvature,
                                                               float32 distance_mm,
                                                               float32 accel_limit_mm_s2)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();
    float32 source_left_coeff;
    float32 source_right_coeff;
    float32 target_left_coeff;
    float32 target_right_coeff;
    float32 source_left_speed_mm_s;
    float32 source_right_speed_mm_s;
    float32 allowed_left_speed_mm_s;
    float32 allowed_right_speed_mm_s;

    if ((distance_mm <= VEHICLE_PATH_PLAN_RESAMPLE_EPSILON_MM)
        || (accel_limit_mm_s2 <= 0.0f))
    {
        return current_cap_mm_s;
    }

    target_left_coeff = vehicle_path_plan_wheel_coeff_get(target_curvature, FALSE);
    target_right_coeff = vehicle_path_plan_wheel_coeff_get(target_curvature, TRUE);
    if ((target_left_coeff <= VEHICLE_PATH_PLAN_WHEEL_COEFF_MIN)
        || (target_right_coeff <= VEHICLE_PATH_PLAN_WHEEL_COEFF_MIN))
    {
        return cfg->plan_min_speed_mm_s;
    }

    source_left_coeff = vehicle_path_plan_wheel_coeff_get(source_curvature, FALSE);
    source_right_coeff = vehicle_path_plan_wheel_coeff_get(source_curvature, TRUE);
    if (source_left_coeff < 0.0f)
    {
        source_left_coeff = 0.0f;
    }
    if (source_right_coeff < 0.0f)
    {
        source_right_coeff = 0.0f;
    }

    accel_limit_mm_s2 *= VEHICLE_PATH_PLAN_WHEEL_ACCEL_RESERVE_RATIO;
    source_left_speed_mm_s = source_speed_mm_s * source_left_coeff;
    source_right_speed_mm_s = source_speed_mm_s * source_right_coeff;
    allowed_left_speed_mm_s =
        sqrtf((source_left_speed_mm_s * source_left_speed_mm_s)
              + (2.0f * accel_limit_mm_s2 * distance_mm));
    allowed_right_speed_mm_s =
        sqrtf((source_right_speed_mm_s * source_right_speed_mm_s)
              + (2.0f * accel_limit_mm_s2 * distance_mm));

    allowed_left_speed_mm_s /= target_left_coeff;
    allowed_right_speed_mm_s /= target_right_coeff;
    if (current_cap_mm_s > allowed_left_speed_mm_s)
    {
        current_cap_mm_s = allowed_left_speed_mm_s;
    }
    if (current_cap_mm_s > allowed_right_speed_mm_s)
    {
        current_cap_mm_s = allowed_right_speed_mm_s;
    }

    return vehicle_path_plan_speed_clamp(current_cap_mm_s);
}

static boolean vehicle_path_plan_forward_step(uint32 max_point_count)
{
    uint32 point_cnt = vehicle_path_runtime.job_point_cnt;
    uint32 processed_count = 0u;

    if (point_cnt < 2u)
    {
        return TRUE;
    }

    while ((processed_count < max_point_count) && (vehicle_path_runtime.job_index_cnt < point_cnt))
    {
        uint32 index_cnt = vehicle_path_runtime.job_index_cnt;
        vehicle_path_point_t* prev_point = &vehicle_path_runtime.points[index_cnt - 1u];
        vehicle_path_point_t* point = &vehicle_path_runtime.points[index_cnt];
        float32 dx = point->x_mm - prev_point->x_mm;
        float32 dy = point->y_mm - prev_point->y_mm;
        float32 distance_mm = sqrtf((dx * dx) + (dy * dy));
        float32 curvature = vehicle_path_plan_effective_curvature_get(index_cnt - 1u);
        float32 point_curvature = vehicle_path_plan_effective_curvature_get(index_cnt);
        float32 accel_mm_s2 = vehicle_path_plan_available_long_accel_get(prev_point->speed_mm_s, curvature, FALSE);
        float32 allowed_speed_mm_s = sqrtf((prev_point->speed_mm_s * prev_point->speed_mm_s)
                                         + (2.0f * accel_mm_s2 * distance_mm));

        allowed_speed_mm_s =
            vehicle_path_plan_wheel_transition_speed_cap_get(allowed_speed_mm_s,
                                                             prev_point->speed_mm_s,
                                                             curvature,
                                                             point_curvature,
                                                             distance_mm,
                                                             accel_mm_s2);

        if (point->speed_mm_s > allowed_speed_mm_s)
        {
            point->speed_mm_s = allowed_speed_mm_s;
        }

        vehicle_path_runtime.job_index_cnt++;
        processed_count++;
    }

    return (vehicle_path_runtime.job_index_cnt >= point_cnt) ? TRUE : FALSE;
}

static boolean vehicle_path_plan_backward_step(uint32 max_point_count)
{
    uint32 point_cnt = vehicle_path_runtime.job_point_cnt;
    uint32 processed_count = 0u;

    if (point_cnt < 2u)
    {
        return TRUE;
    }

    while ((processed_count < max_point_count) && (vehicle_path_runtime.job_index_cnt > 0u))
    {
        uint32 reverse_index_cnt = vehicle_path_runtime.job_index_cnt;
        vehicle_path_point_t* next_point = &vehicle_path_runtime.points[reverse_index_cnt];
        vehicle_path_point_t* point = &vehicle_path_runtime.points[reverse_index_cnt - 1u];
        float32 dx = next_point->x_mm - point->x_mm;
        float32 dy = next_point->y_mm - point->y_mm;
        float32 distance_mm = sqrtf((dx * dx) + (dy * dy));
        float32 curvature = vehicle_path_plan_effective_curvature_get(reverse_index_cnt);
        float32 point_curvature = vehicle_path_plan_effective_curvature_get(reverse_index_cnt - 1u);
        float32 decel_mm_s2 = vehicle_path_plan_available_long_accel_get(next_point->speed_mm_s, curvature, TRUE);
        float32 allowed_speed_mm_s = sqrtf((next_point->speed_mm_s * next_point->speed_mm_s)
                                         + (2.0f * decel_mm_s2 * distance_mm));

        allowed_speed_mm_s =
            vehicle_path_plan_wheel_transition_speed_cap_get(allowed_speed_mm_s,
                                                             next_point->speed_mm_s,
                                                             curvature,
                                                             point_curvature,
                                                             distance_mm,
                                                             decel_mm_s2);

        if (point->speed_mm_s > allowed_speed_mm_s)
        {
            point->speed_mm_s = allowed_speed_mm_s;
        }

        vehicle_path_runtime.job_index_cnt--;
        processed_count++;
    }

    return (vehicle_path_runtime.job_index_cnt == 0u) ? TRUE : FALSE;
}

static boolean vehicle_path_plan_summary_step(uint32 max_point_count)
{
    uint32 processed_count = 0u;

    while ((processed_count < max_point_count)
           && (vehicle_path_runtime.job_index_cnt < vehicle_path_runtime.job_point_cnt))
    {
        vehicle_path_point_t point = vehicle_path_runtime.points[vehicle_path_runtime.job_index_cnt];

        if ((vehicle_path_runtime.job_planned_cnt == 0u)
            || (point.speed_mm_s < vehicle_path_runtime.job_min_speed_mm_s))
        {
            vehicle_path_runtime.job_min_speed_mm_s = point.speed_mm_s;
        }

        if ((vehicle_path_runtime.job_planned_cnt == 0u)
            || (point.speed_mm_s > vehicle_path_runtime.job_max_speed_mm_s))
        {
            vehicle_path_runtime.job_max_speed_mm_s = point.speed_mm_s;
        }

        vehicle_path_runtime.job_planned_cnt++;
        vehicle_path_runtime.job_index_cnt++;
        processed_count++;
    }

    return (vehicle_path_runtime.job_index_cnt >= vehicle_path_runtime.job_point_cnt) ? TRUE : FALSE;
}

static float32 vehicle_path_plan_speed_clamp(float32 speed_mm_s)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();

    if (speed_mm_s < cfg->plan_min_speed_mm_s)
    {
        speed_mm_s = cfg->plan_min_speed_mm_s;
    }

    if (speed_mm_s > cfg->plan_max_speed_mm_s)
    {
        speed_mm_s = cfg->plan_max_speed_mm_s;
    }

    return speed_mm_s;
}

static float32 vehicle_path_plan_stored_speed_clamp(float32 speed_mm_s)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();

    if (speed_mm_s < 0.0f)
    {
        speed_mm_s = 0.0f;
    }
    if (speed_mm_s > cfg->plan_max_speed_mm_s)
    {
        speed_mm_s = cfg->plan_max_speed_mm_s;
    }

    return speed_mm_s;
}

static float32 vehicle_path_wrap_pi(float32 angle_rad);

/**
 * @brief 初始化车体路径记录与复现状态。
 * @param[in] void 无参数。
 * @return void
 */
void module_vehicle_path_init(void)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();

    vehicle_path_state_reset(VEHICLE_PATH_STATUS_IDLE);
    vehicle_path_replay_speed_mm_s = VEHICLE_PATH_DEFAULT_REPLAY_SPEED_MM_S;
    vehicle_path_replay_lookahead_speed_cap_mm_s = 0.0f;
    vehicle_path_replay_lookahead_cnt_runtime = cfg->replay_lookahead_cnt;
    vehicle_path_replay_tangent_gap_cnt_runtime = cfg->replay_tangent_gap_cnt;
    vehicle_path_runtime.save_busy = FALSE;
    vehicle_path_runtime.load_state = VEHICLE_PATH_LOAD_STATE_IDLE;
    vehicle_path_runtime.load_point_length = 0u;
    vehicle_path_runtime.load_point_cnt = 0u;
    vehicle_path_runtime.load_checksum = 0u;
    vehicle_path_runtime.window_start_index_cnt = 0u;
    vehicle_path_runtime.window_valid_cnt = 0u;
    vehicle_path_runtime.window_active_buffer_cnt = 0u;
    vehicle_path_runtime.window_from_flash = FALSE;
    vehicle_path_window_buffers_reset();
    vehicle_path_window_prefetch_reset();
    vehicle_path_runtime.active_slot_offset = VEHICLE_PATH_FLASH_RAW_OFFSET;
    vehicle_path_runtime.marker_tracking_mode = VEHICLE_PATH_MARKER_TRACKING_ENABLED;
    vehicle_path_runtime.planned_speed_enabled = FALSE;
    vehicle_path_planned_slot_erase_reset();
    vehicle_path_marker_reset();
    vehicle_path_runtime.replay_edge_points_valid = FALSE;
    vehicle_path_job_reset();
    vehicle_path_record_stream_reset();
}

/**
 * @brief 推进路径后台保存、实时打印、导出和规划任务。
 * @param[in] void 无参数。
 * @return void
 */
void module_vehicle_path_run(void)
{
    vehicle_path_planned_slot_erase_run();
    vehicle_path_record_queue_drain(VEHICLE_PATH_RECORD_QUEUE_DRAIN_PER_RUN);
    vehicle_path_record_save_finish_try();
    vehicle_path_record_print_queue_drain(VEHICLE_PATH_RECORD_PRINT_QUEUE_DRAIN_PER_RUN);
    vehicle_path_window_prefetch_run();
    vehicle_path_background_job_run();
}

/**
 * @brief 开始路径记录并记录当前位姿为首个路径点。
 * @param[in] void 无参数。
 * @return 成功开始记录返回 TRUE，否则返回 FALSE。
 */
boolean module_vehicle_path_start(void)
{
    if (vehicle_path_operation_busy() != FALSE)
    {
        tools_printf("{pathinfo}path_job_busy\r\n");
        return FALSE;
    }

    vehicle_path_runtime_origin_reset(0.0f, 0.0f, 0.0f);
    module_vehicle_path_clear();
    vehicle_path_runtime.active_slot_offset = VEHICLE_PATH_FLASH_RAW_OFFSET;
    vehicle_path_record_stream_start(VEHICLE_PATH_FLASH_RAW_OFFSET);
    vehicle_path_runtime.state.recording = TRUE;
    vehicle_path_runtime.state.status = VEHICLE_PATH_STATUS_RECORDING;
    tools_printf("{pathfmt}x,y,theta_deg,index,dist,left_dist,right_dist,left_count,right_count,lr_diff,enc_theta_deg,left_sample,right_sample,pose_up,enc_up\r\n");

    return vehicle_path_record_current_pose();
}

/**
 * @brief 停止路径记录。
 * @param[in] void 无参数。
 * @return void
 */
void module_vehicle_path_stop(void)
{
    vehicle_path_runtime.state.recording = FALSE;
    vehicle_path_runtime.state.last_distance_mm = vehicle_path_runtime.record_path_length_mm;

    if (vehicle_path_runtime.state.status == VEHICLE_PATH_STATUS_RECORDING)
    {
        vehicle_path_runtime.state.status = VEHICLE_PATH_STATUS_IDLE;
    }
}

/**
 * @brief 清空路径点与运行计数。
 * @param[in] void 无参数。
 * @return void
 */
void module_vehicle_path_clear(void)
{
    vehicle_path_planned_slot_erase_reset();
    vehicle_path_state_reset(VEHICLE_PATH_STATUS_IDLE);
    vehicle_path_marker_reset();
    vehicle_path_phototube_zone_reset();
    vehicle_path_runtime.replay_edge_points_valid = FALSE;
    vehicle_path_job_reset();
}

/**
 * @brief 根据编码器位姿执行一次路径记录更新。
 * @param[in] void 无参数。
 * @return 成功记录新路径点返回 TRUE，否则返回 FALSE。
 */
boolean module_vehicle_path_update(void)
{
    const module_vehicle_encoder_observation_t* encoder_observation;
    const module_vehicle_pose_fusion_observation_t* pose_observation;
    vehicle_path_point_t current_point;
    float32 current_distance_mm;
    float32 delta_distance_mm;
    float32 distance_step_mm;
    boolean recorded = FALSE;

    if (vehicle_path_runtime.state.recording == FALSE)
    {
        return FALSE;
    }

    if (vehicle_path_runtime.state.status != VEHICLE_PATH_STATUS_RECORDING)
    {
        return FALSE;
    }

    encoder_observation = module_vehicle_encoder_observation_get();
    pose_observation = module_vehicle_pose_fusion_observation_get();

    if ((vehicle_path_runtime.state.point_cnt > 0u)
        && (pose_observation->update_count == vehicle_path_runtime.record_last_pose_update_count))
    {
        return FALSE;
    }

    current_distance_mm = encoder_observation->distance_mm;
    vehicle_path_record_pose_point_make(pose_observation, &current_point);
    distance_step_mm = vehicle_path_min_distance_step_get();
    if (distance_step_mm < 0.001f)
    {
        distance_step_mm = 0.001f;
    }

    delta_distance_mm = current_distance_mm - vehicle_path_runtime.state.last_distance_mm;

    if (delta_distance_mm < 0.0f)
    {
        delta_distance_mm = -delta_distance_mm;
    }

    while (delta_distance_mm >= distance_step_mm)
    {
        float32 signed_delta_distance_mm =
            current_distance_mm - vehicle_path_runtime.state.last_distance_mm;
        float32 signed_step_mm = (signed_delta_distance_mm >= 0.0f) ? distance_step_mm : -distance_step_mm;
        float32 target_distance_mm = vehicle_path_runtime.state.last_distance_mm + signed_step_mm;
        float32 ratio;
        vehicle_path_point_t point;

        if ((signed_delta_distance_mm < 0.001f) && (signed_delta_distance_mm > -0.001f))
        {
            break;
        }

        ratio = (target_distance_mm - vehicle_path_runtime.state.last_distance_mm)
              / signed_delta_distance_mm;

        if (ratio < 0.0f)
        {
            ratio = 0.0f;
        }
        if (ratio > 1.0f)
        {
            ratio = 1.0f;
        }

        point.x_mm = vehicle_path_runtime.record_last_point.x_mm
                   + ((current_point.x_mm - vehicle_path_runtime.record_last_point.x_mm) * ratio);
        point.y_mm = vehicle_path_runtime.record_last_point.y_mm
                   + ((current_point.y_mm - vehicle_path_runtime.record_last_point.y_mm) * ratio);
        point.theta_rad = vehicle_path_runtime.record_last_point.theta_rad
                        + (vehicle_path_wrap_pi(current_point.theta_rad
                                              - vehicle_path_runtime.record_last_point.theta_rad) * ratio);
        point.speed_mm_s = current_point.speed_mm_s;

        if (vehicle_path_record_point_append(&point,
                                             target_distance_mm,
                                             pose_observation->update_count) == FALSE)
        {
            return recorded;
        }

        recorded = TRUE;
        delta_distance_mm = current_distance_mm - vehicle_path_runtime.state.last_distance_mm;
        if (delta_distance_mm < 0.0f)
        {
            delta_distance_mm = -delta_distance_mm;
        }
    }

    return recorded;
}

/**
 * @brief 通过存储服务保存当前路径点到内部 Flash。
 * @param[in] void 无参数。
 * @return 保存成功返回 TRUE，否则返回 FALSE。
 */
boolean module_vehicle_path_save(void)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();
    vehicle_path_flash_header_t header;
    uint8 header_page[VEHICLE_PATH_FLASH_HEADER_LENGTH];
    uint8 point_page[IFXFLASH_PFLASH_PAGE_LENGTH];
    uint32 point_length;
    uint32 aligned_point_length;
    uint32 point_offset;
    uint32 page_offset;
    uint32 page_valid_length;
    uint32 write_length;
    uint32 i;

    if (vehicle_path_runtime.save_busy != FALSE)
    {
        return FALSE;
    }

    if (vehicle_path_job_busy() != FALSE)
    {
        return FALSE;
    }

    if (vehicle_path_runtime.state.point_cnt > vehicle_path_effective_flash_point_cnt_max_get())
    {
        return FALSE;
    }

    if (vehicle_path_runtime.record_stream_active != FALSE)
    {
        if (vehicle_path_runtime.record_stream_failed != FALSE)
        {
            vehicle_path_runtime.record_stream_active = FALSE;
            vehicle_path_runtime.record_save_failed = TRUE;
            vehicle_path_runtime.save_busy = FALSE;
            return FALSE;
        }

        return module_vehicle_path_save_close_request();
    }

    if (vehicle_path_runtime.window_from_flash != FALSE)
    {
        return FALSE;
    }

    if (vehicle_path_runtime.state.point_cnt > vehicle_path_max_point_cnt_get())
    {
        return FALSE;
    }

    point_length = vehicle_path_runtime.state.point_cnt * (uint32)sizeof(vehicle_path_point_t);
    aligned_point_length = ((point_length + IFXFLASH_PFLASH_PAGE_LENGTH - 1u)
                            / IFXFLASH_PFLASH_PAGE_LENGTH) * IFXFLASH_PFLASH_PAGE_LENGTH;

    header.magic = VEHICLE_PATH_MAGIC;
    header.version = VEHICLE_PATH_VERSION;
    header.point_cnt = (uint16)vehicle_path_runtime.state.point_cnt;
    header.checksum = vehicle_path_checksum_calculate((const uint8*)vehicle_path_runtime.points, point_length);

    if (vehicle_path_flash_slot_range_valid(VEHICLE_PATH_FLASH_RAW_OFFSET,
                                            vehicle_path_runtime.state.point_cnt) == FALSE)
    {
        return FALSE;
    }

    vehicle_path_header_page_prepare(header_page, &header);

    write_length = service_storage_writeSync(SERVICE_STORAGE_1,
                                             cfg->flash_offset + VEHICLE_PATH_FLASH_RAW_OFFSET,
                                             header_page,
                                             VEHICLE_PATH_FLASH_HEADER_LENGTH);

    if (write_length != VEHICLE_PATH_FLASH_HEADER_LENGTH)
    {
        return FALSE;
    }

    if (vehicle_path_marker_write(VEHICLE_PATH_FLASH_RAW_OFFSET,
                                  vehicle_path_runtime.state.point_cnt) == FALSE)
    {
        return FALSE;
    }

    for (point_offset = 0u; point_offset < aligned_point_length; point_offset += IFXFLASH_PFLASH_PAGE_LENGTH)
    {
        page_valid_length = point_length - point_offset;
        if (page_valid_length > IFXFLASH_PFLASH_PAGE_LENGTH)
        {
            page_valid_length = IFXFLASH_PFLASH_PAGE_LENGTH;
        }

        for (i = 0u; i < IFXFLASH_PFLASH_PAGE_LENGTH; i++)
        {
            point_page[i] = 0xFFu;
        }

        for (i = 0u; i < page_valid_length; i++)
        {
            point_page[i] = ((const uint8*)vehicle_path_runtime.points)[point_offset + i];
        }

        page_offset = cfg->flash_offset
                    + VEHICLE_PATH_FLASH_RAW_OFFSET
                    + VEHICLE_PATH_FLASH_POINT_OFFSET
                    + point_offset;
        write_length = service_storage_writeSync(SERVICE_STORAGE_1,
                                                 page_offset,
                                                 point_page,
                                                 IFXFLASH_PFLASH_PAGE_LENGTH);

        if (write_length != IFXFLASH_PFLASH_PAGE_LENGTH)
        {
            return FALSE;
        }
    }

    vehicle_path_runtime.save_busy = FALSE;
    vehicle_path_runtime.active_slot_offset = VEHICLE_PATH_FLASH_RAW_OFFSET;
    return TRUE;
}

/**
 * @brief 查询路径保存队列是否仍在后台刷写。
 * @param[in] void 无参数。
 * @return 保存任务忙返回 TRUE，否则返回 FALSE。
 */
boolean module_vehicle_path_save_close_request(void)
{
    if (vehicle_path_runtime.record_stream_active == FALSE)
    {
        return FALSE;
    }

    if (vehicle_path_runtime.record_stream_failed != FALSE)
    {
        vehicle_path_runtime.record_save_failed = TRUE;
        vehicle_path_runtime.record_stream_active = FALSE;
        vehicle_path_runtime.save_busy = FALSE;
        return FALSE;
    }

    if (vehicle_path_runtime.record_stream_close_requested == FALSE)
    {
        if (vehicle_path_record_stream_flush(TRUE) == FALSE)
        {
            vehicle_path_runtime.record_save_failed = TRUE;
            vehicle_path_runtime.record_stream_active = FALSE;
            vehicle_path_runtime.save_busy = FALSE;
            return FALSE;
        }

        vehicle_path_runtime.record_stream_close_requested = TRUE;
        vehicle_path_runtime.save_busy = TRUE;
    }

    return TRUE;
}

boolean module_vehicle_path_save_is_busy(void)
{
    return vehicle_path_runtime.save_busy;
}

boolean module_vehicle_path_save_is_failed(void)
{
    return vehicle_path_runtime.record_save_failed;
}

/**
 * @brief 通过存储服务从内部 Flash 加载路径点。
 * @param[in] void 无参数。
 * @return 加载成功返回 TRUE，否则返回 FALSE。
 */
boolean module_vehicle_path_load(void)
{
    if (vehicle_path_operation_busy() != FALSE)
    {
        vehicle_path_runtime.load_state = VEHICLE_PATH_LOAD_STATE_FAILED;
        return FALSE;
    }

    if (module_vehicle_path_load_planned() != FALSE)
    {
        return TRUE;
    }

    if (module_vehicle_path_load_raw() != FALSE)
    {
        tools_printf("{pathinfo}planned_fallback_raw\r\n");
        return TRUE;
    }

    vehicle_path_runtime.load_state = VEHICLE_PATH_LOAD_STATE_FAILED;
    return FALSE;
}

boolean module_vehicle_path_load_raw(void)
{
    vehicle_path_flash_header_t header;

    if (vehicle_path_operation_busy() != FALSE)
    {
        vehicle_path_runtime.load_state = VEHICLE_PATH_LOAD_STATE_FAILED;
        return FALSE;
    }

    vehicle_path_runtime.load_point_length = 0u;
    vehicle_path_runtime.load_point_cnt = 0u;
    vehicle_path_runtime.load_checksum = 0u;
    vehicle_path_runtime.window_start_index_cnt = 0u;
    vehicle_path_runtime.window_valid_cnt = 0u;
    vehicle_path_runtime.window_active_buffer_cnt = 0u;
    vehicle_path_runtime.window_from_flash = FALSE;
    vehicle_path_window_buffers_reset();
    vehicle_path_window_prefetch_reset();
    vehicle_path_planned_speed_overlay_reset();
    vehicle_path_runtime.load_state = VEHICLE_PATH_LOAD_STATE_FAILED;

    if (vehicle_path_flash_header_valid_at(VEHICLE_PATH_FLASH_RAW_OFFSET, &header) != FALSE)
    {
        if (vehicle_path_load_from_slot(VEHICLE_PATH_FLASH_RAW_OFFSET, &header) != FALSE)
        {
            tools_printf("{pathinfo}load_raw\r\n");
            return TRUE;
        }
    }

    module_vehicle_path_clear();
    vehicle_path_runtime.load_state = VEHICLE_PATH_LOAD_STATE_FAILED;
    return FALSE;
}

boolean module_vehicle_path_load_planned(void)
{
    vehicle_path_flash_header_t header;
    vehicle_path_flash_header_t raw_header;
    uint16 raw_point_cnt = 0u;
    boolean standalone_upload;

    if (vehicle_path_operation_busy() != FALSE)
    {
        vehicle_path_runtime.load_state = VEHICLE_PATH_LOAD_STATE_FAILED;
        return FALSE;
    }

    vehicle_path_runtime.load_point_length = 0u;
    vehicle_path_runtime.load_point_cnt = 0u;
    vehicle_path_runtime.load_checksum = 0u;
    vehicle_path_runtime.window_start_index_cnt = 0u;
    vehicle_path_runtime.window_valid_cnt = 0u;
    vehicle_path_runtime.window_active_buffer_cnt = 0u;
    vehicle_path_runtime.window_from_flash = FALSE;
    vehicle_path_window_buffers_reset();
    vehicle_path_window_prefetch_reset();
    vehicle_path_planned_speed_overlay_reset();
    vehicle_path_runtime.load_state = VEHICLE_PATH_LOAD_STATE_FAILED;

    if (vehicle_path_flash_header_valid_at(VEHICLE_PATH_FLASH_PLANNED_OFFSET, &header) != FALSE)
    {
        standalone_upload = vehicle_path_planned_standalone_valid(&header);
        if ((standalone_upload == FALSE)
            && (vehicle_path_flash_header_valid_at(VEHICLE_PATH_FLASH_RAW_OFFSET, &raw_header) == FALSE))
        {
            tools_printf("{pathinfo}planned_raw_missing\r\n");
            module_vehicle_path_clear();
            vehicle_path_runtime.load_state = VEHICLE_PATH_LOAD_STATE_FAILED;
            return FALSE;
        }

        if ((standalone_upload == FALSE)
            && (vehicle_path_planned_header_matches_raw(&header, &raw_point_cnt) == FALSE))
        {
            tools_printf("{pathinfo}planned_stale,%u,%u\r\n",
                         (unsigned int)header.point_cnt,
                         (unsigned int)raw_point_cnt);
            module_vehicle_path_clear();
            vehicle_path_runtime.load_state = VEHICLE_PATH_LOAD_STATE_FAILED;
            return FALSE;
        }

        if (vehicle_path_planned_speed_table_load(&header) == FALSE)
        {
            tools_printf("{pathinfo}planned_speed_failed\r\n");
            module_vehicle_path_clear();
            vehicle_path_runtime.load_state = VEHICLE_PATH_LOAD_STATE_FAILED;
            return FALSE;
        }

        if (vehicle_path_load_from_slot(VEHICLE_PATH_FLASH_PLANNED_OFFSET, &header) != FALSE)
        {
            vehicle_path_runtime.planned_speed_overlay_active = TRUE;
            vehicle_path_runtime.planned_speed_point_cnt = (uint32)header.point_cnt;
            tools_printf("{pathinfo}load_planned\r\n");
            tools_printf("{pathinfo}planned_speed_overlay,%u,%u,%u,%.1f,%.3f\r\n",
                         (unsigned int)header.point_cnt,
                         (unsigned int)header.point_cnt,
                         (unsigned int)((vehicle_path_runtime.planned_speed_meta_valid != FALSE) ? 1u : 0u),
                         (double)vehicle_path_runtime.planned_speed_source_length_mm,
                         (double)vehicle_path_runtime.planned_speed_step_mm);
            return TRUE;
        }

        tools_printf("{pathinfo}planned_failed\r\n");
    }

    module_vehicle_path_clear();
    vehicle_path_runtime.load_state = VEHICLE_PATH_LOAD_STATE_FAILED;
    return FALSE;
}

static boolean vehicle_path_load_from_slot(uint32 slot_offset, const vehicle_path_flash_header_t* header)
{
    uint32 point_length;

    if (header == NULL_PTR)
    {
        return FALSE;
    }

    vehicle_path_runtime.active_slot_offset = slot_offset;

    point_length = (uint32)header->point_cnt * (uint32)sizeof(vehicle_path_point_t);
    vehicle_path_runtime.load_point_cnt = header->point_cnt;
    vehicle_path_runtime.load_checksum = header->checksum;
    vehicle_path_runtime.load_point_length = point_length;
    vehicle_path_runtime.state.point_cnt = (uint32)header->point_cnt;
    vehicle_path_runtime.state.last_distance_mm = 0.0f;
    if (header->point_cnt > 0u)
    {
        vehicle_path_runtime.state.last_distance_mm =
            (float32)(header->point_cnt - 1u) * vehicle_path_min_distance_step_get();
    }
    vehicle_path_source_distance_cache_reset();

    if (point_length == 0u)
    {
        vehicle_path_runtime.load_state = VEHICLE_PATH_LOAD_STATE_IDLE;
        vehicle_path_marker_reset();
        vehicle_path_runtime.replay_edge_points_valid = FALSE;
        return TRUE;
    }

    vehicle_path_runtime.window_active_buffer_cnt = 0u;
    vehicle_path_window_buffers_reset();
    vehicle_path_window_prefetch_reset();
    vehicle_path_runtime.window_start_index_cnt = 0u;
    vehicle_path_runtime.window_valid_cnt = 0u;
    vehicle_path_runtime.window_from_flash = TRUE;
    vehicle_path_runtime.replay_edge_points_valid = FALSE;
    (void)vehicle_path_marker_load_from_slot(slot_offset);
    vehicle_path_runtime.load_state = VEHICLE_PATH_LOAD_STATE_IDLE;
    return TRUE;
}
/**
 * @brief 查询路径加载是否仍在执行，同步加载接口固定返回空闲。
 * @param[in] void 无参数。
 * @return 加载任务忙返回 TRUE，否则返回 FALSE。
 */
boolean module_vehicle_path_load_is_busy(void)
{
    return FALSE;
}

/**
 * @brief 查询最近一次路径加载是否失败。
 * @param[in] void 无参数。
 * @return 加载失败返回 TRUE，否则返回 FALSE。
 */
boolean module_vehicle_path_load_is_failed(void)
{
    return (boolean)(vehicle_path_runtime.load_state == VEHICLE_PATH_LOAD_STATE_FAILED);
}

boolean module_vehicle_path_job_is_busy(void)
{
    return vehicle_path_operation_busy();
}

boolean module_vehicle_path_plan_last_result_get(boolean* success)
{
    if ((success == NULL_PTR) || (vehicle_path_runtime.job_result_valid == FALSE))
    {
        return FALSE;
    }

    *success = vehicle_path_runtime.job_result_success;
    return TRUE;
}

/**
 * @brief 从当前编码器里程开始路径复现。
 * @param[in] void 无参数。
 * @return 成功开始复现返回 TRUE，否则返回 FALSE。
 */
/**
 * @brief Print all saved path points from Flash for serial export.
 * @return TRUE when dump succeeds, otherwise FALSE.
 */
boolean module_vehicle_path_dump(void)
{
    vehicle_path_flash_header_t header;

    if ((vehicle_path_flash_header_valid_at(VEHICLE_PATH_FLASH_PLANNED_OFFSET, &header) != FALSE)
        && (vehicle_path_planned_header_matches_raw(&header, NULL_PTR) != FALSE))
    {
        return vehicle_path_dump_from_slot(VEHICLE_PATH_FLASH_PLANNED_OFFSET);
    }

    return vehicle_path_dump_from_slot(VEHICLE_PATH_FLASH_RAW_OFFSET);
}

boolean module_vehicle_path_dump_raw(void)
{
    return vehicle_path_dump_from_slot(VEHICLE_PATH_FLASH_RAW_OFFSET);
}

boolean module_vehicle_path_dump_planned(void)
{
    return vehicle_path_dump_from_slot(VEHICLE_PATH_FLASH_PLANNED_OFFSET);
}

static boolean vehicle_path_dump_from_slot(uint32 slot_offset)
{
    if (vehicle_path_job_busy() != FALSE)
    {
        tools_printf("{pathdump}busy\r\n");
        return FALSE;
    }

    if ((vehicle_path_runtime.record_stream_active != FALSE)
        || (vehicle_path_runtime.save_busy != FALSE)
        || (vehicle_path_runtime.state.recording != FALSE)
        || (vehicle_path_runtime.state.replaying != FALSE))
    {
        tools_printf("{pathdump}storage_busy\r\n");
        return FALSE;
    }

    if (vehicle_path_flash_header_valid_at(slot_offset, NULL_PTR) == FALSE)
    {
        tools_printf("{pathdump}slot_empty\r\n");
        return FALSE;
    }

    vehicle_path_runtime.job_read_slot_offset = slot_offset;
    vehicle_path_runtime.job_state = VEHICLE_PATH_JOB_DUMP_HEADER;
    vehicle_path_runtime.job_index_cnt = 0u;
    vehicle_path_runtime.job_point_cnt = 0u;
    vehicle_path_runtime.job_checksum = VEHICLE_PATH_CHECKSUM_SEED;
    tools_printf("{pathdump}queued\r\n");
    return TRUE;
}

/**
 * @brief Plan saved path speed and geometry, then write planned path back to Flash.
 * @return TRUE when planning and saving succeeds.
 */
boolean module_vehicle_path_plan(void)
{
    if (vehicle_path_job_busy() != FALSE)
    {
        tools_printf("{pathplan}busy\r\n");
        return FALSE;
    }

    if ((vehicle_path_runtime.record_stream_active != FALSE)
        || (vehicle_path_runtime.save_busy != FALSE)
        || (vehicle_path_runtime.state.recording != FALSE)
        || (vehicle_path_runtime.state.replaying != FALSE))
    {
        tools_printf("{pathplan}storage_busy\r\n");
        return FALSE;
    }

    vehicle_path_runtime.job_state = VEHICLE_PATH_JOB_PLAN_HEADER;
    vehicle_path_runtime.job_index_cnt = 0u;
    vehicle_path_runtime.job_point_cnt = 0u;
    vehicle_path_runtime.job_source_point_cnt = 0u;
    vehicle_path_runtime.job_checksum = VEHICLE_PATH_CHECKSUM_SEED;
    vehicle_path_runtime.job_write_offset = 0u;
    vehicle_path_runtime.job_read_slot_offset = VEHICLE_PATH_FLASH_RAW_OFFSET;
    vehicle_path_runtime.job_write_slot_offset = VEHICLE_PATH_FLASH_PLANNED_OFFSET;
    vehicle_path_runtime.job_planned_cnt = 0u;
    vehicle_path_runtime.job_min_speed_mm_s = 0.0f;
    vehicle_path_runtime.job_max_speed_mm_s = 0.0f;
    vehicle_path_runtime.job_path_length_mm = 0.0f;
    vehicle_path_runtime.job_has_last_point = FALSE;
    vehicle_path_runtime.job_downsampled = FALSE;
    vehicle_path_runtime.job_result_valid = FALSE;
    vehicle_path_runtime.job_result_success = FALSE;
    tools_printf("{pathplan}queued\r\n");
    return TRUE;
}

boolean module_vehicle_path_import_begin(uint32 point_cnt)
{
    if (vehicle_path_runtime.import_active != FALSE)
    {
        return (vehicle_path_runtime.import_expected_point_cnt == point_cnt) ? TRUE : FALSE;
    }
    if (vehicle_path_runtime.import_erase_active != FALSE)
    {
        return (vehicle_path_runtime.import_expected_point_cnt == point_cnt) ? TRUE : FALSE;
    }
    if ((point_cnt < vehicle_path_cfg_get()->min_valid_point_cnt)
        || (point_cnt > vehicle_path_effective_flash_point_cnt_max_get())
        || (vehicle_path_operation_busy() != FALSE))
    {
        return FALSE;
    }

    module_vehicle_path_clear();
    vehicle_path_runtime.import_expected_point_cnt = point_cnt;
    vehicle_path_runtime.import_received_point_cnt = 0u;
    vehicle_path_runtime.import_active = FALSE;
    vehicle_path_runtime.import_commit_pending = FALSE;
    vehicle_path_runtime.state.point_cnt = 0u;
    vehicle_path_runtime.state.last_distance_mm = 0.0f;
    vehicle_path_runtime.record_path_length_mm = 0.0f;
    vehicle_path_planned_slot_erase_begin(point_cnt);
    return TRUE;
}

boolean module_vehicle_path_import_point(uint32 index_cnt, const vehicle_path_point_t* point)
{
    float32 dx_mm;
    float32 dy_mm;

    if ((point == NULL_PTR)
        || (vehicle_path_runtime.import_active == FALSE)
        || (vehicle_path_runtime.import_commit_pending != FALSE)
        || (index_cnt >= vehicle_path_runtime.import_expected_point_cnt))
    {
        return FALSE;
    }
    if (index_cnt < vehicle_path_runtime.import_received_point_cnt)
    {
        return TRUE;
    }
    if (index_cnt != vehicle_path_runtime.import_received_point_cnt)
    {
        return FALSE;
    }
    if ((point->x_mm < -1000000.0f) || (point->x_mm > 1000000.0f)
        || (point->y_mm < -1000000.0f) || (point->y_mm > 1000000.0f)
        || (point->theta_rad < -1000.0f) || (point->theta_rad > 1000.0f)
        || (point->speed_mm_s < 0.0f) || (point->speed_mm_s > 65535.0f))
    {
        return FALSE;
    }
    if (vehicle_path_record_stream_append(point) == FALSE)
    {
        return FALSE;
    }

    if (index_cnt > 0u)
    {
        dx_mm = point->x_mm - vehicle_path_runtime.record_last_point.x_mm;
        dy_mm = point->y_mm - vehicle_path_runtime.record_last_point.y_mm;
        vehicle_path_runtime.record_path_length_mm += sqrtf((dx_mm * dx_mm) + (dy_mm * dy_mm));
    }
    vehicle_path_runtime.record_last_point = *point;
    if (index_cnt < vehicle_path_max_point_cnt_get())
    {
        vehicle_path_runtime.points[index_cnt] = *point;
    }
    vehicle_path_runtime.import_received_point_cnt++;
    vehicle_path_runtime.state.point_cnt = vehicle_path_runtime.import_received_point_cnt;
    vehicle_path_runtime.state.last_distance_mm = vehicle_path_runtime.record_path_length_mm;
    return TRUE;
}

boolean module_vehicle_path_import_marker(vehicle_path_marker_kind_t kind, uint32 index_cnt)
{
    vehicle_path_marker_t* marker;
    uint32 marker_cnt;

    if ((vehicle_path_runtime.import_active == FALSE)
        || (vehicle_path_runtime.import_commit_pending != FALSE)
        || (vehicle_path_marker_kind_valid(kind) == FALSE)
        || (index_cnt >= vehicle_path_runtime.import_received_point_cnt))
    {
        return FALSE;
    }

    for (marker_cnt = 0u; marker_cnt < vehicle_path_runtime.marker_cnt; marker_cnt++)
    {
        marker = &vehicle_path_runtime.markers[marker_cnt];
        if ((marker->valid != FALSE)
            && (marker->index_cnt == index_cnt)
            && (marker->kind == kind))
        {
            return TRUE;
        }
    }

    if (vehicle_path_runtime.marker_cnt >= VEHICLE_PATH_MARKER_MAX_COUNT)
    {
        return FALSE;
    }

    marker = &vehicle_path_runtime.markers[vehicle_path_runtime.marker_cnt];
    marker->index_cnt = index_cnt;
    marker->kind = kind;
    marker->point.x_mm = 0.0f;
    marker->point.y_mm = 0.0f;
    marker->point.theta_rad = 0.0f;
    marker->point.speed_mm_s = 0.0f;
    marker->valid = TRUE;
    vehicle_path_runtime.marker_cnt++;
    vehicle_path_marker_sort();
    return TRUE;
}

boolean module_vehicle_path_import_phototube_zone(uint32 start_index_cnt, uint32 end_index_cnt)
{
    vehicle_path_phototube_zone_t* previous;

    if ((vehicle_path_runtime.import_active == FALSE)
        || (vehicle_path_runtime.import_commit_pending != FALSE)
        || (start_index_cnt > end_index_cnt)
        || (end_index_cnt >= vehicle_path_runtime.import_received_point_cnt)
        || (end_index_cnt > 65535u))
    {
        return FALSE;
    }
    if (vehicle_path_runtime.phototube_zone_cnt > 0u)
    {
        previous = &vehicle_path_runtime.phototube_zones[
            vehicle_path_runtime.phototube_zone_cnt - 1u];
        if ((previous->start_index_cnt == (uint16)start_index_cnt)
            && (previous->end_index_cnt == (uint16)end_index_cnt))
        {
            return TRUE;
        }
        if (start_index_cnt <= previous->end_index_cnt)
        {
            return FALSE;
        }
    }
    if (vehicle_path_runtime.phototube_zone_cnt >= VEHICLE_PATH_PHOTOTUBE_ZONE_MAX_COUNT)
    {
        return FALSE;
    }
    vehicle_path_runtime.phototube_zones[vehicle_path_runtime.phototube_zone_cnt].start_index_cnt =
        (uint16)start_index_cnt;
    vehicle_path_runtime.phototube_zones[vehicle_path_runtime.phototube_zone_cnt].end_index_cnt =
        (uint16)end_index_cnt;
    vehicle_path_runtime.phototube_zone_cnt++;
    return TRUE;
}

boolean module_vehicle_path_phototube_correction_allowed(uint32 index_cnt)
{
    uint32 zone_cnt;
    for (zone_cnt = 0u; zone_cnt < vehicle_path_runtime.phototube_zone_cnt; zone_cnt++)
    {
        const vehicle_path_phototube_zone_t* zone =
            &vehicle_path_runtime.phototube_zones[zone_cnt];
        if (index_cnt < zone->start_index_cnt)
        {
            break;
        }
        if (index_cnt <= zone->end_index_cnt)
        {
            return FALSE;
        }
    }
    return TRUE;
}

boolean module_vehicle_path_import_commit(void)
{
    if ((vehicle_path_runtime.import_active != FALSE)
        && (vehicle_path_runtime.import_commit_pending != FALSE))
    {
        return TRUE;
    }
    if ((vehicle_path_runtime.import_active == FALSE)
        || (vehicle_path_runtime.import_received_point_cnt
            != vehicle_path_runtime.import_expected_point_cnt))
    {
        return FALSE;
    }
    vehicle_path_runtime.import_commit_pending = TRUE;
    return module_vehicle_path_save_close_request();
}

void module_vehicle_path_import_abort(void)
{
    if ((vehicle_path_runtime.import_active == FALSE)
        && (vehicle_path_runtime.import_erase_active == FALSE))
    {
        return;
    }
    vehicle_path_planned_slot_erase_reset();
    vehicle_path_record_stream_reset();
    module_vehicle_path_clear();
}

boolean module_vehicle_path_import_active_get(void)
{
    return vehicle_path_runtime.import_active;
}

boolean module_vehicle_path_import_preparing_get(void)
{
    return vehicle_path_runtime.import_erase_active;
}

uint32 module_vehicle_path_import_received_count_get(void)
{
    return vehicle_path_runtime.import_received_point_cnt;
}

uint32 module_vehicle_path_import_expected_count_get(void)
{
    return vehicle_path_runtime.import_expected_point_cnt;
}

uint32 module_vehicle_path_import_checksum_get(void)
{
    return vehicle_path_runtime.record_checksum;
}

uint32 module_vehicle_path_flash_point_capacity_get(void)
{
    return vehicle_path_effective_flash_point_cnt_max_get();
}

boolean module_vehicle_path_replay_start(void)
{
    const module_vehicle_pose_fusion_observation_t* pose_observation;
    vehicle_path_point_t start_point;

    if (vehicle_path_runtime.state.point_cnt < vehicle_path_cfg_get()->min_valid_point_cnt)
    {
        return FALSE;
    }

    if (vehicle_path_point_read_preserve_window(0u, &start_point) == FALSE)
    {
        return FALSE;
    }

    (void)vehicle_path_replay_edge_points_refresh();
    if (vehicle_path_runtime.marker_cnt == 0u)
    {
        (void)vehicle_path_marker_load_from_slot(vehicle_path_runtime.active_slot_offset);
    }
    else
    {
        (void)vehicle_path_marker_refresh_points();
    }

    if ((vehicle_path_runtime.window_from_flash != FALSE)
        && (vehicle_path_window_load(0u) == FALSE))
    {
        return FALSE;
    }

    vehicle_path_runtime_origin_reset(0.0f, 0.0f, start_point.theta_rad);

    pose_observation = module_vehicle_pose_fusion_observation_get();

    vehicle_path_runtime.state.recording = FALSE;
    vehicle_path_runtime.state.replaying = TRUE;
    vehicle_path_runtime.state.replay_target_valid = FALSE;
    vehicle_path_runtime.state.status = VEHICLE_PATH_STATUS_REPLAY;
    module_vehicle_pose_fusion_replay_correction_start();
    vehicle_path_runtime.state.replay_start_distance_mm = pose_observation->distance_mm;
    vehicle_path_runtime.state.replay_cursor_cnt = 0u;
    vehicle_path_runtime.state.replay_target_index_cnt = 0u;
    vehicle_path_runtime.state.replay_target_theta_rad = pose_observation->theta_rad;
    vehicle_path_replay_target_reset();
    vehicle_path_runtime.replay_progress_index_float = 0.0f;
    vehicle_path_runtime.replay_progress_traveled_mm = 0.0f;
    vehicle_path_runtime.replay_progress_valid = TRUE;
    vehicle_path_source_distance_cache_reset();

    return module_vehicle_path_replay_update();
}

/**
 * @brief 停止路径复现。
 * @param[in] void 无参数。
 * @return void
 */
void module_vehicle_path_replay_stop(void)
{
    module_vehicle_pose_fusion_replay_correction_stop();
    vehicle_path_runtime.state.replaying = FALSE;
    vehicle_path_runtime.state.replay_target_valid = FALSE;
    vehicle_path_replay_lookahead_speed_cap_mm_s = 0.0f;
    vehicle_path_window_prefetch_reset();
    vehicle_path_replay_target_reset();

    if ((vehicle_path_runtime.state.status == VEHICLE_PATH_STATUS_REPLAY)
        || (vehicle_path_runtime.state.status == VEHICLE_PATH_STATUS_FINISHED))
    {
        vehicle_path_runtime.state.status = VEHICLE_PATH_STATUS_IDLE;
    }
}

/**
 * @brief 根据当前编码器位姿更新路径复现目标。
 * @param[in] void 无参数。
 * @return 复现目标有效返回 TRUE，否则返回 FALSE。
 */
boolean module_vehicle_path_replay_update(void)
{
    const module_vehicle_pose_fusion_observation_t* pose_observation = module_vehicle_pose_fusion_observation_get();
    const module_vehicle_encoder_observation_t* encoder_observation = module_vehicle_encoder_observation_get();
    uint32 base_index_cnt;
    uint32 target_index_cnt;
    uint32 tangent_index_cnt;
    uint32 prefetch_index_cnt;
    uint32 speed_preview_index_cnt;
    float32 base_index_float;
    float32 target_index_float;
    float32 target_theta_rad;
    float32 feedforward_theta_rad;
    float32 lookahead_distance_mm;
    float32 current_speed_abs_mm_s;
    float32 tangent_distance_mm;
    float32 tangent_index_float;
    float32 replay_speed_mm_s;
    float32 raw_plan_speed_mm_s;
    float32 speed_preview_limit_mm_s;
    float32 speed_preview_index_float;
    float32 remaining_mm;
    float32 base_dx_mm;
    float32 base_dy_mm;
    float32 cross_track_error_mm;
    float32 target_yaw_rate_rad_s;
    float32 yaw_ff_mm_s;
    float32 dense_cross_track_error_mm;
    float32 dense_target_theta_rad;
    float32 target_theta_raw_rad;
    float32 base_source_distance_mm = 0.0f;
    float32 marker_weight;
    float32 correction_straight_index_float;
    vehicle_path_point_t interpolated_target_point;
    vehicle_path_point_t dense_target_point;
    vehicle_path_point_t tangent_point;
    vehicle_path_point_t dense_tangent_point;
    vehicle_path_point_t base_point;
    vehicle_path_point_t dense_base_point;
    vehicle_path_point_t correction_straight_point;
    vehicle_path_marker_target_t marker_target;
    boolean marker_target_valid;
    boolean end_brake_active;
    boolean base_source_distance_valid = FALSE;

    if ((vehicle_path_runtime.state.replaying == FALSE)
        || (vehicle_path_runtime.state.point_cnt < vehicle_path_cfg_get()->min_valid_point_cnt))
    {
        vehicle_path_runtime.state.replay_target_valid = FALSE;
        vehicle_path_runtime.replay_target.valid = FALSE;
        module_vehicle_pose_fusion_replay_path_straight_set(FALSE);
        return FALSE;
    }

    if (vehicle_path_replay_pure_progress_find(pose_observation->x_mm,
                                               pose_observation->y_mm,
                                               &base_index_float) == FALSE)
    {
        vehicle_path_runtime.state.replay_target_valid = FALSE;
        vehicle_path_runtime.replay_target.valid = FALSE;
        module_vehicle_pose_fusion_replay_path_straight_set(FALSE);
        return FALSE;
    }
    vehicle_path_runtime.replay_progress_index_float = base_index_float;
    vehicle_path_runtime.replay_progress_traveled_mm = vehicle_path_replay_traveled_mm_get();
    vehicle_path_runtime.replay_progress_predicted_index_float = base_index_float;
    vehicle_path_runtime.replay_progress_projected_index_float = base_index_float;
    vehicle_path_runtime.replay_progress_correction_index_float = 0.0f;
    vehicle_path_runtime.replay_progress_valid = TRUE;

    base_index_cnt = (uint32)base_index_float;
    if (base_index_cnt >= vehicle_path_runtime.state.point_cnt)
    {
        base_index_cnt = vehicle_path_runtime.state.point_cnt - 1u;
    }

    if (vehicle_path_point_at_index_float(base_index_float, &base_point) == FALSE)
    {
        vehicle_path_runtime.state.replay_target_valid = FALSE;
        vehicle_path_runtime.replay_target.valid = FALSE;
        module_vehicle_pose_fusion_replay_path_straight_set(FALSE);
        return FALSE;
    }
    if ((vehicle_path_index_advance_by_distance(
             base_index_float,
             VEHICLE_PATH_REPLAY_CORRECTION_STRAIGHT_CHECK_MM,
             &correction_straight_index_float) != FALSE)
        && (vehicle_path_point_at_index_float(correction_straight_index_float,
                                              &correction_straight_point) != FALSE)
        && (vehicle_path_plan_abs_f32(vehicle_path_wrap_pi(
                correction_straight_point.theta_rad - base_point.theta_rad))
            <= VEHICLE_PATH_REPLAY_CORRECTION_STRAIGHT_HEADING_RAD))
    {
        module_vehicle_pose_fusion_replay_path_straight_set(TRUE);
    }
    else
    {
        module_vehicle_pose_fusion_replay_path_straight_set(FALSE);
    }

    remaining_mm = vehicle_path_replay_remaining_mm_get(base_index_float);
    end_brake_active = (remaining_mm <= VEHICLE_PATH_DEFAULT_REPLAY_END_BRAKE_DISTANCE_MM)
                         ? TRUE
                         : FALSE;

    current_speed_abs_mm_s = encoder_observation->speed_mm_s;
    if (current_speed_abs_mm_s < 0.0f)
    {
        current_speed_abs_mm_s = -current_speed_abs_mm_s;
    }

    lookahead_distance_mm = vehicle_path_replay_fixed_lookahead_distance_mm_get();
    tangent_distance_mm = (float32)vehicle_path_replay_tangent_gap_cnt_get() * vehicle_path_min_distance_step_get();

    if (vehicle_path_replay_tangent_target_find(base_index_float,
                                                lookahead_distance_mm,
                                                tangent_distance_mm,
                                                &target_index_float,
                                                &tangent_index_float,
                                                &interpolated_target_point,
                                                &tangent_point,
                                                &feedforward_theta_rad) == FALSE)
    {
        vehicle_path_runtime.state.replay_target_valid = FALSE;
        vehicle_path_runtime.replay_target.valid = FALSE;
        module_vehicle_pose_fusion_replay_path_straight_set(FALSE);
        return FALSE;
    }

    (void)vehicle_path_replay_smoothed_point_get(target_index_float,
                                                 VEHICLE_PATH_REPLAY_AIM_SMOOTH_WINDOW_MM,
                                                 &interpolated_target_point);
    (void)vehicle_path_replay_smoothed_point_get(tangent_index_float,
                                                 VEHICLE_PATH_REPLAY_AIM_SMOOTH_WINDOW_MM,
                                                 &tangent_point);
    feedforward_theta_rad = vehicle_path_replay_point_heading_get(&interpolated_target_point,
                                                                 &tangent_point,
                                                                 feedforward_theta_rad);

    dense_base_point = base_point;
    dense_target_point = interpolated_target_point;
    dense_tangent_point = tangent_point;
    base_dx_mm = pose_observation->x_mm - base_point.x_mm;
    base_dy_mm = pose_observation->y_mm - base_point.y_mm;
    cross_track_error_mm = (-sinf(base_point.theta_rad) * base_dx_mm) + (cosf(base_point.theta_rad) * base_dy_mm);
    dense_cross_track_error_mm = cross_track_error_mm;
    if (vehicle_path_replay_heading_to_point_get(pose_observation,
                                                 &dense_target_point,
                                                 feedforward_theta_rad,
                                                 &dense_target_theta_rad,
                                                 NULL_PTR) == FALSE)
    {
        dense_target_theta_rad = feedforward_theta_rad;
    }

    marker_target_valid = vehicle_path_marker_tracking_target_get(base_index_float,
                                                                  target_index_float,
                                                                  lookahead_distance_mm,
                                                                  tangent_distance_mm,
                                                                  current_speed_abs_mm_s,
                                                                  &marker_target);
    marker_weight = 0.0f;
    if (marker_target_valid != FALSE)
    {
        marker_weight = marker_target.blend_ratio;
        if (marker_target.mode == VEHICLE_PATH_REPLAY_TRACK_MARKER)
        {
            marker_weight = 1.0f;
        }
        else if (marker_target.mode == VEHICLE_PATH_REPLAY_TRACK_DENSE)
        {
            marker_weight = 0.0f;
        }

        if (marker_weight > 0.0f)
        {
            vehicle_path_point_blend(&dense_target_point,
                                     &marker_target.target_point,
                                     marker_weight,
                                     &interpolated_target_point);
            vehicle_path_point_blend(&dense_tangent_point,
                                     &marker_target.tangent_point,
                                     marker_weight,
                                     &tangent_point);
            vehicle_path_point_blend(&dense_base_point,
                                     &marker_target.base_point,
                                     marker_weight,
                                     &base_point);
            target_index_float +=
                (marker_target.target_index_float - target_index_float) * marker_weight;
            tangent_index_float +=
                (marker_target.tangent_index_float - tangent_index_float) * marker_weight;
            cross_track_error_mm +=
                (marker_target.cross_track_error_mm - cross_track_error_mm) * marker_weight;
            feedforward_theta_rad =
                vehicle_path_angle_blend(feedforward_theta_rad,
                                         marker_target.feedforward_theta_rad,
                                         marker_weight);
        }
    }

    feedforward_theta_rad = vehicle_path_replay_point_heading_get(&interpolated_target_point,
                                                                 &tangent_point,
                                                                 feedforward_theta_rad);
    if ((marker_target_valid != FALSE) && (marker_weight > 0.0f))
    {
        target_theta_raw_rad = vehicle_path_angle_blend(dense_target_theta_rad,
                                                        marker_target.target_theta_rad,
                                                        marker_weight);
    }
    else if (vehicle_path_replay_heading_to_point_get(pose_observation,
                                                      &interpolated_target_point,
                                                      feedforward_theta_rad,
                                                      &target_theta_raw_rad,
                                                      NULL_PTR) == FALSE)
    {
        target_theta_raw_rad = feedforward_theta_rad;
    }

    if (end_brake_active != FALSE)
    {
        target_theta_raw_rad = feedforward_theta_rad;
    }

    target_theta_rad = vehicle_path_replay_target_theta_continuous(target_theta_raw_rad);
    interpolated_target_point.theta_rad = target_theta_rad;

    base_dx_mm = pose_observation->x_mm - base_point.x_mm;
    base_dy_mm = pose_observation->y_mm - base_point.y_mm;

    if (vehicle_path_replay_planned_speed_active() != FALSE)
    {
        float32 base_speed_mm_s;

        base_source_distance_valid =
            vehicle_path_distance_at_index_get(base_index_float, &base_source_distance_mm);
        base_speed_mm_s =
            vehicle_path_planned_speed_at_index_get(base_index_float, dense_base_point.speed_mm_s);
        replay_speed_mm_s = base_speed_mm_s;
        raw_plan_speed_mm_s = replay_speed_mm_s;
        speed_preview_limit_mm_s =
            vehicle_path_planned_speed_preview_limit_get(base_index_float,
                                                         base_source_distance_mm,
                                                         base_source_distance_valid,
                                                         replay_speed_mm_s,
                                                         &speed_preview_index_float);
        if (speed_preview_limit_mm_s < replay_speed_mm_s)
        {
            replay_speed_mm_s = speed_preview_limit_mm_s;
        }
    }
    else
    {
        replay_speed_mm_s = vehicle_path_replay_speed_mm_s;
        raw_plan_speed_mm_s = replay_speed_mm_s;
        speed_preview_limit_mm_s = replay_speed_mm_s;
        speed_preview_index_float = base_index_float;
    }

    if (remaining_mm < VEHICLE_PATH_DEFAULT_END_DECEL_DISTANCE_MM)
    {
        float32 terminal_reference_speed_mm_s = raw_plan_speed_mm_s;
        float32 terminal_speed_limit_mm_s;

        if (vehicle_path_replay_planned_speed_active() != FALSE)
        {
            terminal_reference_speed_mm_s = vehicle_path_cfg_get()->plan_max_speed_mm_s;
        }
        terminal_speed_limit_mm_s = terminal_reference_speed_mm_s
                                  * vehicle_path_end_speed_scale_get(remaining_mm);
        if (replay_speed_mm_s > terminal_speed_limit_mm_s)
        {
            replay_speed_mm_s = terminal_speed_limit_mm_s;
        }
        if (speed_preview_limit_mm_s > terminal_speed_limit_mm_s)
        {
            speed_preview_limit_mm_s = terminal_speed_limit_mm_s;
        }
    }

    if ((vehicle_path_replay_lookahead_speed_cap_mm_s > 0.0f)
        && (replay_speed_mm_s > vehicle_path_replay_lookahead_speed_cap_mm_s))
    {
        replay_speed_mm_s = vehicle_path_replay_lookahead_speed_cap_mm_s;
    }
    if (speed_preview_limit_mm_s > replay_speed_mm_s)
    {
        speed_preview_limit_mm_s = replay_speed_mm_s;
    }

    if (end_brake_active != FALSE)
    {
        replay_speed_mm_s = 0.0f;
        speed_preview_limit_mm_s = 0.0f;
    }
    interpolated_target_point.speed_mm_s = replay_speed_mm_s;
    target_yaw_rate_rad_s = 0.0f;
    yaw_ff_mm_s = 0.0f;

    target_index_cnt = (uint32)target_index_float;
    tangent_index_cnt = (uint32)tangent_index_float;
    if (target_index_cnt >= vehicle_path_runtime.state.point_cnt)
    {
        target_index_cnt = vehicle_path_runtime.state.point_cnt - 1u;
    }
    if (tangent_index_cnt >= vehicle_path_runtime.state.point_cnt)
    {
        tangent_index_cnt = vehicle_path_runtime.state.point_cnt - 1u;
    }

    vehicle_path_runtime.replay_target.base_point = base_point;
    vehicle_path_runtime.replay_target.target_point = interpolated_target_point;
    vehicle_path_runtime.replay_target.tangent_point = tangent_point;
    vehicle_path_runtime.replay_target.base_index_float = base_index_float;
    vehicle_path_runtime.replay_target.target_index_float = target_index_float;
    vehicle_path_runtime.replay_target.tangent_index_float = tangent_index_float;
    vehicle_path_runtime.replay_target.lookahead_distance_mm = lookahead_distance_mm;
    vehicle_path_runtime.replay_target.tangent_distance_mm = tangent_distance_mm;
    vehicle_path_runtime.replay_target.cross_track_error_mm = cross_track_error_mm;
    vehicle_path_runtime.replay_target.along_track_error_mm =
        (cosf(base_point.theta_rad) * base_dx_mm) + (sinf(base_point.theta_rad) * base_dy_mm);
    vehicle_path_runtime.replay_target.target_index_cnt = target_index_cnt;
    vehicle_path_runtime.replay_target.feedforward_theta_rad = feedforward_theta_rad;
    vehicle_path_runtime.replay_target.target_theta_rad = target_theta_rad;
    vehicle_path_runtime.replay_target.target_yaw_rate_rad_s = target_yaw_rate_rad_s;
    vehicle_path_runtime.replay_target.yaw_ff_mm_s = yaw_ff_mm_s;
    vehicle_path_runtime.replay_target.target_speed_mm_s = replay_speed_mm_s;
    vehicle_path_runtime.replay_target.raw_plan_speed_mm_s = raw_plan_speed_mm_s;
    vehicle_path_runtime.replay_target.speed_preview_limit_mm_s = speed_preview_limit_mm_s;
    vehicle_path_runtime.replay_target.marker_segment_cnt =
        (marker_target_valid != FALSE) ? marker_target.segment_cnt : 0u;
    vehicle_path_runtime.replay_target.marker_next_cnt =
        (marker_target_valid != FALSE) ? marker_target.next_cnt : 0u;
    vehicle_path_runtime.replay_target.marker_blend_ratio = marker_weight;
    vehicle_path_runtime.replay_target.marker_cross_track_error_mm =
        (marker_target_valid != FALSE) ? marker_target.cross_track_error_mm : 0.0f;
    vehicle_path_runtime.replay_target.dense_cross_track_error_mm = dense_cross_track_error_mm;
    vehicle_path_runtime.replay_target.marker_target_theta_rad =
        (marker_target_valid != FALSE) ? marker_target.target_theta_rad : 0.0f;
    vehicle_path_runtime.replay_target.dense_target_theta_rad = dense_target_theta_rad;
    vehicle_path_runtime.replay_target.progress_predicted_index_float =
        vehicle_path_runtime.replay_progress_predicted_index_float;
    vehicle_path_runtime.replay_target.progress_projected_index_float =
        vehicle_path_runtime.replay_progress_projected_index_float;
    vehicle_path_runtime.replay_target.progress_correction_index_float =
        vehicle_path_runtime.replay_progress_correction_index_float;
    vehicle_path_runtime.replay_target.track_mode =
        (marker_target_valid != FALSE) ? marker_target.mode : VEHICLE_PATH_REPLAY_TRACK_DENSE;
    vehicle_path_runtime.replay_target.curve_section_active =
        ((marker_target_valid != FALSE)
         && (marker_target.mode != VEHICLE_PATH_REPLAY_TRACK_MARKER))
            ? TRUE
            : FALSE;
    vehicle_path_runtime.replay_target.valid = TRUE;
    vehicle_path_runtime.state.replay_cursor_cnt = base_index_cnt;
    vehicle_path_runtime.state.replay_target_index_cnt = target_index_cnt;
    vehicle_path_runtime.state.replay_target_theta_rad = vehicle_path_runtime.replay_target.target_theta_rad;
    vehicle_path_runtime.state.replay_target_valid = TRUE;
    if (end_brake_active != FALSE)
    {
        module_vehicle_pose_fusion_replay_correction_stop();
        vehicle_path_runtime.state.replaying = FALSE;
        vehicle_path_runtime.state.status = VEHICLE_PATH_STATUS_FINISHED;
    }
    prefetch_index_cnt = target_index_cnt;
    if (tangent_index_cnt > prefetch_index_cnt)
    {
        prefetch_index_cnt = tangent_index_cnt;
    }
    if (base_index_cnt > prefetch_index_cnt)
    {
        prefetch_index_cnt = base_index_cnt;
    }
    speed_preview_index_cnt = (uint32)speed_preview_index_float;
    if (speed_preview_index_cnt >= vehicle_path_runtime.state.point_cnt)
    {
        speed_preview_index_cnt = vehicle_path_runtime.state.point_cnt - 1u;
    }
    if (speed_preview_index_cnt > prefetch_index_cnt)
    {
        prefetch_index_cnt = speed_preview_index_cnt;
    }
    vehicle_path_window_prefetch_update(prefetch_index_cnt);

    return TRUE;
}

/**
 * @brief 获取当前路径复现目标。
 * @param[out] target 复现目标输出指针。
 * @return 复现目标有效并完成复制返回 TRUE，否则返回 FALSE。
 */
boolean module_vehicle_path_replay_target_get(vehicle_path_replay_target_t* target)
{
    if (target == NULL_PTR)
    {
        return FALSE;
    }

    if (vehicle_path_runtime.replay_target.valid == FALSE)
    {
        return FALSE;
    }

    *target = vehicle_path_runtime.replay_target;
    return TRUE;
}

/**
 * @brief 根据索引获取一个已记录路径点。
 * @param[in] index_cnt 路径点索引，单位：点。
 * @param[out] point 路径点输出指针。
 * @return 索引有效并完成复制返回 TRUE，否则返回 FALSE。
 */
boolean module_vehicle_path_point_get(uint32 index_cnt, vehicle_path_point_t* point)
{
    if (point == NULL_PTR)
    {
        return FALSE;
    }

    return vehicle_path_point_read(index_cnt, point);
}

float32 module_vehicle_path_min_distance_step_get(void)
{
    return vehicle_path_min_distance_step_get();
}

boolean module_vehicle_path_project_local(float32 x_mm,
                                          float32 y_mm,
                                          float32 center_index_float,
                                          float32 window_mm,
                                          vehicle_path_projection_t* projection)
{
    uint32 point_cnt = vehicle_path_runtime.state.point_cnt;
    float32 step_mm = vehicle_path_min_distance_step_get();
    uint32 window_cnt;
    uint32 center_index_cnt;
    uint32 start_index_cnt;
    uint32 end_index_cnt;
    uint32 index_cnt;
    vehicle_path_point_t point_a;
    vehicle_path_point_t point_b;
    float32 vx_mm;
    float32 vy_mm;
    float32 wx_mm;
    float32 wy_mm;
    float32 segment_length_sq_mm;
    float32 ratio;
    float32 proj_x_mm;
    float32 proj_y_mm;
    float32 dx_mm;
    float32 dy_mm;
    float32 distance_sq_mm;
    float32 best_distance_sq = 0.0f;
    boolean found = FALSE;

    if (projection == NULL_PTR)
    {
        return FALSE;
    }
    projection->valid = FALSE;
    if (point_cnt < 2u)
    {
        return FALSE;
    }

    if (window_mm < 0.0f)
    {
        window_mm = -window_mm;
    }
    if (step_mm < 1.0f)
    {
        step_mm = 1.0f;
    }

    window_cnt = (uint32)((window_mm / step_mm) + 0.5f);
    if (window_cnt < 2u)
    {
        window_cnt = 2u;
    }

    if (center_index_float < 0.0f)
    {
        center_index_float = 0.0f;
    }
    if (center_index_float > (float32)(point_cnt - 1u))
    {
        center_index_float = (float32)(point_cnt - 1u);
    }

    center_index_cnt = (uint32)center_index_float;
    start_index_cnt = (center_index_cnt > window_cnt) ? (center_index_cnt - window_cnt) : 0u;
    end_index_cnt = center_index_cnt + window_cnt;
    if (end_index_cnt >= (point_cnt - 1u))
    {
        end_index_cnt = point_cnt - 2u;
    }
    if (start_index_cnt > end_index_cnt)
    {
        return FALSE;
    }

    for (index_cnt = start_index_cnt; index_cnt <= end_index_cnt; index_cnt++)
    {
        if ((vehicle_path_point_read_cached(index_cnt, &point_a) == FALSE)
            || (vehicle_path_point_read_cached(index_cnt + 1u, &point_b) == FALSE))
        {
            continue;
        }

        vx_mm = point_b.x_mm - point_a.x_mm;
        vy_mm = point_b.y_mm - point_a.y_mm;
        segment_length_sq_mm = (vx_mm * vx_mm) + (vy_mm * vy_mm);
        if (segment_length_sq_mm < 0.001f)
        {
            continue;
        }

        wx_mm = x_mm - point_a.x_mm;
        wy_mm = y_mm - point_a.y_mm;
        ratio = ((wx_mm * vx_mm) + (wy_mm * vy_mm)) / segment_length_sq_mm;
        if (ratio < 0.0f)
        {
            ratio = 0.0f;
        }
        else if (ratio > 1.0f)
        {
            ratio = 1.0f;
        }

        proj_x_mm = point_a.x_mm + (vx_mm * ratio);
        proj_y_mm = point_a.y_mm + (vy_mm * ratio);
        dx_mm = x_mm - proj_x_mm;
        dy_mm = y_mm - proj_y_mm;
        distance_sq_mm = (dx_mm * dx_mm) + (dy_mm * dy_mm);
        if ((found == FALSE) || (distance_sq_mm < best_distance_sq))
        {
            best_distance_sq = distance_sq_mm;
            projection->point.x_mm = proj_x_mm;
            projection->point.y_mm = proj_y_mm;
            projection->point.theta_rad = vehicle_path_wrap_pi(atan2f(vy_mm, vx_mm));
            projection->point.speed_mm_s =
                point_a.speed_mm_s + ((point_b.speed_mm_s - point_a.speed_mm_s) * ratio);
            projection->index_float = (float32)index_cnt + ratio;
            projection->distance_sq_mm = distance_sq_mm;
            projection->distance_mm = sqrtf(distance_sq_mm);
            projection->segment_index_cnt = index_cnt;
            projection->valid = TRUE;
            found = TRUE;
        }
    }

    if (found == FALSE)
    {
        projection->valid = FALSE;
    }

    return found;
}

void module_vehicle_path_replay_speed_set(float32 speed_mm_s)
{
    if (speed_mm_s < 0.0f)
    {
        speed_mm_s = 0.0f;
    }

    vehicle_path_replay_speed_mm_s = speed_mm_s;
}

float32 module_vehicle_path_replay_speed_get(void)
{
    return vehicle_path_replay_speed_mm_s;
}

boolean module_vehicle_path_replay_planned_speed_active_get(void)
{
    return vehicle_path_replay_planned_speed_active();
}

boolean module_vehicle_path_replay_planned_speed_available_get(void)
{
    return vehicle_path_runtime.planned_speed_overlay_active;
}

boolean module_vehicle_path_replay_planned_speed_requested_get(void)
{
    return vehicle_path_runtime.planned_speed_enabled;
}

boolean module_vehicle_path_replay_planned_speed_enable(boolean enable)
{
    vehicle_path_runtime.planned_speed_enabled = (enable != FALSE) ? TRUE : FALSE;
    return (boolean)((enable == FALSE)
                  || (vehicle_path_runtime.planned_speed_overlay_active != FALSE));
}

uint32 module_vehicle_path_replay_planned_speed_point_count_get(void)
{
    return vehicle_path_runtime.planned_speed_point_cnt;
}

uint32 module_vehicle_path_replay_planned_speed_source_point_count_get(void)
{
    return vehicle_path_runtime.planned_speed_source_point_cnt;
}

float32 module_vehicle_path_replay_planned_speed_source_length_get(void)
{
    return vehicle_path_runtime.planned_speed_source_length_mm;
}

float32 module_vehicle_path_replay_planned_speed_step_get(void)
{
    return vehicle_path_runtime.planned_speed_step_mm;
}

boolean module_vehicle_path_replay_planned_speed_meta_valid_get(void)
{
    return vehicle_path_runtime.planned_speed_meta_valid;
}

void module_vehicle_path_replay_speed_preview_distance_set(float32 distance_mm)
{
    if (distance_mm < 0.0f)
    {
        distance_mm = 0.0f;
    }
    if (distance_mm > 1500.0f)
    {
        distance_mm = 1500.0f;
    }

    vehicle_path_replay_speed_preview_distance_mm = distance_mm;
}

float32 module_vehicle_path_replay_speed_preview_distance_get(void)
{
    return vehicle_path_replay_speed_preview_distance_mm;
}

void module_vehicle_path_replay_ahead_set(float32 lookahead_mm, float32 tangent_mm)
{
    module_vehicle_path_replay_lookahead_set(lookahead_mm);
    module_vehicle_path_replay_tangent_set(tangent_mm);
}

void module_vehicle_path_replay_lookahead_set(float32 lookahead_mm)
{
    if (lookahead_mm < 40.0f)
    {
        lookahead_mm = 40.0f;
    }

    vehicle_path_replay_lookahead_cnt_runtime = vehicle_path_replay_distance_to_cnt(lookahead_mm);
}

void module_vehicle_path_replay_tangent_set(float32 tangent_mm)
{
    if (tangent_mm < vehicle_path_min_distance_step_get())
    {
        tangent_mm = vehicle_path_min_distance_step_get();
    }

    vehicle_path_replay_tangent_gap_cnt_runtime = vehicle_path_replay_distance_to_cnt(tangent_mm);
}

void module_vehicle_path_replay_ahead_reset(void)
{
    vehicle_path_replay_lookahead_cnt_runtime = vehicle_path_cfg_get()->replay_lookahead_cnt;
    vehicle_path_replay_tangent_gap_cnt_runtime = vehicle_path_cfg_get()->replay_tangent_gap_cnt;
}

void module_vehicle_path_replay_ahead_get(float32* lookahead_mm, float32* tangent_mm)
{
    if (lookahead_mm != NULL_PTR)
    {
        *lookahead_mm = vehicle_path_replay_lookahead_distance_mm_get();
    }

    if (tangent_mm != NULL_PTR)
    {
        *tangent_mm = (float32)vehicle_path_replay_tangent_gap_cnt_get()
                    * vehicle_path_min_distance_step_get();
    }
}

void module_vehicle_path_replay_lookahead_speed_cap_set(float32 speed_mm_s)
{
    if (speed_mm_s < 0.0f)
    {
        speed_mm_s = 0.0f;
    }

    vehicle_path_replay_lookahead_speed_cap_mm_s = speed_mm_s;
}

boolean module_vehicle_path_marker_add(vehicle_path_marker_kind_t kind)
{
    const module_vehicle_pose_fusion_observation_t* pose_observation;
    uint32 nearest_index_cnt;

    if ((vehicle_path_marker_kind_valid(kind) == FALSE)
        || (vehicle_path_runtime.state.point_cnt == 0u)
        || (vehicle_path_runtime.marker_cnt >= VEHICLE_PATH_MARKER_MAX_COUNT))
    {
        return FALSE;
    }

    if (vehicle_path_runtime.state.recording != FALSE)
    {
        nearest_index_cnt = vehicle_path_runtime.state.point_cnt - 1u;
    }
    else
    {
        pose_observation = module_vehicle_pose_fusion_observation_get();
        if (vehicle_path_replay_nearest_point_find(0u,
                                                   vehicle_path_runtime.state.point_cnt - 1u,
                                                   pose_observation->x_mm,
                                                   pose_observation->y_mm,
                                                   &nearest_index_cnt) == FALSE)
        {
            return FALSE;
        }
    }

    return module_vehicle_path_marker_set(kind, nearest_index_cnt);
}

boolean module_vehicle_path_marker_set(vehicle_path_marker_kind_t kind, uint32 index_cnt)
{
    vehicle_path_marker_t* marker;
    vehicle_path_point_t marker_point;

    if ((vehicle_path_marker_kind_valid(kind) == FALSE)
        || (vehicle_path_runtime.state.point_cnt == 0u)
        || (index_cnt >= vehicle_path_runtime.state.point_cnt)
        || (vehicle_path_runtime.marker_cnt >= VEHICLE_PATH_MARKER_MAX_COUNT))
    {
        return FALSE;
    }

    if ((vehicle_path_runtime.state.recording != FALSE)
        && (index_cnt == (vehicle_path_runtime.state.point_cnt - 1u))
        && (index_cnt >= vehicle_path_max_point_cnt_get()))
    {
        marker_point = vehicle_path_runtime.record_last_point;
    }
    else if (vehicle_path_point_read(index_cnt, &marker_point) == FALSE)
    {
        return FALSE;
    }

    marker = &vehicle_path_runtime.markers[vehicle_path_runtime.marker_cnt];
    marker->index_cnt = index_cnt;
    marker->kind = kind;
    marker->point = marker_point;
    marker->valid = TRUE;

    vehicle_path_runtime.marker_cnt++;
    vehicle_path_marker_sort();
    tools_printf("{vmark}add,%s,%u,%.3f,%.3f,%.3f\r\n",
                 vehicle_path_marker_kind_name(kind),
                 (unsigned int)index_cnt,
                 (double)marker_point.x_mm,
                 (double)marker_point.y_mm,
                 (double)(marker_point.theta_rad * VEHICLE_PATH_RAD_TO_DEG));
    return TRUE;
}

void module_vehicle_path_marker_clear(void)
{
    vehicle_path_marker_reset();
    tools_printf("{vmark}clear\r\n");
}

void module_vehicle_path_marker_enable(boolean enable)
{
    vehicle_path_runtime.marker_tracking_mode = (enable != FALSE)
        ? VEHICLE_PATH_MARKER_TRACKING_ENABLED
        : VEHICLE_PATH_MARKER_TRACKING_DISABLED;
    tools_printf("{vmark}enable,%u\r\n",
                 (unsigned int)((vehicle_path_marker_tracking_enabled_get() != FALSE) ? 1u : 0u));
}

boolean module_vehicle_path_marker_enabled_get(void)
{
    return vehicle_path_marker_tracking_enabled_get();
}

void module_vehicle_path_marker_print(void)
{
    uint32 index_cnt;

    tools_printf("{vmark}enable,%u,count,%u\r\n",
                 (unsigned int)((vehicle_path_marker_tracking_enabled_get() != FALSE) ? 1u : 0u),
                 (unsigned int)vehicle_path_runtime.marker_cnt);

    for (index_cnt = 0u; index_cnt < vehicle_path_runtime.marker_cnt; index_cnt++)
    {
        const vehicle_path_marker_t* marker = &vehicle_path_runtime.markers[index_cnt];

        if (marker->valid == FALSE)
        {
            continue;
        }

        tools_printf("{vmark}%u,%s,%u,%.3f,%.3f,%.3f\r\n",
                     (unsigned int)index_cnt,
                     vehicle_path_marker_kind_name(marker->kind),
                     (unsigned int)marker->index_cnt,
                     (double)marker->point.x_mm,
                     (double)marker->point.y_mm,
                     (double)(marker->point.theta_rad * VEHICLE_PATH_RAD_TO_DEG));
    }
}

/**
 * @brief 获取当前车体路径记录与复现状态。
 * @param[in] void 无参数。
 * @return 车体路径记录与复现状态指针。
 */
const vehicle_path_state_t* module_vehicle_path_state_get(void)
{
    return &vehicle_path_runtime.state;
}

static void vehicle_path_marker_reset(void)
{
    uint32 index_cnt;

    vehicle_path_runtime.marker_cnt = 0u;
    for (index_cnt = 0u; index_cnt < VEHICLE_PATH_MARKER_MAX_COUNT; index_cnt++)
    {
        vehicle_path_runtime.markers[index_cnt].index_cnt = 0u;
        vehicle_path_runtime.markers[index_cnt].kind = VEHICLE_PATH_MARKER_KIND_TURN_IN;
        vehicle_path_runtime.markers[index_cnt].point.x_mm = 0.0f;
        vehicle_path_runtime.markers[index_cnt].point.y_mm = 0.0f;
        vehicle_path_runtime.markers[index_cnt].point.theta_rad = 0.0f;
        vehicle_path_runtime.markers[index_cnt].point.speed_mm_s = 0.0f;
        vehicle_path_runtime.markers[index_cnt].valid = FALSE;
    }
}

static boolean vehicle_path_marker_kind_valid(vehicle_path_marker_kind_t kind)
{
    return ((kind == VEHICLE_PATH_MARKER_KIND_TURN_IN)
            || (kind == VEHICLE_PATH_MARKER_KIND_TURN_OUT))
               ? TRUE
               : FALSE;
}

static const char* vehicle_path_marker_kind_name(vehicle_path_marker_kind_t kind)
{
    return (kind == VEHICLE_PATH_MARKER_KIND_TURN_OUT) ? "out" : "in";
}

static void vehicle_path_marker_sort(void)
{
    uint32 i;
    uint32 j;

    if (vehicle_path_runtime.marker_cnt < 2u)
    {
        return;
    }

    for (i = 0u; i < (vehicle_path_runtime.marker_cnt - 1u); i++)
    {
        for (j = i + 1u; j < vehicle_path_runtime.marker_cnt; j++)
        {
            if (vehicle_path_runtime.markers[j].index_cnt < vehicle_path_runtime.markers[i].index_cnt)
            {
                vehicle_path_marker_t temp = vehicle_path_runtime.markers[i];
                vehicle_path_runtime.markers[i] = vehicle_path_runtime.markers[j];
                vehicle_path_runtime.markers[j] = temp;
            }
        }
    }
}

static boolean vehicle_path_marker_refresh_points(void)
{
    uint32 index_cnt;
    uint32 write_cnt = 0u;

    for (index_cnt = 0u; index_cnt < vehicle_path_runtime.marker_cnt; index_cnt++)
    {
        vehicle_path_marker_t marker = vehicle_path_runtime.markers[index_cnt];

        if ((marker.valid == FALSE)
            || (marker.index_cnt >= vehicle_path_runtime.state.point_cnt)
            || (vehicle_path_marker_kind_valid(marker.kind) == FALSE))
        {
            continue;
        }

        if (vehicle_path_point_read_preserve_window(marker.index_cnt, &marker.point) == FALSE)
        {
            continue;
        }

        vehicle_path_runtime.markers[write_cnt] = marker;
        vehicle_path_runtime.markers[write_cnt].valid = TRUE;
        write_cnt++;
    }

    vehicle_path_runtime.marker_cnt = write_cnt;
    vehicle_path_marker_sort();
    return TRUE;
}

static uint32 vehicle_path_marker_checksum_calculate(
    const vehicle_path_flash_marker_record_t* markers,
    uint32 marker_cnt)
{
    if ((markers == NULL_PTR) || (marker_cnt == 0u))
    {
        return VEHICLE_PATH_CHECKSUM_SEED;
    }

    return vehicle_path_checksum_calculate((const uint8*)markers,
                                           marker_cnt * (uint32)sizeof(vehicle_path_flash_marker_record_t));
}

static void vehicle_path_plan_meta_page_append(uint32 slot_offset, uint32 point_cnt, uint8* marker_page)
{
    vehicle_path_flash_plan_meta_t meta;
    const uint8* meta_bytes = (const uint8*)&meta;
    uint32 source_point_cnt = vehicle_path_runtime.job_source_point_cnt;
    float32 source_length_mm = vehicle_path_runtime.job_path_length_mm;
    uint32 index_cnt;
    uint32 meta_offset = (uint32)sizeof(vehicle_path_flash_marker_block_t);

    if ((marker_page == NULL_PTR)
        || (slot_offset != VEHICLE_PATH_FLASH_PLANNED_OFFSET)
        || ((meta_offset + (uint32)sizeof(meta)) > VEHICLE_PATH_FLASH_MARKER_LENGTH))
    {
        return;
    }

    if (source_point_cnt == 0u)
    {
        source_point_cnt = point_cnt;
    }
    if ((vehicle_path_runtime.job_downsampled == FALSE)
        || (source_length_mm <= VEHICLE_PATH_PLAN_RESAMPLE_EPSILON_MM))
    {
        source_length_mm = vehicle_path_points_length_calculate(point_cnt);
    }
    if ((source_length_mm <= VEHICLE_PATH_PLAN_RESAMPLE_EPSILON_MM)
        && (source_point_cnt >= 2u))
    {
        source_length_mm =
            (float32)(source_point_cnt - 1u) * vehicle_path_min_distance_step_get();
    }

    meta.magic = VEHICLE_PATH_PLAN_META_MAGIC;
    meta.version = VEHICLE_PATH_PLAN_META_VERSION;
    meta.planned_point_cnt = (uint16)point_cnt;
    meta.source_point_cnt = (uint16)source_point_cnt;
    meta.reserved = (vehicle_path_runtime.import_active != FALSE)
                        ? VEHICLE_PATH_PLAN_META_STANDALONE_UPLOAD
                        : 0u;
    meta.source_checksum = vehicle_path_runtime.job_header.checksum;
    meta.source_length_mm = source_length_mm;
    meta.planned_step_mm = (point_cnt >= 2u)
                               ? (source_length_mm / (float32)(point_cnt - 1u))
                               : 0.0f;

    for (index_cnt = 0u; index_cnt < (uint32)sizeof(meta); index_cnt++)
    {
        marker_page[meta_offset + index_cnt] = meta_bytes[index_cnt];
    }
}

static void vehicle_path_phototube_zone_page_append(uint8* marker_page)
{
    vehicle_path_flash_phototube_zone_block_t* block;
    uint32 block_offset = (uint32)sizeof(vehicle_path_flash_marker_block_t)
                        + (uint32)sizeof(vehicle_path_flash_plan_meta_t);
    uint32 index_cnt;

    if ((marker_page == NULL_PTR)
        || ((block_offset + (uint32)sizeof(*block)) > VEHICLE_PATH_FLASH_MARKER_LENGTH))
    {
        return;
    }
    block = (vehicle_path_flash_phototube_zone_block_t*)&marker_page[block_offset];
    block->magic = VEHICLE_PATH_PHOTOTUBE_ZONE_MAGIC;
    block->version = VEHICLE_PATH_PHOTOTUBE_ZONE_VERSION;
    block->zone_cnt = (uint16)vehicle_path_runtime.phototube_zone_cnt;
    for (index_cnt = 0u; index_cnt < VEHICLE_PATH_PHOTOTUBE_ZONE_MAX_COUNT; index_cnt++)
    {
        block->zones[index_cnt].start_index_cnt = 0u;
        block->zones[index_cnt].end_index_cnt = 0u;
    }
    for (index_cnt = 0u; index_cnt < vehicle_path_runtime.phototube_zone_cnt; index_cnt++)
    {
        block->zones[index_cnt] = vehicle_path_runtime.phototube_zones[index_cnt];
    }
    block->checksum = vehicle_path_checksum_calculate(
        (const uint8*)block->zones,
        vehicle_path_runtime.phototube_zone_cnt * (uint32)sizeof(vehicle_path_phototube_zone_t));
}

static boolean vehicle_path_plan_meta_read(vehicle_path_flash_plan_meta_t* meta)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();
    uint32 read_offset;

    if (meta == NULL_PTR)
    {
        return FALSE;
    }

    read_offset = cfg->flash_offset
                + VEHICLE_PATH_FLASH_PLANNED_OFFSET
                + VEHICLE_PATH_FLASH_MARKER_OFFSET
                + (uint32)sizeof(vehicle_path_flash_marker_block_t);

    if ((read_offset + (uint32)sizeof(*meta))
        > (cfg->flash_offset + VEHICLE_PATH_FLASH_PLANNED_OFFSET + VEHICLE_PATH_FLASH_SLOT_BYTES))
    {
        return FALSE;
    }

    if (service_storage_readSync(SERVICE_STORAGE_1,
                                 read_offset,
                                 (uint8*)meta,
                                 (uint32)sizeof(*meta)) != (uint32)sizeof(*meta))
    {
        return FALSE;
    }

    if ((meta->magic == VEHICLE_PATH_PLAN_META_MAGIC)
        && (meta->version == VEHICLE_PATH_PLAN_META_VERSION))
    {
        return TRUE;
    }

    read_offset = cfg->flash_offset
                + VEHICLE_PATH_FLASH_PLANNED_OFFSET
                + VEHICLE_PATH_FLASH_MARKER_V2_OFFSET
                + (uint32)sizeof(vehicle_path_flash_marker_block_t);
    if (service_storage_readSync(SERVICE_STORAGE_1,
                                 read_offset,
                                 (uint8*)meta,
                                 (uint32)sizeof(*meta)) != (uint32)sizeof(*meta))
    {
        return FALSE;
    }
    return (boolean)((meta->magic == VEHICLE_PATH_PLAN_META_MAGIC)
                     && (meta->version == VEHICLE_PATH_PLAN_META_VERSION));
}

static void vehicle_path_header_page_prepare(uint8* header_page, const vehicle_path_flash_header_t* header)
{
    uint32 index_cnt;

    if ((header_page == NULL_PTR) || (header == NULL_PTR))
    {
        return;
    }

    for (index_cnt = 0u; index_cnt < VEHICLE_PATH_FLASH_HEADER_LENGTH; index_cnt++)
    {
        header_page[index_cnt] = 0xFFu;
    }

    for (index_cnt = 0u; index_cnt < (uint32)sizeof(*header); index_cnt++)
    {
        header_page[index_cnt] = ((const uint8*)header)[index_cnt];
    }
}

static boolean vehicle_path_marker_write(uint32 slot_offset, uint32 point_cnt)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();
    uint8* marker_page = (uint8*)vehicle_path_marker_page_words;
    vehicle_path_flash_marker_block_t* marker_block =
        (vehicle_path_flash_marker_block_t*)marker_page;
    uint32 index_cnt;
    uint32 marker_cnt;
    uint32 write_length;

    if ((slot_offset != VEHICLE_PATH_FLASH_RAW_OFFSET)
        && (slot_offset != VEHICLE_PATH_FLASH_PLANNED_OFFSET))
    {
        return FALSE;
    }

    for (index_cnt = 0u; index_cnt < VEHICLE_PATH_FLASH_MARKER_LENGTH; index_cnt++)
    {
        marker_page[index_cnt] = 0xFFu;
    }

    marker_block->magic = VEHICLE_PATH_MARKER_MAGIC;
    marker_block->version = VEHICLE_PATH_MARKER_VERSION;
    marker_block->marker_cnt = 0u;
    marker_cnt = vehicle_path_runtime.marker_cnt;
    if (marker_cnt > VEHICLE_PATH_MARKER_MAX_COUNT)
    {
        marker_cnt = VEHICLE_PATH_MARKER_MAX_COUNT;
    }

    for (index_cnt = 0u; index_cnt < marker_cnt; index_cnt++)
    {
        if ((vehicle_path_runtime.markers[index_cnt].valid == FALSE)
            || (vehicle_path_runtime.markers[index_cnt].index_cnt >= point_cnt)
            || (vehicle_path_marker_kind_valid(vehicle_path_runtime.markers[index_cnt].kind) == FALSE))
        {
            continue;
        }

        marker_block->markers[marker_block->marker_cnt].index_cnt =
            vehicle_path_runtime.markers[index_cnt].index_cnt;
        marker_block->markers[marker_block->marker_cnt].kind =
            (uint8)vehicle_path_runtime.markers[index_cnt].kind;
        marker_block->markers[marker_block->marker_cnt].reserved[0] = 0u;
        marker_block->markers[marker_block->marker_cnt].reserved[1] = 0u;
        marker_block->markers[marker_block->marker_cnt].reserved[2] = 0u;
        marker_block->marker_cnt++;
    }

    marker_block->checksum =
        vehicle_path_marker_checksum_calculate(marker_block->markers, marker_block->marker_cnt);
    vehicle_path_plan_meta_page_append(slot_offset, point_cnt, marker_page);
    vehicle_path_phototube_zone_page_append(marker_page);

    write_length = service_storage_writeSync(SERVICE_STORAGE_1,
                                             cfg->flash_offset
                                             + slot_offset
                                             + VEHICLE_PATH_FLASH_MARKER_OFFSET,
                                             marker_page,
                                             VEHICLE_PATH_FLASH_MARKER_LENGTH);

    return (write_length == VEHICLE_PATH_FLASH_MARKER_LENGTH) ? TRUE : FALSE;
}

static boolean vehicle_path_marker_load_from_slot(uint32 slot_offset)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();
    union
    {
        vehicle_path_flash_marker_block_t current;
        vehicle_path_flash_marker_legacy_block_t legacy;
    } marker_storage;
    const vehicle_path_flash_marker_record_t* marker_records = NULL_PTR;
    uint32 index_cnt;
    uint32 marker_cnt = 0u;
    boolean marker_block_valid = FALSE;

    vehicle_path_marker_reset();
    vehicle_path_phototube_zone_reset();

    if ((slot_offset != VEHICLE_PATH_FLASH_RAW_OFFSET)
        && (slot_offset != VEHICLE_PATH_FLASH_PLANNED_OFFSET))
    {
        return FALSE;
    }

    if (service_storage_readSync(SERVICE_STORAGE_1,
                                 cfg->flash_offset
                                 + slot_offset
                                 + VEHICLE_PATH_FLASH_MARKER_OFFSET,
                                 (uint8*)&marker_storage.current,
                                 (uint32)sizeof(marker_storage.current))
        == (uint32)sizeof(marker_storage.current))
    {
        if ((marker_storage.current.magic == VEHICLE_PATH_MARKER_MAGIC)
            && (marker_storage.current.version == VEHICLE_PATH_MARKER_VERSION)
            && (marker_storage.current.marker_cnt <= VEHICLE_PATH_MARKER_MAX_COUNT)
            && (marker_storage.current.checksum
                == vehicle_path_marker_checksum_calculate(marker_storage.current.markers,
                                                          marker_storage.current.marker_cnt)))
        {
            marker_records = marker_storage.current.markers;
            marker_cnt = marker_storage.current.marker_cnt;
            marker_block_valid = TRUE;
        }
    }

    if ((marker_block_valid == FALSE)
        && (service_storage_readSync(SERVICE_STORAGE_1,
                                     cfg->flash_offset
                                     + slot_offset
                                     + VEHICLE_PATH_FLASH_MARKER_V2_OFFSET,
                                     (uint8*)&marker_storage.current,
                                     (uint32)sizeof(marker_storage.current))
            == (uint32)sizeof(marker_storage.current)))
    {
        if ((marker_storage.current.magic == VEHICLE_PATH_MARKER_MAGIC)
            && (marker_storage.current.version == VEHICLE_PATH_MARKER_VERSION)
            && (marker_storage.current.marker_cnt <= VEHICLE_PATH_MARKER_MAX_COUNT)
            && (marker_storage.current.checksum
                == vehicle_path_marker_checksum_calculate(marker_storage.current.markers,
                                                          marker_storage.current.marker_cnt)))
        {
            marker_records = marker_storage.current.markers;
            marker_cnt = marker_storage.current.marker_cnt;
            marker_block_valid = TRUE;
        }
    }

    if ((marker_block_valid == FALSE)
        && (service_storage_readSync(SERVICE_STORAGE_1,
                                     cfg->flash_offset
                                     + slot_offset
                                     + VEHICLE_PATH_FLASH_MARKER_LEGACY_OFFSET,
                                     (uint8*)&marker_storage.legacy,
                                     (uint32)sizeof(marker_storage.legacy))
            == (uint32)sizeof(marker_storage.legacy)))
    {
        if ((marker_storage.legacy.magic == VEHICLE_PATH_MARKER_MAGIC)
            && (marker_storage.legacy.version == VEHICLE_PATH_MARKER_LEGACY_VERSION)
            && (marker_storage.legacy.marker_cnt <= VEHICLE_PATH_MARKER_LEGACY_MAX_COUNT)
            && (marker_storage.legacy.checksum
                == vehicle_path_marker_checksum_calculate(marker_storage.legacy.markers,
                                                          marker_storage.legacy.marker_cnt)))
        {
            marker_records = marker_storage.legacy.markers;
            marker_cnt = marker_storage.legacy.marker_cnt;
            marker_block_valid = TRUE;
        }
    }

    if (marker_block_valid == FALSE)
    {
        return FALSE;
    }

    for (index_cnt = 0u; index_cnt < marker_cnt; index_cnt++)
    {
        vehicle_path_marker_kind_t kind = (vehicle_path_marker_kind_t)marker_records[index_cnt].kind;

        if ((vehicle_path_marker_kind_valid(kind) == FALSE)
            || (marker_records[index_cnt].index_cnt >= vehicle_path_runtime.state.point_cnt))
        {
            continue;
        }

        vehicle_path_runtime.markers[vehicle_path_runtime.marker_cnt].index_cnt =
            marker_records[index_cnt].index_cnt;
        vehicle_path_runtime.markers[vehicle_path_runtime.marker_cnt].kind = kind;
        vehicle_path_runtime.markers[vehicle_path_runtime.marker_cnt].valid = TRUE;
        vehicle_path_runtime.marker_cnt++;
    }

    vehicle_path_marker_sort();
    (void)vehicle_path_marker_refresh_points();
    (void)vehicle_path_phototube_zone_load_from_slot(slot_offset);
    return TRUE;
}

static void vehicle_path_phototube_zone_reset(void)
{
    uint32 index_cnt;
    vehicle_path_runtime.phototube_zone_cnt = 0u;
    for (index_cnt = 0u; index_cnt < VEHICLE_PATH_PHOTOTUBE_ZONE_MAX_COUNT; index_cnt++)
    {
        vehicle_path_runtime.phototube_zones[index_cnt].start_index_cnt = 0u;
        vehicle_path_runtime.phototube_zones[index_cnt].end_index_cnt = 0u;
    }
}

static boolean vehicle_path_phototube_zone_load_from_slot(uint32 slot_offset)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();
    vehicle_path_flash_phototube_zone_block_t* block =
        (vehicle_path_flash_phototube_zone_block_t*)vehicle_path_marker_page_words;
    uint32 read_offset = cfg->flash_offset + slot_offset + VEHICLE_PATH_FLASH_MARKER_OFFSET
                       + (uint32)sizeof(vehicle_path_flash_marker_block_t)
                       + (uint32)sizeof(vehicle_path_flash_plan_meta_t);
    uint32 checksum;
    uint32 index_cnt;

    vehicle_path_phototube_zone_reset();
    if (service_storage_readSync(SERVICE_STORAGE_1, read_offset, (uint8*)block,
                                 (uint32)sizeof(*block)) != (uint32)sizeof(*block))
    {
        return FALSE;
    }
    if ((block->magic != VEHICLE_PATH_PHOTOTUBE_ZONE_MAGIC)
        || (block->version != VEHICLE_PATH_PHOTOTUBE_ZONE_VERSION)
        || (block->zone_cnt > VEHICLE_PATH_PHOTOTUBE_ZONE_MAX_COUNT))
    {
        return FALSE;
    }
    checksum = vehicle_path_checksum_calculate(
        (const uint8*)block->zones,
        (uint32)block->zone_cnt * (uint32)sizeof(vehicle_path_phototube_zone_t));
    if (block->checksum != checksum)
    {
        return FALSE;
    }
    for (index_cnt = 0u; index_cnt < (uint32)block->zone_cnt; index_cnt++)
    {
        if ((block->zones[index_cnt].start_index_cnt > block->zones[index_cnt].end_index_cnt)
            || (block->zones[index_cnt].end_index_cnt >= vehicle_path_runtime.state.point_cnt))
        {
            vehicle_path_phototube_zone_reset();
            return FALSE;
        }
        vehicle_path_runtime.phototube_zones[index_cnt] = block->zones[index_cnt];
    }
    vehicle_path_runtime.phototube_zone_cnt = (uint32)block->zone_cnt;
    return TRUE;
}

static boolean vehicle_path_marker_tracking_enabled_get(void)
{
    if (vehicle_path_runtime.marker_tracking_mode == VEHICLE_PATH_MARKER_TRACKING_ENABLED)
    {
        return TRUE;
    }
    if (vehicle_path_runtime.marker_tracking_mode == VEHICLE_PATH_MARKER_TRACKING_DISABLED)
    {
        return FALSE;
    }
    return (vehicle_path_runtime.marker_cnt >= 2u) ? TRUE : FALSE;
}

static boolean vehicle_path_replay_edge_points_refresh(void)
{
    if (vehicle_path_runtime.state.point_cnt == 0u)
    {
        vehicle_path_runtime.replay_edge_points_valid = FALSE;
        return FALSE;
    }

    if ((vehicle_path_point_read_preserve_window(0u, &vehicle_path_runtime.replay_first_point) == FALSE)
        || (vehicle_path_point_read_preserve_window(vehicle_path_runtime.state.point_cnt - 1u,
                                                    &vehicle_path_runtime.replay_last_point) == FALSE))
    {
        vehicle_path_runtime.replay_edge_points_valid = FALSE;
        return FALSE;
    }

    vehicle_path_runtime.replay_edge_points_valid = TRUE;
    return TRUE;
}

static boolean vehicle_path_marker_segment_target_get(uint32 start_index_cnt,
                                                      const vehicle_path_point_t* start_point,
                                                      uint32 end_index_cnt,
                                                      const vehicle_path_point_t* end_point,
                                                      float32 lookahead_mm,
                                                      float32 tangent_distance_mm,
                                                      float32 target_speed_mm_s,
                                                      vehicle_path_marker_target_t* marker_target)
{
    const module_vehicle_pose_fusion_observation_t* pose_observation;
    float32 segment_dx_mm;
    float32 segment_dy_mm;
    float32 segment_length_sq_mm;
    float32 segment_length_mm;
    float32 heading_rad;
    float32 pose_dx_mm;
    float32 pose_dy_mm;
    float32 raw_along_mm;
    float32 base_distance_mm;
    float32 target_distance_mm;
    float32 tangent_distance_along_mm;
    float32 base_ratio;
    float32 target_ratio;
    float32 tangent_ratio;
    float32 index_span_float;

    if ((start_point == NULL_PTR) || (end_point == NULL_PTR) || (marker_target == NULL_PTR)
        || (end_index_cnt <= start_index_cnt))
    {
        return FALSE;
    }

    segment_dx_mm = end_point->x_mm - start_point->x_mm;
    segment_dy_mm = end_point->y_mm - start_point->y_mm;
    segment_length_sq_mm = (segment_dx_mm * segment_dx_mm) + (segment_dy_mm * segment_dy_mm);
    if (segment_length_sq_mm < (VEHICLE_PATH_MARKER_MIN_SEGMENT_MM * VEHICLE_PATH_MARKER_MIN_SEGMENT_MM))
    {
        return FALSE;
    }

    segment_length_mm = sqrtf(segment_length_sq_mm);
    heading_rad = vehicle_path_wrap_pi(atan2f(segment_dy_mm, segment_dx_mm));
    pose_observation = module_vehicle_pose_fusion_observation_get();
    pose_dx_mm = pose_observation->x_mm - start_point->x_mm;
    pose_dy_mm = pose_observation->y_mm - start_point->y_mm;
    raw_along_mm = ((pose_dx_mm * segment_dx_mm) + (pose_dy_mm * segment_dy_mm)) / segment_length_mm;
    base_distance_mm = raw_along_mm;
    if (base_distance_mm < 0.0f)
    {
        base_distance_mm = 0.0f;
    }
    if (base_distance_mm > segment_length_mm)
    {
        base_distance_mm = segment_length_mm;
    }

    target_distance_mm = base_distance_mm + lookahead_mm;
    if (target_distance_mm > segment_length_mm)
    {
        target_distance_mm = segment_length_mm;
    }
    tangent_distance_along_mm = target_distance_mm + tangent_distance_mm;
    if (tangent_distance_along_mm > segment_length_mm)
    {
        tangent_distance_along_mm = segment_length_mm;
    }

    base_ratio = base_distance_mm / segment_length_mm;
    target_ratio = target_distance_mm / segment_length_mm;
    tangent_ratio = tangent_distance_along_mm / segment_length_mm;
    index_span_float = (float32)(end_index_cnt - start_index_cnt);

    marker_target->base_point.x_mm = start_point->x_mm + (segment_dx_mm * base_ratio);
    marker_target->base_point.y_mm = start_point->y_mm + (segment_dy_mm * base_ratio);
    marker_target->base_point.theta_rad = heading_rad;
    marker_target->base_point.speed_mm_s =
        start_point->speed_mm_s + ((end_point->speed_mm_s - start_point->speed_mm_s) * base_ratio);
    marker_target->target_point.x_mm = start_point->x_mm + (segment_dx_mm * target_ratio);
    marker_target->target_point.y_mm = start_point->y_mm + (segment_dy_mm * target_ratio);
    marker_target->target_point.theta_rad = heading_rad;
    marker_target->target_point.speed_mm_s =
        start_point->speed_mm_s + ((end_point->speed_mm_s - start_point->speed_mm_s) * target_ratio);
    marker_target->tangent_point.x_mm = start_point->x_mm + (segment_dx_mm * tangent_ratio);
    marker_target->tangent_point.y_mm = start_point->y_mm + (segment_dy_mm * tangent_ratio);
    marker_target->tangent_point.theta_rad = heading_rad;
    marker_target->tangent_point.speed_mm_s =
        start_point->speed_mm_s + ((end_point->speed_mm_s - start_point->speed_mm_s) * tangent_ratio);
    marker_target->base_index_float = (float32)start_index_cnt + (index_span_float * base_ratio);
    marker_target->target_index_float = (float32)start_index_cnt + (index_span_float * target_ratio);
    marker_target->tangent_index_float = (float32)start_index_cnt + (index_span_float * tangent_ratio);
    marker_target->cross_track_error_mm =
        (-sinf(heading_rad) * pose_dx_mm) + (cosf(heading_rad) * pose_dy_mm);
    marker_target->along_track_error_mm = raw_along_mm - base_distance_mm;
    marker_target->target_theta_rad =
        vehicle_path_replay_target_heading_get(heading_rad,
                                               marker_target->cross_track_error_mm,
                                               lookahead_mm,
                                               target_speed_mm_s,
                                               0.0f);
    marker_target->feedforward_theta_rad = heading_rad;
    marker_target->segment_cnt = start_index_cnt;
    marker_target->next_cnt = end_index_cnt;
    marker_target->blend_ratio = 1.0f;
    marker_target->mode = VEHICLE_PATH_REPLAY_TRACK_MARKER;
    return TRUE;
}

static boolean vehicle_path_marker_tracking_target_get(float32 dense_base_index_float,
                                                       float32 dense_target_index_float,
                                                       float32 lookahead_mm,
                                                       float32 tangent_distance_mm,
                                                       float32 target_speed_mm_s,
                                                       vehicle_path_marker_target_t* marker_target)
{
    uint32 marker_pos_cnt;
    uint32 straight_start_index_cnt = 0u;
    uint32 open_turn_in_index_cnt = 0u;
    uint32 straight_exit_index_cnt = 0u;
    float32 step_mm;
    float32 distance_to_boundary_mm;
    boolean open_turn_in_valid = FALSE;
    boolean straight_exit_blend_valid = FALSE;
    vehicle_path_point_t straight_start_point;
    vehicle_path_point_t open_turn_in_point;

    if ((marker_target == NULL_PTR)
        || (vehicle_path_marker_tracking_enabled_get() == FALSE)
        || (vehicle_path_runtime.marker_cnt == 0u)
        || (vehicle_path_runtime.state.point_cnt < 2u))
    {
        return FALSE;
    }

    if (vehicle_path_runtime.replay_edge_points_valid == FALSE)
    {
        if (vehicle_path_replay_edge_points_refresh() == FALSE)
        {
            return FALSE;
        }
    }

    marker_target->base_point.x_mm = 0.0f;
    marker_target->base_point.y_mm = 0.0f;
    marker_target->base_point.theta_rad = 0.0f;
    marker_target->base_point.speed_mm_s = 0.0f;
    marker_target->target_point = marker_target->base_point;
    marker_target->tangent_point = marker_target->base_point;
    marker_target->base_index_float = dense_base_index_float;
    marker_target->target_index_float = dense_base_index_float;
    marker_target->tangent_index_float = dense_base_index_float;
    marker_target->cross_track_error_mm = 0.0f;
    marker_target->along_track_error_mm = 0.0f;
    marker_target->target_theta_rad = 0.0f;
    marker_target->feedforward_theta_rad = 0.0f;
    marker_target->segment_cnt = 0u;
    marker_target->next_cnt = 0u;
    marker_target->blend_ratio = 0.0f;
    marker_target->mode = VEHICLE_PATH_REPLAY_TRACK_DENSE;

    step_mm = vehicle_path_min_distance_step_get();
    if (step_mm < 1.0f)
    {
        step_mm = 1.0f;
    }

    straight_start_point = vehicle_path_runtime.replay_first_point;
    for (marker_pos_cnt = 0u; marker_pos_cnt < vehicle_path_runtime.marker_cnt; marker_pos_cnt++)
    {
        const vehicle_path_marker_t* marker = &vehicle_path_runtime.markers[marker_pos_cnt];

        if (marker->valid == FALSE)
        {
            continue;
        }

        if (marker->kind == VEHICLE_PATH_MARKER_KIND_TURN_IN)
        {
            if (open_turn_in_valid == FALSE)
            {
                if (dense_target_index_float < (float32)marker->index_cnt)
                {
                    if (vehicle_path_marker_segment_target_get(straight_start_index_cnt,
                                                               &straight_start_point,
                                                               marker->index_cnt,
                                                               &marker->point,
                                                               lookahead_mm,
                                                               tangent_distance_mm,
                                                               target_speed_mm_s,
                                                               marker_target) == FALSE)
                    {
                        return FALSE;
                    }

                    if (straight_exit_blend_valid != FALSE)
                    {
                        distance_to_boundary_mm =
                            (dense_base_index_float - (float32)straight_exit_index_cnt) * step_mm;
                        if (distance_to_boundary_mm < 0.0f)
                        {
                            distance_to_boundary_mm = 0.0f;
                        }
                        if (distance_to_boundary_mm < VEHICLE_PATH_MARKER_BLEND_DISTANCE_MM)
                        {
                            marker_target->mode = VEHICLE_PATH_REPLAY_TRACK_BLEND_TO_MARKER;
                            marker_target->blend_ratio = vehicle_path_smoothstep(
                                distance_to_boundary_mm / VEHICLE_PATH_MARKER_BLEND_DISTANCE_MM);
                        }
                    }
                    return TRUE;
                }

                open_turn_in_index_cnt = marker->index_cnt;
                open_turn_in_point = marker->point;
                open_turn_in_valid = TRUE;
            }
        }
        else if (marker->kind == VEHICLE_PATH_MARKER_KIND_TURN_OUT)
        {
            if (open_turn_in_valid != FALSE)
            {
                if (dense_base_index_float <= (float32)marker->index_cnt)
                {
                    marker_target->mode = VEHICLE_PATH_REPLAY_TRACK_DENSE;
                    marker_target->segment_cnt = open_turn_in_index_cnt;
                    marker_target->next_cnt = marker->index_cnt;
                    distance_to_boundary_mm =
                        (dense_target_index_float - (float32)open_turn_in_index_cnt) * step_mm;
                    if ((distance_to_boundary_mm >= 0.0f)
                        && (distance_to_boundary_mm < VEHICLE_PATH_MARKER_BLEND_DISTANCE_MM)
                        && (vehicle_path_marker_segment_target_get(straight_start_index_cnt,
                                                                   &straight_start_point,
                                                                   open_turn_in_index_cnt,
                                                                   &open_turn_in_point,
                                                                   lookahead_mm,
                                                                   tangent_distance_mm,
                                                                   target_speed_mm_s,
                                                                   marker_target) != FALSE))
                    {
                        marker_target->mode = VEHICLE_PATH_REPLAY_TRACK_BLEND_TO_DENSE;
                        marker_target->blend_ratio = 1.0f - vehicle_path_smoothstep(
                            distance_to_boundary_mm / VEHICLE_PATH_MARKER_BLEND_DISTANCE_MM);
                    }
                    return TRUE;
                }

                straight_start_index_cnt = marker->index_cnt;
                straight_start_point = marker->point;
                straight_exit_index_cnt = marker->index_cnt;
                straight_exit_blend_valid = TRUE;
                open_turn_in_valid = FALSE;
            }
            else if (dense_base_index_float >= (float32)marker->index_cnt)
            {
                straight_start_index_cnt = marker->index_cnt;
                straight_start_point = marker->point;
            }
        }
    }

    if (open_turn_in_valid != FALSE)
    {
        marker_target->mode = VEHICLE_PATH_REPLAY_TRACK_DENSE;
        marker_target->segment_cnt = open_turn_in_index_cnt;
        marker_target->next_cnt = vehicle_path_runtime.state.point_cnt - 1u;
        return TRUE;
    }

    if (vehicle_path_marker_segment_target_get(straight_start_index_cnt,
                                               &straight_start_point,
                                               vehicle_path_runtime.state.point_cnt - 1u,
                                               &vehicle_path_runtime.replay_last_point,
                                               lookahead_mm,
                                               tangent_distance_mm,
                                               target_speed_mm_s,
                                               marker_target) == FALSE)
    {
        return FALSE;
    }

    if (straight_exit_blend_valid != FALSE)
    {
        distance_to_boundary_mm =
            (dense_base_index_float - (float32)straight_exit_index_cnt) * step_mm;
        if (distance_to_boundary_mm < 0.0f)
        {
            distance_to_boundary_mm = 0.0f;
        }
        if (distance_to_boundary_mm < VEHICLE_PATH_MARKER_BLEND_DISTANCE_MM)
        {
            marker_target->mode = VEHICLE_PATH_REPLAY_TRACK_BLEND_TO_MARKER;
            marker_target->blend_ratio = vehicle_path_smoothstep(
                distance_to_boundary_mm / VEHICLE_PATH_MARKER_BLEND_DISTANCE_MM);
        }
    }

    return TRUE;
}

static float32 vehicle_path_smoothstep(float32 ratio)
{
    if (ratio < 0.0f)
    {
        ratio = 0.0f;
    }
    if (ratio > 1.0f)
    {
        ratio = 1.0f;
    }

    return ratio * ratio * (3.0f - (2.0f * ratio));
}

static float32 vehicle_path_angle_blend(float32 start_rad, float32 end_rad, float32 ratio)
{
    return vehicle_path_wrap_pi(start_rad + (vehicle_path_wrap_pi(end_rad - start_rad) * ratio));
}

static void vehicle_path_point_blend(const vehicle_path_point_t* start_point,
                                     const vehicle_path_point_t* end_point,
                                     float32 ratio,
                                     vehicle_path_point_t* output_point)
{
    if ((start_point == NULL_PTR) || (end_point == NULL_PTR) || (output_point == NULL_PTR))
    {
        return;
    }

    if (ratio < 0.0f)
    {
        ratio = 0.0f;
    }
    if (ratio > 1.0f)
    {
        ratio = 1.0f;
    }

    output_point->x_mm = start_point->x_mm + ((end_point->x_mm - start_point->x_mm) * ratio);
    output_point->y_mm = start_point->y_mm + ((end_point->y_mm - start_point->y_mm) * ratio);
    output_point->theta_rad = vehicle_path_angle_blend(start_point->theta_rad, end_point->theta_rad, ratio);
    output_point->speed_mm_s = start_point->speed_mm_s + ((end_point->speed_mm_s - start_point->speed_mm_s) * ratio);
}

/**
 * @brief 获取有效最大路径点数量。
 * @param[in] void 无参数。
 * @return 最大路径点数量，单位：点。
 */
static uint32 vehicle_path_max_point_cnt_get(void)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();

    if (cfg->max_point_cnt > VEHICLE_PATH_DEFAULT_MAX_POINT_CNT)
    {
        return VEHICLE_PATH_DEFAULT_MAX_POINT_CNT;
    }

    return cfg->max_point_cnt;
}

static uint32 vehicle_path_flash_slot_point_capacity_get(void)
{
    if (VEHICLE_PATH_FLASH_MARKER_OFFSET <= VEHICLE_PATH_FLASH_POINT_OFFSET)
    {
        return 0u;
    }

    return (VEHICLE_PATH_FLASH_MARKER_OFFSET - VEHICLE_PATH_FLASH_POINT_OFFSET)
           / (uint32)sizeof(vehicle_path_point_t);
}

static uint32 vehicle_path_effective_flash_point_cnt_max_get(void)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();
    uint32 slot_capacity_cnt = vehicle_path_flash_slot_point_capacity_get();
    uint32 cfg_capacity_cnt = cfg->flash_point_cnt_max;

    return (cfg_capacity_cnt < slot_capacity_cnt) ? cfg_capacity_cnt : slot_capacity_cnt;
}

static boolean vehicle_path_flash_slot_range_valid(uint32 slot_offset, uint32 point_cnt)
{
    uint32 data_length = point_cnt * (uint32)sizeof(vehicle_path_point_t);

    if ((slot_offset != VEHICLE_PATH_FLASH_RAW_OFFSET)
        && (slot_offset != VEHICLE_PATH_FLASH_PLANNED_OFFSET))
    {
        return FALSE;
    }

    if (point_cnt > vehicle_path_effective_flash_point_cnt_max_get())
    {
        return FALSE;
    }

    if ((VEHICLE_PATH_FLASH_POINT_OFFSET + data_length) > VEHICLE_PATH_FLASH_MARKER_OFFSET)
    {
        return FALSE;
    }

    return TRUE;
}

static boolean vehicle_path_flash_slot_read_range_valid(uint32 slot_offset, uint32 point_cnt)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();
    uint32 data_length = point_cnt * (uint32)sizeof(vehicle_path_point_t);

    if ((slot_offset != VEHICLE_PATH_FLASH_RAW_OFFSET)
        && (slot_offset != VEHICLE_PATH_FLASH_PLANNED_OFFSET))
    {
        return FALSE;
    }

    if ((point_cnt > cfg->flash_point_cnt_max)
        || ((VEHICLE_PATH_FLASH_POINT_OFFSET + data_length)
            > VEHICLE_PATH_FLASH_MARKER_LEGACY_OFFSET))
    {
        return FALSE;
    }

    return TRUE;
}

/**
 * @brief 获取有效最小路径记录距离间隔。
 * @param[in] void 无参数。
 * @return 最小路径记录距离间隔，单位：毫米。
 */
static float32 vehicle_path_min_distance_step_get(void)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();

    if (cfg->min_distance_step_mm < 0.0f)
    {
        return 0.0f;
    }

    return cfg->min_distance_step_mm;
}

/**
 * @brief 获取有效路径复现前瞻点数量。
 * @param[in] void 无参数。
 * @return 路径复现前瞻点数量，单位：点。
 */
static uint32 vehicle_path_replay_lookahead_cnt_get(void)
{
    return vehicle_path_replay_lookahead_cnt_runtime;
}

/**
 * @brief 获取有效路径复现切线计算间隔。
 * @param[in] void 无参数。
 * @return 路径复现切线计算间隔，单位：点。
 */
static uint32 vehicle_path_replay_tangent_gap_cnt_get(void)
{
    return vehicle_path_replay_tangent_gap_cnt_runtime;
}

static uint32 vehicle_path_replay_distance_to_cnt(float32 distance_mm)
{
    float32 step_mm = vehicle_path_min_distance_step_get();
    float32 count_float;

    if (distance_mm < 0.0f)
    {
        distance_mm = 0.0f;
    }

    if (step_mm < 0.001f)
    {
        step_mm = 1.0f;
    }

    count_float = (distance_mm / step_mm) + 0.5f;
    if (count_float < 1.0f)
    {
        return 1u;
    }

    return (uint32)count_float;
}

static void vehicle_path_replay_target_reset(void)
{
    vehicle_path_runtime.replay_target.base_point.x_mm = 0.0f;
    vehicle_path_runtime.replay_target.base_point.y_mm = 0.0f;
    vehicle_path_runtime.replay_target.base_point.theta_rad = 0.0f;
    vehicle_path_runtime.replay_target.base_point.speed_mm_s = 0.0f;
    vehicle_path_runtime.replay_target.target_point.x_mm = 0.0f;
    vehicle_path_runtime.replay_target.target_point.y_mm = 0.0f;
    vehicle_path_runtime.replay_target.target_point.theta_rad = 0.0f;
    vehicle_path_runtime.replay_target.target_point.speed_mm_s = 0.0f;
    vehicle_path_runtime.replay_target.tangent_point.x_mm = 0.0f;
    vehicle_path_runtime.replay_target.tangent_point.y_mm = 0.0f;
    vehicle_path_runtime.replay_target.tangent_point.theta_rad = 0.0f;
    vehicle_path_runtime.replay_target.tangent_point.speed_mm_s = 0.0f;
    vehicle_path_runtime.replay_target.base_index_float = 0.0f;
    vehicle_path_runtime.replay_target.target_index_float = 0.0f;
    vehicle_path_runtime.replay_target.tangent_index_float = 0.0f;
    vehicle_path_runtime.replay_target.lookahead_distance_mm = 0.0f;
    vehicle_path_runtime.replay_target.tangent_distance_mm = 0.0f;
    vehicle_path_runtime.replay_target.cross_track_error_mm = 0.0f;
    vehicle_path_runtime.replay_target.along_track_error_mm = 0.0f;
    vehicle_path_runtime.replay_target.feedforward_theta_rad = 0.0f;
    vehicle_path_runtime.replay_target.target_theta_rad = 0.0f;
    vehicle_path_runtime.replay_target.target_yaw_rate_rad_s = 0.0f;
    vehicle_path_runtime.replay_target.yaw_ff_mm_s = 0.0f;
    vehicle_path_runtime.replay_target.target_speed_mm_s = 0.0f;
    vehicle_path_runtime.replay_target.raw_plan_speed_mm_s = 0.0f;
    vehicle_path_runtime.replay_target.speed_preview_limit_mm_s = 0.0f;
    vehicle_path_runtime.replay_target.target_index_cnt = 0u;
    vehicle_path_runtime.replay_target.marker_segment_cnt = 0u;
    vehicle_path_runtime.replay_target.marker_next_cnt = 0u;
    vehicle_path_runtime.replay_target.marker_blend_ratio = 0.0f;
    vehicle_path_runtime.replay_target.marker_cross_track_error_mm = 0.0f;
    vehicle_path_runtime.replay_target.dense_cross_track_error_mm = 0.0f;
    vehicle_path_runtime.replay_target.marker_target_theta_rad = 0.0f;
    vehicle_path_runtime.replay_target.dense_target_theta_rad = 0.0f;
    vehicle_path_runtime.replay_target.progress_predicted_index_float = 0.0f;
    vehicle_path_runtime.replay_target.progress_projected_index_float = 0.0f;
    vehicle_path_runtime.replay_target.progress_correction_index_float = 0.0f;
    vehicle_path_runtime.replay_target.track_mode = VEHICLE_PATH_REPLAY_TRACK_DENSE;
    vehicle_path_runtime.replay_target.curve_section_active = FALSE;
    vehicle_path_runtime.replay_target.valid = FALSE;
    vehicle_path_runtime.replay_last_target_theta_rad = 0.0f;
    vehicle_path_runtime.replay_target_theta_initialized = FALSE;
    vehicle_path_runtime.replay_progress_index_float = 0.0f;
    vehicle_path_runtime.replay_progress_traveled_mm = 0.0f;
    vehicle_path_runtime.replay_progress_predicted_index_float = 0.0f;
    vehicle_path_runtime.replay_progress_projected_index_float = 0.0f;
    vehicle_path_runtime.replay_progress_correction_index_float = 0.0f;
    vehicle_path_runtime.replay_progress_valid = FALSE;
    vehicle_path_source_distance_cache_reset();
}

/**
 * @brief 重置路径记录与复现运行状态。
 * @param[in] status 目标路径模块状态。
 * @return void
 */
static void vehicle_path_state_reset(vehicle_path_status_t status)
{
    uint32 i;

    module_vehicle_pose_fusion_replay_correction_stop();
    for (i = 0u; i < VEHICLE_PATH_DEFAULT_MAX_POINT_CNT; i++)
    {
        vehicle_path_runtime.points[i].x_mm = 0.0f;
        vehicle_path_runtime.points[i].y_mm = 0.0f;
    }

    vehicle_path_runtime.state.status = status;
    vehicle_path_runtime.state.recording = FALSE;
    vehicle_path_runtime.state.replaying = FALSE;
    vehicle_path_runtime.state.replay_target_valid = FALSE;
    vehicle_path_runtime.state.point_cnt = 0u;
    vehicle_path_runtime.state.dropped_cnt = 0u;
    vehicle_path_runtime.state.replay_cursor_cnt = 0u;
    vehicle_path_runtime.state.replay_target_index_cnt = 0u;
    vehicle_path_runtime.state.last_distance_mm = 0.0f;
    vehicle_path_runtime.state.replay_start_distance_mm = 0.0f;
    vehicle_path_runtime.state.replay_target_theta_rad = 0.0f;
    vehicle_path_runtime.window_start_index_cnt = 0u;
    vehicle_path_runtime.window_valid_cnt = 0u;
    vehicle_path_runtime.window_active_buffer_cnt = 0u;
    vehicle_path_runtime.window_from_flash = FALSE;
    vehicle_path_window_buffers_reset();
    vehicle_path_window_prefetch_reset();
    vehicle_path_runtime.active_slot_offset = VEHICLE_PATH_FLASH_RAW_OFFSET;
    vehicle_path_planned_speed_overlay_reset();
    vehicle_path_record_stream_reset();
    vehicle_path_replay_target_reset();
    vehicle_path_runtime.replay_target.base_point.x_mm = 0.0f;
    vehicle_path_runtime.replay_target.base_point.y_mm = 0.0f;
    vehicle_path_runtime.replay_target.base_point.theta_rad = 0.0f;
    vehicle_path_runtime.replay_target.base_point.speed_mm_s = 0.0f;
    vehicle_path_runtime.replay_target.target_point.x_mm = 0.0f;
    vehicle_path_runtime.replay_target.target_point.y_mm = 0.0f;
    vehicle_path_runtime.replay_target.target_point.theta_rad = 0.0f;
    vehicle_path_runtime.replay_target.target_point.speed_mm_s = 0.0f;
    vehicle_path_runtime.replay_target.base_index_float = 0.0f;
    vehicle_path_runtime.replay_target.target_index_float = 0.0f;
    vehicle_path_runtime.replay_target.lookahead_distance_mm = 0.0f;
    vehicle_path_runtime.replay_target.cross_track_error_mm = 0.0f;
    vehicle_path_runtime.replay_target.along_track_error_mm = 0.0f;
    vehicle_path_runtime.replay_target.feedforward_theta_rad = 0.0f;
    vehicle_path_runtime.replay_target.target_theta_rad = 0.0f;
    vehicle_path_runtime.replay_target.target_yaw_rate_rad_s = 0.0f;
    vehicle_path_runtime.replay_target.yaw_ff_mm_s = 0.0f;
    vehicle_path_runtime.replay_target.target_speed_mm_s = 0.0f;
    vehicle_path_runtime.replay_target.target_index_cnt = 0u;
    vehicle_path_runtime.replay_target.curve_section_active = FALSE;
    vehicle_path_runtime.replay_target.valid = FALSE;
}

static void vehicle_path_runtime_origin_reset(float32 x_mm, float32 y_mm, float32 theta_rad)
{
    module_vehicle_encoder_reset_baseline();
    module_vehicle_gyro_reset_heading(0.0f);
    module_vehicle_pose_fusion_reset(x_mm, y_mm, theta_rad);
}

/**
 * @brief 将当前编码器位姿记录到路径缓存。
 * @param[in] void 无参数。
 * @return 成功记录路径点返回 TRUE，否则返回 FALSE。
 */
static boolean vehicle_path_record_current_pose(void)
{
    const module_vehicle_encoder_observation_t* encoder_observation = module_vehicle_encoder_observation_get();
    const module_vehicle_pose_fusion_observation_t* pose_observation = module_vehicle_pose_fusion_observation_get();
    vehicle_path_point_t point_value;

    if ((vehicle_path_runtime.state.point_cnt > 0u)
        && (pose_observation->update_count == vehicle_path_runtime.record_last_pose_update_count))
    {
        return FALSE;
    }

    vehicle_path_record_pose_point_make(pose_observation, &point_value);

    return vehicle_path_record_point_append(&point_value,
                                            encoder_observation->distance_mm,
                                            pose_observation->update_count);
}

static void vehicle_path_record_pose_point_make(const module_vehicle_pose_fusion_observation_t* pose_observation,
                                                vehicle_path_point_t* point)
{
    if ((pose_observation == NULL_PTR) || (point == NULL_PTR))
    {
        return;
    }

    point->x_mm = pose_observation->x_mm * VEHICLE_PATH_RECORD_COORD_SCALE_X;
    point->y_mm = pose_observation->y_mm * VEHICLE_PATH_RECORD_COORD_SCALE_Y;
    point->theta_rad = pose_observation->theta_accum_rad;
    point->speed_mm_s = VEHICLE_PATH_RECORD_DEFAULT_SPEED_MM_S;
}

static boolean vehicle_path_record_point_append(const vehicle_path_point_t* point_value,
                                                float32 distance_mm,
                                                uint32 pose_update_count)
{
    vehicle_path_point_t point_copy;
    vehicle_path_point_t* point;
    uint32 point_index_cnt;

    if (point_value == NULL_PTR)
    {
        return FALSE;
    }

    if (vehicle_path_runtime.state.point_cnt >= vehicle_path_effective_flash_point_cnt_max_get())
    {
        vehicle_path_runtime.state.status = VEHICLE_PATH_STATUS_FULL;
        vehicle_path_runtime.state.recording = FALSE;
        vehicle_path_runtime.state.dropped_cnt++;
        return FALSE;
    }

    point_index_cnt = vehicle_path_runtime.state.point_cnt;
    point_copy = *point_value;

    if (vehicle_path_runtime.state.point_cnt < vehicle_path_max_point_cnt_get())
    {
        point = &vehicle_path_runtime.points[vehicle_path_runtime.state.point_cnt];
    }
    else
    {
        point = &point_copy;
    }

    *point = point_copy;

    if (vehicle_path_record_stream_append(point) == FALSE)
    {
        vehicle_path_runtime.state.status = VEHICLE_PATH_STATUS_FULL;
        vehicle_path_runtime.state.recording = FALSE;
        vehicle_path_runtime.state.dropped_cnt++;
        return FALSE;
    }

    if (vehicle_path_runtime.state.point_cnt > 0u)
    {
        vehicle_path_runtime.record_path_length_mm +=
            vehicle_path_segment_length_mm(&vehicle_path_runtime.record_last_point, point);
    }

    vehicle_path_runtime.record_last_point = *point;
    vehicle_path_runtime.state.last_distance_mm = distance_mm;
    vehicle_path_runtime.state.point_cnt++;
    vehicle_path_runtime.record_last_pose_update_count = pose_update_count;
    if (vehicle_path_record_print_queue_push(point,
                                             vehicle_path_runtime.record_path_length_mm,
                                             point_index_cnt,
                                             pose_update_count) == FALSE)
    {
        vehicle_path_runtime.record_print_dropped_cnt++;
    }

    return TRUE;
}

/**
 * @brief 计算路径点数据校验值。
 * @param[in] data 路径点原始字节数据指针。
 * @param[in] length 数据长度，单位：字节。
 * @return 路径点数据校验值。
 */
static uint32 vehicle_path_checksum_calculate(const uint8* data, uint32 length)
{
    return vehicle_path_checksum_update(VEHICLE_PATH_CHECKSUM_SEED, data, length);
}

static uint32 vehicle_path_checksum_update(uint32 checksum, const uint8* data, uint32 length)
{
    uint32 index;

    if ((data == NULL_PTR) && (length > 0u))
    {
        return 0u;
    }

    for (index = 0u; index < length; index++)
    {
        checksum ^= (uint32)data[index];
        checksum = (checksum << 5u) | (checksum >> 27u);
    }

    return checksum;
}

static void vehicle_path_record_stream_reset(void)
{
    uint32 i;

    for (i = 0u; i < IFXFLASH_PFLASH_PAGE_LENGTH; i++)
    {
        vehicle_path_runtime.record_page[i] = 0xFFu;
    }

    vehicle_path_runtime.record_page_start_offset = 0u;
    vehicle_path_runtime.record_page_used_length = 0u;
    vehicle_path_runtime.record_checksum = VEHICLE_PATH_CHECKSUM_SEED;
    vehicle_path_runtime.record_stream_slot_offset = VEHICLE_PATH_FLASH_RAW_OFFSET;
    vehicle_path_runtime.import_expected_point_cnt = 0u;
    vehicle_path_runtime.import_received_point_cnt = 0u;
    vehicle_path_runtime.import_active = FALSE;
    vehicle_path_runtime.import_commit_pending = FALSE;
    vehicle_path_runtime.record_stream_active = FALSE;
    vehicle_path_runtime.record_stream_failed = FALSE;
    vehicle_path_runtime.record_stream_close_requested = FALSE;
    vehicle_path_runtime.record_save_failed = FALSE;
    vehicle_path_runtime.record_last_pose_update_count = 0u;
    vehicle_path_runtime.record_last_point.x_mm = 0.0f;
    vehicle_path_runtime.record_last_point.y_mm = 0.0f;
    vehicle_path_runtime.record_last_point.theta_rad = 0.0f;
    vehicle_path_runtime.record_last_point.speed_mm_s = 0.0f;
    vehicle_path_runtime.record_path_length_mm = 0.0f;
    vehicle_path_record_print_queue_reset();
    vehicle_path_record_queue_reset();
}

static void vehicle_path_record_stream_start(uint32 slot_offset)
{
    vehicle_path_record_stream_reset();
    vehicle_path_runtime.record_stream_slot_offset = slot_offset;
    vehicle_path_runtime.record_stream_active = TRUE;
}

static boolean vehicle_path_record_stream_append(const vehicle_path_point_t* point)
{
    const uint8* point_bytes;
    uint32 point_offset;
    uint32 copy_length;
    uint32 free_length;

    if ((point == NULL_PTR) || (vehicle_path_runtime.record_stream_active == FALSE))
    {
        return FALSE;
    }

    if (vehicle_path_runtime.record_stream_failed != FALSE)
    {
        return FALSE;
    }

    point_bytes = (const uint8*)point;
    point_offset = 0u;

    while (point_offset < (uint32)sizeof(vehicle_path_point_t))
    {
        free_length = IFXFLASH_PFLASH_PAGE_LENGTH - vehicle_path_runtime.record_page_used_length;
        copy_length = (uint32)sizeof(vehicle_path_point_t) - point_offset;
        if (copy_length > free_length)
        {
            copy_length = free_length;
        }

        for (free_length = 0u; free_length < copy_length; free_length++)
        {
            vehicle_path_runtime.record_page[vehicle_path_runtime.record_page_used_length + free_length] =
                point_bytes[point_offset + free_length];
        }

        vehicle_path_runtime.record_page_used_length += copy_length;
        point_offset += copy_length;

        if (vehicle_path_runtime.record_page_used_length >= IFXFLASH_PFLASH_PAGE_LENGTH)
        {
            if (vehicle_path_record_stream_flush(FALSE) == FALSE)
            {
                vehicle_path_runtime.record_stream_failed = TRUE;
                return FALSE;
            }
        }
    }

    vehicle_path_runtime.record_checksum =
        vehicle_path_checksum_update(vehicle_path_runtime.record_checksum,
                                     (const uint8*)point,
                                     (uint32)sizeof(vehicle_path_point_t));
    return TRUE;
}

static boolean vehicle_path_record_stream_flush(boolean force)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();
    uint32 write_offset;
    uint32 write_length;
    uint32 i;

    if (vehicle_path_runtime.record_page_used_length == 0u)
    {
        return TRUE;
    }

    if ((force == FALSE) && (vehicle_path_runtime.record_page_used_length < IFXFLASH_PFLASH_PAGE_LENGTH))
    {
        return TRUE;
    }

    for (i = vehicle_path_runtime.record_page_used_length; i < IFXFLASH_PFLASH_PAGE_LENGTH; i++)
    {
        vehicle_path_runtime.record_page[i] = 0xFFu;
    }

    write_offset = cfg->flash_offset
                 + vehicle_path_runtime.record_stream_slot_offset
                 + VEHICLE_PATH_FLASH_POINT_OFFSET
                 + vehicle_path_runtime.record_page_start_offset;
    write_length = vehicle_path_record_queue_push(write_offset,
                                                  vehicle_path_runtime.record_page)
                 ? IFXFLASH_PFLASH_PAGE_LENGTH
                 : 0u;

    if (write_length != IFXFLASH_PFLASH_PAGE_LENGTH)
    {
        vehicle_path_runtime.record_stream_failed = TRUE;
        return FALSE;
    }

    vehicle_path_runtime.record_page_start_offset += IFXFLASH_PFLASH_PAGE_LENGTH;
    vehicle_path_runtime.record_page_used_length = 0u;
    for (i = 0u; i < IFXFLASH_PFLASH_PAGE_LENGTH; i++)
    {
        vehicle_path_runtime.record_page[i] = 0xFFu;
    }

    return TRUE;
}

static void vehicle_path_record_queue_reset(void)
{
    vehicle_path_runtime.record_queue_head = 0u;
    vehicle_path_runtime.record_queue_tail = 0u;
}

static boolean vehicle_path_record_queue_push(uint32 offset, const uint8* page)
{
    uint32 tail = vehicle_path_runtime.record_queue_tail;
    uint32 next_tail = tail + 1u;
    uint32 i;

    if (next_tail >= VEHICLE_PATH_RECORD_QUEUE_SLOT_COUNT)
    {
        next_tail = 0u;
    }

    if (next_tail == vehicle_path_runtime.record_queue_head)
    {
        return FALSE;
    }

    vehicle_path_runtime.record_queue_offset[tail] = offset;

    for (i = 0u; i < IFXFLASH_PFLASH_PAGE_LENGTH; i++)
    {
        vehicle_path_runtime.record_queue[tail][i] = page[i];
    }

    vehicle_path_runtime.record_queue_tail = next_tail;
    return TRUE;
}

static boolean vehicle_path_record_queue_is_empty(void)
{
    return (vehicle_path_runtime.record_queue_head == vehicle_path_runtime.record_queue_tail) ? TRUE : FALSE;
}

static void vehicle_path_record_queue_drain(uint32 max_page_count)
{
    uint32 drained_count = 0u;
    uint32 head;
    uint32 next_head;
    uint32 write_length;

    while ((drained_count < max_page_count) && (vehicle_path_record_queue_is_empty() == FALSE))
    {
        head = vehicle_path_runtime.record_queue_head;
        write_length = service_storage_writeSync(SERVICE_STORAGE_1,
                                                 vehicle_path_runtime.record_queue_offset[head],
                                                 vehicle_path_runtime.record_queue[head],
                                                 IFXFLASH_PFLASH_PAGE_LENGTH);

        if (write_length != IFXFLASH_PFLASH_PAGE_LENGTH)
        {
            vehicle_path_runtime.record_save_failed = TRUE;
            vehicle_path_runtime.record_stream_failed = TRUE;
            vehicle_path_runtime.save_busy = FALSE;
            vehicle_path_runtime.record_stream_active = FALSE;
            vehicle_path_runtime.import_active = FALSE;
            vehicle_path_runtime.import_commit_pending = FALSE;
            tools_printf("{pathupload}failed,flash\r\n");
            return;
        }

        next_head = head + 1u;
        if (next_head >= VEHICLE_PATH_RECORD_QUEUE_SLOT_COUNT)
        {
            next_head = 0u;
        }

        vehicle_path_runtime.record_queue_head = next_head;
        drained_count++;
    }
}

static void vehicle_path_record_save_finish_try(void)
{
    uint32 slot_offset;
    boolean imported;

    if ((vehicle_path_runtime.record_stream_active == FALSE)
        || (vehicle_path_runtime.record_stream_close_requested == FALSE))
    {
        return;
    }

    if (vehicle_path_runtime.record_save_failed != FALSE)
    {
        boolean import_failed = vehicle_path_runtime.import_active;
        vehicle_path_runtime.record_stream_active = FALSE;
        vehicle_path_runtime.save_busy = FALSE;
        vehicle_path_runtime.import_active = FALSE;
        vehicle_path_runtime.import_commit_pending = FALSE;
        if (import_failed != FALSE)
        {
            tools_printf("{pathupload}failed,flash\r\n");
        }
        return;
    }

    if (vehicle_path_record_queue_is_empty() == FALSE)
    {
        return;
    }

    slot_offset = vehicle_path_runtime.record_stream_slot_offset;
    imported = vehicle_path_runtime.import_active;
    if (imported != FALSE)
    {
        vehicle_path_runtime.job_source_point_cnt = vehicle_path_runtime.state.point_cnt;
        vehicle_path_runtime.job_path_length_mm = vehicle_path_runtime.record_path_length_mm;
        vehicle_path_runtime.job_header.checksum = vehicle_path_runtime.record_checksum;
        vehicle_path_runtime.job_downsampled = TRUE;
    }

    if (vehicle_path_header_write(slot_offset,
                                  vehicle_path_runtime.state.point_cnt,
                                  vehicle_path_runtime.record_checksum) == FALSE)
    {
        vehicle_path_runtime.record_save_failed = TRUE;
        vehicle_path_runtime.record_stream_active = FALSE;
        vehicle_path_runtime.save_busy = FALSE;
        vehicle_path_runtime.import_active = FALSE;
        vehicle_path_runtime.import_commit_pending = FALSE;
        tools_printf("{pathupload}failed,commit\r\n");
        return;
    }

    vehicle_path_runtime.record_stream_active = FALSE;
    vehicle_path_runtime.record_stream_close_requested = FALSE;
    vehicle_path_runtime.save_busy = FALSE;
    vehicle_path_runtime.active_slot_offset = slot_offset;
    vehicle_path_runtime.state.last_distance_mm = vehicle_path_runtime.record_path_length_mm;
    vehicle_path_runtime.window_from_flash = FALSE;
    vehicle_path_runtime.window_active_buffer_cnt = 0u;
    vehicle_path_window_buffers_reset();
    vehicle_path_window_prefetch_reset();
    if (vehicle_path_runtime.state.point_cnt > vehicle_path_max_point_cnt_get())
    {
        (void)vehicle_path_window_load(0u);
    }
    else
    {
        vehicle_path_runtime.window_start_index_cnt = 0u;
        vehicle_path_runtime.window_valid_cnt = vehicle_path_runtime.state.point_cnt;
    }

    if (imported != FALSE)
    {
        uint32 imported_count = vehicle_path_runtime.import_received_point_cnt;
        uint32 imported_checksum = vehicle_path_runtime.record_checksum;
        float32 imported_length_mm = vehicle_path_runtime.record_path_length_mm;
        vehicle_path_runtime.import_active = FALSE;
        vehicle_path_runtime.import_commit_pending = FALSE;
        if (module_vehicle_path_load_planned() == FALSE)
        {
            tools_printf("{pathupload}failed,load\r\n");
            return;
        }
        /* Preserve a previously requested mode across asynchronous upload completion. */
        tools_printf("{pathupload}done,%u,%lu,%.1f\r\n",
                     (unsigned int)imported_count,
                     (unsigned long)imported_checksum,
                     (double)imported_length_mm);
    }

    if (vehicle_path_runtime.record_print_dropped_cnt > 0u)
    {
        tools_printf("{pathdrop}%lu\r\n", (unsigned long)vehicle_path_runtime.record_print_dropped_cnt);
    }
}

static void vehicle_path_record_print_queue_reset(void)
{
    vehicle_path_runtime.record_print_queue_head = 0u;
    vehicle_path_runtime.record_print_queue_tail = 0u;
    vehicle_path_runtime.record_print_dropped_cnt = 0u;
}

static boolean vehicle_path_record_print_queue_push(const vehicle_path_point_t* point,
                                                    float32 record_distance_mm,
                                                    uint32 point_index_cnt,
                                                    uint32 pose_update_count)
{
    const module_vehicle_encoder_observation_t* encoder_observation =
        module_vehicle_encoder_observation_get();
    vehicle_path_record_print_item_t item;
    uint32 tail = vehicle_path_runtime.record_print_queue_tail;
    uint32 next_tail = tail + 1u;

    if (point == NULL_PTR)
    {
        return FALSE;
    }

    if (next_tail >= VEHICLE_PATH_RECORD_PRINT_QUEUE_SLOT_COUNT)
    {
        next_tail = 0u;
    }

    if (next_tail == vehicle_path_runtime.record_print_queue_head)
    {
        return FALSE;
    }

    item.point = *point;
    item.record_distance_mm = record_distance_mm;
    item.left_total_distance_mm = encoder_observation->left_total_distance_mm;
    item.right_total_distance_mm = encoder_observation->right_total_distance_mm;
    item.left_total_count = encoder_observation->left_total_count;
    item.right_total_count = encoder_observation->right_total_count;
    item.left_sample_count = encoder_observation->left_sample_count;
    item.right_sample_count = encoder_observation->right_sample_count;
    item.point_index_cnt = point_index_cnt;
    item.pose_update_count = pose_update_count;
    item.encoder_update_count = encoder_observation->update_count;

    vehicle_path_runtime.record_print_queue[tail] = item;
    vehicle_path_runtime.record_print_queue_tail = next_tail;
    return TRUE;
}

static boolean vehicle_path_record_print_queue_is_empty(void)
{
    return (vehicle_path_runtime.record_print_queue_head == vehicle_path_runtime.record_print_queue_tail) ? TRUE : FALSE;
}

static void vehicle_path_record_print_queue_drain(uint32 max_point_count)
{
    uint32 drained_count = 0u;
    uint32 head;
    uint32 next_head;
    vehicle_path_record_print_item_t item;

    while ((drained_count < max_point_count) && (vehicle_path_record_print_queue_is_empty() == FALSE))
    {
        head = vehicle_path_runtime.record_print_queue_head;
        item = vehicle_path_runtime.record_print_queue[head];

        next_head = head + 1u;
        if (next_head >= VEHICLE_PATH_RECORD_PRINT_QUEUE_SLOT_COUNT)
        {
            next_head = 0u;
        }

        vehicle_path_runtime.record_print_queue_head = next_head;
        drained_count++;

        tools_printf("{path}%.3f,%.3f,%.3f,%lu,%.3f,%.3f,%.3f,%ld,%ld,%.3f,%.3f,%lu,%lu,%lu,%lu\r\n",
                     (double)item.point.x_mm,
                     (double)item.point.y_mm,
                     (double)(item.point.theta_rad * VEHICLE_PATH_RAD_TO_DEG),
                     (unsigned long)item.point_index_cnt,
                     (double)item.record_distance_mm,
                     (double)item.left_total_distance_mm,
                     (double)item.right_total_distance_mm,
                     (long)item.left_total_count,
                     (long)item.right_total_count,
                     (double)(item.right_total_distance_mm - item.left_total_distance_mm),
                     (double)(((item.right_total_distance_mm - item.left_total_distance_mm)
                               / VEHICLE_PATH_ENCODER_WHEEL_BASE_MM)
                              * VEHICLE_PATH_RAD_TO_DEG),
                     (unsigned long)item.left_sample_count,
                     (unsigned long)item.right_sample_count,
                     (unsigned long)item.pose_update_count,
                     (unsigned long)item.encoder_update_count);
    }
}

static void vehicle_path_job_reset(void)
{
    uint32 i;

    if ((vehicle_path_runtime.job_state >= VEHICLE_PATH_JOB_PLAN_HEADER)
        && (vehicle_path_runtime.job_result_valid == FALSE))
    {
        vehicle_path_runtime.job_result_valid = TRUE;
        vehicle_path_runtime.job_result_success = FALSE;
    }

    vehicle_path_runtime.job_state = VEHICLE_PATH_JOB_IDLE;
    vehicle_path_runtime.job_index_cnt = 0u;
    vehicle_path_runtime.job_point_cnt = 0u;
    vehicle_path_runtime.job_source_point_cnt = 0u;
    vehicle_path_runtime.job_checksum = VEHICLE_PATH_CHECKSUM_SEED;
    vehicle_path_runtime.job_write_offset = 0u;
    vehicle_path_runtime.job_planned_cnt = 0u;
    vehicle_path_runtime.job_min_speed_mm_s = 0.0f;
    vehicle_path_runtime.job_max_speed_mm_s = 0.0f;
    vehicle_path_runtime.job_path_length_mm = 0.0f;
    vehicle_path_runtime.job_last_point.x_mm = 0.0f;
    vehicle_path_runtime.job_last_point.y_mm = 0.0f;
    vehicle_path_runtime.job_last_point.theta_rad = 0.0f;
    vehicle_path_runtime.job_last_point.speed_mm_s = 0.0f;
    vehicle_path_runtime.job_has_last_point = FALSE;
    vehicle_path_runtime.job_downsampled = FALSE;
    vehicle_path_runtime.job_resample_next_index_cnt = 0u;
    vehicle_path_runtime.job_resample_target_index_cnt = 0u;
    vehicle_path_runtime.job_resample_step_mm = 0.0f;
    vehicle_path_runtime.job_resample_segment_start_mm = 0.0f;
    vehicle_path_runtime.job_resample_segment_end_mm = 0.0f;
    vehicle_path_runtime.job_resample_prev_point.x_mm = 0.0f;
    vehicle_path_runtime.job_resample_prev_point.y_mm = 0.0f;
    vehicle_path_runtime.job_resample_prev_point.theta_rad = 0.0f;
    vehicle_path_runtime.job_resample_prev_point.speed_mm_s = 0.0f;
    vehicle_path_runtime.job_resample_next_point = vehicle_path_runtime.job_resample_prev_point;
    vehicle_path_runtime.job_chunk_core_start_cnt = 0u;
    vehicle_path_runtime.job_chunk_core_count = 0u;
    vehicle_path_runtime.job_chunk_input_start_cnt = 0u;
    vehicle_path_runtime.job_chunk_input_count = 0u;
    vehicle_path_runtime.job_chunk_loaded_count = 0u;
    vehicle_path_runtime.job_chunk_core_offset_cnt = 0u;
    vehicle_path_runtime.job_chunk_write_offset_cnt = 0u;
    vehicle_path_runtime.job_chunk_pass = VEHICLE_PATH_PLAN_CHUNK_PASS_FORWARD;
    vehicle_path_runtime.job_speed_pass_carry_mm_s = 0.0f;
    vehicle_path_runtime.job_speed_pass_carry_valid = FALSE;
    for (i = 0u; i < VEHICLE_PATH_MARKER_MAX_COUNT; i++)
    {
        vehicle_path_runtime.job_marker_source_distance_mm[i] = 0.0f;
        vehicle_path_runtime.job_marker_source_distance_valid[i] = FALSE;
    }
    for (i = 0u; i < VEHICLE_PATH_PLAN_MAX_CHUNK_COUNT; i++)
    {
        vehicle_path_runtime.job_chunk_forward_start_speed_mm_s[i] = 0.0f;
        vehicle_path_runtime.job_chunk_backward_next_speed_mm_s[i] = 0.0f;
        vehicle_path_runtime.job_chunk_backward_next_valid[i] = FALSE;
        vehicle_path_runtime.job_chunk_source_next_index_cnt[i] = 0u;
        vehicle_path_runtime.job_chunk_source_segment_start_mm[i] = 0.0f;
        vehicle_path_runtime.job_chunk_source_segment_end_mm[i] = 0.0f;
        vehicle_path_runtime.job_chunk_source_valid[i] = FALSE;
    }
    vehicle_path_runtime.job_header.magic = 0u;
    vehicle_path_runtime.job_header.version = 0u;
    vehicle_path_runtime.job_header.point_cnt = 0u;
    vehicle_path_runtime.job_header.checksum = 0u;

    for (i = 0u; i < IFXFLASH_PFLASH_PAGE_LENGTH; i++)
    {
        vehicle_path_runtime.job_page[i] = 0xFFu;
    }
}

static boolean vehicle_path_job_busy(void)
{
    return (vehicle_path_runtime.job_state == VEHICLE_PATH_JOB_IDLE) ? FALSE : TRUE;
}

static boolean vehicle_path_operation_busy(void)
{
    return ((vehicle_path_job_busy() != FALSE)
            || (vehicle_path_runtime.save_busy != FALSE)
            || (vehicle_path_runtime.import_erase_active != FALSE)
            || (vehicle_path_runtime.record_stream_active != FALSE)
            || (vehicle_path_runtime.state.recording != FALSE)
            || (vehicle_path_runtime.state.replaying != FALSE))
           ? TRUE
           : FALSE;
}

static boolean vehicle_path_flash_header_read_from(uint32 slot_offset, vehicle_path_flash_header_t* header)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();

    if (header == NULL_PTR)
    {
        return FALSE;
    }

    return (service_storage_readSync(SERVICE_STORAGE_1,
                                     cfg->flash_offset + slot_offset,
                                     (uint8*)header,
                                     (uint32)sizeof(*header)) == (uint32)sizeof(*header))
           ? TRUE
           : FALSE;
}

static boolean vehicle_path_flash_header_valid_at(uint32 slot_offset, vehicle_path_flash_header_t* header)
{
    vehicle_path_flash_header_t local_header;
    vehicle_path_flash_header_t* target_header = (header != NULL_PTR) ? header : &local_header;

    if (vehicle_path_flash_header_read_from(slot_offset, target_header) == FALSE)
    {
        return FALSE;
    }

    if ((target_header->magic != VEHICLE_PATH_MAGIC)
        || (target_header->version != VEHICLE_PATH_VERSION)
        || (vehicle_path_flash_slot_read_range_valid(slot_offset,
                                                     (uint32)target_header->point_cnt) == FALSE))
    {
        return FALSE;
    }

    return TRUE;
}

static boolean vehicle_path_planned_standalone_valid(const vehicle_path_flash_header_t* planned_header)
{
    vehicle_path_flash_plan_meta_t plan_meta;

    if ((planned_header == NULL_PTR)
        || (vehicle_path_plan_meta_read(&plan_meta) == FALSE))
    {
        return FALSE;
    }

    return (boolean)(((plan_meta.reserved & VEHICLE_PATH_PLAN_META_STANDALONE_UPLOAD) != 0u)
                 && (plan_meta.planned_point_cnt == planned_header->point_cnt)
                 && (plan_meta.source_point_cnt == planned_header->point_cnt)
                 && (plan_meta.source_checksum == planned_header->checksum)
                 && (plan_meta.source_length_mm > VEHICLE_PATH_PLAN_RESAMPLE_EPSILON_MM)
                 && (plan_meta.planned_step_mm > VEHICLE_PATH_PLAN_RESAMPLE_EPSILON_MM));
}

static boolean vehicle_path_planned_header_matches_raw(const vehicle_path_flash_header_t* planned_header,
                                                       uint16* raw_point_cnt)
{
    vehicle_path_flash_header_t raw_header;
    vehicle_path_flash_plan_meta_t plan_meta;

    if (raw_point_cnt != NULL_PTR)
    {
        *raw_point_cnt = 0u;
    }

    if (planned_header == NULL_PTR)
    {
        return FALSE;
    }

    if (vehicle_path_flash_header_valid_at(VEHICLE_PATH_FLASH_RAW_OFFSET, &raw_header) == FALSE)
    {
        return TRUE;
    }

    if (raw_point_cnt != NULL_PTR)
    {
        *raw_point_cnt = raw_header.point_cnt;
    }

    if (vehicle_path_plan_meta_read(&plan_meta) == FALSE)
    {
        return (planned_header->point_cnt == raw_header.point_cnt) ? TRUE : FALSE;
    }

    if ((plan_meta.planned_point_cnt != planned_header->point_cnt)
        || (plan_meta.source_point_cnt != raw_header.point_cnt)
        || (plan_meta.source_checksum != raw_header.checksum)
        || (plan_meta.source_length_mm <= VEHICLE_PATH_PLAN_RESAMPLE_EPSILON_MM)
        || (plan_meta.planned_step_mm <= VEHICLE_PATH_PLAN_RESAMPLE_EPSILON_MM))
    {
        return FALSE;
    }

    return TRUE;
}

static void vehicle_path_planned_slot_erase_begin(uint32 point_cnt)
{
    uint32 point_end_offset = VEHICLE_PATH_FLASH_POINT_OFFSET
                            + (point_cnt * (uint32)sizeof(vehicle_path_point_t));
    uint32 data_sector_count;

    if (point_end_offset <= 0x00010000u)
    {
        data_sector_count = 1u;
    }
    else if (point_end_offset <= 0x00020000u)
    {
        data_sector_count = 2u;
    }
    else if (point_end_offset <= 0x00040000u)
    {
        data_sector_count = 3u;
    }
    else if (point_end_offset <= 0x00060000u)
    {
        data_sector_count = 4u;
    }
    else
    {
        data_sector_count = 5u;
    }
    vehicle_path_runtime.import_erase_data_sector_count = data_sector_count;
    vehicle_path_runtime.import_erase_sector_count =
        data_sector_count + ((data_sector_count < 5u) ? 1u : 0u);
    vehicle_path_runtime.import_erase_sector_index = 0u;
    vehicle_path_runtime.import_erase_delay = TRUE;
    vehicle_path_runtime.import_erase_active = TRUE;
}

static void vehicle_path_planned_slot_erase_run(void)
{
    static const uint32 sector_offsets[5] =
    {
        0x00000000u,
        0x00010000u,
        0x00020000u,
        0x00040000u,
        0x00060000u,
    };
    const vehicle_path_cfg_t* cfg;
    uint32 address;
    uint32 erase_index;
    uint32 offset_index;
    uint32 expected_point_cnt;

    if (vehicle_path_runtime.import_erase_active == FALSE)
    {
        return;
    }
    if (vehicle_path_runtime.import_erase_delay != FALSE)
    {
        vehicle_path_runtime.import_erase_delay = FALSE;
        return;
    }

    erase_index = vehicle_path_runtime.import_erase_sector_index;
    offset_index = (erase_index < vehicle_path_runtime.import_erase_data_sector_count)
                 ? erase_index : 4u;
    cfg = vehicle_path_cfg_get();
    address = device_int_flash_start_address_get(DEVICE_INT_FLASH_1)
            + cfg->flash_offset
            + VEHICLE_PATH_FLASH_PLANNED_OFFSET
            + sector_offsets[offset_index];
    tools_printf("{pathupload}erase,%u,%u,%u\r\n",
                 (unsigned int)(erase_index + 1u),
                 (unsigned int)vehicle_path_runtime.import_erase_sector_count,
                 (unsigned int)address);
    driver_flash_eraseSector(address);
    vehicle_path_runtime.import_erase_sector_index++;
    tools_printf("{pathupload}erase_done,%u,%u\r\n",
                 (unsigned int)vehicle_path_runtime.import_erase_sector_index,
                 (unsigned int)vehicle_path_runtime.import_erase_sector_count);

    if (vehicle_path_runtime.import_erase_sector_index
        >= vehicle_path_runtime.import_erase_sector_count)
    {
        expected_point_cnt = vehicle_path_runtime.import_expected_point_cnt;
        vehicle_path_runtime.import_erase_active = FALSE;
        vehicle_path_record_stream_start(VEHICLE_PATH_FLASH_PLANNED_OFFSET);
        vehicle_path_runtime.import_expected_point_cnt = expected_point_cnt;
        vehicle_path_runtime.import_received_point_cnt = 0u;
        vehicle_path_runtime.import_active = TRUE;
    }
}

static void vehicle_path_planned_slot_erase_reset(void)
{
    vehicle_path_runtime.import_erase_active = FALSE;
    vehicle_path_runtime.import_erase_delay = FALSE;
    vehicle_path_runtime.import_erase_data_sector_count = 0u;
    vehicle_path_runtime.import_erase_sector_count = 0u;
    vehicle_path_runtime.import_erase_sector_index = 0u;
}

static void vehicle_path_planned_speed_overlay_reset(void)
{
    vehicle_path_runtime.planned_speed_overlay_active = FALSE;
    vehicle_path_runtime.planned_speed_point_cnt = 0u;
    vehicle_path_planned_speed_meta_reset();
    vehicle_path_source_distance_cache_reset();
}

static void vehicle_path_planned_speed_meta_reset(void)
{
    vehicle_path_runtime.planned_speed_source_point_cnt = 0u;
    vehicle_path_runtime.planned_speed_source_length_mm = 0.0f;
    vehicle_path_runtime.planned_speed_step_mm = 0.0f;
    vehicle_path_runtime.planned_speed_meta_valid = FALSE;
}

static void vehicle_path_planned_speed_meta_set(uint32 source_point_cnt,
                                                float32 source_length_mm,
                                                uint32 planned_point_cnt,
                                                boolean meta_valid)
{
    if (source_point_cnt == 0u)
    {
        source_point_cnt = planned_point_cnt;
    }

    if ((source_length_mm <= VEHICLE_PATH_PLAN_RESAMPLE_EPSILON_MM)
        && (source_point_cnt >= 2u))
    {
        source_length_mm =
            (float32)(source_point_cnt - 1u) * vehicle_path_min_distance_step_get();
    }

    vehicle_path_runtime.planned_speed_source_point_cnt = source_point_cnt;
    vehicle_path_runtime.planned_speed_source_length_mm = source_length_mm;
    vehicle_path_runtime.planned_speed_step_mm = 0.0f;
    vehicle_path_runtime.planned_speed_meta_valid = FALSE;

    if ((planned_point_cnt >= 2u)
        && (source_length_mm > VEHICLE_PATH_PLAN_RESAMPLE_EPSILON_MM))
    {
        vehicle_path_runtime.planned_speed_step_mm =
            source_length_mm / (float32)(planned_point_cnt - 1u);
        vehicle_path_runtime.planned_speed_meta_valid =
            (meta_valid != FALSE) ? TRUE : FALSE;
    }
}

static void vehicle_path_planned_speed_meta_set_from_job(uint32 planned_point_cnt)
{
    uint32 source_point_cnt = vehicle_path_runtime.job_source_point_cnt;
    float32 source_length_mm = vehicle_path_runtime.job_path_length_mm;

    if (source_point_cnt == 0u)
    {
        source_point_cnt = planned_point_cnt;
    }

    if ((vehicle_path_runtime.job_downsampled == FALSE)
        || (source_length_mm <= VEHICLE_PATH_PLAN_RESAMPLE_EPSILON_MM))
    {
        source_length_mm = vehicle_path_points_length_calculate(planned_point_cnt);
    }

    vehicle_path_planned_speed_meta_set(source_point_cnt,
                                        source_length_mm,
                                        planned_point_cnt,
                                        TRUE);
}

static boolean vehicle_path_planned_speed_distance_map_valid(void)
{
    return (boolean)((vehicle_path_runtime.planned_speed_overlay_active != FALSE)
                 && (vehicle_path_runtime.planned_speed_meta_valid != FALSE)
                 && (vehicle_path_runtime.planned_speed_point_cnt >= 2u)
                 && (vehicle_path_runtime.planned_speed_source_length_mm
                     > VEHICLE_PATH_PLAN_RESAMPLE_EPSILON_MM)
                 && (vehicle_path_runtime.planned_speed_step_mm
                     > VEHICLE_PATH_PLAN_RESAMPLE_EPSILON_MM));
}

static uint16 vehicle_path_speed_mm_s_to_u16(float32 speed_mm_s)
{
    if (speed_mm_s < 0.0f)
    {
        speed_mm_s = 0.0f;
    }
    if (speed_mm_s > 65535.0f)
    {
        speed_mm_s = 65535.0f;
    }

    return (uint16)(speed_mm_s + 0.5f);
}

static boolean vehicle_path_planned_speed_table_load(const vehicle_path_flash_header_t* planned_header)
{
    vehicle_path_flash_plan_meta_t plan_meta;
    boolean plan_meta_valid = FALSE;

    vehicle_path_planned_speed_overlay_reset();

    if ((planned_header == NULL_PTR)
        || (vehicle_path_flash_slot_read_range_valid(VEHICLE_PATH_FLASH_PLANNED_OFFSET,
                                                     (uint32)planned_header->point_cnt) == FALSE))
    {
        return FALSE;
    }

    if ((vehicle_path_plan_meta_read(&plan_meta) != FALSE)
        && (plan_meta.planned_point_cnt == planned_header->point_cnt))
    {
        vehicle_path_planned_speed_meta_set(plan_meta.source_point_cnt,
                                            plan_meta.source_length_mm,
                                            (uint32)planned_header->point_cnt,
                                            TRUE);
        plan_meta_valid = TRUE;
    }
    if (plan_meta_valid == FALSE)
    {
        vehicle_path_planned_speed_meta_set((uint32)planned_header->point_cnt,
                                            0.0f,
                                            (uint32)planned_header->point_cnt,
                                            FALSE);
    }

    vehicle_path_runtime.planned_speed_point_cnt = (uint32)planned_header->point_cnt;
    vehicle_path_runtime.planned_speed_overlay_active =
        (vehicle_path_runtime.planned_speed_point_cnt >= vehicle_path_cfg_get()->min_valid_point_cnt)
            ? TRUE
            : FALSE;
    return vehicle_path_runtime.planned_speed_overlay_active;
}

static void vehicle_path_planned_speed_table_store_from_points(uint32 point_cnt)
{
    uint32 index_cnt;

    vehicle_path_planned_speed_overlay_reset();

    if (point_cnt > VEHICLE_PATH_DEFAULT_MAX_POINT_CNT)
    {
        point_cnt = VEHICLE_PATH_DEFAULT_MAX_POINT_CNT;
    }

    for (index_cnt = 0u; index_cnt < point_cnt; index_cnt++)
    {
        vehicle_path_runtime.planned_speed_table_mm_s[index_cnt] =
            vehicle_path_speed_mm_s_to_u16(vehicle_path_runtime.points[index_cnt].speed_mm_s);
    }

    vehicle_path_runtime.planned_speed_point_cnt = point_cnt;
    vehicle_path_runtime.planned_speed_overlay_active =
        (vehicle_path_runtime.planned_speed_point_cnt >= vehicle_path_cfg_get()->min_valid_point_cnt)
            ? TRUE
            : FALSE;
    vehicle_path_planned_speed_meta_set_from_job(point_cnt);
}

static void vehicle_path_plan_speed_overlay_activate_failed_reset(void)
{
    vehicle_path_planned_speed_overlay_reset();
    vehicle_path_runtime.active_slot_offset = VEHICLE_PATH_FLASH_RAW_OFFSET;
    vehicle_path_runtime.state.point_cnt = 0u;
    vehicle_path_runtime.state.last_distance_mm = 0.0f;
    vehicle_path_runtime.state.replay_cursor_cnt = 0u;
    vehicle_path_runtime.state.replay_target_index_cnt = 0u;
    vehicle_path_runtime.state.replay_target_valid = FALSE;
    vehicle_path_runtime.window_start_index_cnt = 0u;
    vehicle_path_runtime.window_valid_cnt = 0u;
    vehicle_path_runtime.window_active_buffer_cnt = 0u;
    vehicle_path_runtime.window_from_flash = FALSE;
    vehicle_path_window_buffers_reset();
    vehicle_path_window_prefetch_reset();
    vehicle_path_marker_reset();
    vehicle_path_runtime.replay_edge_points_valid = FALSE;
    vehicle_path_replay_target_reset();
}

static boolean vehicle_path_plan_speed_overlay_activate_from_job(void)
{
    vehicle_path_flash_header_t raw_header;
    uint32 planned_point_cnt = vehicle_path_runtime.job_point_cnt;

    if (planned_point_cnt > VEHICLE_PATH_DEFAULT_MAX_POINT_CNT)
    {
        planned_point_cnt = VEHICLE_PATH_DEFAULT_MAX_POINT_CNT;
    }

    vehicle_path_planned_speed_table_store_from_points(planned_point_cnt);
    if (vehicle_path_runtime.planned_speed_overlay_active == FALSE)
    {
        vehicle_path_plan_speed_overlay_activate_failed_reset();
        return FALSE;
    }

    if (vehicle_path_flash_header_valid_at(VEHICLE_PATH_FLASH_RAW_OFFSET, &raw_header) == FALSE)
    {
        vehicle_path_plan_speed_overlay_activate_failed_reset();
        return FALSE;
    }

    if (vehicle_path_load_from_slot(VEHICLE_PATH_FLASH_RAW_OFFSET, &raw_header) == FALSE)
    {
        vehicle_path_plan_speed_overlay_activate_failed_reset();
        return FALSE;
    }

    vehicle_path_runtime.planned_speed_overlay_active = TRUE;
    vehicle_path_runtime.planned_speed_point_cnt = planned_point_cnt;
    tools_printf("{pathinfo}planned_speed_overlay,%u,%u,%u,%.1f,%.3f\r\n",
                 (unsigned int)raw_header.point_cnt,
                 (unsigned int)planned_point_cnt,
                 (unsigned int)((vehicle_path_runtime.planned_speed_meta_valid != FALSE) ? 1u : 0u),
                 (double)vehicle_path_runtime.planned_speed_source_length_mm,
                 (double)vehicle_path_runtime.planned_speed_step_mm);
    return TRUE;
}

static float32 vehicle_path_planned_speed_at_index_get(float32 raw_index_float, float32 fallback_speed_mm_s)
{
    return vehicle_path_planned_speed_at_scaled_index_get(raw_index_float, fallback_speed_mm_s);
}

static float32 vehicle_path_planned_speed_at_scaled_index_get(float32 raw_index_float,
                                                             float32 fallback_speed_mm_s)
{
    uint32 raw_point_cnt = vehicle_path_runtime.state.point_cnt;
    uint32 planned_point_cnt = vehicle_path_runtime.planned_speed_point_cnt;
    float32 planned_index_float;
    vehicle_path_point_t point;

    if ((vehicle_path_runtime.planned_speed_overlay_active == FALSE)
        || (raw_point_cnt < 2u)
        || (planned_point_cnt < 2u))
    {
        return fallback_speed_mm_s;
    }

    if (raw_index_float < 0.0f)
    {
        raw_index_float = 0.0f;
    }
    if (raw_index_float > (float32)(raw_point_cnt - 1u))
    {
        raw_index_float = (float32)(raw_point_cnt - 1u);
    }

    /* Planned speed may be a downsampled overlay; map raw replay progress by normalized index. */
    planned_index_float = raw_index_float
                        * ((float32)(planned_point_cnt - 1u) / (float32)(raw_point_cnt - 1u));
    if (planned_index_float < 0.0f)
    {
        planned_index_float = 0.0f;
    }
    if (planned_index_float > (float32)(planned_point_cnt - 1u))
    {
        planned_index_float = (float32)(planned_point_cnt - 1u);
    }

    if (vehicle_path_point_at_index_float(planned_index_float, &point) == FALSE)
    {
        return fallback_speed_mm_s;
    }

    return vehicle_path_plan_stored_speed_clamp(point.speed_mm_s);
}

static float32 vehicle_path_planned_speed_preview_limit_get(float32 base_index_float,
                                                           float32 base_source_distance_mm,
                                                           boolean base_source_distance_valid,
                                                           float32 fallback_speed_mm_s,
                                                           float32* preview_index_float)
{
    uint32 raw_point_cnt = vehicle_path_runtime.state.point_cnt;
    float32 speed_limit_mm_s;
    float32 preview_distance_mm;
    float32 future_index_float;
    float32 future_speed_mm_s;

    speed_limit_mm_s = vehicle_path_plan_stored_speed_clamp(fallback_speed_mm_s);
    if (preview_index_float != NULL_PTR)
    {
        *preview_index_float = base_index_float;
    }
    if ((vehicle_path_runtime.planned_speed_overlay_active == FALSE) || (raw_point_cnt < 2u))
    {
        return speed_limit_mm_s;
    }

    preview_distance_mm = vehicle_path_replay_speed_preview_distance_mm;
    if (preview_distance_mm < 0.0f)
    {
        preview_distance_mm = 0.0f;
    }

    (void)base_source_distance_mm;
    (void)base_source_distance_valid;

    {
        float32 remaining_mm = vehicle_path_replay_remaining_mm_get(base_index_float);
        float32 preview_before_end_decel_mm = remaining_mm
                                            - VEHICLE_PATH_DEFAULT_END_DECEL_DISTANCE_MM;

        if (preview_before_end_decel_mm <= 0.0f)
        {
            preview_distance_mm = 0.0f;
        }
        else if (preview_distance_mm > preview_before_end_decel_mm)
        {
            preview_distance_mm = preview_before_end_decel_mm;
        }
    }

    if (vehicle_path_index_advance_by_distance(base_index_float,
                                               preview_distance_mm,
                                               &future_index_float) == FALSE)
    {
        return speed_limit_mm_s;
    }

    if (future_index_float > (float32)(raw_point_cnt - 1u))
    {
        future_index_float = (float32)(raw_point_cnt - 1u);
    }
    if (preview_index_float != NULL_PTR)
    {
        *preview_index_float = future_index_float;
    }

    future_speed_mm_s =
        vehicle_path_planned_speed_at_scaled_index_get(future_index_float, speed_limit_mm_s);

    if (future_speed_mm_s < speed_limit_mm_s)
    {
        speed_limit_mm_s = future_speed_mm_s;
    }

    return vehicle_path_plan_stored_speed_clamp(speed_limit_mm_s);
}

static void vehicle_path_background_job_run(void)
{
    if (vehicle_path_runtime.job_state == VEHICLE_PATH_JOB_IDLE)
    {
        return;
    }

    if ((vehicle_path_runtime.job_state == VEHICLE_PATH_JOB_DUMP_HEADER)
        || (vehicle_path_runtime.job_state == VEHICLE_PATH_JOB_DUMP_POINTS))
    {
        vehicle_path_dump_job_run();
    }
    else
    {
        vehicle_path_plan_job_run();
    }
}

static void vehicle_path_dump_job_run(void)
{
    uint32 processed_count = 0u;
    vehicle_path_point_t point;

    if (vehicle_path_runtime.job_state == VEHICLE_PATH_JOB_DUMP_HEADER)
    {
        if (vehicle_path_flash_header_read_from(vehicle_path_runtime.job_read_slot_offset,
                                                &vehicle_path_runtime.job_header) == FALSE)
        {
            tools_printf("{pathdump}load_failed\r\n");
            vehicle_path_job_reset();
            return;
        }

        if ((vehicle_path_runtime.job_header.magic != VEHICLE_PATH_MAGIC)
            || (vehicle_path_runtime.job_header.version != VEHICLE_PATH_VERSION)
            || (vehicle_path_flash_slot_read_range_valid(vehicle_path_runtime.job_read_slot_offset,
                                                         (uint32)vehicle_path_runtime.job_header.point_cnt)
                == FALSE))
        {
            tools_printf("{pathdump}header_failed\r\n");
            vehicle_path_job_reset();
            return;
        }

        vehicle_path_runtime.active_slot_offset = vehicle_path_runtime.job_read_slot_offset;
        vehicle_path_runtime.state.point_cnt = (uint32)vehicle_path_runtime.job_header.point_cnt;
        vehicle_path_runtime.window_start_index_cnt = 0u;
        vehicle_path_runtime.window_valid_cnt = 0u;
        vehicle_path_runtime.window_active_buffer_cnt = 0u;
        vehicle_path_runtime.window_from_flash = TRUE;
        vehicle_path_window_buffers_reset();
        vehicle_path_window_prefetch_reset();
        vehicle_path_runtime.job_index_cnt = 0u;
        vehicle_path_runtime.job_point_cnt = (uint32)vehicle_path_runtime.job_header.point_cnt;
        vehicle_path_runtime.job_checksum = VEHICLE_PATH_CHECKSUM_SEED;
        vehicle_path_runtime.job_path_length_mm = 0.0f;
        vehicle_path_runtime.job_has_last_point = FALSE;
        tools_printf("{pathdump}slot,%s\r\n",
                     (vehicle_path_runtime.job_read_slot_offset == VEHICLE_PATH_FLASH_PLANNED_OFFSET)
                         ? "planned"
                         : "raw");
        tools_printf("{pathdump}begin,%u\r\n", (unsigned int)vehicle_path_runtime.job_point_cnt);
        vehicle_path_runtime.job_state = VEHICLE_PATH_JOB_DUMP_POINTS;
    }

    while ((processed_count < VEHICLE_PATH_DUMP_POINTS_PER_RUN)
           && (vehicle_path_runtime.job_index_cnt < vehicle_path_runtime.job_point_cnt))
    {
        if (vehicle_path_point_read(vehicle_path_runtime.job_index_cnt, &point) == FALSE)
        {
            tools_printf("{pathdump}read_failed,%u\r\n",
                         (unsigned int)vehicle_path_runtime.job_index_cnt);
            vehicle_path_job_reset();
            return;
        }

        vehicle_path_runtime.job_checksum =
            vehicle_path_checksum_update(vehicle_path_runtime.job_checksum,
                                         (const uint8*)&point,
                                         (uint32)sizeof(point));

        if (vehicle_path_runtime.job_has_last_point != FALSE)
        {
            vehicle_path_runtime.job_path_length_mm +=
                vehicle_path_segment_length_mm(&vehicle_path_runtime.job_last_point, &point);
        }
        vehicle_path_runtime.job_last_point = point;
        vehicle_path_runtime.job_has_last_point = TRUE;

        tools_printf("{pathdump}%u,%.3f,%.3f,%.6f,%.3f\r\n",
                     (unsigned int)vehicle_path_runtime.job_index_cnt,
                     (double)point.x_mm,
                     (double)point.y_mm,
                     (double)point.theta_rad,
                     (double)point.speed_mm_s);

        vehicle_path_runtime.job_index_cnt++;
        processed_count++;
    }

    if (vehicle_path_runtime.job_index_cnt >= vehicle_path_runtime.job_point_cnt)
    {
        if (vehicle_path_runtime.job_checksum != vehicle_path_runtime.job_header.checksum)
        {
            tools_printf("{pathdump}checksum_failed,%lu,%lu\r\n",
                         (unsigned long)vehicle_path_runtime.job_checksum,
                         (unsigned long)vehicle_path_runtime.job_header.checksum);
        }

        tools_printf("{pathdump}length,%.3f\r\n", (double)vehicle_path_runtime.job_path_length_mm);
        tools_printf("{pathdump}end,%u\r\n", (unsigned int)vehicle_path_runtime.job_point_cnt);
        vehicle_path_job_reset();
    }
}

static void vehicle_path_plan_job_run(void)
{
    switch (vehicle_path_runtime.job_state)
    {
        case VEHICLE_PATH_JOB_PLAN_HEADER:
            if (vehicle_path_flash_header_read_from(vehicle_path_runtime.job_read_slot_offset,
                                                    &vehicle_path_runtime.job_header) == FALSE)
            {
                tools_printf("{pathplan}load_failed\r\n");
                vehicle_path_job_reset();
                return;
            }

            if ((vehicle_path_runtime.job_header.magic != VEHICLE_PATH_MAGIC)
                || (vehicle_path_runtime.job_header.version != VEHICLE_PATH_VERSION)
                || (vehicle_path_flash_slot_read_range_valid(vehicle_path_runtime.job_read_slot_offset,
                                                             (uint32)vehicle_path_runtime.job_header.point_cnt)
                    == FALSE))
            {
                tools_printf("{pathplan}header_failed\r\n");
                vehicle_path_job_reset();
                return;
            }

            vehicle_path_runtime.job_source_point_cnt = (uint32)vehicle_path_runtime.job_header.point_cnt;
            if (vehicle_path_runtime.job_source_point_cnt < vehicle_path_cfg_get()->min_valid_point_cnt)
            {
                tools_printf("{pathplan}too_short,%u\r\n",
                             (unsigned int)vehicle_path_runtime.job_source_point_cnt);
                vehicle_path_job_reset();
                return;
            }

            vehicle_path_runtime.active_slot_offset = vehicle_path_runtime.job_read_slot_offset;
            vehicle_path_runtime.state.point_cnt = vehicle_path_runtime.job_source_point_cnt;
            vehicle_path_runtime.window_start_index_cnt = 0u;
            vehicle_path_runtime.window_valid_cnt = 0u;
            vehicle_path_runtime.window_active_buffer_cnt = 0u;
            vehicle_path_runtime.window_from_flash = TRUE;
            vehicle_path_window_buffers_reset();
            vehicle_path_window_prefetch_reset();
            vehicle_path_runtime.job_index_cnt = 0u;
            vehicle_path_runtime.job_checksum = VEHICLE_PATH_CHECKSUM_SEED;
            (void)vehicle_path_marker_load_from_slot(vehicle_path_runtime.job_read_slot_offset);
            {
                uint32 marker_cnt;
                for (marker_cnt = 0u; marker_cnt < VEHICLE_PATH_MARKER_MAX_COUNT; marker_cnt++)
                {
                    vehicle_path_runtime.job_marker_source_distance_mm[marker_cnt] = 0.0f;
                    vehicle_path_runtime.job_marker_source_distance_valid[marker_cnt] = FALSE;
                }
            }
            vehicle_path_runtime.job_path_length_mm = 0.0f;
            vehicle_path_runtime.job_has_last_point = FALSE;
            vehicle_path_runtime.job_state = VEHICLE_PATH_JOB_PLAN_LONG_SCAN;
            tools_printf("{pathplan}scan,%u\r\n",
                         (unsigned int)vehicle_path_runtime.job_source_point_cnt);
            break;

        case VEHICLE_PATH_JOB_PLAN_LONG_SCAN:
            if (vehicle_path_plan_long_scan_step(VEHICLE_PATH_PLAN_LONG_SCAN_POINTS_PER_RUN) == FALSE)
            {
                vehicle_path_job_reset();
                return;
            }

            if (vehicle_path_runtime.job_index_cnt >= vehicle_path_runtime.job_source_point_cnt)
            {
                if (vehicle_path_runtime.job_checksum != vehicle_path_runtime.job_header.checksum)
                {
                    tools_printf("{pathplan}checksum_failed,%lu,%lu\r\n",
                                 (unsigned long)vehicle_path_runtime.job_checksum,
                                 (unsigned long)vehicle_path_runtime.job_header.checksum);
                    vehicle_path_job_reset();
                    return;
                }

                if (vehicle_path_plan_fixed_begin() == FALSE)
                {
                    tools_printf("{pathplan}fixed_failed\r\n");
                    vehicle_path_runtime.job_result_valid = TRUE;
                    vehicle_path_runtime.job_result_success = FALSE;
                    vehicle_path_job_reset();
                    return;
                }
                vehicle_path_runtime.job_state = VEHICLE_PATH_JOB_PLAN_FIXED_CHUNK_LOAD;
            }
            break;

        case VEHICLE_PATH_JOB_PLAN_LONG_RESAMPLE:
            if (vehicle_path_plan_long_resample_step(VEHICLE_PATH_PLAN_LONG_RESAMPLE_POINTS_PER_RUN) == FALSE)
            {
                vehicle_path_job_reset();
                return;
            }

            if (vehicle_path_runtime.job_index_cnt >= vehicle_path_runtime.job_point_cnt)
            {
                vehicle_path_runtime.window_from_flash = FALSE;
                vehicle_path_runtime.state.point_cnt = vehicle_path_runtime.job_point_cnt;
                vehicle_path_runtime.window_start_index_cnt = 0u;
                vehicle_path_runtime.window_valid_cnt = vehicle_path_runtime.job_point_cnt;
                vehicle_path_runtime.window_active_buffer_cnt = 0u;
                vehicle_path_window_buffers_reset();
                vehicle_path_window_prefetch_reset();
                vehicle_path_runtime.job_state = VEHICLE_PATH_JOB_PLAN_GEOMETRY;
            }
            break;

        case VEHICLE_PATH_JOB_PLAN_LOAD_POINTS:
            if (vehicle_path_plan_load_points_step(VEHICLE_PATH_PLAN_LOAD_POINTS_PER_RUN) == FALSE)
            {
                vehicle_path_job_reset();
                return;
            }

            if (vehicle_path_runtime.job_index_cnt >= vehicle_path_runtime.job_point_cnt)
            {
                if (vehicle_path_runtime.job_checksum != vehicle_path_runtime.job_header.checksum)
                {
                    tools_printf("{pathplan}checksum_failed,%lu,%lu\r\n",
                                 (unsigned long)vehicle_path_runtime.job_checksum,
                                 (unsigned long)vehicle_path_runtime.job_header.checksum);
                    vehicle_path_job_reset();
                    return;
                }

                vehicle_path_runtime.window_from_flash = FALSE;
                vehicle_path_runtime.window_start_index_cnt = 0u;
                vehicle_path_runtime.window_valid_cnt = vehicle_path_runtime.job_point_cnt;
                vehicle_path_runtime.window_active_buffer_cnt = 0u;
                vehicle_path_window_buffers_reset();
                vehicle_path_window_prefetch_reset();
                (void)vehicle_path_marker_load_from_slot(vehicle_path_runtime.job_read_slot_offset);
                vehicle_path_runtime.job_state = VEHICLE_PATH_JOB_PLAN_GEOMETRY;
            }
            break;

        case VEHICLE_PATH_JOB_PLAN_GEOMETRY:
            vehicle_path_plan_geometry_compute(vehicle_path_runtime.job_point_cnt, TRUE);
            vehicle_path_plan_end_speed_apply(vehicle_path_runtime.job_point_cnt);
            vehicle_path_runtime.job_index_cnt = 1u;
            vehicle_path_runtime.job_state = VEHICLE_PATH_JOB_PLAN_FORWARD;
            break;

        case VEHICLE_PATH_JOB_PLAN_FORWARD:
            if (vehicle_path_plan_forward_step(VEHICLE_PATH_PLAN_PASS_POINTS_PER_RUN) != FALSE)
            {
                vehicle_path_runtime.job_index_cnt =
                    (vehicle_path_runtime.job_point_cnt > 0u) ? (vehicle_path_runtime.job_point_cnt - 1u) : 0u;
                vehicle_path_runtime.job_state = VEHICLE_PATH_JOB_PLAN_BACKWARD;
            }
            break;

        case VEHICLE_PATH_JOB_PLAN_BACKWARD:
            if (vehicle_path_plan_backward_step(VEHICLE_PATH_PLAN_PASS_POINTS_PER_RUN) != FALSE)
            {
                vehicle_path_plan_start_speed_apply(vehicle_path_runtime.job_point_cnt);
                vehicle_path_runtime.job_index_cnt = 0u;
                vehicle_path_runtime.job_planned_cnt = 0u;
                vehicle_path_runtime.job_min_speed_mm_s = 0.0f;
                vehicle_path_runtime.job_max_speed_mm_s = 0.0f;
                vehicle_path_runtime.job_state = VEHICLE_PATH_JOB_PLAN_SUMMARY;
            }
            break;

        case VEHICLE_PATH_JOB_PLAN_SUMMARY:
            if (vehicle_path_plan_summary_step(VEHICLE_PATH_PLAN_PASS_POINTS_PER_RUN) != FALSE)
            {
                vehicle_path_runtime.job_state = VEHICLE_PATH_JOB_PLAN_PREPARE_WRITE;
            }
            break;

        case VEHICLE_PATH_JOB_PLAN_PREPARE_WRITE:
            vehicle_path_runtime.job_write_offset = 0u;
            vehicle_path_runtime.job_checksum = vehicle_path_checksum_calculate(
                (const uint8*)vehicle_path_runtime.points,
                vehicle_path_runtime.job_point_cnt * (uint32)sizeof(vehicle_path_point_t));
            vehicle_path_runtime.job_state = VEHICLE_PATH_JOB_PLAN_WRITE_POINTS;
            break;

        case VEHICLE_PATH_JOB_PLAN_WRITE_POINTS:
            if (vehicle_path_plan_write_points_step(VEHICLE_PATH_PLAN_WRITE_PAGES_PER_RUN) == FALSE)
            {
                tools_printf("{pathplan}save_failed\r\n");
                vehicle_path_job_reset();
                return;
            }

            if (vehicle_path_runtime.job_write_offset
                >= (vehicle_path_runtime.job_point_cnt * (uint32)sizeof(vehicle_path_point_t)))
            {
                vehicle_path_runtime.job_state = VEHICLE_PATH_JOB_PLAN_WRITE_HEADER;
            }
            break;

        case VEHICLE_PATH_JOB_PLAN_WRITE_HEADER:
            vehicle_path_plan_markers_remap_to_planned();
            if (vehicle_path_header_write(vehicle_path_runtime.job_write_slot_offset,
                                          vehicle_path_runtime.job_point_cnt,
                                          vehicle_path_runtime.job_checksum) == FALSE)
            {
                tools_printf("{pathplan}save_failed\r\n");
                vehicle_path_job_reset();
                return;
            }

            if (vehicle_path_plan_speed_overlay_activate_from_job() == FALSE)
            {
                tools_printf("{pathplan}raw_activate_failed\r\n");
            }
            (void)vehicle_path_replay_edge_points_refresh();
            (void)vehicle_path_marker_refresh_points();
            tools_printf("{pathplan}done,%u,%.3f,%.3f,%u,%.1f,%.3f\r\n",
                         (unsigned int)vehicle_path_runtime.job_planned_cnt,
                         (double)vehicle_path_runtime.job_min_speed_mm_s,
                         (double)vehicle_path_runtime.job_max_speed_mm_s,
                         (unsigned int)vehicle_path_runtime.planned_speed_source_point_cnt,
                         (double)vehicle_path_runtime.planned_speed_source_length_mm,
                         (double)vehicle_path_runtime.planned_speed_step_mm);
            vehicle_path_job_reset();
            break;

        case VEHICLE_PATH_JOB_PLAN_FIXED_CHUNK_LOAD:
            if (vehicle_path_plan_chunk_load_step(VEHICLE_PATH_PLAN_LONG_RESAMPLE_POINTS_PER_RUN) == FALSE)
            {
                tools_printf("{pathplan}chunk_read_failed,%u\r\n",
                             (unsigned int)vehicle_path_runtime.job_chunk_core_start_cnt);
                vehicle_path_runtime.job_result_valid = TRUE;
                vehicle_path_runtime.job_result_success = FALSE;
                vehicle_path_job_reset();
                return;
            }
            if (vehicle_path_runtime.job_chunk_loaded_count >= vehicle_path_runtime.job_chunk_input_count)
            {
                vehicle_path_runtime.job_state = VEHICLE_PATH_JOB_PLAN_FIXED_CHUNK_APPLY;
            }
            break;

        case VEHICLE_PATH_JOB_PLAN_FIXED_CHUNK_APPLY:
            vehicle_path_plan_chunk_apply();
            vehicle_path_runtime.job_chunk_core_start_cnt += vehicle_path_runtime.job_chunk_core_count;
            if (vehicle_path_runtime.job_chunk_core_start_cnt >= vehicle_path_runtime.job_point_cnt)
            {
                vehicle_path_runtime.job_chunk_core_start_cnt =
                    ((vehicle_path_runtime.job_point_cnt - 1u)
                     / VEHICLE_PATH_PLAN_CHUNK_CORE_POINT_CNT)
                    * VEHICLE_PATH_PLAN_CHUNK_CORE_POINT_CNT;
                vehicle_path_runtime.job_speed_pass_carry_valid = FALSE;
                vehicle_path_plan_chunk_begin(VEHICLE_PATH_PLAN_CHUNK_PASS_REVERSE);
                vehicle_path_runtime.job_state = VEHICLE_PATH_JOB_PLAN_FIXED_REVERSE_LOAD;
            }
            else
            {
                vehicle_path_plan_chunk_begin(VEHICLE_PATH_PLAN_CHUNK_PASS_FORWARD);
                vehicle_path_runtime.job_state = VEHICLE_PATH_JOB_PLAN_FIXED_CHUNK_LOAD;
            }
            break;

        case VEHICLE_PATH_JOB_PLAN_FIXED_REVERSE_LOAD:
            if (vehicle_path_plan_chunk_load_step(VEHICLE_PATH_PLAN_LONG_RESAMPLE_POINTS_PER_RUN) == FALSE)
            {
                tools_printf("{pathplan}chunk_read_failed,%u\r\n",
                             (unsigned int)vehicle_path_runtime.job_chunk_core_start_cnt);
                vehicle_path_runtime.job_result_valid = TRUE;
                vehicle_path_runtime.job_result_success = FALSE;
                vehicle_path_job_reset();
                return;
            }
            if (vehicle_path_runtime.job_chunk_loaded_count >= vehicle_path_runtime.job_chunk_input_count)
            {
                vehicle_path_runtime.job_state = VEHICLE_PATH_JOB_PLAN_FIXED_REVERSE_APPLY;
            }
            break;

        case VEHICLE_PATH_JOB_PLAN_FIXED_REVERSE_APPLY:
            vehicle_path_plan_chunk_apply();
            if (vehicle_path_runtime.job_chunk_core_start_cnt == 0u)
            {
                vehicle_path_runtime.job_planned_cnt = 0u;
                vehicle_path_runtime.job_min_speed_mm_s = 0.0f;
                vehicle_path_runtime.job_max_speed_mm_s = 0.0f;
                vehicle_path_runtime.job_checksum = VEHICLE_PATH_CHECKSUM_SEED;
                vehicle_path_runtime.job_chunk_core_start_cnt = 0u;
                vehicle_path_plan_chunk_begin(VEHICLE_PATH_PLAN_CHUNK_PASS_WRITE);
                vehicle_path_runtime.job_state = VEHICLE_PATH_JOB_PLAN_FIXED_WRITE_LOAD;
            }
            else
            {
                vehicle_path_runtime.job_chunk_core_start_cnt -=
                    VEHICLE_PATH_PLAN_CHUNK_CORE_POINT_CNT;
                vehicle_path_plan_chunk_begin(VEHICLE_PATH_PLAN_CHUNK_PASS_REVERSE);
                vehicle_path_runtime.job_state = VEHICLE_PATH_JOB_PLAN_FIXED_REVERSE_LOAD;
            }
            break;

        case VEHICLE_PATH_JOB_PLAN_FIXED_WRITE_LOAD:
            if (vehicle_path_plan_chunk_load_step(VEHICLE_PATH_PLAN_LONG_RESAMPLE_POINTS_PER_RUN) == FALSE)
            {
                tools_printf("{pathplan}chunk_read_failed,%u\r\n",
                             (unsigned int)vehicle_path_runtime.job_chunk_core_start_cnt);
                vehicle_path_runtime.job_result_valid = TRUE;
                vehicle_path_runtime.job_result_success = FALSE;
                vehicle_path_job_reset();
                return;
            }
            if (vehicle_path_runtime.job_chunk_loaded_count >= vehicle_path_runtime.job_chunk_input_count)
            {
                vehicle_path_runtime.job_state = VEHICLE_PATH_JOB_PLAN_FIXED_WRITE_APPLY;
            }
            break;

        case VEHICLE_PATH_JOB_PLAN_FIXED_WRITE_APPLY:
            vehicle_path_plan_chunk_apply();
            vehicle_path_runtime.job_state = VEHICLE_PATH_JOB_PLAN_FIXED_WRITE_PAGES;
            break;

        case VEHICLE_PATH_JOB_PLAN_FIXED_WRITE_PAGES:
            if (vehicle_path_plan_chunk_write_step(VEHICLE_PATH_PLAN_WRITE_PAGES_PER_RUN) == FALSE)
            {
                tools_printf("{pathplan}save_failed\r\n");
                vehicle_path_runtime.job_result_valid = TRUE;
                vehicle_path_runtime.job_result_success = FALSE;
                vehicle_path_job_reset();
                return;
            }
            if (vehicle_path_runtime.job_chunk_write_offset_cnt >= vehicle_path_runtime.job_chunk_core_count)
            {
                vehicle_path_runtime.job_chunk_core_start_cnt += vehicle_path_runtime.job_chunk_core_count;
                if (vehicle_path_runtime.job_chunk_core_start_cnt >= vehicle_path_runtime.job_point_cnt)
                {
                    if (vehicle_path_plan_fixed_finish() == FALSE)
                    {
                        tools_printf("{pathplan}finish_failed\r\n");
                        vehicle_path_runtime.job_result_valid = TRUE;
                        vehicle_path_runtime.job_result_success = FALSE;
                    }
                    else
                    {
                        vehicle_path_runtime.job_result_valid = TRUE;
                        vehicle_path_runtime.job_result_success = TRUE;
                    }
                    vehicle_path_job_reset();
                }
                else
                {
                    vehicle_path_plan_chunk_begin(VEHICLE_PATH_PLAN_CHUNK_PASS_WRITE);
                    vehicle_path_runtime.job_state = VEHICLE_PATH_JOB_PLAN_FIXED_WRITE_LOAD;
                }
            }
            break;

        default:
            vehicle_path_job_reset();
            break;
    }
}

static boolean vehicle_path_plan_load_points_step(uint32 max_point_count)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();
    uint32 processed_count = 0u;

    while ((processed_count < max_point_count)
           && (vehicle_path_runtime.job_index_cnt < vehicle_path_runtime.job_point_cnt))
    {
        uint32 remaining_count = vehicle_path_runtime.job_point_cnt - vehicle_path_runtime.job_index_cnt;
        uint32 read_count = max_point_count - processed_count;
        uint32 read_length;
        uint32 read_offset;
        uint32 local_index_cnt;

        if (read_count > remaining_count)
        {
            read_count = remaining_count;
        }

        read_length = read_count * (uint32)sizeof(vehicle_path_point_t);
        read_offset = cfg->flash_offset
                    + vehicle_path_runtime.job_read_slot_offset
                    + VEHICLE_PATH_FLASH_POINT_OFFSET
                    + (vehicle_path_runtime.job_index_cnt * (uint32)sizeof(vehicle_path_point_t));

        if (service_storage_readSync(SERVICE_STORAGE_1,
                                     read_offset,
                                     (uint8*)&vehicle_path_runtime.points[vehicle_path_runtime.job_index_cnt],
                                     read_length) != read_length)
        {
            tools_printf("{pathplan}read_failed,%u\r\n",
                         (unsigned int)vehicle_path_runtime.job_index_cnt);
            return FALSE;
        }

        vehicle_path_runtime.job_checksum =
            vehicle_path_checksum_update(vehicle_path_runtime.job_checksum,
                                         (const uint8*)&vehicle_path_runtime.points[vehicle_path_runtime.job_index_cnt],
                                         read_length);

        for (local_index_cnt = 0u; local_index_cnt < read_count; local_index_cnt++)
        {
            vehicle_path_runtime.points[vehicle_path_runtime.job_index_cnt + local_index_cnt].speed_mm_s =
                vehicle_path_cfg_get()->plan_max_speed_mm_s;
        }

        vehicle_path_runtime.job_index_cnt += read_count;
        processed_count += read_count;
    }

    return TRUE;
}

static boolean vehicle_path_plan_source_point_read(uint32 index_cnt, vehicle_path_point_t* point)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();
    uint32 read_offset;

    if ((point == NULL_PTR) || (index_cnt >= vehicle_path_runtime.job_source_point_cnt))
    {
        return FALSE;
    }

    read_offset = cfg->flash_offset
                + vehicle_path_runtime.job_read_slot_offset
                + VEHICLE_PATH_FLASH_POINT_OFFSET
                + (index_cnt * (uint32)sizeof(vehicle_path_point_t));

    return (service_storage_readSync(SERVICE_STORAGE_1,
                                     read_offset,
                                     (uint8*)point,
                                     (uint32)sizeof(vehicle_path_point_t))
            == (uint32)sizeof(vehicle_path_point_t)) ? TRUE : FALSE;
}

static boolean vehicle_path_plan_long_scan_step(uint32 max_point_count)
{
    uint32 processed_count = 0u;

    while ((processed_count < max_point_count)
           && (vehicle_path_runtime.job_index_cnt < vehicle_path_runtime.job_source_point_cnt))
    {
        vehicle_path_point_t point;

        if (vehicle_path_plan_source_point_read(vehicle_path_runtime.job_index_cnt, &point) == FALSE)
        {
            tools_printf("{pathplan}read_failed,%u\r\n",
                         (unsigned int)vehicle_path_runtime.job_index_cnt);
            return FALSE;
        }

        vehicle_path_runtime.job_checksum =
            vehicle_path_checksum_update(vehicle_path_runtime.job_checksum,
                                         (const uint8*)&point,
                                         (uint32)sizeof(point));

        if (vehicle_path_runtime.job_has_last_point != FALSE)
        {
            float32 dx = point.x_mm - vehicle_path_runtime.job_last_point.x_mm;
            float32 dy = point.y_mm - vehicle_path_runtime.job_last_point.y_mm;

            vehicle_path_runtime.job_path_length_mm += sqrtf((dx * dx) + (dy * dy));
        }

        {
            uint32 marker_cnt;
            for (marker_cnt = 0u; marker_cnt < vehicle_path_runtime.marker_cnt; marker_cnt++)
            {
                const vehicle_path_marker_t* marker = &vehicle_path_runtime.markers[marker_cnt];
                if ((marker->valid != FALSE)
                    && (marker->index_cnt == vehicle_path_runtime.job_index_cnt))
                {
                    vehicle_path_runtime.job_marker_source_distance_mm[marker_cnt] =
                        vehicle_path_runtime.job_path_length_mm;
                    vehicle_path_runtime.job_marker_source_distance_valid[marker_cnt] = TRUE;
                }
            }
        }

        vehicle_path_runtime.job_last_point = point;
        vehicle_path_runtime.job_has_last_point = TRUE;
        vehicle_path_runtime.job_index_cnt++;
        processed_count++;
    }

    return TRUE;
}

static boolean vehicle_path_plan_long_resample_begin(void)
{
    float32 dx;
    float32 dy;

    if ((vehicle_path_runtime.job_source_point_cnt < 2u)
        || (vehicle_path_runtime.job_point_cnt < 2u)
        || (vehicle_path_runtime.job_path_length_mm <= VEHICLE_PATH_PLAN_RESAMPLE_EPSILON_MM))
    {
        return FALSE;
    }

    if (vehicle_path_plan_source_point_read(0u, &vehicle_path_runtime.job_resample_prev_point) == FALSE)
    {
        return FALSE;
    }

    if (vehicle_path_plan_source_point_read(1u, &vehicle_path_runtime.job_resample_next_point) == FALSE)
    {
        return FALSE;
    }

    dx = vehicle_path_runtime.job_resample_next_point.x_mm
       - vehicle_path_runtime.job_resample_prev_point.x_mm;
    dy = vehicle_path_runtime.job_resample_next_point.y_mm
       - vehicle_path_runtime.job_resample_prev_point.y_mm;

    vehicle_path_runtime.job_index_cnt = 0u;
    vehicle_path_runtime.job_resample_next_index_cnt = 1u;
    vehicle_path_runtime.job_resample_step_mm =
        vehicle_path_runtime.job_path_length_mm / (float32)(vehicle_path_runtime.job_point_cnt - 1u);
    vehicle_path_runtime.job_resample_segment_start_mm = 0.0f;
    vehicle_path_runtime.job_resample_segment_end_mm = sqrtf((dx * dx) + (dy * dy));
    return TRUE;
}

static boolean vehicle_path_plan_long_resample_step(uint32 max_point_count)
{
    uint32 processed_count = 0u;

    while ((processed_count < max_point_count)
           && (vehicle_path_runtime.job_index_cnt < vehicle_path_runtime.job_point_cnt))
    {
        vehicle_path_point_t point;
        float32 target_distance_mm =
            vehicle_path_runtime.job_resample_step_mm * (float32)vehicle_path_runtime.job_index_cnt;

        if ((vehicle_path_runtime.job_index_cnt + 1u) >= vehicle_path_runtime.job_point_cnt)
        {
            target_distance_mm = vehicle_path_runtime.job_path_length_mm;
        }

        while (((target_distance_mm - vehicle_path_runtime.job_resample_segment_end_mm)
                > VEHICLE_PATH_PLAN_RESAMPLE_EPSILON_MM)
               && ((vehicle_path_runtime.job_resample_next_index_cnt + 1u)
                   < vehicle_path_runtime.job_source_point_cnt))
        {
            float32 dx;
            float32 dy;

            vehicle_path_runtime.job_resample_prev_point =
                vehicle_path_runtime.job_resample_next_point;
            vehicle_path_runtime.job_resample_segment_start_mm =
                vehicle_path_runtime.job_resample_segment_end_mm;
            vehicle_path_runtime.job_resample_next_index_cnt++;

            if (vehicle_path_plan_source_point_read(vehicle_path_runtime.job_resample_next_index_cnt,
                                                    &vehicle_path_runtime.job_resample_next_point) == FALSE)
            {
                tools_printf("{pathplan}read_failed,%u\r\n",
                             (unsigned int)vehicle_path_runtime.job_resample_next_index_cnt);
                return FALSE;
            }

            dx = vehicle_path_runtime.job_resample_next_point.x_mm
               - vehicle_path_runtime.job_resample_prev_point.x_mm;
            dy = vehicle_path_runtime.job_resample_next_point.y_mm
               - vehicle_path_runtime.job_resample_prev_point.y_mm;
            vehicle_path_runtime.job_resample_segment_end_mm += sqrtf((dx * dx) + (dy * dy));
        }

        if (vehicle_path_runtime.job_index_cnt == 0u)
        {
            point = vehicle_path_runtime.job_resample_prev_point;
        }
        else
        {
            float32 segment_length_mm = vehicle_path_runtime.job_resample_segment_end_mm
                                      - vehicle_path_runtime.job_resample_segment_start_mm;
            float32 ratio = 0.0f;

            if (segment_length_mm > VEHICLE_PATH_PLAN_RESAMPLE_EPSILON_MM)
            {
                ratio = (target_distance_mm - vehicle_path_runtime.job_resample_segment_start_mm)
                      / segment_length_mm;
            }

            if (ratio < 0.0f)
            {
                ratio = 0.0f;
            }
            if (ratio > 1.0f)
            {
                ratio = 1.0f;
            }

            point.x_mm = vehicle_path_runtime.job_resample_prev_point.x_mm
                       + ((vehicle_path_runtime.job_resample_next_point.x_mm
                           - vehicle_path_runtime.job_resample_prev_point.x_mm) * ratio);
            point.y_mm = vehicle_path_runtime.job_resample_prev_point.y_mm
                       + ((vehicle_path_runtime.job_resample_next_point.y_mm
                           - vehicle_path_runtime.job_resample_prev_point.y_mm) * ratio);
            point.theta_rad = vehicle_path_runtime.job_resample_prev_point.theta_rad
                            + (vehicle_path_wrap_pi(vehicle_path_runtime.job_resample_next_point.theta_rad
                                                   - vehicle_path_runtime.job_resample_prev_point.theta_rad)
                               * ratio);
        }

        point.speed_mm_s = vehicle_path_cfg_get()->plan_max_speed_mm_s;
        vehicle_path_runtime.points[vehicle_path_runtime.job_index_cnt] = point;
        vehicle_path_runtime.job_index_cnt++;
        processed_count++;
    }

    return TRUE;
}

static void vehicle_path_plan_markers_remap_to_planned(void)
{
    uint32 marker_cnt;

    if ((vehicle_path_runtime.job_downsampled == FALSE)
        || (vehicle_path_runtime.marker_cnt == 0u)
        || (vehicle_path_runtime.job_point_cnt == 0u))
    {
        return;
    }

    for (marker_cnt = 0u; marker_cnt < vehicle_path_runtime.marker_cnt; marker_cnt++)
    {
        vehicle_path_marker_t* marker = &vehicle_path_runtime.markers[marker_cnt];
        uint32 planned_index_cnt;
        uint32 read_offset;

        if ((marker->valid == FALSE)
            || (vehicle_path_marker_kind_valid(marker->kind) == FALSE)
            || (vehicle_path_runtime.job_marker_source_distance_valid[marker_cnt] == FALSE))
        {
            continue;
        }

        if (vehicle_path_runtime.job_resample_step_mm > VEHICLE_PATH_PLAN_RESAMPLE_EPSILON_MM)
        {
            planned_index_cnt = (uint32)((vehicle_path_runtime.job_marker_source_distance_mm[marker_cnt]
                                        / vehicle_path_runtime.job_resample_step_mm) + 0.5f);
        }
        else
        {
            planned_index_cnt = 0u;
        }
        if (planned_index_cnt >= vehicle_path_runtime.job_point_cnt)
        {
            planned_index_cnt = vehicle_path_runtime.job_point_cnt - 1u;
        }

        read_offset = vehicle_path_cfg_get()->flash_offset
                    + VEHICLE_PATH_FLASH_PLANNED_OFFSET
                    + VEHICLE_PATH_FLASH_POINT_OFFSET
                    + (planned_index_cnt * (uint32)sizeof(vehicle_path_point_t));
        if (service_storage_readSync(SERVICE_STORAGE_1,
                                     read_offset,
                                     (uint8*)&marker->point,
                                     (uint32)sizeof(marker->point)) == (uint32)sizeof(marker->point))
        {
            marker->index_cnt = planned_index_cnt;
            marker->valid = TRUE;
        }
        else
        {
            marker->valid = FALSE;
        }
    }

    vehicle_path_marker_sort();
}

static void vehicle_path_plan_geometry_compute(uint32 point_cnt, boolean apply_start_speed)
{
#if VEHICLE_PATH_ONBOARD_GEOMETRY_ENABLE
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();
    uint32 index_cnt;
    boolean corridor_available = (point_cnt <= VEHICLE_PATH_PLAN_CHUNK_MAX_POINT_CNT) ? TRUE : FALSE;

    if (corridor_available != FALSE)
    {
        for (index_cnt = 0u; index_cnt < point_cnt; index_cnt++)
        {
            vehicle_path_runtime.plan_source_x_mm[index_cnt] = vehicle_path_runtime.points[index_cnt].x_mm;
            vehicle_path_runtime.plan_source_y_mm[index_cnt] = vehicle_path_runtime.points[index_cnt].y_mm;
        }
    }

    if (cfg->plan_geometry_enable != FALSE)
    {
        vehicle_path_plan_smooth_points(point_cnt);
        if (cfg->plan_bspline_enable != FALSE)
        {
            vehicle_path_plan_bspline_apply(point_cnt);
        }
        if (cfg->plan_clothoid_enable != FALSE)
        {
            vehicle_path_plan_clothoid_apply(point_cnt);
        }
        if (cfg->plan_curvature_smooth_enable != FALSE)
        {
            vehicle_path_plan_curvature_smooth_apply(point_cnt);
        }
        if (cfg->plan_radius_lock_enable != FALSE)
        {
            vehicle_path_plan_radius_lock_apply(point_cnt);
        }
        vehicle_path_recalculate_theta(point_cnt);

        if (corridor_available != FALSE)
        {
            for (index_cnt = 0u; index_cnt < point_cnt; index_cnt++)
            {
                float32 dx = vehicle_path_runtime.points[index_cnt].x_mm
                           - vehicle_path_runtime.plan_source_x_mm[index_cnt];
                float32 dy = vehicle_path_runtime.points[index_cnt].y_mm
                           - vehicle_path_runtime.plan_source_y_mm[index_cnt];
                float32 offset_mm = sqrtf((dx * dx) + (dy * dy));

                if (offset_mm > VEHICLE_PATH_PLAN_GEOMETRY_MAX_OFFSET_MM)
                {
                    float32 scale = VEHICLE_PATH_PLAN_GEOMETRY_MAX_OFFSET_MM / offset_mm;
                    vehicle_path_runtime.points[index_cnt].x_mm =
                        vehicle_path_runtime.plan_source_x_mm[index_cnt] + (dx * scale);
                    vehicle_path_runtime.points[index_cnt].y_mm =
                        vehicle_path_runtime.plan_source_y_mm[index_cnt] + (dy * scale);
                }
            }
            vehicle_path_recalculate_theta(point_cnt);
        }
    }
#endif

    vehicle_path_plan_curve_speed_apply(point_cnt);
    if (apply_start_speed != FALSE)
    {
        vehicle_path_plan_start_speed_apply(point_cnt);
    }
}

static boolean vehicle_path_plan_write_points_step(uint32 max_page_count)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();
    uint32 point_length = vehicle_path_runtime.job_point_cnt * (uint32)sizeof(vehicle_path_point_t);
    uint32 page_count = 0u;
    uint32 i;

    while ((page_count < max_page_count) && (vehicle_path_runtime.job_write_offset < point_length))
    {
        uint32 page_valid_length = point_length - vehicle_path_runtime.job_write_offset;
        uint32 write_offset;
        uint32 write_length;

        if (page_valid_length > IFXFLASH_PFLASH_PAGE_LENGTH)
        {
            page_valid_length = IFXFLASH_PFLASH_PAGE_LENGTH;
        }

        for (i = 0u; i < IFXFLASH_PFLASH_PAGE_LENGTH; i++)
        {
            vehicle_path_runtime.job_page[i] = 0xFFu;
        }

        for (i = 0u; i < page_valid_length; i++)
        {
            vehicle_path_runtime.job_page[i] =
                ((const uint8*)vehicle_path_runtime.points)[vehicle_path_runtime.job_write_offset + i];
        }

        write_offset = cfg->flash_offset
                     + vehicle_path_runtime.job_write_slot_offset
                     + VEHICLE_PATH_FLASH_POINT_OFFSET
                     + vehicle_path_runtime.job_write_offset;
        write_length = service_storage_writeSync(SERVICE_STORAGE_1,
                                                 write_offset,
                                                 vehicle_path_runtime.job_page,
                                                 IFXFLASH_PFLASH_PAGE_LENGTH);
        if (write_length != IFXFLASH_PFLASH_PAGE_LENGTH)
        {
            return FALSE;
        }

        vehicle_path_runtime.job_write_offset += page_valid_length;
        page_count++;
    }

    return TRUE;
}

#if VEHICLE_PATH_ONBOARD_GEOMETRY_ENABLE
static void vehicle_path_recalculate_theta(uint32 point_cnt)
{
    uint32 index_cnt;

    if (point_cnt < 2u)
    {
        return;
    }

    for (index_cnt = 0u; index_cnt < (point_cnt - 1u); index_cnt++)
    {
        float32 dx = vehicle_path_runtime.points[index_cnt + 1u].x_mm
                   - vehicle_path_runtime.points[index_cnt].x_mm;
        float32 dy = vehicle_path_runtime.points[index_cnt + 1u].y_mm
                   - vehicle_path_runtime.points[index_cnt].y_mm;

        if (((dx * dx) + (dy * dy)) > 0.001f)
        {
            vehicle_path_runtime.points[index_cnt].theta_rad = atan2f(dy, dx);
        }
        else if (index_cnt > 0u)
        {
            vehicle_path_runtime.points[index_cnt].theta_rad =
                vehicle_path_runtime.points[index_cnt - 1u].theta_rad;
        }
    }

    vehicle_path_runtime.points[point_cnt - 1u].theta_rad =
        vehicle_path_runtime.points[point_cnt - 2u].theta_rad;
}
#endif

static boolean vehicle_path_header_write(uint32 slot_offset, uint32 point_cnt, uint32 checksum)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();
    vehicle_path_flash_header_t header;
    uint8 header_page[VEHICLE_PATH_FLASH_HEADER_LENGTH];
    uint32 write_length;

    if (vehicle_path_flash_slot_range_valid(slot_offset, point_cnt) == FALSE)
    {
        return FALSE;
    }

    header.magic = VEHICLE_PATH_MAGIC;
    header.version = VEHICLE_PATH_VERSION;
    header.point_cnt = (uint16)point_cnt;
    header.checksum = checksum;

    vehicle_path_header_page_prepare(header_page, &header);

    /* Commit metadata first. The valid header is the final atomic commit marker. */
    if (vehicle_path_marker_write(slot_offset, point_cnt) == FALSE)
    {
        return FALSE;
    }

    write_length = service_storage_writeSync(SERVICE_STORAGE_1,
                                             cfg->flash_offset + slot_offset,
                                             header_page,
                                             VEHICLE_PATH_FLASH_HEADER_LENGTH);

    if (write_length != VEHICLE_PATH_FLASH_HEADER_LENGTH)
    {
        return FALSE;
    }

    return TRUE;
}

static vehicle_path_point_t* vehicle_path_window_buffer_get(uint32 buffer_index_cnt)
{
    uint32 point_offset_cnt = 0u;

    if (buffer_index_cnt != 0u)
    {
        point_offset_cnt = VEHICLE_PATH_WINDOW_BUFFER_POINT_CNT;
    }

    return &vehicle_path_runtime.points[point_offset_cnt];
}

static uint32 vehicle_path_window_inactive_buffer_get(void)
{
    return (vehicle_path_runtime.window_active_buffer_cnt == 0u) ? 1u : 0u;
}

static void vehicle_path_window_buffers_reset(void)
{
    uint32 buffer_index_cnt;

    for (buffer_index_cnt = 0u; buffer_index_cnt < VEHICLE_PATH_WINDOW_BUFFER_COUNT; buffer_index_cnt++)
    {
        vehicle_path_runtime.window_buffer_start_index_cnt[buffer_index_cnt] = 0u;
        vehicle_path_runtime.window_buffer_valid_cnt[buffer_index_cnt] = 0u;
        vehicle_path_runtime.window_buffer_loaded[buffer_index_cnt] = FALSE;
    }
}

static void vehicle_path_window_buffer_invalidate(uint32 buffer_index_cnt)
{
    if (buffer_index_cnt >= VEHICLE_PATH_WINDOW_BUFFER_COUNT)
    {
        return;
    }

    vehicle_path_runtime.window_buffer_start_index_cnt[buffer_index_cnt] = 0u;
    vehicle_path_runtime.window_buffer_valid_cnt[buffer_index_cnt] = 0u;
    vehicle_path_runtime.window_buffer_loaded[buffer_index_cnt] = FALSE;
}

static void vehicle_path_window_buffer_mark(uint32 buffer_index_cnt,
                                            uint32 start_index_cnt,
                                            uint32 valid_cnt)
{
    if (buffer_index_cnt >= VEHICLE_PATH_WINDOW_BUFFER_COUNT)
    {
        return;
    }

    vehicle_path_runtime.window_buffer_start_index_cnt[buffer_index_cnt] = start_index_cnt;
    vehicle_path_runtime.window_buffer_valid_cnt[buffer_index_cnt] = valid_cnt;
    vehicle_path_runtime.window_buffer_loaded[buffer_index_cnt] = (valid_cnt > 0u) ? TRUE : FALSE;
}

static boolean vehicle_path_window_buffer_contains(uint32 buffer_index_cnt, uint32 index_cnt)
{
    uint32 window_end_index_cnt;

    if ((buffer_index_cnt >= VEHICLE_PATH_WINDOW_BUFFER_COUNT)
        || (vehicle_path_runtime.window_buffer_loaded[buffer_index_cnt] == FALSE))
    {
        return FALSE;
    }

    window_end_index_cnt = vehicle_path_runtime.window_buffer_start_index_cnt[buffer_index_cnt]
                         + vehicle_path_runtime.window_buffer_valid_cnt[buffer_index_cnt];

    return (boolean)((index_cnt >= vehicle_path_runtime.window_buffer_start_index_cnt[buffer_index_cnt])
                  && (index_cnt < window_end_index_cnt));
}

static boolean vehicle_path_window_buffer_read(uint32 buffer_index_cnt,
                                               uint32 index_cnt,
                                               vehicle_path_point_t* point)
{
    uint32 local_index_cnt;
    vehicle_path_point_t* buffer;

    if ((point == NULL_PTR) || (vehicle_path_window_buffer_contains(buffer_index_cnt, index_cnt) == FALSE))
    {
        return FALSE;
    }

    local_index_cnt = index_cnt - vehicle_path_runtime.window_buffer_start_index_cnt[buffer_index_cnt];
    buffer = vehicle_path_window_buffer_get(buffer_index_cnt);
    *point = buffer[local_index_cnt];
    return TRUE;
}

static void vehicle_path_window_active_set(uint32 buffer_index_cnt)
{
    if (buffer_index_cnt >= VEHICLE_PATH_WINDOW_BUFFER_COUNT)
    {
        buffer_index_cnt = 0u;
    }

    vehicle_path_runtime.window_active_buffer_cnt = buffer_index_cnt;
    if (vehicle_path_runtime.window_buffer_loaded[buffer_index_cnt] != FALSE)
    {
        vehicle_path_runtime.window_start_index_cnt =
            vehicle_path_runtime.window_buffer_start_index_cnt[buffer_index_cnt];
        vehicle_path_runtime.window_valid_cnt =
            vehicle_path_runtime.window_buffer_valid_cnt[buffer_index_cnt];
    }
    else
    {
        vehicle_path_runtime.window_start_index_cnt = 0u;
        vehicle_path_runtime.window_valid_cnt = 0u;
    }
}

static void vehicle_path_window_prefetch_reset(void)
{
    vehicle_path_runtime.window_prefetch_start_index_cnt = 0u;
    vehicle_path_runtime.window_prefetch_valid_cnt = 0u;
    vehicle_path_runtime.window_prefetch_buffer_cnt = vehicle_path_window_inactive_buffer_get();
    vehicle_path_runtime.window_prefetch_request_start_index_cnt = 0u;
    vehicle_path_runtime.window_prefetch_valid = FALSE;
    vehicle_path_runtime.window_prefetch_pending = FALSE;
}

static boolean vehicle_path_window_read_to_buffer(uint32 start_index_cnt,
                                                  uint32 buffer_index_cnt,
                                                  uint32* read_count)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();
    uint32 point_count = vehicle_path_runtime.state.point_cnt;
    uint32 local_read_count;
    uint32 read_length;
    uint32 read_offset;
    vehicle_path_point_t* buffer;

    if (read_count != NULL_PTR)
    {
        *read_count = 0u;
    }

    if ((point_count == 0u) || (start_index_cnt >= point_count))
    {
        return FALSE;
    }

    if (buffer_index_cnt >= VEHICLE_PATH_WINDOW_BUFFER_COUNT)
    {
        return FALSE;
    }

    local_read_count = point_count - start_index_cnt;
    if (local_read_count > VEHICLE_PATH_WINDOW_BUFFER_POINT_CNT)
    {
        local_read_count = VEHICLE_PATH_WINDOW_BUFFER_POINT_CNT;
    }

    read_length = local_read_count * (uint32)sizeof(vehicle_path_point_t);
    read_offset = cfg->flash_offset
                + vehicle_path_runtime.active_slot_offset
                + VEHICLE_PATH_FLASH_POINT_OFFSET
                + (start_index_cnt * (uint32)sizeof(vehicle_path_point_t));
    buffer = vehicle_path_window_buffer_get(buffer_index_cnt);
    vehicle_path_window_buffer_invalidate(buffer_index_cnt);

    if (service_storage_readSync(SERVICE_STORAGE_1,
                                 read_offset,
                                 (uint8*)buffer,
                                 read_length) != read_length)
    {
        vehicle_path_window_buffer_invalidate(buffer_index_cnt);
        return FALSE;
    }

    vehicle_path_window_buffer_mark(buffer_index_cnt, start_index_cnt, local_read_count);

    if (read_count != NULL_PTR)
    {
        *read_count = local_read_count;
    }

    return TRUE;
}

static void vehicle_path_window_prefetch_schedule(uint32 start_index_cnt)
{
    uint32 buffer_index_cnt;
    uint32 point_count = vehicle_path_runtime.state.point_cnt;

    if ((vehicle_path_runtime.window_from_flash == FALSE)
        || (point_count <= VEHICLE_PATH_WINDOW_BUFFER_POINT_CNT)
        || (start_index_cnt >= point_count))
    {
        return;
    }

    if ((vehicle_path_runtime.window_prefetch_valid != FALSE)
        && (vehicle_path_runtime.window_prefetch_start_index_cnt == start_index_cnt)
        && (vehicle_path_runtime.window_prefetch_buffer_cnt < VEHICLE_PATH_WINDOW_BUFFER_COUNT)
        && (vehicle_path_runtime.window_buffer_loaded[vehicle_path_runtime.window_prefetch_buffer_cnt] != FALSE)
        && (vehicle_path_runtime.window_buffer_start_index_cnt[vehicle_path_runtime.window_prefetch_buffer_cnt]
            == start_index_cnt))
    {
        return;
    }

    if ((vehicle_path_runtime.window_prefetch_pending != FALSE)
        && (vehicle_path_runtime.window_prefetch_request_start_index_cnt == start_index_cnt))
    {
        return;
    }

    for (buffer_index_cnt = 0u;
         buffer_index_cnt < VEHICLE_PATH_WINDOW_BUFFER_COUNT;
         buffer_index_cnt++)
    {
        if ((vehicle_path_runtime.window_buffer_loaded[buffer_index_cnt] != FALSE)
            && (vehicle_path_runtime.window_buffer_start_index_cnt[buffer_index_cnt]
                == start_index_cnt))
        {
            if (buffer_index_cnt == vehicle_path_runtime.window_active_buffer_cnt)
            {
                vehicle_path_window_prefetch_reset();
                return;
            }

            vehicle_path_runtime.window_prefetch_start_index_cnt = start_index_cnt;
            vehicle_path_runtime.window_prefetch_valid_cnt =
                vehicle_path_runtime.window_buffer_valid_cnt[buffer_index_cnt];
            vehicle_path_runtime.window_prefetch_buffer_cnt = buffer_index_cnt;
            vehicle_path_runtime.window_prefetch_request_start_index_cnt = start_index_cnt;
            vehicle_path_runtime.window_prefetch_valid = TRUE;
            vehicle_path_runtime.window_prefetch_pending = FALSE;
            return;
        }
    }

    vehicle_path_runtime.window_prefetch_request_start_index_cnt = start_index_cnt;
    vehicle_path_runtime.window_prefetch_pending = TRUE;
}

static void vehicle_path_window_prefetch_update(uint32 high_index_cnt)
{
    uint32 point_count = vehicle_path_runtime.state.point_cnt;
    uint32 active_buffer_cnt = vehicle_path_runtime.window_active_buffer_cnt;
    uint32 window_end_index_cnt;
    uint32 next_start_index_cnt;

    if ((vehicle_path_runtime.window_from_flash == FALSE)
        || (active_buffer_cnt >= VEHICLE_PATH_WINDOW_BUFFER_COUNT)
        || (vehicle_path_runtime.window_buffer_loaded[active_buffer_cnt] == FALSE)
        || (point_count <= VEHICLE_PATH_WINDOW_BUFFER_POINT_CNT))
    {
        return;
    }

    if (high_index_cnt >= point_count)
    {
        high_index_cnt = point_count - 1u;
    }

    window_end_index_cnt = vehicle_path_runtime.window_buffer_start_index_cnt[active_buffer_cnt]
                         + vehicle_path_runtime.window_buffer_valid_cnt[active_buffer_cnt];
    if (window_end_index_cnt >= point_count)
    {
        return;
    }

    if ((high_index_cnt + VEHICLE_PATH_WINDOW_PREFETCH_MARGIN_CNT) < window_end_index_cnt)
    {
        return;
    }

    next_start_index_cnt = vehicle_path_window_start_align(window_end_index_cnt);

    vehicle_path_window_prefetch_schedule(next_start_index_cnt);
}

static void vehicle_path_window_prefetch_run(void)
{
    uint32 read_count;
    uint32 buffer_index_cnt;
    uint32 request_start_index_cnt;

    if ((vehicle_path_runtime.window_prefetch_pending == FALSE)
        || (vehicle_path_runtime.state.replaying == FALSE)
        || (vehicle_path_runtime.window_from_flash == FALSE)
        || (vehicle_path_runtime.job_state != VEHICLE_PATH_JOB_IDLE))
    {
        return;
    }

    request_start_index_cnt = vehicle_path_runtime.window_prefetch_request_start_index_cnt;
    buffer_index_cnt = vehicle_path_window_inactive_buffer_get();

    if (vehicle_path_window_read_to_buffer(request_start_index_cnt,
                                           buffer_index_cnt,
                                           &read_count) == FALSE)
    {
        vehicle_path_runtime.window_prefetch_pending = FALSE;
        vehicle_path_runtime.window_prefetch_valid = FALSE;
        vehicle_path_runtime.window_prefetch_valid_cnt = 0u;
        return;
    }

    vehicle_path_runtime.window_prefetch_start_index_cnt = request_start_index_cnt;
    vehicle_path_runtime.window_prefetch_valid_cnt = read_count;
    vehicle_path_runtime.window_prefetch_buffer_cnt = buffer_index_cnt;
    vehicle_path_runtime.window_prefetch_valid = TRUE;
    vehicle_path_runtime.window_prefetch_pending = FALSE;
    tools_printf("{vwin}prefetch,%u,%u,%u\r\n",
                 (unsigned int)buffer_index_cnt,
                 (unsigned int)request_start_index_cnt,
                 (unsigned int)read_count);
}

static boolean vehicle_path_window_prefetch_contains(uint32 index_cnt)
{
    uint32 window_end_index_cnt;

    if ((vehicle_path_runtime.window_prefetch_valid == FALSE)
        || (vehicle_path_runtime.window_prefetch_buffer_cnt >= VEHICLE_PATH_WINDOW_BUFFER_COUNT)
        || (vehicle_path_window_buffer_contains(vehicle_path_runtime.window_prefetch_buffer_cnt,
                                                index_cnt) == FALSE)
        || (vehicle_path_runtime.window_buffer_start_index_cnt[vehicle_path_runtime.window_prefetch_buffer_cnt]
            != vehicle_path_runtime.window_prefetch_start_index_cnt)
        || (vehicle_path_runtime.window_buffer_valid_cnt[vehicle_path_runtime.window_prefetch_buffer_cnt]
            != vehicle_path_runtime.window_prefetch_valid_cnt))
    {
        return FALSE;
    }

    window_end_index_cnt = vehicle_path_runtime.window_prefetch_start_index_cnt
                         + vehicle_path_runtime.window_prefetch_valid_cnt;

    return (boolean)((index_cnt >= vehicle_path_runtime.window_prefetch_start_index_cnt)
                  && (index_cnt < window_end_index_cnt));
}

static boolean vehicle_path_window_prefetch_promote(uint32 index_cnt)
{
    if (vehicle_path_window_prefetch_contains(index_cnt) == FALSE)
    {
        return FALSE;
    }

    vehicle_path_window_active_set(vehicle_path_runtime.window_prefetch_buffer_cnt);
    vehicle_path_runtime.window_prefetch_valid = FALSE;
    vehicle_path_runtime.window_prefetch_pending = FALSE;
    vehicle_path_runtime.window_prefetch_valid_cnt = 0u;
    vehicle_path_runtime.window_prefetch_buffer_cnt = vehicle_path_window_inactive_buffer_get();
    tools_printf("{vwin}promote,%u,%u,%u,%u\r\n",
                 (unsigned int)vehicle_path_runtime.window_active_buffer_cnt,
                 (unsigned int)vehicle_path_runtime.window_start_index_cnt,
                 (unsigned int)vehicle_path_runtime.window_valid_cnt,
                 (unsigned int)index_cnt);
    return TRUE;
}

static boolean vehicle_path_window_load(uint32 start_index_cnt)
{
    uint32 read_count;
    uint32 load_buffer_cnt;

    if (vehicle_path_runtime.state.point_cnt == 0u)
    {
        vehicle_path_runtime.window_start_index_cnt = 0u;
        vehicle_path_runtime.window_valid_cnt = 0u;
        vehicle_path_runtime.window_from_flash = FALSE;
        vehicle_path_window_buffers_reset();
        vehicle_path_window_prefetch_reset();
        return TRUE;
    }

    load_buffer_cnt = vehicle_path_runtime.window_active_buffer_cnt;
    if ((load_buffer_cnt < VEHICLE_PATH_WINDOW_BUFFER_COUNT)
        && (vehicle_path_runtime.window_buffer_loaded[load_buffer_cnt] != FALSE)
        && (vehicle_path_runtime.window_buffer_start_index_cnt[load_buffer_cnt] != start_index_cnt))
    {
        load_buffer_cnt = vehicle_path_window_inactive_buffer_get();
    }

    if (vehicle_path_window_read_to_buffer(start_index_cnt,
                                           load_buffer_cnt,
                                           &read_count) == FALSE)
    {
        vehicle_path_window_active_set(vehicle_path_runtime.window_active_buffer_cnt);
        vehicle_path_window_prefetch_reset();
        return FALSE;
    }

    vehicle_path_window_active_set(load_buffer_cnt);
    vehicle_path_runtime.window_from_flash = TRUE;
    vehicle_path_window_prefetch_reset();
    tools_printf("{vwin}load,%u,%u,%u\r\n",
                 (unsigned int)vehicle_path_runtime.window_active_buffer_cnt,
                 (unsigned int)vehicle_path_runtime.window_start_index_cnt,
                 (unsigned int)read_count);
    return TRUE;
}

static boolean vehicle_path_window_contains(uint32 index_cnt)
{
    return vehicle_path_window_buffer_contains(vehicle_path_runtime.window_active_buffer_cnt, index_cnt);
}

static uint32 vehicle_path_window_start_align(uint32 index_cnt)
{
    uint32 point_count = vehicle_path_runtime.state.point_cnt;
    uint32 window_cnt = VEHICLE_PATH_WINDOW_BUFFER_POINT_CNT;
    uint32 start_index_cnt;

    if (window_cnt == 0u)
    {
        return 0u;
    }

    if ((point_count > 0u) && (index_cnt >= point_count))
    {
        index_cnt = point_count - 1u;
    }

    start_index_cnt = (index_cnt / window_cnt) * window_cnt;
    return start_index_cnt;
}

static boolean vehicle_path_point_read_cached(uint32 index_cnt, vehicle_path_point_t* point)
{
    uint32 buffer_index_cnt;

    if ((point == NULL_PTR) || (index_cnt >= vehicle_path_runtime.state.point_cnt))
    {
        return FALSE;
    }

    if (vehicle_path_runtime.window_from_flash == FALSE)
    {
        if (index_cnt >= vehicle_path_max_point_cnt_get())
        {
            return FALSE;
        }

        *point = vehicle_path_runtime.points[index_cnt];
        return TRUE;
    }

    for (buffer_index_cnt = 0u;
         buffer_index_cnt < VEHICLE_PATH_WINDOW_BUFFER_COUNT;
         buffer_index_cnt++)
    {
        if ((vehicle_path_runtime.window_buffer_loaded[buffer_index_cnt] != FALSE)
            && (vehicle_path_window_buffer_contains(buffer_index_cnt, index_cnt) != FALSE))
        {
            return vehicle_path_window_buffer_read(buffer_index_cnt, index_cnt, point);
        }
    }

    return FALSE;
}

static boolean vehicle_path_point_read_preserve_window(uint32 index_cnt, vehicle_path_point_t* point)
{
    const vehicle_path_cfg_t* cfg;
    uint32 read_offset;

    if ((point == NULL_PTR) || (index_cnt >= vehicle_path_runtime.state.point_cnt))
    {
        return FALSE;
    }

    if (vehicle_path_runtime.window_from_flash == FALSE)
    {
        if (index_cnt >= vehicle_path_max_point_cnt_get())
        {
            return FALSE;
        }

        *point = vehicle_path_runtime.points[index_cnt];
        return TRUE;
    }

    cfg = vehicle_path_cfg_get();
    read_offset = cfg->flash_offset
                + vehicle_path_runtime.active_slot_offset
                + VEHICLE_PATH_FLASH_POINT_OFFSET
                + (index_cnt * (uint32)sizeof(vehicle_path_point_t));

    return (service_storage_readSync(SERVICE_STORAGE_1,
                                     read_offset,
                                     (uint8*)point,
                                     (uint32)sizeof(vehicle_path_point_t))
            == (uint32)sizeof(vehicle_path_point_t)) ? TRUE : FALSE;
}

static boolean vehicle_path_point_read(uint32 index_cnt, vehicle_path_point_t* point)
{
    uint32 window_start_index_cnt;
    uint32 inactive_buffer_cnt;

    if (point == NULL_PTR)
    {
        return FALSE;
    }

    if (index_cnt >= vehicle_path_runtime.state.point_cnt)
    {
        return FALSE;
    }

    if (vehicle_path_runtime.window_from_flash == FALSE)
    {
        if (index_cnt >= vehicle_path_max_point_cnt_get())
        {
            return FALSE;
        }

        *point = vehicle_path_runtime.points[index_cnt];
        return TRUE;
    }

    if (vehicle_path_window_contains(index_cnt) == FALSE)
    {
        if (vehicle_path_window_prefetch_promote(index_cnt) == FALSE)
        {
            inactive_buffer_cnt = vehicle_path_window_inactive_buffer_get();
            if (vehicle_path_window_buffer_read(inactive_buffer_cnt, index_cnt, point) != FALSE)
            {
                uint32 active_start_index_cnt = vehicle_path_runtime.window_start_index_cnt;
                uint32 active_buffer_cnt = vehicle_path_runtime.window_active_buffer_cnt;

                if ((active_buffer_cnt < VEHICLE_PATH_WINDOW_BUFFER_COUNT)
                    && (vehicle_path_runtime.window_buffer_loaded[active_buffer_cnt] != FALSE))
                {
                    active_start_index_cnt = vehicle_path_runtime.window_buffer_start_index_cnt[active_buffer_cnt];
                }

                if (index_cnt >= active_start_index_cnt)
                {
                    vehicle_path_window_active_set(inactive_buffer_cnt);
                    vehicle_path_window_prefetch_reset();
                    tools_printf("{vwin}recover,%u,%u,%u,%u\r\n",
                                 (unsigned int)vehicle_path_runtime.window_active_buffer_cnt,
                                 (unsigned int)vehicle_path_runtime.window_start_index_cnt,
                                 (unsigned int)vehicle_path_runtime.window_valid_cnt,
                                 (unsigned int)index_cnt);
                }
                return TRUE;
            }

            window_start_index_cnt = vehicle_path_window_start_align(index_cnt);

            if (vehicle_path_window_load(window_start_index_cnt) == FALSE)
            {
                return FALSE;
            }
        }
    }

    return vehicle_path_window_buffer_read(vehicle_path_runtime.window_active_buffer_cnt, index_cnt, point);
}

static void vehicle_path_source_distance_cache_reset(void)
{
    vehicle_path_runtime.source_distance_cache_index_cnt = 0u;
    vehicle_path_runtime.source_distance_cache_mm = 0.0f;
    vehicle_path_runtime.source_distance_cache_valid = FALSE;
}

static boolean vehicle_path_distance_point_read(uint32 index_cnt, vehicle_path_point_t* point)
{
    if (vehicle_path_point_read_cached(index_cnt, point) != FALSE)
    {
        return TRUE;
    }

    return vehicle_path_point_read_preserve_window(index_cnt, point);
}

static boolean vehicle_path_distance_at_index_get(float32 index_float, float32* distance_mm)
{
    uint32 point_cnt = vehicle_path_runtime.state.point_cnt;
    uint32 index_cnt;
    float32 ratio;

    if ((distance_mm == NULL_PTR) || (point_cnt == 0u))
    {
        return FALSE;
    }

    if (index_float <= 0.0f)
    {
        *distance_mm = 0.0f;
        return TRUE;
    }
    if (index_float >= (float32)(point_cnt - 1u))
    {
        index_float = (float32)(point_cnt - 1u);
    }

    index_cnt = (uint32)index_float;
    ratio = index_float - (float32)index_cnt;
    if (ratio < 0.0f)
    {
        ratio = 0.0f;
    }
    if (ratio > 1.0f)
    {
        ratio = 1.0f;
    }

    if (vehicle_path_runtime.source_distance_cache_valid == FALSE)
    {
        vehicle_path_runtime.source_distance_cache_index_cnt = 0u;
        vehicle_path_runtime.source_distance_cache_mm = 0.0f;
        vehicle_path_runtime.source_distance_cache_valid = TRUE;
    }

    while (vehicle_path_runtime.source_distance_cache_index_cnt < index_cnt)
    {
        vehicle_path_point_t point_a;
        vehicle_path_point_t point_b;
        uint32 cache_index_cnt = vehicle_path_runtime.source_distance_cache_index_cnt;

        if ((vehicle_path_distance_point_read(cache_index_cnt, &point_a) == FALSE)
            || (vehicle_path_distance_point_read(cache_index_cnt + 1u, &point_b) == FALSE))
        {
            vehicle_path_source_distance_cache_reset();
            return FALSE;
        }

        vehicle_path_runtime.source_distance_cache_mm +=
            vehicle_path_segment_length_mm(&point_a, &point_b);
        vehicle_path_runtime.source_distance_cache_index_cnt++;
    }

    while (vehicle_path_runtime.source_distance_cache_index_cnt > index_cnt)
    {
        vehicle_path_point_t point_a;
        vehicle_path_point_t point_b;
        uint32 cache_index_cnt = vehicle_path_runtime.source_distance_cache_index_cnt;
        float32 segment_length_mm;

        if ((vehicle_path_distance_point_read(cache_index_cnt - 1u, &point_a) == FALSE)
            || (vehicle_path_distance_point_read(cache_index_cnt, &point_b) == FALSE))
        {
            vehicle_path_source_distance_cache_reset();
            return FALSE;
        }

        segment_length_mm = vehicle_path_segment_length_mm(&point_a, &point_b);
        if (segment_length_mm >= vehicle_path_runtime.source_distance_cache_mm)
        {
            vehicle_path_runtime.source_distance_cache_mm = 0.0f;
        }
        else
        {
            vehicle_path_runtime.source_distance_cache_mm -= segment_length_mm;
        }
        vehicle_path_runtime.source_distance_cache_index_cnt--;
    }

    *distance_mm = vehicle_path_runtime.source_distance_cache_mm;
    if ((ratio > 0.0f) && ((index_cnt + 1u) < point_cnt))
    {
        vehicle_path_point_t point_a;
        vehicle_path_point_t point_b;

        if ((vehicle_path_distance_point_read(index_cnt, &point_a) == FALSE)
            || (vehicle_path_distance_point_read(index_cnt + 1u, &point_b) == FALSE))
        {
            return FALSE;
        }

        *distance_mm += vehicle_path_segment_length_mm(&point_a, &point_b) * ratio;
    }

    return TRUE;
}

static boolean vehicle_path_plan_fixed_begin(void)
{
    float32 step_mm = VEHICLE_PATH_DEFAULT_PLAN_SAMPLE_STEP_MM;
    float32 covered_mm;
    uint32 point_cnt;

    if ((vehicle_path_runtime.job_source_point_cnt < 2u)
        || (vehicle_path_runtime.job_path_length_mm <= VEHICLE_PATH_PLAN_RESAMPLE_EPSILON_MM)
        || (step_mm <= VEHICLE_PATH_PLAN_RESAMPLE_EPSILON_MM))
    {
        return FALSE;
    }

    point_cnt = (uint32)(vehicle_path_runtime.job_path_length_mm / step_mm) + 1u;
    covered_mm = (float32)(point_cnt - 1u) * step_mm;
    if ((vehicle_path_runtime.job_path_length_mm - covered_mm) > VEHICLE_PATH_PLAN_RESAMPLE_EPSILON_MM)
    {
        point_cnt++;
    }

    if ((point_cnt > (uint32)65535u)
        || (vehicle_path_flash_slot_range_valid(VEHICLE_PATH_FLASH_PLANNED_OFFSET, point_cnt) == FALSE))
    {
        tools_printf("{pathplan}fixed_capacity,%u,%u,%.1f\r\n",
                     (unsigned int)point_cnt,
                     (unsigned int)vehicle_path_effective_flash_point_cnt_max_get(),
                     (double)step_mm);
        return FALSE;
    }

    vehicle_path_runtime.job_point_cnt = point_cnt;
    vehicle_path_runtime.job_resample_step_mm = step_mm;
    vehicle_path_runtime.job_downsampled = TRUE;
    vehicle_path_runtime.job_chunk_core_start_cnt = 0u;
    vehicle_path_runtime.job_chunk_pass = VEHICLE_PATH_PLAN_CHUNK_PASS_FORWARD;
    vehicle_path_runtime.job_speed_pass_carry_valid = FALSE;
    vehicle_path_plan_chunk_begin(VEHICLE_PATH_PLAN_CHUNK_PASS_FORWARD);
    tools_printf("{pathplan}fixed,%u,%u,%.1f,%.1f\r\n",
                 (unsigned int)vehicle_path_runtime.job_source_point_cnt,
                 (unsigned int)point_cnt,
                 (double)step_mm,
                 (double)vehicle_path_runtime.job_path_length_mm);
    return TRUE;
}

static void vehicle_path_plan_chunk_begin(vehicle_path_plan_chunk_pass_t pass)
{
    uint32 core_start = vehicle_path_runtime.job_chunk_core_start_cnt;
    uint32 chunk_cnt = core_start / VEHICLE_PATH_PLAN_CHUNK_CORE_POINT_CNT;
    uint32 core_end = core_start + VEHICLE_PATH_PLAN_CHUNK_CORE_POINT_CNT;
    uint32 input_start;
    uint32 input_end;

    if (core_end > vehicle_path_runtime.job_point_cnt)
    {
        core_end = vehicle_path_runtime.job_point_cnt;
    }
    input_start = (core_start > VEHICLE_PATH_PLAN_CHUNK_OVERLAP_POINT_CNT)
                    ? (core_start - VEHICLE_PATH_PLAN_CHUNK_OVERLAP_POINT_CNT)
                    : 0u;
    input_end = core_end + VEHICLE_PATH_PLAN_CHUNK_OVERLAP_POINT_CNT;
    if (input_end > vehicle_path_runtime.job_point_cnt)
    {
        input_end = vehicle_path_runtime.job_point_cnt;
    }

    vehicle_path_runtime.job_chunk_core_count = core_end - core_start;
    vehicle_path_runtime.job_chunk_input_start_cnt = input_start;
    vehicle_path_runtime.job_chunk_input_count = input_end - input_start;
    vehicle_path_runtime.job_chunk_core_offset_cnt = core_start - input_start;
    vehicle_path_runtime.job_chunk_loaded_count = 0u;
    vehicle_path_runtime.job_chunk_write_offset_cnt = 0u;
    vehicle_path_runtime.job_chunk_pass = pass;
    vehicle_path_runtime.job_resample_target_index_cnt = input_start;

    if (vehicle_path_plan_chunk_resample_begin(chunk_cnt) == FALSE)
    {
        vehicle_path_runtime.job_chunk_input_count = 0u;
    }
    vehicle_path_runtime.job_resample_step_mm = VEHICLE_PATH_DEFAULT_PLAN_SAMPLE_STEP_MM;
}

static boolean vehicle_path_plan_chunk_resample_begin(uint32 chunk_cnt)
{
    uint32 next_index_cnt;

    if (chunk_cnt >= VEHICLE_PATH_PLAN_MAX_CHUNK_COUNT)
    {
        return FALSE;
    }
    if (chunk_cnt == 0u)
    {
        if (vehicle_path_plan_long_resample_begin() == FALSE)
        {
            return FALSE;
        }
        vehicle_path_runtime.job_chunk_source_next_index_cnt[0u] =
            vehicle_path_runtime.job_resample_next_index_cnt;
        vehicle_path_runtime.job_chunk_source_segment_start_mm[0u] =
            vehicle_path_runtime.job_resample_segment_start_mm;
        vehicle_path_runtime.job_chunk_source_segment_end_mm[0u] =
            vehicle_path_runtime.job_resample_segment_end_mm;
        vehicle_path_runtime.job_chunk_source_valid[0u] = TRUE;
        return TRUE;
    }
    if (vehicle_path_runtime.job_chunk_source_valid[chunk_cnt] == FALSE)
    {
        return FALSE;
    }

    next_index_cnt = vehicle_path_runtime.job_chunk_source_next_index_cnt[chunk_cnt];
    if ((next_index_cnt == 0u)
        || (next_index_cnt >= vehicle_path_runtime.job_source_point_cnt)
        || (vehicle_path_plan_source_point_read(next_index_cnt - 1u,
                                                &vehicle_path_runtime.job_resample_prev_point) == FALSE)
        || (vehicle_path_plan_source_point_read(next_index_cnt,
                                                &vehicle_path_runtime.job_resample_next_point) == FALSE))
    {
        return FALSE;
    }

    vehicle_path_runtime.job_resample_next_index_cnt = next_index_cnt;
    vehicle_path_runtime.job_resample_segment_start_mm =
        vehicle_path_runtime.job_chunk_source_segment_start_mm[chunk_cnt];
    vehicle_path_runtime.job_resample_segment_end_mm =
        vehicle_path_runtime.job_chunk_source_segment_end_mm[chunk_cnt];
    return TRUE;
}

static boolean vehicle_path_plan_chunk_load_step(uint32 max_point_count)
{
    uint32 processed_count = 0u;
    uint32 input_end = vehicle_path_runtime.job_chunk_input_start_cnt
                     + vehicle_path_runtime.job_chunk_input_count;

    if (vehicle_path_runtime.job_chunk_input_count == 0u)
    {
        return FALSE;
    }

    while ((processed_count < max_point_count)
           && (vehicle_path_runtime.job_resample_target_index_cnt < input_end))
    {
        vehicle_path_point_t point;
        uint32 target_index_cnt = vehicle_path_runtime.job_resample_target_index_cnt;
        float32 target_distance_mm = vehicle_path_runtime.job_resample_step_mm * (float32)target_index_cnt;

        if ((target_index_cnt + 1u) >= vehicle_path_runtime.job_point_cnt)
        {
            target_distance_mm = vehicle_path_runtime.job_path_length_mm;
        }

        while (((target_distance_mm - vehicle_path_runtime.job_resample_segment_end_mm)
                > VEHICLE_PATH_PLAN_RESAMPLE_EPSILON_MM)
               && ((vehicle_path_runtime.job_resample_next_index_cnt + 1u)
                   < vehicle_path_runtime.job_source_point_cnt))
        {
            float32 dx;
            float32 dy;

            vehicle_path_runtime.job_resample_prev_point = vehicle_path_runtime.job_resample_next_point;
            vehicle_path_runtime.job_resample_segment_start_mm =
                vehicle_path_runtime.job_resample_segment_end_mm;
            vehicle_path_runtime.job_resample_next_index_cnt++;
            if (vehicle_path_plan_source_point_read(vehicle_path_runtime.job_resample_next_index_cnt,
                                                    &vehicle_path_runtime.job_resample_next_point) == FALSE)
            {
                return FALSE;
            }
            dx = vehicle_path_runtime.job_resample_next_point.x_mm
               - vehicle_path_runtime.job_resample_prev_point.x_mm;
            dy = vehicle_path_runtime.job_resample_next_point.y_mm
               - vehicle_path_runtime.job_resample_prev_point.y_mm;
            vehicle_path_runtime.job_resample_segment_end_mm += sqrtf((dx * dx) + (dy * dy));
        }

        if (vehicle_path_runtime.job_chunk_pass == VEHICLE_PATH_PLAN_CHUNK_PASS_FORWARD)
        {
            uint32 shifted_target_cnt = target_index_cnt
                                      + VEHICLE_PATH_PLAN_CHUNK_OVERLAP_POINT_CNT;
            if ((shifted_target_cnt >= VEHICLE_PATH_PLAN_CHUNK_CORE_POINT_CNT)
                && ((shifted_target_cnt % VEHICLE_PATH_PLAN_CHUNK_CORE_POINT_CNT) == 0u))
            {
                uint32 next_chunk_cnt = shifted_target_cnt
                                      / VEHICLE_PATH_PLAN_CHUNK_CORE_POINT_CNT;
                if (next_chunk_cnt < VEHICLE_PATH_PLAN_MAX_CHUNK_COUNT)
                {
                    vehicle_path_runtime.job_chunk_source_next_index_cnt[next_chunk_cnt] =
                        vehicle_path_runtime.job_resample_next_index_cnt;
                    vehicle_path_runtime.job_chunk_source_segment_start_mm[next_chunk_cnt] =
                        vehicle_path_runtime.job_resample_segment_start_mm;
                    vehicle_path_runtime.job_chunk_source_segment_end_mm[next_chunk_cnt] =
                        vehicle_path_runtime.job_resample_segment_end_mm;
                    vehicle_path_runtime.job_chunk_source_valid[next_chunk_cnt] = TRUE;
                }
            }
        }

        if (target_index_cnt == 0u)
        {
            point = vehicle_path_runtime.job_resample_prev_point;
        }
        else
        {
            float32 segment_length_mm = vehicle_path_runtime.job_resample_segment_end_mm
                                      - vehicle_path_runtime.job_resample_segment_start_mm;
            float32 ratio = 0.0f;
            if (segment_length_mm > VEHICLE_PATH_PLAN_RESAMPLE_EPSILON_MM)
            {
                ratio = (target_distance_mm - vehicle_path_runtime.job_resample_segment_start_mm)
                      / segment_length_mm;
            }
            if (ratio < 0.0f) { ratio = 0.0f; }
            if (ratio > 1.0f) { ratio = 1.0f; }
            point.x_mm = vehicle_path_runtime.job_resample_prev_point.x_mm
                       + ((vehicle_path_runtime.job_resample_next_point.x_mm
                           - vehicle_path_runtime.job_resample_prev_point.x_mm) * ratio);
            point.y_mm = vehicle_path_runtime.job_resample_prev_point.y_mm
                       + ((vehicle_path_runtime.job_resample_next_point.y_mm
                           - vehicle_path_runtime.job_resample_prev_point.y_mm) * ratio);
            point.theta_rad = vehicle_path_runtime.job_resample_prev_point.theta_rad
                            + (vehicle_path_wrap_pi(vehicle_path_runtime.job_resample_next_point.theta_rad
                                                   - vehicle_path_runtime.job_resample_prev_point.theta_rad) * ratio);
        }
        point.speed_mm_s = vehicle_path_cfg_get()->plan_max_speed_mm_s;

        if (target_index_cnt >= vehicle_path_runtime.job_chunk_input_start_cnt)
        {
            uint32 local_cnt = target_index_cnt - vehicle_path_runtime.job_chunk_input_start_cnt;
            vehicle_path_runtime.points[local_cnt] = point;
            vehicle_path_runtime.job_chunk_loaded_count++;
        }
        vehicle_path_runtime.job_resample_target_index_cnt++;
        processed_count++;
    }

    return TRUE;
}

static void vehicle_path_plan_chunk_apply(void)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();
    uint32 local_cnt;
    uint32 input_count = vehicle_path_runtime.job_chunk_input_count;
    uint32 core_count = vehicle_path_runtime.job_chunk_core_count;
    uint32 core_offset = vehicle_path_runtime.job_chunk_core_offset_cnt;
    uint32 chunk_cnt = vehicle_path_runtime.job_chunk_core_start_cnt
                     / VEHICLE_PATH_PLAN_CHUNK_CORE_POINT_CNT;
    boolean apply_start = (vehicle_path_runtime.job_chunk_input_start_cnt == 0u) ? TRUE : FALSE;

    vehicle_path_runtime.window_from_flash = FALSE;
    vehicle_path_runtime.window_start_index_cnt = 0u;
    vehicle_path_runtime.window_valid_cnt = input_count;
    vehicle_path_runtime.state.point_cnt = input_count;
    vehicle_path_plan_geometry_compute(input_count, apply_start);
    vehicle_path_plan_chunk_end_speed_apply();

    if (vehicle_path_runtime.job_chunk_pass == VEHICLE_PATH_PLAN_CHUNK_PASS_FORWARD)
    {
        for (local_cnt = 0u; local_cnt < core_count; local_cnt++)
        {
            uint32 input_cnt = core_offset + local_cnt;
            float32 speed_mm_s = vehicle_path_runtime.points[input_cnt].speed_mm_s;

            if (vehicle_path_runtime.job_speed_pass_carry_valid != FALSE)
            {
                float32 allowed_speed_mm_s = sqrtf(
                    (vehicle_path_runtime.job_speed_pass_carry_mm_s
                     * vehicle_path_runtime.job_speed_pass_carry_mm_s)
                    + (2.0f * cfg->plan_accel_mm_s2
                       * VEHICLE_PATH_DEFAULT_PLAN_SAMPLE_STEP_MM));
                if (speed_mm_s > allowed_speed_mm_s)
                {
                    speed_mm_s = allowed_speed_mm_s;
                }
            }
            if (local_cnt == 0u)
            {
                vehicle_path_runtime.job_chunk_forward_start_speed_mm_s[chunk_cnt] = speed_mm_s;
            }
            vehicle_path_runtime.job_speed_pass_carry_mm_s = speed_mm_s;
            vehicle_path_runtime.job_speed_pass_carry_valid = TRUE;
        }
        return;
    }

    if (core_count == 0u)
    {
        return;
    }

    vehicle_path_runtime.points[core_offset].speed_mm_s =
        vehicle_path_runtime.job_chunk_forward_start_speed_mm_s[chunk_cnt];
    for (local_cnt = 1u; local_cnt < core_count; local_cnt++)
    {
        uint32 input_cnt = core_offset + local_cnt;
        float32 previous_speed_mm_s = vehicle_path_runtime.points[input_cnt - 1u].speed_mm_s;
        float32 allowed_speed_mm_s = sqrtf(
            (previous_speed_mm_s * previous_speed_mm_s)
            + (2.0f * cfg->plan_accel_mm_s2 * VEHICLE_PATH_DEFAULT_PLAN_SAMPLE_STEP_MM));
        if (vehicle_path_runtime.points[input_cnt].speed_mm_s > allowed_speed_mm_s)
        {
            vehicle_path_runtime.points[input_cnt].speed_mm_s = allowed_speed_mm_s;
        }
    }

    if (vehicle_path_runtime.job_chunk_backward_next_valid[chunk_cnt] != FALSE)
    {
        vehicle_path_runtime.job_speed_pass_carry_mm_s =
            vehicle_path_runtime.job_chunk_backward_next_speed_mm_s[chunk_cnt];
        vehicle_path_runtime.job_speed_pass_carry_valid = TRUE;
        local_cnt = core_count;
    }
    else
    {
        local_cnt = core_count - 1u;
        vehicle_path_runtime.job_speed_pass_carry_mm_s =
            vehicle_path_runtime.points[core_offset + local_cnt].speed_mm_s;
        vehicle_path_runtime.job_speed_pass_carry_valid = TRUE;
    }

    while (local_cnt > 0u)
    {
        uint32 input_cnt;
        float32 allowed_speed_mm_s;

        local_cnt--;
        input_cnt = core_offset + local_cnt;
        allowed_speed_mm_s = sqrtf(
            (vehicle_path_runtime.job_speed_pass_carry_mm_s
             * vehicle_path_runtime.job_speed_pass_carry_mm_s)
            + (2.0f * cfg->plan_decel_mm_s2 * VEHICLE_PATH_DEFAULT_PLAN_SAMPLE_STEP_MM));
        if (vehicle_path_runtime.points[input_cnt].speed_mm_s > allowed_speed_mm_s)
        {
            vehicle_path_runtime.points[input_cnt].speed_mm_s = allowed_speed_mm_s;
        }
        vehicle_path_runtime.job_speed_pass_carry_mm_s =
            vehicle_path_runtime.points[input_cnt].speed_mm_s;
    }

    if (vehicle_path_runtime.job_chunk_pass == VEHICLE_PATH_PLAN_CHUNK_PASS_REVERSE)
    {
        if (chunk_cnt > 0u)
        {
            vehicle_path_runtime.job_chunk_backward_next_speed_mm_s[chunk_cnt - 1u] =
                vehicle_path_runtime.points[core_offset].speed_mm_s;
            vehicle_path_runtime.job_chunk_backward_next_valid[chunk_cnt - 1u] = TRUE;
        }
        return;
    }

    for (local_cnt = 0u; local_cnt < core_count; local_cnt++)
    {
        float32 speed_mm_s = vehicle_path_runtime.points[core_offset + local_cnt].speed_mm_s;
        if ((vehicle_path_runtime.job_planned_cnt == 0u)
            || (speed_mm_s < vehicle_path_runtime.job_min_speed_mm_s))
        {
            vehicle_path_runtime.job_min_speed_mm_s = speed_mm_s;
        }
        if ((vehicle_path_runtime.job_planned_cnt == 0u)
            || (speed_mm_s > vehicle_path_runtime.job_max_speed_mm_s))
        {
            vehicle_path_runtime.job_max_speed_mm_s = speed_mm_s;
        }
        vehicle_path_runtime.job_planned_cnt++;
    }
}

static boolean vehicle_path_plan_chunk_write_step(uint32 max_page_count)
{
    const vehicle_path_cfg_t* cfg = vehicle_path_cfg_get();
    uint32 points_per_page = IFXFLASH_PFLASH_PAGE_LENGTH / (uint32)sizeof(vehicle_path_point_t);
    uint32 page_count = 0u;

    if (points_per_page == 0u)
    {
        return FALSE;
    }

    while ((page_count < max_page_count)
           && (vehicle_path_runtime.job_chunk_write_offset_cnt < vehicle_path_runtime.job_chunk_core_count))
    {
        uint32 valid_points = vehicle_path_runtime.job_chunk_core_count
                            - vehicle_path_runtime.job_chunk_write_offset_cnt;
        uint32 local_point = vehicle_path_runtime.job_chunk_core_offset_cnt
                           + vehicle_path_runtime.job_chunk_write_offset_cnt;
        uint32 global_point = vehicle_path_runtime.job_chunk_core_start_cnt
                            + vehicle_path_runtime.job_chunk_write_offset_cnt;
        uint32 valid_bytes;
        uint32 write_offset;
        uint32 index_cnt;

        if (valid_points > points_per_page) { valid_points = points_per_page; }
        valid_bytes = valid_points * (uint32)sizeof(vehicle_path_point_t);
        for (index_cnt = 0u; index_cnt < IFXFLASH_PFLASH_PAGE_LENGTH; index_cnt++)
        {
            vehicle_path_runtime.job_page[index_cnt] = 0xFFu;
        }
        for (index_cnt = 0u; index_cnt < valid_bytes; index_cnt++)
        {
            vehicle_path_runtime.job_page[index_cnt] =
                ((const uint8*)&vehicle_path_runtime.points[local_point])[index_cnt];
        }

        write_offset = cfg->flash_offset + VEHICLE_PATH_FLASH_PLANNED_OFFSET
                     + VEHICLE_PATH_FLASH_POINT_OFFSET
                     + (global_point * (uint32)sizeof(vehicle_path_point_t));
        if (service_storage_writeSync(SERVICE_STORAGE_1,
                                      write_offset,
                                      vehicle_path_runtime.job_page,
                                      IFXFLASH_PFLASH_PAGE_LENGTH) != IFXFLASH_PFLASH_PAGE_LENGTH)
        {
            return FALSE;
        }
        vehicle_path_runtime.job_checksum =
            vehicle_path_checksum_update(vehicle_path_runtime.job_checksum,
                                         vehicle_path_runtime.job_page,
                                         valid_bytes);
        vehicle_path_runtime.job_chunk_write_offset_cnt += valid_points;
        page_count++;
    }

    return TRUE;
}

static boolean vehicle_path_plan_fixed_finish(void)
{
    vehicle_path_flash_header_t header;

    vehicle_path_plan_markers_remap_to_planned();
    if (vehicle_path_header_write(VEHICLE_PATH_FLASH_PLANNED_OFFSET,
                                  vehicle_path_runtime.job_point_cnt,
                                  vehicle_path_runtime.job_checksum) == FALSE)
    {
        return FALSE;
    }

    header.magic = VEHICLE_PATH_MAGIC;
    header.version = VEHICLE_PATH_VERSION;
    header.point_cnt = (uint16)vehicle_path_runtime.job_point_cnt;
    header.checksum = vehicle_path_runtime.job_checksum;
    if (vehicle_path_load_from_slot(VEHICLE_PATH_FLASH_PLANNED_OFFSET, &header) == FALSE)
    {
        return FALSE;
    }

    vehicle_path_runtime.planned_speed_point_cnt = vehicle_path_runtime.job_point_cnt;
    vehicle_path_runtime.planned_speed_overlay_active = TRUE;
    vehicle_path_planned_speed_meta_set(vehicle_path_runtime.job_source_point_cnt,
                                        vehicle_path_runtime.job_path_length_mm,
                                        vehicle_path_runtime.job_point_cnt,
                                        TRUE);
    (void)vehicle_path_marker_load_from_slot(VEHICLE_PATH_FLASH_PLANNED_OFFSET);
    (void)vehicle_path_replay_edge_points_refresh();
    tools_printf("{pathplan}done,%u,%.3f,%.3f,%u,%.1f,%.3f\r\n",
                 (unsigned int)vehicle_path_runtime.job_planned_cnt,
                 (double)vehicle_path_runtime.job_min_speed_mm_s,
                 (double)vehicle_path_runtime.job_max_speed_mm_s,
                 (unsigned int)vehicle_path_runtime.job_source_point_cnt,
                 (double)vehicle_path_runtime.job_path_length_mm,
                 (double)vehicle_path_runtime.job_resample_step_mm);
    tools_printf("{pathplan}success,%u,%.1f\r\n",
                 (unsigned int)vehicle_path_runtime.job_point_cnt,
                 (double)vehicle_path_runtime.job_path_length_mm);
    return TRUE;
}

static float32 vehicle_path_points_length_calculate(uint32 point_cnt)
{
    uint32 index_cnt;
    float32 length_mm = 0.0f;

    if (point_cnt > vehicle_path_max_point_cnt_get())
    {
        point_cnt = vehicle_path_max_point_cnt_get();
    }
    if (point_cnt < 2u)
    {
        return 0.0f;
    }

    for (index_cnt = 1u; index_cnt < point_cnt; index_cnt++)
    {
        length_mm += vehicle_path_segment_length_mm(&vehicle_path_runtime.points[index_cnt - 1u],
                                                    &vehicle_path_runtime.points[index_cnt]);
    }

    return length_mm;
}

/**
 * @brief 将角度限制到负圆周率到正圆周率范围内。
 * @param[in] angle_rad 原始角度，单位：弧度。
 * @return 归一化后的角度，单位：弧度。
 */
static float32 vehicle_path_wrap_pi(float32 angle_rad)
{
    while (angle_rad > VEHICLE_PATH_PI)
    {
        angle_rad -= 2.0f * VEHICLE_PATH_PI;
    }

    while (angle_rad < -VEHICLE_PATH_PI)
    {
        angle_rad += 2.0f * VEHICLE_PATH_PI;
    }

    return angle_rad;
}
