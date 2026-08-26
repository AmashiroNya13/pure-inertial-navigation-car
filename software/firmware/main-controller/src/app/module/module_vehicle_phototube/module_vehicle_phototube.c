#include "../../../../inc/app/module/module_vehicle_phototube/module_vehicle_phototube.h"

#include "../../../../inc/app/module/module_vehicle_control/module_vehicle_control.h"
#include "../../../../inc/app/module/module_vehicle_encoder/module_vehicle_encoder.h"
#include "../../../../inc/app/module/module_vehicle_path/module_vehicle_path.h"
#include "../../../../inc/app/module/module_vehicle_pose_fusion/module_vehicle_pose_fusion.h"
#include "../../../../inc/device/device_phototube/device_phototube.h"
#include "../../../../inc/device/device_int_flash/device_int_flash.h"
#include "../../../../inc/driver/driver_flash/driver_flash.h"
#include "../../../../inc/middleware/sysTick/sysTick.h"
#include "../../../../inc/middleware/tools/tools_print/tools_print.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define MODULE_VEHICLE_PHOTOTUBE_COUNT         ((uint32)16u)
#define MODULE_VEHICLE_PHOTOTUBE_FLASH_MAGIC   ((uint32)0x50544341u)
#define MODULE_VEHICLE_PHOTOTUBE_FLASH_VERSION ((uint16)1u)
#define MODULE_VEHICLE_PHOTOTUBE_FLASH_OFFSET  ((uint32)0x00100000u)
#define MODULE_VEHICLE_PHOTOTUBE_FLASH_ADDRESS ((uint32)0xA0200000u)
#define MODULE_VEHICLE_PHOTOTUBE_FLASH_SECTOR_BYTES ((uint32)0x00040000u)
#define MODULE_VEHICLE_PHOTOTUBE_FLASH_CHECKSUM_SEED ((uint32)0xFFFFFFFFu)
#define MODULE_VEHICLE_PHOTOTUBE_ADC_SHIFT     ((uint16)0u)
#define MODULE_VEHICLE_PHOTOTUBE_ADC_MAX       ((uint16)4095u)
#define MODULE_VEHICLE_PHOTOTUBE_GAIN_Q15_BASE ((uint32)32768u)
#define MODULE_VEHICLE_PHOTOTUBE_WEIGHT_Q15_BASE ((uint32)32768u)
#define MODULE_VEHICLE_PHOTOTUBE_MIN_SPAN      ((uint16)32u)
#define MODULE_VEHICLE_PHOTOTUBE_NORM_MAX      ((uint16)1000u)
#define MODULE_VEHICLE_PHOTOTUBE_AUTO_BLUE_SAMPLE_COUNT ((uint32)64u)
#define MODULE_VEHICLE_PHOTOTUBE_AUTO_LINE_DELTA ((uint16)35u)
#define MODULE_VEHICLE_PHOTOTUBE_AUTO_BACK_DELTA ((uint16)70u)
#define MODULE_VEHICLE_PHOTOTUBE_AUTO_BACK_CHANNEL_COUNT ((uint32)14u)
#define MODULE_VEHICLE_PHOTOTUBE_AUTO_BACK_STABLE_COUNT ((uint32)16u)
#define MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_SPEED_DEFAULT_MM_S (500.0f)
#define MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_SPEED_MIN_MM_S (400.0f)
#define MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_SPEED_MAX_MM_S (1000.0f)
#define MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_WHITE_CONFIRM_COUNT ((uint32)4u)
#define MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_STOP_STABLE_COUNT ((uint32)16u)
#define MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_STOP_SPEED_MM_S (50.0f)
#define MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_SEARCH_MAX_MM (600.0f)
#define MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_LAUNCH_TIMEOUT_S (2.0f)
#define MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_TOTAL_TIMEOUT_S (8.0f)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_PRINT_DIVIDER_DEFAULT ((uint32)50u)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_PRINT_DIVIDER_MIN ((uint32)20u)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_POSITION_STEP (100.0f)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_MIN_SUM ((uint32)80u)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_KP_DEFAULT_MM_S (0.8f)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_KD_DEFAULT_MM_S (0.20f)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_FILTER_ALPHA_DEFAULT (0.35f)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_EDGE_BOOST_DEFAULT (0.20f)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_DIFF_LIMIT_MM_S (600.0f)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_MIN_SPEED_MM_S (300.0f)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_SPEED_DEFAULT_MM_S (1200.0f)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_ELEMENT_SUM_MIN ((uint32)3720u)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_ACTIVE_THRESHOLD ((uint16)30u)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_ELEMENT_CONFIRM_S (0.0025f)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_ELEMENT_CONFIRM_COUNT ((uint32)2u)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_HEADING_ANCHOR_ERROR_MAX (100.0f)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_HEADING_ANCHOR_STABLE_S (0.010f)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_HEADING_ANCHOR_MAX_AGE_S (0.100f)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_NORMAL_SUM_MIN ((uint32)200u)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_NORMAL_SUM_MAX ((uint32)3500u)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_NORMAL_ACTIVE_MIN ((uint32)3u)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_NORMAL_ACTIVE_MAX ((uint32)8u)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_TURN_RADIUS_DEFAULT_MM (20.0f)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_TURN_ANGLE_RAD (1.5707963267948966f)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_REACQUIRE_ANGLE_RAD (0.20943951023931956f)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_REACQUIRE_MIN_DISTANCE_MM (50.0f)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_REACQUIRE_STABLE_S (0.010f)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_REACQUIRE_STABLE_DISTANCE_MM (20.0f)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_REACQUIRE_STABLE_COUNT ((uint32)3u)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_REACQUIRE_BLEND_S (0.050f)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_MANEUVER_TIMEOUT_S (5.0f)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_MANEUVER_MAX_DISTANCE_MM (1000.0f)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_LOST_SUM_MIN MODULE_VEHICLE_PHOTOTUBE_LINE_MIN_SUM
#define MODULE_VEHICLE_PHOTOTUBE_LINE_LOST_TIMEOUT_S (0.050f)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_LOST_DISTANCE_MM (200.0f)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_LOST_RECOVERY_RATIO (2.0f)
#define MODULE_VEHICLE_PHOTOTUBE_LINE_LOST_SUCTION_DELAY_US ((uint64)300000u)
#define MODULE_VEHICLE_PHOTOTUBE_TURN_TABLE_MAX ((uint32)32u)
#define MODULE_VEHICLE_PHOTOTUBE_US_TO_S (0.000001f)
/* Physical installation position, measured forward from the turning center. */
#define MODULE_VEHICLE_PHOTOTUBE_CORR_SENSOR_X_DEFAULT_MM (107.2f)
#define MODULE_VEHICLE_PHOTOTUBE_CORR_SENSOR_SPACING_DEFAULT_MM (4.0f)
#define MODULE_VEHICLE_PHOTOTUBE_CORR_SENSOR_Y_SIGN_DEFAULT (-1.0f)
#define MODULE_VEHICLE_PHOTOTUBE_CORR_GAIN_DEFAULT (0.06f)
#define MODULE_VEHICLE_PHOTOTUBE_CORR_MAX_STEP_DEFAULT_MM (0.60f)
#define MODULE_VEHICLE_PHOTOTUBE_CORR_REJECT_DEFAULT_MM (120.0f)
#define MODULE_VEHICLE_PHOTOTUBE_CORR_WINDOW_DEFAULT_MM (300.0f)
#define MODULE_VEHICLE_PHOTOTUBE_CORR_MIN_CONFIDENCE_DEFAULT (0.05f)
#define MODULE_VEHICLE_PHOTOTUBE_CORR_STRENGTH_THRESHOLD_DEFAULT ((uint16)30u)
#define MODULE_VEHICLE_PHOTOTUBE_CORR_SUM_MAX_DEFAULT ((uint32)4500u)
#define MODULE_VEHICLE_PHOTOTUBE_CORR_ACTIVE_COUNT_MAX_DEFAULT ((uint32)6u)
#define MODULE_VEHICLE_PHOTOTUBE_CORR_ACTIVE_WIDTH_MAX_DEFAULT_MM (28.0f)
#define MODULE_VEHICLE_PHOTOTUBE_CORR_CONFIDENCE_FULL_SUM (2000.0f)
#define MODULE_VEHICLE_PHOTOTUBE_CORR_PRINT_DIVIDER_DEFAULT ((uint32)192u)
#define MODULE_VEHICLE_PHOTOTUBE_CORR_EVENT_RELEASE_COUNT ((uint32)20u)
#define MODULE_VEHICLE_PHOTOTUBE_CORR_GATE_YAW_RATE_DEFAULT_DEG_S (40.0f)
#define MODULE_VEHICLE_PHOTOTUBE_CORR_GATE_HEADING_DEFAULT_DEG (8.0f)
#define MODULE_VEHICLE_PHOTOTUBE_CORR_GATE_STABLE_COUNT_DEFAULT ((uint32)20u)
#define MODULE_VEHICLE_PHOTOTUBE_PI (3.14159265358979323846f)
#define MODULE_VEHICLE_PHOTOTUBE_RAD_TO_DEG (57.295779513082320876f)
#define MODULE_VEHICLE_PHOTOTUBE_DEG_TO_RAD (0.017453292519943295f)

static const uint16 module_vehicle_phototube_line_weight_q15[MODULE_VEHICLE_PHOTOTUBE_COUNT] =
{
    11469u, 14746u, 19661u, 24576u,
    29491u, 34406u, 37683u, 39322u,
    39322u, 37683u, 34406u, 29491u,
    24576u, 19661u, 14746u, 11469u,
};

static const uint8 module_vehicle_phototube_default_map[MODULE_VEHICLE_PHOTOTUBE_COUNT] =
{
    15u, 11u, 14u, 10u,
    13u, 9u, 12u, 8u,
    7u, 3u, 6u, 2u,
    5u, 1u, 4u, 0u,
};

static const module_vehicle_phototube_calibration_t module_vehicle_phototube_default_calibration =
{
    .valid = TRUE,
    .sample_count = 19806u,
    .blue_value =
    {
        2112u, 2015u, 1945u, 1883u,
        2243u, 2242u, 1806u, 1783u,
        1715u, 1893u, 1696u, 1872u,
        2071u, 1994u, 2297u, 2265u,
    },
    .white_value =
    {
        1166u, 1106u, 1050u, 924u,
        1286u, 1320u, 896u, 852u,
        776u, 934u, 786u, 952u,
        1054u, 1042u, 1276u, 1310u,
    },
    .min_value =
    {
        1166u, 1106u, 1050u, 924u,
        1286u, 1320u, 896u, 852u,
        776u, 934u, 786u, 952u,
        1054u, 1042u, 1276u, 1310u,
    },
    .max_value =
    {
        2140u, 2046u, 1976u, 1910u,
        2276u, 2278u, 1828u, 1808u,
        1744u, 1918u, 1726u, 1906u,
        2104u, 2032u, 2330u, 2298u,
    },
    .center_value =
    {
        1653u, 1576u, 1513u, 1417u,
        1781u, 1799u, 1362u, 1330u,
        1260u, 1426u, 1256u, 1429u,
        1579u, 1537u, 1803u, 1804u,
    },
    .span_value =
    {
        974u, 940u, 926u, 986u,
        990u, 958u, 932u, 956u,
        968u, 984u, 940u, 954u,
        1050u, 990u, 1054u, 988u,
    },
    .offset_value =
    {
        121, 44, -19, -115,
        249, 267, -170, -202,
        -272, -106, -276, -103,
        47, 5, 271, 272,
    },
    .gain_q15 =
    {
        32768u, 33953u, 34466u, 32369u,
        32238u, 33315u, 34244u, 33384u,
        32971u, 32434u, 33953u, 33454u,
        30396u, 32238u, 30280u, 32303u,
    },
};

typedef enum
{
    MODULE_VEHICLE_PHOTOTUBE_COMMAND_NONE = 0,
    MODULE_VEHICLE_PHOTOTUBE_COMMAND_START,
    MODULE_VEHICLE_PHOTOTUBE_COMMAND_AUTO,
    MODULE_VEHICLE_PHOTOTUBE_COMMAND_STOP,
    MODULE_VEHICLE_PHOTOTUBE_COMMAND_SHOW,
    MODULE_VEHICLE_PHOTOTUBE_COMMAND_ABORT,
    MODULE_VEHICLE_PHOTOTUBE_COMMAND_SAVE,
    MODULE_VEHICLE_PHOTOTUBE_COMMAND_LOAD,
    MODULE_VEHICLE_PHOTOTUBE_COMMAND_ERASE,
    MODULE_VEHICLE_PHOTOTUBE_COMMAND_FLASH,
    MODULE_VEHICLE_PHOTOTUBE_COMMAND_USAGE,
} module_vehicle_phototube_command_t;

typedef struct
{
    uint32 magic;
    uint16 version;
    uint16 record_length;
    uint32 checksum;
    uint32 sample_count;
    uint8 physical_map[16u];
    uint16 blue_value[16u];
    uint16 white_value[16u];
    uint16 min_value[16u];
    uint16 max_value[16u];
    uint16 center_value[16u];
    uint16 span_value[16u];
    sint16 offset_value[16u];
    uint32 gain_q15[16u];
} module_vehicle_phototube_flash_record_t;

typedef char module_vehicle_phototube_flash_page_check_t[
    (IFXFLASH_PFLASH_PAGE_LENGTH == 32u) ? 1 : -1];
typedef char module_vehicle_phototube_flash_size_check_t[
    (sizeof(module_vehicle_phototube_flash_record_t)
     <= MODULE_VEHICLE_PHOTOTUBE_FLASH_SECTOR_BYTES) ? 1 : -1];

typedef enum
{
    MODULE_VEHICLE_PHOTOTUBE_AUTO_IDLE = 0,
    MODULE_VEHICLE_PHOTOTUBE_AUTO_BLUE,
    MODULE_VEHICLE_PHOTOTUBE_AUTO_SCAN,
} module_vehicle_phototube_auto_state_t;

typedef enum
{
    MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_IDLE = 0,
    MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_BLUE_SAMPLE,
    MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_LAUNCH_WHITE,
    MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_SEARCH_WHITE,
    MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_WHITE_STOP,
    MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_WHITE_SAMPLE,
    MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_LAUNCH_BLUE,
    MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_SEARCH_BLUE,
    MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_BLUE_STOP,
    MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_BLUE_VERIFY,
} module_vehicle_phototube_drive_cal_state_t;

typedef enum
{
    MODULE_VEHICLE_PHOTOTUBE_LINE_OFF = 0,
    MODULE_VEHICLE_PHOTOTUBE_LINE_PUSH,
    MODULE_VEHICLE_PHOTOTUBE_LINE_RUN,
} module_vehicle_phototube_line_mode_t;

typedef enum
{
    MODULE_VEHICLE_PHOTOTUBE_LAUNCH_OFF = 0,
    MODULE_VEHICLE_PHOTOTUBE_LAUNCH_WAIT_START,
    MODULE_VEHICLE_PHOTOTUBE_LAUNCH_ACTIVE,
} module_vehicle_phototube_launch_state_t;

typedef enum
{
    MODULE_VEHICLE_PHOTOTUBE_TURN_LEFT = 0,
    MODULE_VEHICLE_PHOTOTUBE_TURN_RIGHT = 1,
    MODULE_VEHICLE_PHOTOTUBE_TURN_EVALUATE = 2,
    MODULE_VEHICLE_PHOTOTUBE_TURN_STRAIGHT = 3,
} module_vehicle_phototube_turn_action_t;

typedef enum
{
    MODULE_VEHICLE_PHOTOTUBE_MANEUVER_FOLLOW = 0,
    MODULE_VEHICLE_PHOTOTUBE_MANEUVER_HEADING,
} module_vehicle_phototube_maneuver_state_t;

typedef struct
{
    uint16 value[16u];
    uint8 physical_map[16u];
    boolean frame_ready;
    boolean calibration_active;
    module_vehicle_phototube_auto_state_t auto_state;
    uint32 auto_blue_sum[16u];
    uint32 auto_back_stable_count;
    boolean auto_line_seen;
    module_vehicle_phototube_drive_cal_state_t drive_cal_state;
    float32 drive_cal_speed_mm_s;
    float32 drive_cal_heading_rad;
    float32 drive_cal_state_start_distance_mm;
    uint64 drive_cal_start_tick;
    uint64 drive_cal_state_start_tick;
    boolean drive_cal_launch_seen;
    uint32 drive_cal_confirm_count;
    uint32 drive_cal_peak_confirm_count;
    uint32 drive_cal_peak_delta;
    uint32 drive_cal_sample_count;
    uint32 drive_cal_white_sum[16u];
    uint32 drive_cal_final_blue_sum[16u];
    module_vehicle_phototube_calibration_t drive_cal_backup;
    module_vehicle_phototube_line_mode_t line_mode;
    module_vehicle_phototube_launch_state_t line_launch_state;
    float32 line_base_speed_mm_s;
    float32 line_kp_mm_s;
    float32 line_kd_mm_s;
    float32 line_filter_alpha;
    float32 line_edge_boost;
    float32 line_diff_limit_mm_s;
    float32 line_min_speed_mm_s;
    float32 line_direction;
    boolean line_weight_enabled;
    boolean line_detail_enabled;
    boolean line_filter_initialized;
    uint32 line_print_divider;
    uint32 line_print_count;
    float32 line_position;
    float32 line_error;
    float32 line_filtered_error;
    float32 line_last_filtered_error;
    float32 line_correction_mm_s;
    float32 line_left_speed_mm_s;
    float32 line_right_speed_mm_s;
    uint32 line_sum;
    uint32 line_active_count;
    boolean line_valid;
    module_vehicle_phototube_maneuver_state_t maneuver_state;
    module_vehicle_phototube_turn_action_t maneuver_action;
    module_vehicle_phototube_turn_action_t turn_table[MODULE_VEHICLE_PHOTOTUBE_TURN_TABLE_MAX];
    uint32 turn_table_count;
    uint32 turn_event_index;
    boolean element_detection_enabled;
    boolean element_guard_enabled;
    float32 turn_radius_mm;
    float32 maneuver_entry_heading_rad;
    float32 maneuver_final_heading_rad;
    float32 maneuver_target_heading_rad;
    float32 maneuver_entry_distance_mm;
    float32 maneuver_last_distance_mm;
    float32 maneuver_elapsed_s;
    float32 element_candidate_s;
    uint32 element_candidate_count;
    float32 stable_heading_rad;
    float32 stable_heading_s;
    float32 stable_heading_age_s;
    boolean stable_heading_valid;
    float32 element_entry_heading_rad;
    boolean element_entry_heading_valid;
    float32 normal_confidence_s;
    float32 normal_confidence_distance_mm;
    uint32 normal_confidence_count;
    float32 line_follow_blend;
    float32 line_lost_confidence_s;
    float32 line_lost_confidence_distance_mm;
    float32 line_lost_last_distance_mm;
    boolean line_lost_distance_initialized;
    boolean line_lost_drive_stop_pending;
    boolean line_lost_suction_pending;
    uint64 line_lost_stop_tick;
    uint64 line_last_tick;
    boolean line_last_tick_valid;
    boolean power_user_enabled;
    boolean correction_enabled;
    boolean correction_monitor_enabled;
    boolean correction_print_enabled;
    uint32 correction_print_divider;
    uint32 correction_print_count;
    boolean correction_event_active;
    uint32 correction_event_release_count;
    uint32 correction_event_sample_count;
    float32 correction_event_start_index;
    float32 correction_event_end_index;
    float32 correction_event_start_x_mm;
    float32 correction_event_start_y_mm;
    float32 correction_event_end_x_mm;
    float32 correction_event_end_y_mm;
    float32 correction_event_sum_x_mm;
    float32 correction_event_sum_y_mm;
    float32 correction_event_peak_step_mm;
    float32 correction_event_max_confidence;
    float32 correction_sensor_x_mm;
    float32 correction_sensor_spacing_mm;
    float32 correction_sensor_y_sign;
    float32 correction_gain;
    float32 correction_max_step_mm;
    float32 correction_reject_distance_mm;
    float32 correction_window_mm;
    float32 correction_min_confidence;
    uint16 correction_strength_threshold;
    uint32 correction_sum_max;
    uint32 correction_active_count_max;
    float32 correction_active_width_max_mm;
    boolean correction_shape_pass;
    boolean correction_gate_enabled;
    uint32 correction_gate_stable_required_count;
    uint32 correction_gate_stable_count;
    boolean correction_gate_pass;
    boolean flash_auto_load_pending;
    float32 correction_gate_yaw_rate_max_rad_s;
    float32 correction_gate_heading_delta_max_rad;
    float32 correction_gate_target_yaw_rate_deg_s;
    float32 correction_gate_heading_delta_deg;
    module_vehicle_phototube_line_observation_t line_observation;
    volatile module_vehicle_phototube_command_t pending_command;
    module_vehicle_phototube_calibration_t calibration;
} module_vehicle_phototube_runtime_t;

static module_vehicle_phototube_runtime_t module_vehicle_phototube_runtime;

static uint16 module_vehicle_phototube_value_read(uint32 phototube_index);
static void module_vehicle_phototube_frame_capture(void);
static void module_vehicle_phototube_calibration_start(void);
static void module_vehicle_phototube_calibration_auto_start(void);
static void module_vehicle_phototube_calibration_abort(void);
static void module_vehicle_phototube_calibration_update(void);
static void module_vehicle_phototube_calibration_auto_update(void);
static void module_vehicle_phototube_drive_calibration_start(float32 speed_mm_s);
static void module_vehicle_phototube_drive_calibration_update(void);
static void module_vehicle_phototube_drive_calibration_abort(const char* reason);
static void module_vehicle_phototube_drive_calibration_status(void);
static boolean module_vehicle_phototube_drive_calibration_launch(
    module_vehicle_phototube_drive_cal_state_t launch_state);
static boolean module_vehicle_phototube_drive_calibration_stop(void);
static float32 module_vehicle_phototube_drive_calibration_elapsed_s(uint64 start_tick);
static float32 module_vehicle_phototube_drive_calibration_distance_get(void);
static const char* module_vehicle_phototube_drive_calibration_state_name(
    module_vehicle_phototube_drive_cal_state_t state);
static boolean module_vehicle_phototube_calibration_finish(void);
static void module_vehicle_phototube_calibration_print(void);
static boolean module_vehicle_phototube_calibration_save_to_flash(void);
static boolean module_vehicle_phototube_calibration_load_from_flash(void);
static boolean module_vehicle_phototube_calibration_erase_flash(void);
static void module_vehicle_phototube_calibration_auto_load_process(void);
static boolean module_vehicle_phototube_calibration_flash_record_read(
    module_vehicle_phototube_flash_record_t* record);
static boolean module_vehicle_phototube_calibration_flash_record_validate(
    const module_vehicle_phototube_flash_record_t* record);
static void module_vehicle_phototube_calibration_flash_record_pack(
    module_vehicle_phototube_flash_record_t* record);
static void module_vehicle_phototube_calibration_flash_record_apply(
    const module_vehicle_phototube_flash_record_t* record);
static uint32 module_vehicle_phototube_flash_address_get(void);
static boolean module_vehicle_phototube_flash_runtime_ready(void);
static uint32 module_vehicle_phototube_flash_aligned_length_get(uint32 length);
static uint32 module_vehicle_phototube_checksum_calculate(const uint8* data, uint32 length);
static uint32 module_vehicle_phototube_checksum_update(uint32 checksum, const uint8* data, uint32 length);
static void module_vehicle_phototube_flash_page_pack(const uint8 page[IFXFLASH_PFLASH_PAGE_LENGTH],
                                                     uint32 (*word_l)[4],
                                                     uint32 (*word_u)[4]);
static void module_vehicle_phototube_line_update(void);
static void module_vehicle_phototube_line_print(void);
void module_vehicle_phototube_line_stop(void);
static void module_vehicle_phototube_line_control_reset(void);
static void module_vehicle_phototube_line_maneuver_reset(boolean reset_event_index);
static float32 module_vehicle_phototube_line_frame_dt_get(void);
static uint32 module_vehicle_phototube_line_active_count_get(void);
static void module_vehicle_phototube_line_heading_anchor_update(float32 dt_s);
static boolean module_vehicle_phototube_line_element_detected(float32 dt_s);
static boolean module_vehicle_phototube_line_normal_detected(void);
static void module_vehicle_phototube_line_lost_reset(void);
static boolean module_vehicle_phototube_line_lost_update(float32 dt_s);
static void module_vehicle_phototube_line_lost_stop(void);
static void module_vehicle_phototube_line_lost_suction_update(void);
static void module_vehicle_phototube_line_maneuver_start(void);
static boolean module_vehicle_phototube_line_maneuver_update(float32 dt_s);
static module_vehicle_phototube_turn_action_t module_vehicle_phototube_turn_action_get(uint32 index);
static boolean module_vehicle_phototube_turn_action_parse(
    uint8* text,
    module_vehicle_phototube_turn_action_t* action);
static const char* module_vehicle_phototube_turn_action_name(
    module_vehicle_phototube_turn_action_t action);
static void module_vehicle_phototube_correction_process(void);
static void module_vehicle_phototube_correction_event_miss(void);
static void module_vehicle_phototube_correction_event_apply(
    const vehicle_path_projection_t* projection,
    float32 confidence,
    float32 correction_x_mm,
    float32 correction_y_mm);
static void module_vehicle_phototube_correction_event_finish(void);
static void module_vehicle_phototube_correction_print(
    const module_vehicle_phototube_line_observation_t* line_observation,
    float32 line_world_x_mm,
    float32 line_world_y_mm,
    const vehicle_path_projection_t* projection,
    float32 error_x_mm,
    float32 error_y_mm,
    float32 correction_x_mm,
    float32 correction_y_mm,
    boolean applied);
static void module_vehicle_phototube_correction_show(void);
static boolean module_vehicle_phototube_control_speed_test_enable_post(boolean enable);
static boolean module_vehicle_phototube_control_speed_test_stop_post(void);
static boolean module_vehicle_phototube_control_suction_post(uint32 duty);
static boolean module_vehicle_phototube_control_drive_stop_keep_suction_post(uint32 duty);
static boolean module_vehicle_phototube_control_heading_target_post(float32 speed_mm_s,
                                                                    float32 theta_rad);
static boolean module_vehicle_phototube_control_speed_target_post(float32 left_speed_mm_s,
                                                                  float32 right_speed_mm_s);
static float32 module_vehicle_phototube_line_correction_calculate(float32 error);
static float32 module_vehicle_phototube_clamp_f32(float32 value, float32 min_value, float32 max_value);
static float32 module_vehicle_phototube_vector_limit(float32* x_mm, float32* y_mm, float32 limit_mm);
static float32 module_vehicle_phototube_wrap_pi(float32 angle_rad);
static boolean module_vehicle_phototube_correction_gate_update(
    const vehicle_path_replay_target_t* replay_target);
static void module_vehicle_phototube_correction_gate_reset(void);
static boolean module_vehicle_phototube_line_calculate(float32* position,
                                                       float32* error,
                                                       uint32* line_sum);
static boolean module_vehicle_phototube_line_observation_calculate(
    module_vehicle_phototube_line_observation_t* observation);
static void module_vehicle_phototube_map_reset(void);
static boolean module_vehicle_phototube_map_validate(const uint8 map[16u]);
static boolean module_vehicle_phototube_map_parse(uint8 argc, uint8* argv[], uint8 map[16u]);
static void module_vehicle_phototube_map_apply(const uint8 map[16u]);
static void module_vehicle_phototube_map_print(void);
static void module_vehicle_phototube_adc_values_get(uint16 values[16u]);
static void module_vehicle_phototube_adc_print(void);
static void module_vehicle_phototube_power_update(void);
static boolean module_vehicle_phototube_power_needed(void);
static boolean module_vehicle_phototube_correction_runtime_allowed(void);
static void module_vehicle_phototube_power_show(void);
static void module_vehicle_phototube_command_process(void);
static boolean module_vehicle_phototube_command_is(uint8* argument, const char* command);
static float32 module_vehicle_phototube_command_float_get(uint8* text);
static uint32 module_vehicle_phototube_command_uint_get(uint8* text);
static uint16 module_vehicle_phototube_normalized_value_get(uint32 index);
static uint16 module_vehicle_phototube_clamp_u16(sint32 value);

void module_vehicle_phototube_init(void)
{
    uint32 index;

    (void)memset(&module_vehicle_phototube_runtime, 0, sizeof(module_vehicle_phototube_runtime));
    module_vehicle_phototube_map_reset();
    module_vehicle_phototube_runtime.calibration =
        module_vehicle_phototube_default_calibration;
    module_vehicle_phototube_runtime.drive_cal_speed_mm_s =
        MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_SPEED_DEFAULT_MM_S;
    module_vehicle_phototube_runtime.line_base_speed_mm_s =
        MODULE_VEHICLE_PHOTOTUBE_LINE_SPEED_DEFAULT_MM_S;
    module_vehicle_phototube_runtime.line_kp_mm_s =
        MODULE_VEHICLE_PHOTOTUBE_LINE_KP_DEFAULT_MM_S;
    module_vehicle_phototube_runtime.line_kd_mm_s =
        MODULE_VEHICLE_PHOTOTUBE_LINE_KD_DEFAULT_MM_S;
    module_vehicle_phototube_runtime.line_filter_alpha =
        MODULE_VEHICLE_PHOTOTUBE_LINE_FILTER_ALPHA_DEFAULT;
    module_vehicle_phototube_runtime.line_edge_boost =
        MODULE_VEHICLE_PHOTOTUBE_LINE_EDGE_BOOST_DEFAULT;
    module_vehicle_phototube_runtime.line_diff_limit_mm_s =
        MODULE_VEHICLE_PHOTOTUBE_LINE_DIFF_LIMIT_MM_S;
    module_vehicle_phototube_runtime.line_min_speed_mm_s =
        MODULE_VEHICLE_PHOTOTUBE_LINE_MIN_SPEED_MM_S;
    module_vehicle_phototube_runtime.line_direction = -1.0f;
    module_vehicle_phototube_runtime.line_weight_enabled = TRUE;
    module_vehicle_phototube_runtime.line_detail_enabled = FALSE;
    module_vehicle_phototube_runtime.line_print_divider =
        MODULE_VEHICLE_PHOTOTUBE_LINE_PRINT_DIVIDER_DEFAULT;
    module_vehicle_phototube_runtime.turn_radius_mm =
        MODULE_VEHICLE_PHOTOTUBE_LINE_TURN_RADIUS_DEFAULT_MM;
    module_vehicle_phototube_runtime.element_detection_enabled = TRUE;
    module_vehicle_phototube_runtime.element_guard_enabled = FALSE;
    module_vehicle_phototube_runtime.line_follow_blend = 1.0f;
    for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_TURN_TABLE_MAX; index++)
    {
        module_vehicle_phototube_runtime.turn_table[index] =
            MODULE_VEHICLE_PHOTOTUBE_TURN_EVALUATE;
    }
    module_vehicle_phototube_line_maneuver_reset(TRUE);
    module_vehicle_phototube_runtime.power_user_enabled = FALSE;
    module_vehicle_phototube_runtime.correction_enabled = FALSE;
    module_vehicle_phototube_runtime.correction_monitor_enabled = FALSE;
    module_vehicle_phototube_runtime.correction_print_enabled = FALSE;
    module_vehicle_phototube_runtime.correction_print_divider =
        MODULE_VEHICLE_PHOTOTUBE_CORR_PRINT_DIVIDER_DEFAULT;
    module_vehicle_phototube_runtime.correction_sensor_x_mm =
        MODULE_VEHICLE_PHOTOTUBE_CORR_SENSOR_X_DEFAULT_MM;
    module_vehicle_phototube_runtime.correction_sensor_spacing_mm =
        MODULE_VEHICLE_PHOTOTUBE_CORR_SENSOR_SPACING_DEFAULT_MM;
    module_vehicle_phototube_runtime.correction_sensor_y_sign =
        MODULE_VEHICLE_PHOTOTUBE_CORR_SENSOR_Y_SIGN_DEFAULT;
    module_vehicle_phototube_runtime.correction_gain =
        MODULE_VEHICLE_PHOTOTUBE_CORR_GAIN_DEFAULT;
    module_vehicle_phototube_runtime.correction_max_step_mm =
        MODULE_VEHICLE_PHOTOTUBE_CORR_MAX_STEP_DEFAULT_MM;
    module_vehicle_phototube_runtime.correction_reject_distance_mm =
        MODULE_VEHICLE_PHOTOTUBE_CORR_REJECT_DEFAULT_MM;
    module_vehicle_phototube_runtime.correction_window_mm =
        MODULE_VEHICLE_PHOTOTUBE_CORR_WINDOW_DEFAULT_MM;
    module_vehicle_phototube_runtime.correction_min_confidence =
        MODULE_VEHICLE_PHOTOTUBE_CORR_MIN_CONFIDENCE_DEFAULT;
    module_vehicle_phototube_runtime.correction_strength_threshold =
        MODULE_VEHICLE_PHOTOTUBE_CORR_STRENGTH_THRESHOLD_DEFAULT;
    module_vehicle_phototube_runtime.correction_sum_max =
        MODULE_VEHICLE_PHOTOTUBE_CORR_SUM_MAX_DEFAULT;
    module_vehicle_phototube_runtime.correction_active_count_max =
        MODULE_VEHICLE_PHOTOTUBE_CORR_ACTIVE_COUNT_MAX_DEFAULT;
    module_vehicle_phototube_runtime.correction_active_width_max_mm =
        MODULE_VEHICLE_PHOTOTUBE_CORR_ACTIVE_WIDTH_MAX_DEFAULT_MM;
    module_vehicle_phototube_runtime.correction_shape_pass = FALSE;
    module_vehicle_phototube_runtime.correction_gate_enabled = TRUE;
    module_vehicle_phototube_runtime.correction_gate_stable_required_count =
        MODULE_VEHICLE_PHOTOTUBE_CORR_GATE_STABLE_COUNT_DEFAULT;
    module_vehicle_phototube_runtime.correction_gate_yaw_rate_max_rad_s =
        MODULE_VEHICLE_PHOTOTUBE_CORR_GATE_YAW_RATE_DEFAULT_DEG_S
        * MODULE_VEHICLE_PHOTOTUBE_DEG_TO_RAD;
    module_vehicle_phototube_runtime.correction_gate_heading_delta_max_rad =
        MODULE_VEHICLE_PHOTOTUBE_CORR_GATE_HEADING_DEFAULT_DEG
        * MODULE_VEHICLE_PHOTOTUBE_DEG_TO_RAD;
    module_vehicle_phototube_runtime.flash_auto_load_pending = TRUE;
    device_phototube_set_enabled(FALSE);

}

void module_vehicle_phototube_run(void)
{
    module_vehicle_phototube_calibration_auto_load_process();
    module_vehicle_phototube_command_process();
    module_vehicle_phototube_line_lost_suction_update();
    module_vehicle_phototube_power_update();

    if (module_vehicle_phototube_runtime.frame_ready == FALSE)
    {
        return;
    }

    module_vehicle_phototube_runtime.frame_ready = FALSE;
    if (module_vehicle_phototube_runtime.calibration_active != FALSE)
    {
        module_vehicle_phototube_calibration_update();
    }
    if (module_vehicle_phototube_runtime.auto_state != MODULE_VEHICLE_PHOTOTUBE_AUTO_IDLE)
    {
        module_vehicle_phototube_calibration_auto_update();
    }
    if (module_vehicle_phototube_runtime.drive_cal_state
        != MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_IDLE)
    {
        module_vehicle_phototube_drive_calibration_update();
    }
    if (module_vehicle_phototube_runtime.line_mode != MODULE_VEHICLE_PHOTOTUBE_LINE_OFF)
    {
        module_vehicle_phototube_line_update();
    }
    module_vehicle_phototube_correction_process();
    module_vehicle_phototube_power_update();
}

void module_vehicle_phototube_frame_update(void)
{
    module_vehicle_phototube_frame_capture();
    module_vehicle_phototube_runtime.frame_ready = TRUE;
}

boolean module_vehicle_phototube_frame_ready_get(void)
{
    return module_vehicle_phototube_runtime.frame_ready;
}

void module_vehicle_phototube_frame_ready_clear(void)
{
    module_vehicle_phototube_runtime.frame_ready = FALSE;
}

void module_vehicle_phototube_calibration_command(uint8 argc, uint8* argv[])
{
    module_vehicle_phototube_command_t command =
        MODULE_VEHICLE_PHOTOTUBE_COMMAND_USAGE;

    if ((argc > 1u) && (module_vehicle_phototube_command_is(argv[1u], "drive") != FALSE))
    {
        if ((argc > 2u) && (module_vehicle_phototube_command_is(argv[2u], "status") != FALSE))
        {
            module_vehicle_phototube_drive_calibration_status();
        }
        else if ((argc > 2u) && (module_vehicle_phototube_command_is(argv[2u], "abort") != FALSE))
        {
            module_vehicle_phototube_drive_calibration_abort("user_abort");
        }
        else
        {
            float32 speed_mm_s = MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_SPEED_DEFAULT_MM_S;

            if (argc > 2u)
            {
                speed_mm_s = module_vehicle_phototube_command_float_get(argv[2u]) * 1000.0f;
            }
            module_vehicle_phototube_drive_calibration_start(speed_mm_s);
        }
        module_vehicle_phototube_power_update();
        return;
    }

    if ((argc <= 1u) || (module_vehicle_phototube_command_is(argv[1u], "start") != FALSE))
    {
        command = MODULE_VEHICLE_PHOTOTUBE_COMMAND_START;
    }
    else if (module_vehicle_phototube_command_is(argv[1u], "auto") != FALSE)
    {
        command = MODULE_VEHICLE_PHOTOTUBE_COMMAND_AUTO;
    }
    else if (module_vehicle_phototube_command_is(argv[1u], "stop") != FALSE)
    {
        command = MODULE_VEHICLE_PHOTOTUBE_COMMAND_STOP;
    }
    else if (module_vehicle_phototube_command_is(argv[1u], "show") != FALSE)
    {
        command = MODULE_VEHICLE_PHOTOTUBE_COMMAND_SHOW;
    }
    else if (module_vehicle_phototube_command_is(argv[1u], "abort") != FALSE)
    {
        command = MODULE_VEHICLE_PHOTOTUBE_COMMAND_ABORT;
    }
    else if (module_vehicle_phototube_command_is(argv[1u], "save") != FALSE)
    {
        command = MODULE_VEHICLE_PHOTOTUBE_COMMAND_SAVE;
    }
    else if (module_vehicle_phototube_command_is(argv[1u], "load") != FALSE)
    {
        command = MODULE_VEHICLE_PHOTOTUBE_COMMAND_LOAD;
    }
    else if ((module_vehicle_phototube_command_is(argv[1u], "erase") != FALSE)
             || (module_vehicle_phototube_command_is(argv[1u], "clear") != FALSE))
    {
        command = MODULE_VEHICLE_PHOTOTUBE_COMMAND_ERASE;
    }
    else if ((module_vehicle_phototube_command_is(argv[1u], "flash") != FALSE)
             || (module_vehicle_phototube_command_is(argv[1u], "status") != FALSE))
    {
        command = MODULE_VEHICLE_PHOTOTUBE_COMMAND_FLASH;
    }

    module_vehicle_phototube_runtime.pending_command = command;
    module_vehicle_phototube_command_process();
    module_vehicle_phototube_power_update();
}

void module_vehicle_phototube_line_command(uint8 argc, uint8* argv[])
{
    if (argc <= 1u)
    {
        tools_printf("{ptline}mode,%u,speed,%.3f,kp,%.3f,kd,%.3f,filter,%.3f,boost,%.3f,limit,%.3f,min,%.3f,dir,%.0f,weight,%u,print,%u,detail,%u\r\n",
                     (unsigned int)module_vehicle_phototube_runtime.line_mode,
                     (double)(module_vehicle_phototube_runtime.line_base_speed_mm_s * 0.001f),
                     (double)module_vehicle_phototube_runtime.line_kp_mm_s,
                     (double)module_vehicle_phototube_runtime.line_kd_mm_s,
                     (double)module_vehicle_phototube_runtime.line_filter_alpha,
                     (double)module_vehicle_phototube_runtime.line_edge_boost,
                     (double)(module_vehicle_phototube_runtime.line_diff_limit_mm_s * 0.001f),
                     (double)(module_vehicle_phototube_runtime.line_min_speed_mm_s * 0.001f),
                     (double)module_vehicle_phototube_runtime.line_direction,
                     (unsigned int)((module_vehicle_phototube_runtime.line_weight_enabled != FALSE) ? 1u : 0u),
                     (unsigned int)module_vehicle_phototube_runtime.line_print_divider,
                     (unsigned int)((module_vehicle_phototube_runtime.line_detail_enabled != FALSE) ? 1u : 0u));
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "off") != FALSE)
        || (module_vehicle_phototube_command_is(argv[1u], "stop") != FALSE))
    {
        module_vehicle_phototube_line_stop();
        tools_printf("{ptline}off\r\n");
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "push") != FALSE)
        || (module_vehicle_phototube_command_is(argv[1u], "manual") != FALSE))
    {
        module_vehicle_phototube_line_stop();
        module_vehicle_phototube_runtime.line_mode = MODULE_VEHICLE_PHOTOTUBE_LINE_PUSH;
        module_vehicle_phototube_runtime.power_user_enabled = TRUE;
        if (module_vehicle_phototube_runtime.line_print_divider < MODULE_VEHICLE_PHOTOTUBE_LINE_PRINT_DIVIDER_MIN)
        {
            module_vehicle_phototube_runtime.line_print_divider = MODULE_VEHICLE_PHOTOTUBE_LINE_PRINT_DIVIDER_MIN;
        }
        module_vehicle_phototube_runtime.line_print_count =
            module_vehicle_phototube_runtime.line_print_divider - 1u;
        module_vehicle_phototube_power_update();
        if (module_vehicle_phototube_runtime.calibration.valid == FALSE)
        {
            tools_printf("{ptline}push,cal_invalid,use_ptadc_or_detail\r\n");
        }
        else
        {
            tools_printf("{ptline}push\r\n");
        }
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "run") != FALSE)
        || (module_vehicle_phototube_command_is(argv[1u], "drive") != FALSE))
    {
        uint32 suction_duty;

        if (module_vehicle_phototube_runtime.calibration.valid == FALSE)
        {
            tools_printf("{ptline}cal_invalid\r\n");
            return;
        }
        if (argc > 2u)
        {
            module_vehicle_phototube_runtime.line_base_speed_mm_s =
                module_vehicle_phototube_command_float_get(argv[2u]) * 1000.0f;
            if (module_vehicle_phototube_runtime.line_base_speed_mm_s < 0.0f)
            {
                module_vehicle_phototube_runtime.line_base_speed_mm_s =
                    -module_vehicle_phototube_runtime.line_base_speed_mm_s;
            }
            module_vehicle_phototube_runtime.line_base_speed_mm_s =
                module_vehicle_phototube_clamp_f32(
                    module_vehicle_phototube_runtime.line_base_speed_mm_s,
                    0.0f,
                    2000.0f);
        }
        suction_duty = (vehicle_control_auto_suction_enabled_get() != FALSE)
                         ? VEHICLE_CONTROL_DEFAULT_RUN_SUCTION_DUTY
                         : 0u;
        if (module_vehicle_phototube_control_speed_test_enable_post(FALSE) == FALSE)
        {
            return;
        }
        if (module_vehicle_phototube_control_suction_post(suction_duty) == FALSE)
        {
            return;
        }
        if (module_vehicle_phototube_control_speed_test_enable_post(TRUE) == FALSE)
        {
            (void)module_vehicle_phototube_control_suction_post(0u);
            return;
        }
        if (module_vehicle_phototube_control_speed_target_post(
                module_vehicle_phototube_runtime.line_base_speed_mm_s,
                module_vehicle_phototube_runtime.line_base_speed_mm_s) == FALSE)
        {
            (void)module_vehicle_phototube_control_speed_test_enable_post(FALSE);
            (void)module_vehicle_phototube_control_suction_post(0u);
            return;
        }
        module_vehicle_phototube_line_control_reset();
        module_vehicle_phototube_line_maneuver_reset(TRUE);
        module_vehicle_phototube_line_lost_reset();
        module_vehicle_phototube_runtime.line_lost_drive_stop_pending = FALSE;
        module_vehicle_phototube_runtime.line_lost_suction_pending = FALSE;
        module_vehicle_phototube_runtime.power_user_enabled = TRUE;
        if (module_vehicle_phototube_runtime.line_print_divider < MODULE_VEHICLE_PHOTOTUBE_LINE_PRINT_DIVIDER_MIN)
        {
            module_vehicle_phototube_runtime.line_print_divider = MODULE_VEHICLE_PHOTOTUBE_LINE_PRINT_DIVIDER_MIN;
        }
        module_vehicle_phototube_runtime.line_mode = MODULE_VEHICLE_PHOTOTUBE_LINE_RUN;
        module_vehicle_phototube_runtime.line_launch_state =
            MODULE_VEHICLE_PHOTOTUBE_LAUNCH_WAIT_START;
        module_vehicle_phototube_runtime.line_print_count = 0u;
        module_vehicle_phototube_power_update();
        tools_printf("{ptline}launch_begin,drag_pwm,750,drag_time,0.400,hold_speed,0.400,hold_time,0.300\r\n");
        tools_printf("{ptline}run,%.3f,suction,%u\r\n",
                     (double)(module_vehicle_phototube_runtime.line_base_speed_mm_s * 0.001f),
                     (unsigned int)suction_duty);
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "kd") != FALSE) && (argc > 2u))
    {
        module_vehicle_phototube_runtime.line_kd_mm_s =
            module_vehicle_phototube_command_float_get(argv[2u]);
        if (module_vehicle_phototube_runtime.line_kd_mm_s < 0.0f)
        {
            module_vehicle_phototube_runtime.line_kd_mm_s =
                -module_vehicle_phototube_runtime.line_kd_mm_s;
        }
        module_vehicle_phototube_line_control_reset();
        tools_printf("{ptline}kd,%.3f\r\n",
                     (double)module_vehicle_phototube_runtime.line_kd_mm_s);
        return;
    }

    if (((module_vehicle_phototube_command_is(argv[1u], "filter") != FALSE)
         || (module_vehicle_phototube_command_is(argv[1u], "filt") != FALSE))
        && (argc > 2u))
    {
        module_vehicle_phototube_runtime.line_filter_alpha =
            module_vehicle_phototube_clamp_f32(module_vehicle_phototube_command_float_get(argv[2u]),
                                               0.05f,
                                               1.0f);
        module_vehicle_phototube_line_control_reset();
        tools_printf("{ptline}filter,%.3f\r\n",
                     (double)module_vehicle_phototube_runtime.line_filter_alpha);
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "boost") != FALSE) && (argc > 2u))
    {
        module_vehicle_phototube_runtime.line_edge_boost =
            module_vehicle_phototube_clamp_f32(module_vehicle_phototube_command_float_get(argv[2u]),
                                               0.0f,
                                               2.0f);
        tools_printf("{ptline}boost,%.3f\r\n",
                     (double)module_vehicle_phototube_runtime.line_edge_boost);
        return;
    }

    if (((module_vehicle_phototube_command_is(argv[1u], "limit") != FALSE)
         || (module_vehicle_phototube_command_is(argv[1u], "diff") != FALSE))
        && (argc > 2u))
    {
        module_vehicle_phototube_runtime.line_diff_limit_mm_s =
            module_vehicle_phototube_command_float_get(argv[2u]) * 1000.0f;
        if (module_vehicle_phototube_runtime.line_diff_limit_mm_s < 0.0f)
        {
            module_vehicle_phototube_runtime.line_diff_limit_mm_s =
                -module_vehicle_phototube_runtime.line_diff_limit_mm_s;
        }
        tools_printf("{ptline}limit,%.3f\r\n",
                     (double)(module_vehicle_phototube_runtime.line_diff_limit_mm_s * 0.001f));
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "min") != FALSE) && (argc > 2u))
    {
        module_vehicle_phototube_runtime.line_min_speed_mm_s =
            module_vehicle_phototube_command_float_get(argv[2u]) * 1000.0f;
        if (module_vehicle_phototube_runtime.line_min_speed_mm_s < 0.0f)
        {
            module_vehicle_phototube_runtime.line_min_speed_mm_s = 0.0f;
        }
        tools_printf("{ptline}min,%.3f\r\n",
                     (double)(module_vehicle_phototube_runtime.line_min_speed_mm_s * 0.001f));
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "kp") != FALSE) && (argc > 2u))
    {
        module_vehicle_phototube_runtime.line_kp_mm_s =
            module_vehicle_phototube_command_float_get(argv[2u]);
        if (module_vehicle_phototube_runtime.line_kp_mm_s < 0.0f)
        {
            module_vehicle_phototube_runtime.line_kp_mm_s =
                -module_vehicle_phototube_runtime.line_kp_mm_s;
        }
        tools_printf("{ptline}kp,%.3f\r\n",
                     (double)module_vehicle_phototube_runtime.line_kp_mm_s);
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "print") != FALSE) && (argc > 2u))
    {
        module_vehicle_phototube_runtime.line_print_divider =
            module_vehicle_phototube_command_uint_get(argv[2u]);
        if (module_vehicle_phototube_runtime.line_print_divider < MODULE_VEHICLE_PHOTOTUBE_LINE_PRINT_DIVIDER_MIN)
        {
            module_vehicle_phototube_runtime.line_print_divider = MODULE_VEHICLE_PHOTOTUBE_LINE_PRINT_DIVIDER_MIN;
        }
        tools_printf("{ptline}print,%u\r\n",
                     (unsigned int)module_vehicle_phototube_runtime.line_print_divider);
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "detail") != FALSE)
        || (module_vehicle_phototube_command_is(argv[1u], "raw") != FALSE))
    {
        if (argc > 2u)
        {
            if ((module_vehicle_phototube_command_is(argv[2u], "on") != FALSE)
                || (module_vehicle_phototube_command_is(argv[2u], "enable") != FALSE)
                || (module_vehicle_phototube_command_is(argv[2u], "1") != FALSE))
            {
                module_vehicle_phototube_runtime.line_detail_enabled = TRUE;
            }
            else if ((module_vehicle_phototube_command_is(argv[2u], "off") != FALSE)
                     || (module_vehicle_phototube_command_is(argv[2u], "disable") != FALSE)
                     || (module_vehicle_phototube_command_is(argv[2u], "0") != FALSE))
            {
                module_vehicle_phototube_runtime.line_detail_enabled = FALSE;
            }
        }

        tools_printf("{ptline}detail,%u\r\n",
                     (unsigned int)((module_vehicle_phototube_runtime.line_detail_enabled != FALSE) ? 1u : 0u));
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "dir") != FALSE) && (argc > 2u))
    {
        module_vehicle_phototube_runtime.line_direction =
            (module_vehicle_phototube_command_float_get(argv[2u]) < 0.0f) ? -1.0f : 1.0f;
        tools_printf("{ptline}dir,%.0f\r\n",
                     (double)module_vehicle_phototube_runtime.line_direction);
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "weight") != FALSE)
        || (module_vehicle_phototube_command_is(argv[1u], "w") != FALSE))
    {
        if (argc > 2u)
        {
            if ((module_vehicle_phototube_command_is(argv[2u], "on") != FALSE)
                || (module_vehicle_phototube_command_is(argv[2u], "enable") != FALSE)
                || (module_vehicle_phototube_command_is(argv[2u], "1") != FALSE))
            {
                module_vehicle_phototube_runtime.line_weight_enabled = TRUE;
            }
            else if ((module_vehicle_phototube_command_is(argv[2u], "off") != FALSE)
                     || (module_vehicle_phototube_command_is(argv[2u], "disable") != FALSE)
                     || (module_vehicle_phototube_command_is(argv[2u], "0") != FALSE))
            {
                module_vehicle_phototube_runtime.line_weight_enabled = FALSE;
            }
        }

        tools_printf("{ptline}weight,%u\r\n",
                     (unsigned int)((module_vehicle_phototube_runtime.line_weight_enabled != FALSE) ? 1u : 0u));
        return;
    }

    tools_printf("{ptline}usage: ptline push|run [mps]|stop|kp <gain>|kd <gain>|filter <0.05-1>|boost <gain>|limit <mps>|min <mps>|dir <-1|1>|weight on|off|print <n>|detail on|off\r\n");
}

void module_vehicle_phototube_turn_command(uint8 argc, uint8* argv[])
{
    module_vehicle_phototube_turn_action_t action;
    uint32 index;

    if ((argc <= 1u)
        || (module_vehicle_phototube_command_is(argv[1u], "show") != FALSE))
    {
        tools_printf("{ptturn}count,%u,next,%u,radius,%.3f,detect,%u,guard,%u,state,%u,action,%s\r\n",
                     (unsigned int)module_vehicle_phototube_runtime.turn_table_count,
                     (unsigned int)module_vehicle_phototube_runtime.turn_event_index,
                     (double)(module_vehicle_phototube_runtime.turn_radius_mm * 0.001f),
                     (unsigned int)((module_vehicle_phototube_runtime.element_detection_enabled
                                     != FALSE) ? 1u : 0u),
                     (unsigned int)((module_vehicle_phototube_runtime.element_guard_enabled
                                     != FALSE) ? 1u : 0u),
                     (unsigned int)module_vehicle_phototube_runtime.maneuver_state,
                     module_vehicle_phototube_turn_action_name(
                         module_vehicle_phototube_runtime.maneuver_action));
        for (index = 0u; index < module_vehicle_phototube_runtime.turn_table_count; index++)
        {
            action = module_vehicle_phototube_runtime.turn_table[index];
            tools_printf("{ptturn}item,%u,%u,%s\r\n",
                         (unsigned int)index,
                         (unsigned int)action,
                         module_vehicle_phototube_turn_action_name(action));
        }
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "guard") != FALSE)
        || (module_vehicle_phototube_command_is(argv[1u], "confirm") != FALSE))
    {
        if (argc > 2u)
        {
            if ((module_vehicle_phototube_command_is(argv[2u], "on") != FALSE)
                || (module_vehicle_phototube_command_is(argv[2u], "enable") != FALSE)
                || (module_vehicle_phototube_command_is(argv[2u], "1") != FALSE))
            {
                module_vehicle_phototube_runtime.element_guard_enabled = TRUE;
            }
            else if ((module_vehicle_phototube_command_is(argv[2u], "off") != FALSE)
                     || (module_vehicle_phototube_command_is(argv[2u], "disable") != FALSE)
                     || (module_vehicle_phototube_command_is(argv[2u], "0") != FALSE)
                     || (module_vehicle_phototube_command_is(argv[2u], "once") != FALSE))
            {
                module_vehicle_phototube_runtime.element_guard_enabled = FALSE;
            }
        }
        tools_printf("{ptturn}guard,%u\r\n",
                     (unsigned int)((module_vehicle_phototube_runtime.element_guard_enabled
                                     != FALSE) ? 1u : 0u));
        return;
    }

    if (module_vehicle_phototube_command_is(argv[1u], "clear") != FALSE)
    {
        for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_TURN_TABLE_MAX; index++)
        {
            module_vehicle_phototube_runtime.turn_table[index] =
                MODULE_VEHICLE_PHOTOTUBE_TURN_EVALUATE;
        }
        module_vehicle_phototube_runtime.turn_table_count = 0u;
        module_vehicle_phototube_runtime.turn_event_index = 0u;
        tools_printf("{ptturn}clear\r\n");
        return;
    }

    if (module_vehicle_phototube_command_is(argv[1u], "reset") != FALSE)
    {
        module_vehicle_phototube_runtime.turn_event_index = 0u;
        tools_printf("{ptturn}reset\r\n");
        return;
    }

    if (module_vehicle_phototube_command_is(argv[1u], "detect") != FALSE)
    {
        if (argc > 2u)
        {
            if ((module_vehicle_phototube_command_is(argv[2u], "on") != FALSE)
                || (module_vehicle_phototube_command_is(argv[2u], "enable") != FALSE)
                || (module_vehicle_phototube_command_is(argv[2u], "1") != FALSE))
            {
                module_vehicle_phototube_runtime.element_detection_enabled = TRUE;
            }
            else if ((module_vehicle_phototube_command_is(argv[2u], "off") != FALSE)
                     || (module_vehicle_phototube_command_is(argv[2u], "disable") != FALSE)
                     || (module_vehicle_phototube_command_is(argv[2u], "0") != FALSE))
            {
                module_vehicle_phototube_runtime.element_detection_enabled = FALSE;
                module_vehicle_phototube_runtime.element_candidate_s = 0.0f;
                module_vehicle_phototube_runtime.element_candidate_count = 0u;
                module_vehicle_phototube_runtime.element_entry_heading_valid = FALSE;
                if (module_vehicle_phototube_runtime.maneuver_state
                    == MODULE_VEHICLE_PHOTOTUBE_MANEUVER_FOLLOW)
                {
                    module_vehicle_phototube_runtime.line_follow_blend = 0.0f;
                    module_vehicle_phototube_line_control_reset();
                }
            }
        }
        tools_printf("{ptturn}detect,%u\r\n",
                     (unsigned int)((module_vehicle_phototube_runtime.element_detection_enabled
                                     != FALSE) ? 1u : 0u));
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "radius") != FALSE)
        && (argc > 2u))
    {
        module_vehicle_phototube_runtime.turn_radius_mm =
            module_vehicle_phototube_clamp_f32(
                module_vehicle_phototube_command_float_get(argv[2u]) * 1000.0f,
                20.0f,
                2000.0f);
        tools_printf("{ptturn}radius,%.3f\r\n",
                     (double)(module_vehicle_phototube_runtime.turn_radius_mm * 0.001f));
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "add") != FALSE)
        && (argc > 2u))
    {
        if ((module_vehicle_phototube_runtime.turn_table_count
             >= MODULE_VEHICLE_PHOTOTUBE_TURN_TABLE_MAX)
            || (module_vehicle_phototube_turn_action_parse(argv[2u], &action) == FALSE))
        {
            tools_printf("{ptturn}add_failed\r\n");
            return;
        }
        index = module_vehicle_phototube_runtime.turn_table_count;
        module_vehicle_phototube_runtime.turn_table[index] = action;
        module_vehicle_phototube_runtime.turn_table_count++;
        tools_printf("{ptturn}item,%u,%u,%s\r\n",
                     (unsigned int)index,
                     (unsigned int)action,
                     module_vehicle_phototube_turn_action_name(action));
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "set") != FALSE)
        && (argc > 3u))
    {
        index = module_vehicle_phototube_command_uint_get(argv[2u]);
        if ((index >= MODULE_VEHICLE_PHOTOTUBE_TURN_TABLE_MAX)
            || (module_vehicle_phototube_turn_action_parse(argv[3u], &action) == FALSE))
        {
            tools_printf("{ptturn}set_failed\r\n");
            return;
        }
        while (module_vehicle_phototube_runtime.turn_table_count <= index)
        {
            module_vehicle_phototube_runtime.turn_table[
                module_vehicle_phototube_runtime.turn_table_count] =
                MODULE_VEHICLE_PHOTOTUBE_TURN_EVALUATE;
            module_vehicle_phototube_runtime.turn_table_count++;
        }
        module_vehicle_phototube_runtime.turn_table[index] = action;
        tools_printf("{ptturn}item,%u,%u,%s\r\n",
                     (unsigned int)index,
                     (unsigned int)action,
                     module_vehicle_phototube_turn_action_name(action));
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "table") != FALSE)
        && (argc > 2u))
    {
        uint32 next_count = (uint32)argc - 2u;

        if (next_count > MODULE_VEHICLE_PHOTOTUBE_TURN_TABLE_MAX)
        {
            tools_printf("{ptturn}table_too_long,%u,%u\r\n",
                         (unsigned int)next_count,
                         (unsigned int)MODULE_VEHICLE_PHOTOTUBE_TURN_TABLE_MAX);
            return;
        }
        for (index = 0u; index < next_count; index++)
        {
            if (module_vehicle_phototube_turn_action_parse(argv[index + 2u], &action) == FALSE)
            {
                tools_printf("{ptturn}table_invalid,%u\r\n", (unsigned int)index);
                return;
            }
        }
        for (index = 0u; index < next_count; index++)
        {
            (void)module_vehicle_phototube_turn_action_parse(argv[index + 2u], &action);
            module_vehicle_phototube_runtime.turn_table[index] = action;
        }
        module_vehicle_phototube_runtime.turn_table_count = next_count;
        module_vehicle_phototube_runtime.turn_event_index = 0u;
        tools_printf("{ptturn}table,%u\r\n", (unsigned int)next_count);
        return;
    }

    tools_printf("{ptturn}usage: ptturn show|clear|reset|detect on|off|guard on|off|radius <m>|add <0|1|2|3>|set <index> <0|1|2|3>|table <actions...>\r\n");
}

void module_vehicle_phototube_map_command(uint8 argc, uint8* argv[])
{
    uint8 next_map[16u];

    if ((argc <= 1u) || (module_vehicle_phototube_command_is(argv[1u], "show") != FALSE))
    {
        module_vehicle_phototube_map_print();
        return;
    }

    if (module_vehicle_phototube_command_is(argv[1u], "reset") != FALSE)
    {
        module_vehicle_phototube_map_apply(module_vehicle_phototube_default_map);
        module_vehicle_phototube_map_print();
        return;
    }

    if (module_vehicle_phototube_command_is(argv[1u], "set") != FALSE)
    {
        if (module_vehicle_phototube_map_parse(argc, argv, next_map) == FALSE)
        {
            tools_printf("{ptmap}usage: ptmap set <logical0_raw> ... <logical15_raw>\r\n");
            return;
        }

        if (module_vehicle_phototube_map_validate(next_map) == FALSE)
        {
            tools_printf("{ptmap}invalid\r\n");
            return;
        }

        module_vehicle_phototube_map_apply(next_map);
        module_vehicle_phototube_map_print();
        return;
    }

    tools_printf("{ptmap}usage: ptmap show|reset|set <logical0_raw> ... <logical15_raw>\r\n");
}

void module_vehicle_phototube_adc_command(uint8 argc, uint8* argv[])
{
    (void)argc;
    (void)argv;

    module_vehicle_phototube_adc_print();
}

void module_vehicle_phototube_power_command(uint8 argc, uint8* argv[])
{
    if ((argc <= 1u)
        || (module_vehicle_phototube_command_is(argv[1u], "show") != FALSE)
        || (module_vehicle_phototube_command_is(argv[1u], "status") != FALSE))
    {
        module_vehicle_phototube_power_show();
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "on") != FALSE)
        || (module_vehicle_phototube_command_is(argv[1u], "enable") != FALSE)
        || (module_vehicle_phototube_command_is(argv[1u], "1") != FALSE))
    {
        module_vehicle_phototube_runtime.power_user_enabled = TRUE;
        module_vehicle_phototube_power_update();
        module_vehicle_phototube_power_show();
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "off") != FALSE)
        || (module_vehicle_phototube_command_is(argv[1u], "disable") != FALSE)
        || (module_vehicle_phototube_command_is(argv[1u], "0") != FALSE))
    {
        module_vehicle_phototube_runtime.power_user_enabled = FALSE;
        module_vehicle_phototube_runtime.calibration_active = FALSE;
        module_vehicle_phototube_runtime.auto_state = MODULE_VEHICLE_PHOTOTUBE_AUTO_IDLE;
        module_vehicle_phototube_line_stop();
        module_vehicle_phototube_correction_gate_reset();
        module_vehicle_phototube_power_update();
        module_vehicle_phototube_power_show();
        return;
    }

    tools_printf("{ptpower}usage: ptpower on|off|show\r\n");
}

void module_vehicle_phototube_correction_command(uint8 argc, uint8* argv[])
{
    if (argc <= 1u)
    {
        module_vehicle_phototube_correction_show();
        return;
    }

    if (module_vehicle_phototube_command_is(argv[1u], "on") != FALSE)
    {
        module_vehicle_phototube_runtime.power_user_enabled = TRUE;
        module_vehicle_phototube_runtime.correction_monitor_enabled = FALSE;
        module_vehicle_phototube_runtime.correction_enabled = TRUE;
        module_vehicle_phototube_power_update();
        module_vehicle_phototube_correction_show();
        return;
    }

    if (module_vehicle_phototube_command_is(argv[1u], "off") != FALSE)
    {
        module_vehicle_phototube_runtime.correction_enabled = FALSE;
        module_vehicle_phototube_correction_gate_reset();
        module_vehicle_phototube_power_update();
        module_vehicle_phototube_correction_show();
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "monitor") != FALSE)
        && (argc > 2u))
    {
        if ((module_vehicle_phototube_command_is(argv[2u], "on") != FALSE)
            || (module_vehicle_phototube_command_is(argv[2u], "1") != FALSE))
        {
            module_vehicle_phototube_runtime.power_user_enabled = TRUE;
            module_vehicle_phototube_runtime.correction_enabled = FALSE;
            module_vehicle_phototube_runtime.correction_monitor_enabled = TRUE;
            module_vehicle_phototube_runtime.correction_print_enabled = TRUE;
        }
        else if ((module_vehicle_phototube_command_is(argv[2u], "off") != FALSE)
                 || (module_vehicle_phototube_command_is(argv[2u], "0") != FALSE))
        {
            module_vehicle_phototube_runtime.correction_monitor_enabled = FALSE;
            module_vehicle_phototube_runtime.correction_print_enabled = FALSE;
            module_vehicle_phototube_correction_gate_reset();
        }
        module_vehicle_phototube_power_update();
        module_vehicle_phototube_correction_show();
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "show") != FALSE)
        || (module_vehicle_phototube_command_is(argv[1u], "status") != FALSE))
    {
        module_vehicle_phototube_correction_show();
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "print") != FALSE)
        && (argc > 2u))
    {
        if ((module_vehicle_phototube_command_is(argv[2u], "on") != FALSE)
            || (module_vehicle_phototube_command_is(argv[2u], "1") != FALSE))
        {
            module_vehicle_phototube_runtime.correction_print_enabled = TRUE;
        }
        else if ((module_vehicle_phototube_command_is(argv[2u], "off") != FALSE)
                 || (module_vehicle_phototube_command_is(argv[2u], "0") != FALSE))
        {
            module_vehicle_phototube_runtime.correction_print_enabled = FALSE;
        }
        else
        {
            module_vehicle_phototube_runtime.correction_print_divider =
                module_vehicle_phototube_command_uint_get(argv[2u]);
            module_vehicle_phototube_runtime.correction_print_enabled = TRUE;
            if (module_vehicle_phototube_runtime.correction_print_divider == 0u)
            {
                module_vehicle_phototube_runtime.correction_print_divider = 1u;
            }
        }
        module_vehicle_phototube_correction_show();
        return;
    }

    if (module_vehicle_phototube_command_is(argv[1u], "gate") != FALSE)
    {
        if (argc > 2u)
        {
            if ((module_vehicle_phototube_command_is(argv[2u], "on") != FALSE)
                || (module_vehicle_phototube_command_is(argv[2u], "1") != FALSE))
            {
                module_vehicle_phototube_runtime.correction_gate_enabled = TRUE;
            }
            else if ((module_vehicle_phototube_command_is(argv[2u], "off") != FALSE)
                     || (module_vehicle_phototube_command_is(argv[2u], "0") != FALSE))
            {
                module_vehicle_phototube_runtime.correction_gate_enabled = FALSE;
            }
        }
        module_vehicle_phototube_correction_gate_reset();
        module_vehicle_phototube_correction_show();
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "yaw") != FALSE)
        && (argc > 2u))
    {
        module_vehicle_phototube_runtime.correction_gate_yaw_rate_max_rad_s =
            module_vehicle_phototube_clamp_f32(module_vehicle_phototube_command_float_get(argv[2u]),
                                               0.0f,
                                               1000.0f)
            * MODULE_VEHICLE_PHOTOTUBE_DEG_TO_RAD;
        module_vehicle_phototube_correction_gate_reset();
        module_vehicle_phototube_correction_show();
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "heading") != FALSE)
        && (argc > 2u))
    {
        module_vehicle_phototube_runtime.correction_gate_heading_delta_max_rad =
            module_vehicle_phototube_clamp_f32(module_vehicle_phototube_command_float_get(argv[2u]),
                                               0.0f,
                                               180.0f)
            * MODULE_VEHICLE_PHOTOTUBE_DEG_TO_RAD;
        module_vehicle_phototube_correction_gate_reset();
        module_vehicle_phototube_correction_show();
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "stable") != FALSE)
        && (argc > 2u))
    {
        module_vehicle_phototube_runtime.correction_gate_stable_required_count =
            module_vehicle_phototube_command_uint_get(argv[2u]);
        module_vehicle_phototube_correction_gate_reset();
        module_vehicle_phototube_correction_show();
        return;
    }

    if (((module_vehicle_phototube_command_is(argv[1u], "sensor") != FALSE)
         || (module_vehicle_phototube_command_is(argv[1u], "front") != FALSE)
         || (module_vehicle_phototube_command_is(argv[1u], "x") != FALSE))
        && (argc > 2u))
    {
        module_vehicle_phototube_runtime.correction_sensor_x_mm =
            module_vehicle_phototube_command_float_get(argv[2u]);
        module_vehicle_phototube_correction_show();
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "spacing") != FALSE)
        && (argc > 2u))
    {
        module_vehicle_phototube_runtime.correction_sensor_spacing_mm =
            module_vehicle_phototube_clamp_f32(module_vehicle_phototube_command_float_get(argv[2u]),
                                               0.5f,
                                               20.0f);
        module_vehicle_phototube_correction_show();
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "sign") != FALSE)
        && (argc > 2u))
    {
        module_vehicle_phototube_runtime.correction_sensor_y_sign =
            (module_vehicle_phototube_command_float_get(argv[2u]) < 0.0f) ? -1.0f : 1.0f;
        module_vehicle_phototube_correction_show();
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "gain") != FALSE)
        && (argc > 2u))
    {
        module_vehicle_phototube_runtime.correction_gain =
            module_vehicle_phototube_clamp_f32(module_vehicle_phototube_command_float_get(argv[2u]),
                                               0.0f,
                                               1.0f);
        module_vehicle_phototube_correction_show();
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "step") != FALSE)
        && (argc > 2u))
    {
        module_vehicle_phototube_runtime.correction_max_step_mm =
            module_vehicle_phototube_clamp_f32(module_vehicle_phototube_command_float_get(argv[2u]),
                                               0.0f,
                                               20.0f);
        module_vehicle_phototube_correction_show();
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "window") != FALSE)
        && (argc > 2u))
    {
        module_vehicle_phototube_runtime.correction_window_mm =
            module_vehicle_phototube_clamp_f32(module_vehicle_phototube_command_float_get(argv[2u]),
                                               20.0f,
                                               1000.0f);
        module_vehicle_phototube_correction_show();
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "reject") != FALSE)
        && (argc > 2u))
    {
        module_vehicle_phototube_runtime.correction_reject_distance_mm =
            module_vehicle_phototube_clamp_f32(module_vehicle_phototube_command_float_get(argv[2u]),
                                               1.0f,
                                               1000.0f);
        module_vehicle_phototube_correction_show();
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "conf") != FALSE)
        && (argc > 2u))
    {
        module_vehicle_phototube_runtime.correction_min_confidence =
            module_vehicle_phototube_clamp_f32(module_vehicle_phototube_command_float_get(argv[2u]),
                                               0.0f,
                                               1.0f);
        module_vehicle_phototube_correction_show();
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "threshold") != FALSE)
        && (argc > 2u))
    {
        module_vehicle_phototube_runtime.correction_strength_threshold =
            (uint16)module_vehicle_phototube_clamp_f32(
                module_vehicle_phototube_command_float_get(argv[2u]),
                0.0f,
                (float32)MODULE_VEHICLE_PHOTOTUBE_NORM_MAX);
        module_vehicle_phototube_correction_show();
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "summax") != FALSE)
        && (argc > 2u))
    {
        module_vehicle_phototube_runtime.correction_sum_max =
            module_vehicle_phototube_command_uint_get(argv[2u]);
        module_vehicle_phototube_correction_show();
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "countmax") != FALSE)
        && (argc > 2u))
    {
        module_vehicle_phototube_runtime.correction_active_count_max =
            module_vehicle_phototube_command_uint_get(argv[2u]);
        module_vehicle_phototube_correction_show();
        return;
    }

    if ((module_vehicle_phototube_command_is(argv[1u], "widthmax") != FALSE)
        && (argc > 2u))
    {
        module_vehicle_phototube_runtime.correction_active_width_max_mm =
            module_vehicle_phototube_clamp_f32(module_vehicle_phototube_command_float_get(argv[2u]),
                                               0.0f,
                                               200.0f);
        module_vehicle_phototube_correction_show();
        return;
    }

    tools_printf("{ptcorr}usage:on|off|monitor on|off|print on|off|<n>|gate on|off|yaw <deg_s>|heading <deg>|stable <frames>|sensor <mm>|spacing <mm>|sign <-1|1>|gain <0-1>|step <mm>|window <mm>|reject <mm>|conf <0-1>|threshold <0-1000>|summax <n>|countmax <n>|widthmax <mm>\r\n");
}

boolean module_vehicle_phototube_calibration_flash_save(void)
{
    return module_vehicle_phototube_calibration_save_to_flash();
}

const module_vehicle_phototube_calibration_t* module_vehicle_phototube_calibration_get(void)
{
    return &module_vehicle_phototube_runtime.calibration;
}

boolean module_vehicle_phototube_line_observation_get(
    module_vehicle_phototube_line_observation_t* observation)
{
    return module_vehicle_phototube_line_observation_calculate(observation);
}

void module_vehicle_phototube_values_get(uint16 values[16u])
{
    uint32 index;

    if (values == NULL_PTR)
    {
        return;
    }

    for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
    {
        values[index] = module_vehicle_phototube_runtime.value[index];
    }
}

void module_vehicle_phototube_calibrated_values_get(uint16 values[16u])
{
    const module_vehicle_phototube_calibration_t *calibration =
        &module_vehicle_phototube_runtime.calibration;
    uint32 index;

    if (values == NULL_PTR)
    {
        return;
    }

    for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
    {
        sint32 value = (sint32)module_vehicle_phototube_runtime.value[index];

        if (calibration->valid != FALSE)
        {
            value -= (sint32)calibration->offset_value[index];
            value = (value * (sint32)calibration->gain_q15[index])
                    / (sint32)MODULE_VEHICLE_PHOTOTUBE_GAIN_Q15_BASE;
        }

        values[index] = module_vehicle_phototube_clamp_u16(value);
    }
}

void module_vehicle_phototube_normalized_values_get(uint16 values[16u])
{
    uint32 index;

    if (values == NULL_PTR)
    {
        return;
    }

    for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
    {
        values[index] = module_vehicle_phototube_normalized_value_get(index);
    }
}

static uint16 module_vehicle_phototube_value_read(uint32 phototube_index)
{
    device_phototube_runtime_t *runtime = device_phototube_runtime_table_get();

    return (uint16)(runtime[phototube_index].dma_receive_buffer[0u]
                    >> MODULE_VEHICLE_PHOTOTUBE_ADC_SHIFT);
}

static void module_vehicle_phototube_frame_capture(void)
{
    uint32 index;

    for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
    {
        uint32 physical_index =
            (uint32)module_vehicle_phototube_runtime.physical_map[index];

        module_vehicle_phototube_runtime.value[index] =
            module_vehicle_phototube_value_read(physical_index);
    }
}

static void module_vehicle_phototube_map_reset(void)
{
    uint32 index;

    for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
    {
        module_vehicle_phototube_runtime.physical_map[index] =
            module_vehicle_phototube_default_map[index];
    }
}

static boolean module_vehicle_phototube_map_validate(const uint8 map[16u])
{
    boolean used[16u];
    uint32 index;

    if (map == NULL_PTR)
    {
        return FALSE;
    }

    for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
    {
        used[index] = FALSE;
    }

    for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
    {
        uint8 physical_index = map[index];

        if (physical_index >= MODULE_VEHICLE_PHOTOTUBE_COUNT)
        {
            return FALSE;
        }
        if (used[physical_index] != FALSE)
        {
            return FALSE;
        }
        used[physical_index] = TRUE;
    }

    return TRUE;
}

static boolean module_vehicle_phototube_map_parse(uint8 argc, uint8* argv[], uint8 map[16u])
{
    uint32 value[16u];
    uint32 index;
    boolean has_zero = FALSE;
    boolean all_one_based_range = TRUE;

    if ((argv == NULL_PTR) || (map == NULL_PTR))
    {
        return FALSE;
    }
    if (argc < (MODULE_VEHICLE_PHOTOTUBE_COUNT + 2u))
    {
        return FALSE;
    }

    for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
    {
        value[index] = module_vehicle_phototube_command_uint_get(argv[index + 2u]);
        if (value[index] == 0u)
        {
            has_zero = TRUE;
        }
        if ((value[index] < 1u) || (value[index] > MODULE_VEHICLE_PHOTOTUBE_COUNT))
        {
            all_one_based_range = FALSE;
        }
    }

    if ((has_zero == FALSE) && (all_one_based_range != FALSE))
    {
        for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
        {
            map[index] = (uint8)(value[index] - 1u);
        }
    }
    else
    {
        for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
        {
            if (value[index] >= MODULE_VEHICLE_PHOTOTUBE_COUNT)
            {
                return FALSE;
            }
            map[index] = (uint8)value[index];
        }
    }

    return TRUE;
}

static void module_vehicle_phototube_map_apply(const uint8 map[16u])
{
    uint32 index;

    if (map == NULL_PTR)
    {
        return;
    }

    module_vehicle_phototube_line_stop();
    module_vehicle_phototube_runtime.calibration_active = FALSE;
    module_vehicle_phototube_runtime.auto_state = MODULE_VEHICLE_PHOTOTUBE_AUTO_IDLE;
    module_vehicle_phototube_runtime.calibration.valid = FALSE;

    for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
    {
        module_vehicle_phototube_runtime.physical_map[index] = map[index];
    }
    module_vehicle_phototube_power_update();
}

static void module_vehicle_phototube_map_print(void)
{
    tools_printf("{ptmap}%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                 (unsigned int)module_vehicle_phototube_runtime.physical_map[0u],
                 (unsigned int)module_vehicle_phototube_runtime.physical_map[1u],
                 (unsigned int)module_vehicle_phototube_runtime.physical_map[2u],
                 (unsigned int)module_vehicle_phototube_runtime.physical_map[3u],
                 (unsigned int)module_vehicle_phototube_runtime.physical_map[4u],
                 (unsigned int)module_vehicle_phototube_runtime.physical_map[5u],
                 (unsigned int)module_vehicle_phototube_runtime.physical_map[6u],
                 (unsigned int)module_vehicle_phototube_runtime.physical_map[7u],
                 (unsigned int)module_vehicle_phototube_runtime.physical_map[8u],
                 (unsigned int)module_vehicle_phototube_runtime.physical_map[9u],
                 (unsigned int)module_vehicle_phototube_runtime.physical_map[10u],
                 (unsigned int)module_vehicle_phototube_runtime.physical_map[11u],
                 (unsigned int)module_vehicle_phototube_runtime.physical_map[12u],
                 (unsigned int)module_vehicle_phototube_runtime.physical_map[13u],
                 (unsigned int)module_vehicle_phototube_runtime.physical_map[14u],
                 (unsigned int)module_vehicle_phototube_runtime.physical_map[15u]);
}

static void module_vehicle_phototube_adc_values_get(uint16 values[16u])
{
    uint32 index;

    if (values == NULL_PTR)
    {
        return;
    }

    for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
    {
        values[index] = module_vehicle_phototube_value_read(index);
    }
}

static void module_vehicle_phototube_adc_print(void)
{
    uint16 adc[16u];

    module_vehicle_phototube_adc_values_get(adc);
    tools_printf("{ptadc}%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                 (unsigned int)adc[0u],
                 (unsigned int)adc[1u],
                 (unsigned int)adc[2u],
                 (unsigned int)adc[3u],
                 (unsigned int)adc[4u],
                 (unsigned int)adc[5u],
                 (unsigned int)adc[6u],
                 (unsigned int)adc[7u],
                 (unsigned int)adc[8u],
                 (unsigned int)adc[9u],
                 (unsigned int)adc[10u],
                 (unsigned int)adc[11u],
                 (unsigned int)adc[12u],
                 (unsigned int)adc[13u],
                 (unsigned int)adc[14u],
                 (unsigned int)adc[15u]);
}

static void module_vehicle_phototube_calibration_start(void)
{
    uint32 index;

    if (module_vehicle_phototube_runtime.drive_cal_state
        != MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_IDLE)
    {
        tools_printf("{ptcal}busy,drive\r\n");
        return;
    }

    module_vehicle_phototube_runtime.power_user_enabled = TRUE;
    module_vehicle_phototube_runtime.calibration_active = TRUE;
    module_vehicle_phototube_runtime.auto_state = MODULE_VEHICLE_PHOTOTUBE_AUTO_IDLE;
    module_vehicle_phototube_runtime.calibration.valid = FALSE;
    module_vehicle_phototube_runtime.calibration.sample_count = 0u;

    for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
    {
        module_vehicle_phototube_runtime.calibration.blue_value[index] = 0u;
        module_vehicle_phototube_runtime.calibration.white_value[index] = 0u;
        module_vehicle_phototube_runtime.calibration.min_value[index] =
            MODULE_VEHICLE_PHOTOTUBE_ADC_MAX;
        module_vehicle_phototube_runtime.calibration.max_value[index] = 0u;
        module_vehicle_phototube_runtime.calibration.center_value[index] = 0u;
        module_vehicle_phototube_runtime.calibration.span_value[index] = 0u;
        module_vehicle_phototube_runtime.calibration.offset_value[index] = 0;
        module_vehicle_phototube_runtime.calibration.gain_q15[index] =
            MODULE_VEHICLE_PHOTOTUBE_GAIN_Q15_BASE;
    }

    tools_printf("{ptcal}start\r\n");
}

static void module_vehicle_phototube_calibration_auto_start(void)
{
    uint32 index;

    if (module_vehicle_phototube_runtime.drive_cal_state
        != MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_IDLE)
    {
        tools_printf("{ptcal}busy,drive\r\n");
        return;
    }

    module_vehicle_phototube_runtime.power_user_enabled = TRUE;
    module_vehicle_phototube_runtime.calibration_active = FALSE;
    module_vehicle_phototube_runtime.auto_state = MODULE_VEHICLE_PHOTOTUBE_AUTO_BLUE;
    module_vehicle_phototube_runtime.auto_line_seen = FALSE;
    module_vehicle_phototube_runtime.auto_back_stable_count = 0u;
    module_vehicle_phototube_runtime.calibration.valid = FALSE;
    module_vehicle_phototube_runtime.calibration.sample_count = 0u;

    for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
    {
        module_vehicle_phototube_runtime.auto_blue_sum[index] = 0u;
        module_vehicle_phototube_runtime.calibration.blue_value[index] = 0u;
        module_vehicle_phototube_runtime.calibration.white_value[index] = 0u;
        module_vehicle_phototube_runtime.calibration.min_value[index] =
            MODULE_VEHICLE_PHOTOTUBE_ADC_MAX;
        module_vehicle_phototube_runtime.calibration.max_value[index] = 0u;
        module_vehicle_phototube_runtime.calibration.center_value[index] = 0u;
        module_vehicle_phototube_runtime.calibration.span_value[index] = 0u;
        module_vehicle_phototube_runtime.calibration.offset_value[index] = 0;
        module_vehicle_phototube_runtime.calibration.gain_q15[index] =
            MODULE_VEHICLE_PHOTOTUBE_GAIN_Q15_BASE;
    }

    tools_printf("{ptcal}auto_blue\r\n");
}

static void module_vehicle_phototube_calibration_abort(void)
{
    if (module_vehicle_phototube_runtime.drive_cal_state
        != MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_IDLE)
    {
        module_vehicle_phototube_drive_calibration_abort("user_abort");
        return;
    }
    module_vehicle_phototube_runtime.calibration_active = FALSE;
    module_vehicle_phototube_runtime.auto_state = MODULE_VEHICLE_PHOTOTUBE_AUTO_IDLE;
    module_vehicle_phototube_power_update();
    tools_printf("{ptcal}abort,%u\r\n",
                 (unsigned int)module_vehicle_phototube_runtime.calibration.sample_count);
}

static void module_vehicle_phototube_calibration_update(void)
{
    module_vehicle_phototube_calibration_t *calibration =
        &module_vehicle_phototube_runtime.calibration;
    uint32 index;

    calibration->sample_count++;
    for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
    {
        uint16 value = module_vehicle_phototube_runtime.value[index];

        if (value < calibration->min_value[index])
        {
            calibration->min_value[index] = value;
        }
        if (value > calibration->max_value[index])
        {
            calibration->max_value[index] = value;
        }
    }
}

static void module_vehicle_phototube_calibration_auto_update(void)
{
    module_vehicle_phototube_calibration_t *calibration =
        &module_vehicle_phototube_runtime.calibration;
    uint32 index;
    uint32 max_delta = 0u;
    uint32 blue_channel_count = 0u;

    if (module_vehicle_phototube_runtime.auto_state == MODULE_VEHICLE_PHOTOTUBE_AUTO_BLUE)
    {
        calibration->sample_count++;
        for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
        {
            module_vehicle_phototube_runtime.auto_blue_sum[index] +=
                module_vehicle_phototube_runtime.value[index];
        }

        if (calibration->sample_count >= MODULE_VEHICLE_PHOTOTUBE_AUTO_BLUE_SAMPLE_COUNT)
        {
            for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
            {
                uint16 blue_value =
                    (uint16)(module_vehicle_phototube_runtime.auto_blue_sum[index]
                             / MODULE_VEHICLE_PHOTOTUBE_AUTO_BLUE_SAMPLE_COUNT);
                calibration->blue_value[index] = blue_value;
                calibration->min_value[index] = blue_value;
                calibration->max_value[index] = blue_value;
            }
            calibration->sample_count = 0u;
            module_vehicle_phototube_runtime.auto_state = MODULE_VEHICLE_PHOTOTUBE_AUTO_SCAN;
            tools_printf("{ptcal}auto_scan\r\n");
        }
        return;
    }

    calibration->sample_count++;
    for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
    {
        uint16 value = module_vehicle_phototube_runtime.value[index];
        uint16 blue_value = calibration->blue_value[index];
        uint32 delta = (value >= blue_value)
                     ? ((uint32)value - (uint32)blue_value)
                     : ((uint32)blue_value - (uint32)value);

        if (value < calibration->min_value[index])
        {
            calibration->min_value[index] = value;
        }
        if (value > calibration->max_value[index])
        {
            calibration->max_value[index] = value;
        }
        if (delta > max_delta)
        {
            max_delta = delta;
        }
        if (delta <= MODULE_VEHICLE_PHOTOTUBE_AUTO_BACK_DELTA)
        {
            blue_channel_count++;
        }
    }

    if (max_delta >= MODULE_VEHICLE_PHOTOTUBE_AUTO_LINE_DELTA)
    {
        module_vehicle_phototube_runtime.auto_line_seen = TRUE;
    }

    if ((module_vehicle_phototube_runtime.auto_line_seen != FALSE)
        && (blue_channel_count >= MODULE_VEHICLE_PHOTOTUBE_AUTO_BACK_CHANNEL_COUNT)
        && (max_delta < MODULE_VEHICLE_PHOTOTUBE_AUTO_LINE_DELTA))
    {
        module_vehicle_phototube_runtime.auto_back_stable_count++;
        if (module_vehicle_phototube_runtime.auto_back_stable_count
            >= MODULE_VEHICLE_PHOTOTUBE_AUTO_BACK_STABLE_COUNT)
        {
            (void)module_vehicle_phototube_calibration_finish();
        }
    }
    else if (module_vehicle_phototube_runtime.auto_back_stable_count > 0u)
    {
        module_vehicle_phototube_runtime.auto_back_stable_count--;
    }
}

static void module_vehicle_phototube_drive_calibration_start(float32 speed_mm_s)
{
    const vehicle_path_state_t* path_state = module_vehicle_path_state_get();
    uint32 index;

    if (module_vehicle_phototube_runtime.drive_cal_state
        != MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_IDLE)
    {
        tools_printf("{ptcaldrive}busy,%s\r\n",
                     module_vehicle_phototube_drive_calibration_state_name(
                         module_vehicle_phototube_runtime.drive_cal_state));
        return;
    }
    if ((path_state->recording != FALSE) || (path_state->replaying != FALSE)
        || (module_vehicle_phototube_runtime.line_mode
            != MODULE_VEHICLE_PHOTOTUBE_LINE_OFF)
        || (vehicle_control_speed_test_is_enabled() != FALSE))
    {
        tools_printf("{ptcaldrive}fail,vehicle_busy\r\n");
        return;
    }

    speed_mm_s = module_vehicle_phototube_clamp_f32(
        speed_mm_s,
        MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_SPEED_MIN_MM_S,
        MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_SPEED_MAX_MM_S);
    module_vehicle_phototube_runtime.drive_cal_backup =
        module_vehicle_phototube_runtime.calibration;
    module_vehicle_phototube_runtime.drive_cal_speed_mm_s = speed_mm_s;
    module_vehicle_phototube_runtime.drive_cal_start_tick = sysTick_getTick(SYSTICK1);
    module_vehicle_phototube_runtime.drive_cal_state_start_tick =
        module_vehicle_phototube_runtime.drive_cal_start_tick;
    module_vehicle_phototube_runtime.drive_cal_state_start_distance_mm =
        module_vehicle_pose_fusion_observation_get()->distance_mm;
    module_vehicle_phototube_runtime.drive_cal_state =
        MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_BLUE_SAMPLE;
    module_vehicle_phototube_runtime.drive_cal_confirm_count = 0u;
    module_vehicle_phototube_runtime.drive_cal_peak_confirm_count = 0u;
    module_vehicle_phototube_runtime.drive_cal_peak_delta = 0u;
    module_vehicle_phototube_runtime.drive_cal_sample_count = 0u;
    module_vehicle_phototube_runtime.drive_cal_launch_seen = FALSE;
    module_vehicle_phototube_runtime.calibration_active = FALSE;
    module_vehicle_phototube_runtime.auto_state = MODULE_VEHICLE_PHOTOTUBE_AUTO_IDLE;
    module_vehicle_phototube_runtime.calibration.valid = FALSE;
    module_vehicle_phototube_runtime.calibration.sample_count = 0u;
    module_vehicle_phototube_runtime.power_user_enabled = TRUE;

    for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
    {
        module_vehicle_phototube_runtime.auto_blue_sum[index] = 0u;
        module_vehicle_phototube_runtime.drive_cal_white_sum[index] = 0u;
        module_vehicle_phototube_runtime.drive_cal_final_blue_sum[index] = 0u;
    }
    tools_printf("{ptcaldrive}blue_sample,0,%u,speed,%.3f\r\n",
                 (unsigned int)MODULE_VEHICLE_PHOTOTUBE_AUTO_BLUE_SAMPLE_COUNT,
                 (double)(speed_mm_s * 0.001f));
}

static boolean module_vehicle_phototube_drive_calibration_launch(
    module_vehicle_phototube_drive_cal_state_t launch_state)
{
    const module_vehicle_pose_fusion_observation_t* pose =
        module_vehicle_pose_fusion_observation_get();
    uint32 suction_duty = (vehicle_control_auto_suction_enabled_get() != FALSE)
                              ? VEHICLE_CONTROL_DEFAULT_RUN_SUCTION_DUTY
                              : 0u;

    module_vehicle_phototube_runtime.drive_cal_heading_rad = pose->theta_rad;
    if (module_vehicle_phototube_control_speed_test_enable_post(FALSE) == FALSE)
    {
        return FALSE;
    }
    if (module_vehicle_phototube_control_suction_post(suction_duty) == FALSE)
    {
        return FALSE;
    }
    if (module_vehicle_phototube_control_speed_test_enable_post(TRUE) == FALSE)
    {
        return FALSE;
    }
    if (module_vehicle_phototube_control_speed_target_post(
            module_vehicle_phototube_runtime.drive_cal_speed_mm_s,
            module_vehicle_phototube_runtime.drive_cal_speed_mm_s) == FALSE)
    {
        return FALSE;
    }

    module_vehicle_phototube_runtime.drive_cal_state = launch_state;
    module_vehicle_phototube_runtime.drive_cal_launch_seen = FALSE;
    module_vehicle_phototube_runtime.drive_cal_confirm_count = 0u;
    module_vehicle_phototube_runtime.drive_cal_state_start_tick = sysTick_getTick(SYSTICK1);
    module_vehicle_phototube_runtime.drive_cal_state_start_distance_mm = pose->distance_mm;
    tools_printf("{ptcaldrive}launch,%s,speed,%.3f,heading,%.3f,suction,%u\r\n",
                 (launch_state == MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_LAUNCH_WHITE)
                     ? "white"
                     : "blue",
                 (double)(module_vehicle_phototube_runtime.drive_cal_speed_mm_s * 0.001f),
                 (double)(module_vehicle_phototube_runtime.drive_cal_heading_rad
                          * MODULE_VEHICLE_PHOTOTUBE_RAD_TO_DEG),
                 (unsigned int)suction_duty);
    return TRUE;
}

static boolean module_vehicle_phototube_drive_calibration_stop(void)
{
    if (module_vehicle_phototube_control_speed_test_stop_post() == FALSE)
    {
        return FALSE;
    }
    module_vehicle_phototube_runtime.drive_cal_confirm_count = 0u;
    module_vehicle_phototube_runtime.drive_cal_state_start_tick = sysTick_getTick(SYSTICK1);
    module_vehicle_phototube_runtime.drive_cal_state_start_distance_mm =
        module_vehicle_pose_fusion_observation_get()->distance_mm;
    return TRUE;
}

static void module_vehicle_phototube_drive_calibration_update(void)
{
    module_vehicle_phototube_calibration_t* calibration =
        &module_vehicle_phototube_runtime.calibration;
    const module_vehicle_encoder_observation_t* encoder =
        module_vehicle_encoder_observation_get();
    module_vehicle_phototube_drive_cal_state_t state =
        module_vehicle_phototube_runtime.drive_cal_state;
    uint32 index;

    if (module_vehicle_phototube_drive_calibration_elapsed_s(
            module_vehicle_phototube_runtime.drive_cal_start_tick)
        > MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_TOTAL_TIMEOUT_S)
    {
        module_vehicle_phototube_drive_calibration_abort("total_timeout");
        return;
    }

    if (state == MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_BLUE_SAMPLE)
    {
        module_vehicle_phototube_runtime.drive_cal_sample_count++;
        for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
        {
            module_vehicle_phototube_runtime.auto_blue_sum[index] +=
                module_vehicle_phototube_runtime.value[index];
        }
        if (module_vehicle_phototube_runtime.drive_cal_sample_count
            >= MODULE_VEHICLE_PHOTOTUBE_AUTO_BLUE_SAMPLE_COUNT)
        {
            for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
            {
                uint16 blue_value =
                    (uint16)(module_vehicle_phototube_runtime.auto_blue_sum[index]
                             / MODULE_VEHICLE_PHOTOTUBE_AUTO_BLUE_SAMPLE_COUNT);
                calibration->blue_value[index] = blue_value;
                calibration->min_value[index] = blue_value;
                calibration->max_value[index] = blue_value;
            }
            module_vehicle_phototube_runtime.drive_cal_sample_count = 0u;
            if (module_vehicle_phototube_drive_calibration_launch(
                    MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_LAUNCH_WHITE) == FALSE)
            {
                module_vehicle_phototube_drive_calibration_abort("launch_queue");
            }
        }
        return;
    }

    if ((state == MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_LAUNCH_WHITE)
        || (state == MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_LAUNCH_BLUE))
    {
        if (vehicle_control_test_launch_active_get() != FALSE)
        {
            module_vehicle_phototube_runtime.drive_cal_launch_seen = TRUE;
            return;
        }
        if (module_vehicle_phototube_runtime.drive_cal_launch_seen != FALSE)
        {
            if (module_vehicle_phototube_control_heading_target_post(
                    module_vehicle_phototube_runtime.drive_cal_speed_mm_s,
                    module_vehicle_phototube_runtime.drive_cal_heading_rad) == FALSE)
            {
                module_vehicle_phototube_drive_calibration_abort("heading_queue");
                return;
            }
            module_vehicle_phototube_runtime.drive_cal_state =
                (state == MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_LAUNCH_WHITE)
                    ? MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_SEARCH_WHITE
                    : MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_SEARCH_BLUE;
            module_vehicle_phototube_runtime.drive_cal_state_start_tick =
                sysTick_getTick(SYSTICK1);
            module_vehicle_phototube_runtime.drive_cal_state_start_distance_mm =
                module_vehicle_pose_fusion_observation_get()->distance_mm;
            tools_printf("{ptcaldrive}%s\r\n",
                         (state == MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_LAUNCH_WHITE)
                             ? "search_white"
                             : "search_blue");
            return;
        }
        if (module_vehicle_phototube_drive_calibration_elapsed_s(
                module_vehicle_phototube_runtime.drive_cal_state_start_tick)
            > MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_LAUNCH_TIMEOUT_S)
        {
            module_vehicle_phototube_drive_calibration_abort("launch_timeout");
        }
        return;
    }

    if (state == MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_SEARCH_WHITE)
    {
        uint32 max_delta = 0u;

        for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
        {
            uint16 value = module_vehicle_phototube_runtime.value[index];
            uint16 blue = calibration->blue_value[index];
            uint32 delta = (value >= blue) ? ((uint32)value - (uint32)blue)
                                           : ((uint32)blue - (uint32)value);
            if (delta > max_delta)
            {
                max_delta = delta;
            }
        }
        if (max_delta >= MODULE_VEHICLE_PHOTOTUBE_AUTO_LINE_DELTA)
        {
            module_vehicle_phototube_runtime.drive_cal_confirm_count++;
        }
        else if (module_vehicle_phototube_runtime.drive_cal_confirm_count > 0u)
        {
            module_vehicle_phototube_runtime.drive_cal_confirm_count--;
        }
        if (max_delta > module_vehicle_phototube_runtime.drive_cal_peak_delta)
        {
            module_vehicle_phototube_runtime.drive_cal_peak_delta = max_delta;
        }
        if (module_vehicle_phototube_runtime.drive_cal_confirm_count
            > module_vehicle_phototube_runtime.drive_cal_peak_confirm_count)
        {
            module_vehicle_phototube_runtime.drive_cal_peak_confirm_count =
                module_vehicle_phototube_runtime.drive_cal_confirm_count;
        }
        if (module_vehicle_phototube_runtime.drive_cal_confirm_count
            >= MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_WHITE_CONFIRM_COUNT)
        {
            if (module_vehicle_phototube_drive_calibration_stop() == FALSE)
            {
                module_vehicle_phototube_drive_calibration_abort("stop_queue");
                return;
            }
            module_vehicle_phototube_runtime.drive_cal_state =
                MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_WHITE_STOP;
            tools_printf("{ptcaldrive}white_stop,delta,%u,distance,%.1f\r\n",
                         (unsigned int)max_delta,
                         (double)module_vehicle_phototube_drive_calibration_distance_get());
        }
        else if (module_vehicle_phototube_drive_calibration_distance_get()
                 > MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_SEARCH_MAX_MM)
        {
            tools_printf("{ptcaldrive}white_peak,delta,%u,confirm,%u,distance,%.1f\r\n",
                         (unsigned int)module_vehicle_phototube_runtime.drive_cal_peak_delta,
                         (unsigned int)module_vehicle_phototube_runtime.drive_cal_peak_confirm_count,
                         (double)module_vehicle_phototube_drive_calibration_distance_get());
            module_vehicle_phototube_drive_calibration_abort("white_not_found");
        }
        return;
    }

    if ((state == MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_WHITE_STOP)
        || (state == MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_BLUE_STOP))
    {
        boolean stopped = ((encoder->left_speed_mm_s
                            < MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_STOP_SPEED_MM_S)
                           && (encoder->left_speed_mm_s
                               > -MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_STOP_SPEED_MM_S)
                           && (encoder->right_speed_mm_s
                               < MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_STOP_SPEED_MM_S)
                           && (encoder->right_speed_mm_s
                               > -MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_STOP_SPEED_MM_S))
                              ? TRUE
                              : FALSE;
        if (stopped != FALSE)
        {
            module_vehicle_phototube_runtime.drive_cal_confirm_count++;
        }
        else
        {
            module_vehicle_phototube_runtime.drive_cal_confirm_count = 0u;
        }
        if (module_vehicle_phototube_runtime.drive_cal_confirm_count
            >= MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_STOP_STABLE_COUNT)
        {
            uint32 suction_duty = (vehicle_control_auto_suction_enabled_get() != FALSE)
                                      ? VEHICLE_CONTROL_DEFAULT_RUN_SUCTION_DUTY
                                      : 0u;
            if (module_vehicle_phototube_control_suction_post(suction_duty) == FALSE)
            {
                module_vehicle_phototube_drive_calibration_abort("suction_queue");
                return;
            }
            module_vehicle_phototube_runtime.drive_cal_sample_count = 0u;
            module_vehicle_phototube_runtime.drive_cal_state =
                (state == MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_WHITE_STOP)
                    ? MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_WHITE_SAMPLE
                    : MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_BLUE_VERIFY;
            tools_printf("{ptcaldrive}%s,0,%u\r\n",
                         (state == MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_WHITE_STOP)
                             ? "white_sample"
                             : "blue_verify",
                         (unsigned int)MODULE_VEHICLE_PHOTOTUBE_AUTO_BLUE_SAMPLE_COUNT);
        }
        else if (module_vehicle_phototube_drive_calibration_elapsed_s(
                     module_vehicle_phototube_runtime.drive_cal_state_start_tick) > 1.0f)
        {
            module_vehicle_phototube_drive_calibration_abort("stop_timeout");
        }
        return;
    }

    if (state == MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_WHITE_SAMPLE)
    {
        module_vehicle_phototube_runtime.drive_cal_sample_count++;
        for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
        {
            module_vehicle_phototube_runtime.drive_cal_white_sum[index] +=
                module_vehicle_phototube_runtime.value[index];
        }
        if (module_vehicle_phototube_runtime.drive_cal_sample_count
            >= MODULE_VEHICLE_PHOTOTUBE_AUTO_BLUE_SAMPLE_COUNT)
        {
            for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
            {
                uint16 white_value =
                    (uint16)(module_vehicle_phototube_runtime.drive_cal_white_sum[index]
                             / MODULE_VEHICLE_PHOTOTUBE_AUTO_BLUE_SAMPLE_COUNT);
                uint16 blue_value = calibration->blue_value[index];
                calibration->white_value[index] = white_value;
                calibration->min_value[index] =
                    (white_value < blue_value) ? white_value : blue_value;
                calibration->max_value[index] =
                    (white_value > blue_value) ? white_value : blue_value;
            }
            module_vehicle_phototube_runtime.drive_cal_sample_count = 0u;
            if (module_vehicle_phototube_drive_calibration_launch(
                    MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_LAUNCH_BLUE) == FALSE)
            {
                module_vehicle_phototube_drive_calibration_abort("relaunch_queue");
            }
        }
        return;
    }

    if (state == MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_SEARCH_BLUE)
    {
        uint32 blue_channel_count = 0u;
        uint32 max_delta = 0u;

        for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
        {
            uint16 value = module_vehicle_phototube_runtime.value[index];
            uint16 blue = calibration->blue_value[index];
            uint32 delta = (value >= blue) ? ((uint32)value - (uint32)blue)
                                           : ((uint32)blue - (uint32)value);
            if (delta <= MODULE_VEHICLE_PHOTOTUBE_AUTO_BACK_DELTA)
            {
                blue_channel_count++;
            }
            if (delta > max_delta)
            {
                max_delta = delta;
            }
        }
        if ((blue_channel_count >= MODULE_VEHICLE_PHOTOTUBE_AUTO_BACK_CHANNEL_COUNT)
            && (max_delta < MODULE_VEHICLE_PHOTOTUBE_AUTO_LINE_DELTA))
        {
            module_vehicle_phototube_runtime.drive_cal_confirm_count++;
        }
        else if (module_vehicle_phototube_runtime.drive_cal_confirm_count > 0u)
        {
            module_vehicle_phototube_runtime.drive_cal_confirm_count--;
        }
        if (module_vehicle_phototube_runtime.drive_cal_confirm_count
            >= MODULE_VEHICLE_PHOTOTUBE_AUTO_BACK_STABLE_COUNT)
        {
            if (module_vehicle_phototube_drive_calibration_stop() == FALSE)
            {
                module_vehicle_phototube_drive_calibration_abort("final_stop_queue");
                return;
            }
            module_vehicle_phototube_runtime.drive_cal_state =
                MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_BLUE_STOP;
            tools_printf("{ptcaldrive}blue_stop,distance,%.1f\r\n",
                         (double)module_vehicle_phototube_drive_calibration_distance_get());
        }
        else if (module_vehicle_phototube_drive_calibration_distance_get()
                 > MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_SEARCH_MAX_MM)
        {
            module_vehicle_phototube_drive_calibration_abort("blue_not_found");
        }
        return;
    }

    if (state == MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_BLUE_VERIFY)
    {
        module_vehicle_phototube_runtime.drive_cal_sample_count++;
        for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
        {
            module_vehicle_phototube_runtime.drive_cal_final_blue_sum[index] +=
                module_vehicle_phototube_runtime.value[index];
        }
        if (module_vehicle_phototube_runtime.drive_cal_sample_count
            >= MODULE_VEHICLE_PHOTOTUBE_AUTO_BLUE_SAMPLE_COUNT)
        {
            uint32 worst_delta = 0u;
            uint32 blue_channel_count = 0u;
            boolean line_residue = FALSE;

            for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
            {
                uint16 first_blue = calibration->blue_value[index];
                uint16 final_blue =
                    (uint16)(module_vehicle_phototube_runtime.drive_cal_final_blue_sum[index]
                             / MODULE_VEHICLE_PHOTOTUBE_AUTO_BLUE_SAMPLE_COUNT);
                uint32 delta = (final_blue >= first_blue)
                                   ? ((uint32)final_blue - (uint32)first_blue)
                                   : ((uint32)first_blue - (uint32)final_blue);
                uint16 blue_value = (uint16)(((uint32)first_blue + (uint32)final_blue) / 2u);
                uint16 white_value = calibration->white_value[index];

                if (delta > worst_delta)
                {
                    worst_delta = delta;
                }
                if (delta <= MODULE_VEHICLE_PHOTOTUBE_AUTO_BACK_DELTA)
                {
                    blue_channel_count++;
                }
                if (delta >= MODULE_VEHICLE_PHOTOTUBE_AUTO_LINE_DELTA)
                {
                    line_residue = TRUE;
                }
                calibration->blue_value[index] = blue_value;
                calibration->min_value[index] =
                    (white_value < blue_value) ? white_value : blue_value;
                calibration->max_value[index] =
                    (white_value > blue_value) ? white_value : blue_value;
            }
            if ((blue_channel_count < MODULE_VEHICLE_PHOTOTUBE_AUTO_BACK_CHANNEL_COUNT)
                || (line_residue != FALSE))
            {
                module_vehicle_phototube_drive_calibration_abort("blue_mismatch");
                return;
            }
            calibration->sample_count =
                MODULE_VEHICLE_PHOTOTUBE_AUTO_BLUE_SAMPLE_COUNT * 3u;
            if (module_vehicle_phototube_calibration_finish() != FALSE)
            {
                module_vehicle_phototube_runtime.drive_cal_state =
                    MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_IDLE;
                (void)module_vehicle_phototube_control_suction_post(0u);
                module_vehicle_phototube_power_update();
                tools_printf("{ptcaldrive}done,samples,%u,blue_delta,%u\r\n",
                             (unsigned int)calibration->sample_count,
                             (unsigned int)worst_delta);
            }
            else
            {
                module_vehicle_phototube_drive_calibration_abort("flash_save");
            }
        }
    }
}

static void module_vehicle_phototube_drive_calibration_abort(const char* reason)
{
    if (module_vehicle_phototube_runtime.drive_cal_state
        == MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_IDLE)
    {
        tools_printf("{ptcaldrive}idle\r\n");
        return;
    }

    module_vehicle_phototube_runtime.drive_cal_state =
        MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_IDLE;
    module_vehicle_phototube_runtime.calibration =
        module_vehicle_phototube_runtime.drive_cal_backup;
    module_vehicle_phototube_runtime.calibration_active = FALSE;
    module_vehicle_phototube_runtime.auto_state = MODULE_VEHICLE_PHOTOTUBE_AUTO_IDLE;
    (void)module_vehicle_phototube_control_speed_test_stop_post();
    (void)module_vehicle_phototube_control_suction_post(0u);
    module_vehicle_phototube_power_update();
    tools_printf("{ptcaldrive}fail,%s\r\n", reason);
}

static void module_vehicle_phototube_drive_calibration_status(void)
{
    const module_vehicle_encoder_observation_t* encoder =
        module_vehicle_encoder_observation_get();
    boolean active = (module_vehicle_phototube_runtime.drive_cal_state
                      != MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_IDLE)
                         ? TRUE
                         : FALSE;

    tools_printf("{ptcaldrive}status,%s,speed,%.3f,distance,%.1f,left,%.1f,right,%.1f,elapsed,%.3f\r\n",
                 module_vehicle_phototube_drive_calibration_state_name(
                     module_vehicle_phototube_runtime.drive_cal_state),
                 (double)(module_vehicle_phototube_runtime.drive_cal_speed_mm_s * 0.001f),
                 (double)((active != FALSE)
                              ? module_vehicle_phototube_drive_calibration_distance_get()
                              : 0.0f),
                 (double)encoder->left_speed_mm_s,
                 (double)encoder->right_speed_mm_s,
                 (double)((active != FALSE)
                              ? module_vehicle_phototube_drive_calibration_elapsed_s(
                                    module_vehicle_phototube_runtime.drive_cal_start_tick)
                              : 0.0f));
}

static float32 module_vehicle_phototube_drive_calibration_elapsed_s(uint64 start_tick)
{
    uint64 elapsed_tick = sysTick_getTick(SYSTICK1) - start_tick;
    uint64 elapsed_us = sysTick_ticksToMicroseconds(SYSTICK1, elapsed_tick);

    return (float32)elapsed_us * MODULE_VEHICLE_PHOTOTUBE_US_TO_S;
}

static float32 module_vehicle_phototube_drive_calibration_distance_get(void)
{
    float32 distance = module_vehicle_pose_fusion_observation_get()->distance_mm
                     - module_vehicle_phototube_runtime.drive_cal_state_start_distance_mm;

    return (distance >= 0.0f) ? distance : -distance;
}

static const char* module_vehicle_phototube_drive_calibration_state_name(
    module_vehicle_phototube_drive_cal_state_t state)
{
    static const char* const names[] =
    {
        "idle", "blue_sample", "launch_white", "search_white", "white_stop",
        "white_sample", "launch_blue", "search_blue", "blue_stop", "blue_verify"
    };

    if ((uint32)state >= (uint32)(sizeof(names) / sizeof(names[0u])))
    {
        return "unknown";
    }
    return names[(uint32)state];
}

static boolean module_vehicle_phototube_calibration_finish(void)
{
    uint32 center_sum = 0u;
    uint32 span_sum = 0u;
    uint16 center_mean;
    uint16 span_mean;
    module_vehicle_phototube_calibration_t *calibration =
        &module_vehicle_phototube_runtime.calibration;
    uint32 index;

    module_vehicle_phototube_runtime.calibration_active = FALSE;
    module_vehicle_phototube_runtime.auto_state = MODULE_VEHICLE_PHOTOTUBE_AUTO_IDLE;
    module_vehicle_phototube_power_update();
    if (calibration->sample_count == 0u)
    {
        tools_printf("{ptcal}empty\r\n");
        return FALSE;
    }

    for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
    {
        uint16 min_value = calibration->min_value[index];
        uint16 max_value = calibration->max_value[index];
        uint16 span_value = (uint16)(max_value - min_value);
        uint16 blue_value = calibration->blue_value[index];
        uint32 high_delta = (max_value >= blue_value) ? ((uint32)max_value - (uint32)blue_value) : 0u;
        uint32 low_delta = (blue_value >= min_value) ? ((uint32)blue_value - (uint32)min_value) : 0u;

        if (calibration->blue_value[index] == 0u)
        {
            calibration->blue_value[index] = min_value;
            blue_value = min_value;
        }
        calibration->white_value[index] = (high_delta >= low_delta) ? max_value : min_value;
        calibration->center_value[index] = (uint16)((min_value + max_value) / 2u);
        calibration->span_value[index] = span_value;
        center_sum += calibration->center_value[index];
        span_sum += span_value;
    }

    center_mean = (uint16)(center_sum / MODULE_VEHICLE_PHOTOTUBE_COUNT);
    span_mean = (uint16)(span_sum / MODULE_VEHICLE_PHOTOTUBE_COUNT);
    if (span_mean < MODULE_VEHICLE_PHOTOTUBE_MIN_SPAN)
    {
        span_mean = MODULE_VEHICLE_PHOTOTUBE_MIN_SPAN;
    }

    for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
    {
        uint16 span_value = calibration->span_value[index];

        calibration->offset_value[index] =
            (sint16)((sint32)calibration->center_value[index] - (sint32)center_mean);
        if (span_value < MODULE_VEHICLE_PHOTOTUBE_MIN_SPAN)
        {
            span_value = MODULE_VEHICLE_PHOTOTUBE_MIN_SPAN;
        }
        calibration->gain_q15[index] =
            (MODULE_VEHICLE_PHOTOTUBE_GAIN_Q15_BASE * (uint32)span_mean) / (uint32)span_value;
    }

    calibration->valid = TRUE;
    tools_printf("{ptcal}done,%u,%u,%u\r\n",
                 (unsigned int)calibration->sample_count,
                 (unsigned int)center_mean,
                 (unsigned int)span_mean);
    module_vehicle_phototube_calibration_print();
    if (module_vehicle_phototube_calibration_save_to_flash() != FALSE)
    {
        tools_printf("{ptcal}autosave,ok\r\n");
        return TRUE;
    }
    tools_printf("{ptcal}autosave,fail\r\n");
    return FALSE;
}

static void module_vehicle_phototube_calibration_print(void)
{
    const module_vehicle_phototube_calibration_t *calibration =
        &module_vehicle_phototube_runtime.calibration;

    tools_printf("{ptcal}valid,%u,samples,%u\r\n",
                 (unsigned int)calibration->valid,
                 (unsigned int)calibration->sample_count);
    tools_printf("{ptmin}%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                 (unsigned int)calibration->min_value[0u],
                 (unsigned int)calibration->min_value[1u],
                 (unsigned int)calibration->min_value[2u],
                 (unsigned int)calibration->min_value[3u],
                 (unsigned int)calibration->min_value[4u],
                 (unsigned int)calibration->min_value[5u],
                 (unsigned int)calibration->min_value[6u],
                 (unsigned int)calibration->min_value[7u],
                 (unsigned int)calibration->min_value[8u],
                 (unsigned int)calibration->min_value[9u],
                 (unsigned int)calibration->min_value[10u],
                 (unsigned int)calibration->min_value[11u],
                 (unsigned int)calibration->min_value[12u],
                 (unsigned int)calibration->min_value[13u],
                 (unsigned int)calibration->min_value[14u],
                 (unsigned int)calibration->min_value[15u]);
    tools_printf("{ptmax}%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                 (unsigned int)calibration->max_value[0u],
                 (unsigned int)calibration->max_value[1u],
                 (unsigned int)calibration->max_value[2u],
                 (unsigned int)calibration->max_value[3u],
                 (unsigned int)calibration->max_value[4u],
                 (unsigned int)calibration->max_value[5u],
                 (unsigned int)calibration->max_value[6u],
                 (unsigned int)calibration->max_value[7u],
                 (unsigned int)calibration->max_value[8u],
                 (unsigned int)calibration->max_value[9u],
                 (unsigned int)calibration->max_value[10u],
                 (unsigned int)calibration->max_value[11u],
                 (unsigned int)calibration->max_value[12u],
                 (unsigned int)calibration->max_value[13u],
                 (unsigned int)calibration->max_value[14u],
                 (unsigned int)calibration->max_value[15u]);
    tools_printf("{ptctr}%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                 (unsigned int)calibration->center_value[0u],
                 (unsigned int)calibration->center_value[1u],
                 (unsigned int)calibration->center_value[2u],
                 (unsigned int)calibration->center_value[3u],
                 (unsigned int)calibration->center_value[4u],
                 (unsigned int)calibration->center_value[5u],
                 (unsigned int)calibration->center_value[6u],
                 (unsigned int)calibration->center_value[7u],
                 (unsigned int)calibration->center_value[8u],
                 (unsigned int)calibration->center_value[9u],
                 (unsigned int)calibration->center_value[10u],
                 (unsigned int)calibration->center_value[11u],
                 (unsigned int)calibration->center_value[12u],
                 (unsigned int)calibration->center_value[13u],
                 (unsigned int)calibration->center_value[14u],
                 (unsigned int)calibration->center_value[15u]);
    tools_printf("{ptblue}%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                 (unsigned int)calibration->blue_value[0u],
                 (unsigned int)calibration->blue_value[1u],
                 (unsigned int)calibration->blue_value[2u],
                 (unsigned int)calibration->blue_value[3u],
                 (unsigned int)calibration->blue_value[4u],
                 (unsigned int)calibration->blue_value[5u],
                 (unsigned int)calibration->blue_value[6u],
                 (unsigned int)calibration->blue_value[7u],
                 (unsigned int)calibration->blue_value[8u],
                 (unsigned int)calibration->blue_value[9u],
                 (unsigned int)calibration->blue_value[10u],
                 (unsigned int)calibration->blue_value[11u],
                 (unsigned int)calibration->blue_value[12u],
                 (unsigned int)calibration->blue_value[13u],
                 (unsigned int)calibration->blue_value[14u],
                 (unsigned int)calibration->blue_value[15u]);
    tools_printf("{ptwhite}%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                 (unsigned int)calibration->white_value[0u],
                 (unsigned int)calibration->white_value[1u],
                 (unsigned int)calibration->white_value[2u],
                 (unsigned int)calibration->white_value[3u],
                 (unsigned int)calibration->white_value[4u],
                 (unsigned int)calibration->white_value[5u],
                 (unsigned int)calibration->white_value[6u],
                 (unsigned int)calibration->white_value[7u],
                 (unsigned int)calibration->white_value[8u],
                 (unsigned int)calibration->white_value[9u],
                 (unsigned int)calibration->white_value[10u],
                 (unsigned int)calibration->white_value[11u],
                 (unsigned int)calibration->white_value[12u],
                 (unsigned int)calibration->white_value[13u],
                 (unsigned int)calibration->white_value[14u],
                 (unsigned int)calibration->white_value[15u]);
    tools_printf("{ptoff}%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                 (int)calibration->offset_value[0u],
                 (int)calibration->offset_value[1u],
                 (int)calibration->offset_value[2u],
                 (int)calibration->offset_value[3u],
                 (int)calibration->offset_value[4u],
                 (int)calibration->offset_value[5u],
                 (int)calibration->offset_value[6u],
                 (int)calibration->offset_value[7u],
                 (int)calibration->offset_value[8u],
                 (int)calibration->offset_value[9u],
                 (int)calibration->offset_value[10u],
                 (int)calibration->offset_value[11u],
                 (int)calibration->offset_value[12u],
                 (int)calibration->offset_value[13u],
                 (int)calibration->offset_value[14u],
                 (int)calibration->offset_value[15u]);
    tools_printf("{ptgain}%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                 (unsigned int)calibration->gain_q15[0u],
                 (unsigned int)calibration->gain_q15[1u],
                 (unsigned int)calibration->gain_q15[2u],
                 (unsigned int)calibration->gain_q15[3u],
                 (unsigned int)calibration->gain_q15[4u],
                 (unsigned int)calibration->gain_q15[5u],
                 (unsigned int)calibration->gain_q15[6u],
                 (unsigned int)calibration->gain_q15[7u],
                 (unsigned int)calibration->gain_q15[8u],
                 (unsigned int)calibration->gain_q15[9u],
                 (unsigned int)calibration->gain_q15[10u],
                 (unsigned int)calibration->gain_q15[11u],
                 (unsigned int)calibration->gain_q15[12u],
                 (unsigned int)calibration->gain_q15[13u],
                 (unsigned int)calibration->gain_q15[14u],
                 (unsigned int)calibration->gain_q15[15u]);
}

static boolean module_vehicle_phototube_calibration_save_to_flash(void)
{
    module_vehicle_phototube_flash_record_t record;
    module_vehicle_phototube_flash_record_t verify_record;
    uint8 page[IFXFLASH_PFLASH_PAGE_LENGTH];
    uint32 word_l[4u];
    uint32 word_u[4u];
    const uint8* record_bytes;
    uint32 flash_address = module_vehicle_phototube_flash_address_get();
    uint32 record_length = (uint32)sizeof(record);
    uint32 aligned_length = module_vehicle_phototube_flash_aligned_length_get(record_length);
    uint32 offset;
    uint32 index;

    if (module_vehicle_phototube_flash_runtime_ready() == FALSE)
    {
        return FALSE;
    }
    if (module_vehicle_phototube_runtime.calibration.valid == FALSE)
    {
        return FALSE;
    }
    if ((aligned_length == 0u) || (aligned_length > MODULE_VEHICLE_PHOTOTUBE_FLASH_SECTOR_BYTES))
    {
        return FALSE;
    }

    module_vehicle_phototube_calibration_flash_record_pack(&record);
    record_bytes = (const uint8*)&record;

    driver_flash_eraseSector(flash_address);

    for (offset = 0u; offset < aligned_length; offset += IFXFLASH_PFLASH_PAGE_LENGTH)
    {
        uint32 copy_length = record_length - offset;

        if (copy_length > IFXFLASH_PFLASH_PAGE_LENGTH)
        {
            copy_length = IFXFLASH_PFLASH_PAGE_LENGTH;
        }

        for (index = 0u; index < IFXFLASH_PFLASH_PAGE_LENGTH; index++)
        {
            page[index] = 0xFFu;
        }
        (void)memcpy(page, &record_bytes[offset], copy_length);
        module_vehicle_phototube_flash_page_pack(page, &word_l, &word_u);
        if (device_int_flash_writePFlashPageData(DEVICE_INT_FLASH_1,
                                                 flash_address + offset,
                                                 &word_l,
                                                 &word_u) == FALSE)
        {
            return FALSE;
        }
    }

    if (module_vehicle_phototube_calibration_flash_record_read(&verify_record) == FALSE)
    {
        return FALSE;
    }

    return module_vehicle_phototube_calibration_flash_record_validate(&verify_record);
}

static boolean module_vehicle_phototube_calibration_load_from_flash(void)
{
    module_vehicle_phototube_flash_record_t record;

    if (module_vehicle_phototube_flash_runtime_ready() == FALSE)
    {
        return FALSE;
    }
    if (module_vehicle_phototube_calibration_flash_record_read(&record) == FALSE)
    {
        return FALSE;
    }
    if (module_vehicle_phototube_calibration_flash_record_validate(&record) == FALSE)
    {
        return FALSE;
    }

    module_vehicle_phototube_calibration_flash_record_apply(&record);
    return TRUE;
}

static boolean module_vehicle_phototube_calibration_erase_flash(void)
{
    if (module_vehicle_phototube_flash_runtime_ready() == FALSE)
    {
        return FALSE;
    }

    driver_flash_eraseSector(module_vehicle_phototube_flash_address_get());
    module_vehicle_phototube_runtime.calibration.valid = FALSE;
    return TRUE;
}

static void module_vehicle_phototube_calibration_auto_load_process(void)
{
    if (module_vehicle_phototube_runtime.flash_auto_load_pending == FALSE)
    {
        return;
    }

    module_vehicle_phototube_runtime.flash_auto_load_pending = FALSE;
    if (module_vehicle_phototube_calibration_load_from_flash() != FALSE)
    {
        tools_printf("{ptcal}autoload,ok\r\n");
    }
}

static boolean module_vehicle_phototube_calibration_flash_record_read(
    module_vehicle_phototube_flash_record_t* record)
{
    if (record == NULL_PTR)
    {
        return FALSE;
    }

    (void)memset(record, 0, sizeof(*record));
    device_int_flash_read(DEVICE_INT_FLASH_1,
                          module_vehicle_phototube_flash_address_get(),
                          (uint8*)record,
                          (uint32)sizeof(*record));

    return TRUE;
}

static boolean module_vehicle_phototube_calibration_flash_record_validate(
    const module_vehicle_phototube_flash_record_t* record)
{
    uint32 checksum;

    if (record == NULL_PTR)
    {
        return FALSE;
    }
    if (record->magic != MODULE_VEHICLE_PHOTOTUBE_FLASH_MAGIC)
    {
        return FALSE;
    }
    if (record->version != MODULE_VEHICLE_PHOTOTUBE_FLASH_VERSION)
    {
        return FALSE;
    }
    if (record->record_length != (uint16)sizeof(*record))
    {
        return FALSE;
    }
    if (module_vehicle_phototube_map_validate(record->physical_map) == FALSE)
    {
        return FALSE;
    }

    checksum = module_vehicle_phototube_checksum_calculate(
        &((const uint8*)record)[offsetof(module_vehicle_phototube_flash_record_t, sample_count)],
        (uint32)sizeof(*record)
        - (uint32)offsetof(module_vehicle_phototube_flash_record_t, sample_count));

    return (boolean)(checksum == record->checksum);
}

static void module_vehicle_phototube_calibration_flash_record_pack(
    module_vehicle_phototube_flash_record_t* record)
{
    const module_vehicle_phototube_calibration_t* calibration =
        &module_vehicle_phototube_runtime.calibration;

    if (record == NULL_PTR)
    {
        return;
    }

    (void)memset(record, 0xFF, sizeof(*record));
    record->magic = MODULE_VEHICLE_PHOTOTUBE_FLASH_MAGIC;
    record->version = MODULE_VEHICLE_PHOTOTUBE_FLASH_VERSION;
    record->record_length = (uint16)sizeof(*record);
    record->checksum = 0u;
    record->sample_count = calibration->sample_count;
    (void)memcpy(record->physical_map,
                 module_vehicle_phototube_runtime.physical_map,
                 sizeof(record->physical_map));
    (void)memcpy(record->blue_value, calibration->blue_value, sizeof(record->blue_value));
    (void)memcpy(record->white_value, calibration->white_value, sizeof(record->white_value));
    (void)memcpy(record->min_value, calibration->min_value, sizeof(record->min_value));
    (void)memcpy(record->max_value, calibration->max_value, sizeof(record->max_value));
    (void)memcpy(record->center_value, calibration->center_value, sizeof(record->center_value));
    (void)memcpy(record->span_value, calibration->span_value, sizeof(record->span_value));
    (void)memcpy(record->offset_value, calibration->offset_value, sizeof(record->offset_value));
    (void)memcpy(record->gain_q15, calibration->gain_q15, sizeof(record->gain_q15));

    record->checksum = module_vehicle_phototube_checksum_calculate(
        &((const uint8*)record)[offsetof(module_vehicle_phototube_flash_record_t, sample_count)],
        (uint32)sizeof(*record)
        - (uint32)offsetof(module_vehicle_phototube_flash_record_t, sample_count));
}

static void module_vehicle_phototube_calibration_flash_record_apply(
    const module_vehicle_phototube_flash_record_t* record)
{
    module_vehicle_phototube_calibration_t* calibration =
        &module_vehicle_phototube_runtime.calibration;

    if (record == NULL_PTR)
    {
        return;
    }

    module_vehicle_phototube_runtime.calibration_active = FALSE;
    module_vehicle_phototube_runtime.auto_state = MODULE_VEHICLE_PHOTOTUBE_AUTO_IDLE;
    calibration->valid = TRUE;
    calibration->sample_count = record->sample_count;
    (void)memcpy(module_vehicle_phototube_runtime.physical_map,
                 record->physical_map,
                 sizeof(module_vehicle_phototube_runtime.physical_map));
    (void)memcpy(calibration->blue_value, record->blue_value, sizeof(calibration->blue_value));
    (void)memcpy(calibration->white_value, record->white_value, sizeof(calibration->white_value));
    (void)memcpy(calibration->min_value, record->min_value, sizeof(calibration->min_value));
    (void)memcpy(calibration->max_value, record->max_value, sizeof(calibration->max_value));
    (void)memcpy(calibration->center_value, record->center_value, sizeof(calibration->center_value));
    (void)memcpy(calibration->span_value, record->span_value, sizeof(calibration->span_value));
    (void)memcpy(calibration->offset_value, record->offset_value, sizeof(calibration->offset_value));
    (void)memcpy(calibration->gain_q15, record->gain_q15, sizeof(calibration->gain_q15));
}

static uint32 module_vehicle_phototube_flash_address_get(void)
{
    return MODULE_VEHICLE_PHOTOTUBE_FLASH_ADDRESS;
}

static boolean module_vehicle_phototube_flash_runtime_ready(void)
{
    device_int_flash_runtime_t* runtime = device_int_flash_runtime_table_get();

    if (runtime[DEVICE_INT_FLASH_1].flash_runtime.sector_group == 0u)
    {
        return FALSE;
    }
    if (runtime[DEVICE_INT_FLASH_1].flash_runtime.group_capacity
        < (MODULE_VEHICLE_PHOTOTUBE_FLASH_OFFSET + IFXFLASH_PFLASH_PAGE_LENGTH))
    {
        return FALSE;
    }

    return TRUE;
}

static uint32 module_vehicle_phototube_flash_aligned_length_get(uint32 length)
{
    if (length == 0u)
    {
        return 0u;
    }

    return ((length + IFXFLASH_PFLASH_PAGE_LENGTH - 1u)
            / IFXFLASH_PFLASH_PAGE_LENGTH)
           * IFXFLASH_PFLASH_PAGE_LENGTH;
}

static uint32 module_vehicle_phototube_checksum_calculate(const uint8* data, uint32 length)
{
    return module_vehicle_phototube_checksum_update(
        MODULE_VEHICLE_PHOTOTUBE_FLASH_CHECKSUM_SEED,
        data,
        length);
}

static uint32 module_vehicle_phototube_checksum_update(uint32 checksum, const uint8* data, uint32 length)
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

static void module_vehicle_phototube_flash_page_pack(const uint8 page[IFXFLASH_PFLASH_PAGE_LENGTH],
                                                     uint32 (*word_l)[4],
                                                     uint32 (*word_u)[4])
{
    uint32 index;

    if ((page == NULL_PTR) || (word_l == NULL_PTR) || (word_u == NULL_PTR))
    {
        return;
    }

    for (index = 0u; index < 4u; index++)
    {
        (*word_l)[index] = ((uint32)page[index * 8u])
                         | (((uint32)page[(index * 8u) + 1u]) << 8u)
                         | (((uint32)page[(index * 8u) + 2u]) << 16u)
                         | (((uint32)page[(index * 8u) + 3u]) << 24u);
        (*word_u)[index] = ((uint32)page[(index * 8u) + 4u])
                         | (((uint32)page[(index * 8u) + 5u]) << 8u)
                         | (((uint32)page[(index * 8u) + 6u]) << 16u)
                         | (((uint32)page[(index * 8u) + 7u]) << 24u);
    }
}

static void module_vehicle_phototube_command_process(void)
{
    module_vehicle_phototube_command_t command =
        module_vehicle_phototube_runtime.pending_command;

    module_vehicle_phototube_runtime.pending_command =
        MODULE_VEHICLE_PHOTOTUBE_COMMAND_NONE;

    if (command == MODULE_VEHICLE_PHOTOTUBE_COMMAND_START)
    {
        module_vehicle_phototube_calibration_start();
    }
    else if (command == MODULE_VEHICLE_PHOTOTUBE_COMMAND_AUTO)
    {
        module_vehicle_phototube_calibration_auto_start();
    }
    else if (command == MODULE_VEHICLE_PHOTOTUBE_COMMAND_STOP)
    {
        if (module_vehicle_phototube_runtime.drive_cal_state
            != MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_IDLE)
        {
            module_vehicle_phototube_drive_calibration_abort("user_stop");
        }
        else
        {
            (void)module_vehicle_phototube_calibration_finish();
        }
    }
    else if (command == MODULE_VEHICLE_PHOTOTUBE_COMMAND_SHOW)
    {
        module_vehicle_phototube_calibration_print();
    }
    else if (command == MODULE_VEHICLE_PHOTOTUBE_COMMAND_ABORT)
    {
        module_vehicle_phototube_calibration_abort();
    }
    else if (command == MODULE_VEHICLE_PHOTOTUBE_COMMAND_SAVE)
    {
        if (module_vehicle_phototube_runtime.calibration.valid == FALSE)
        {
            tools_printf("{ptcal}save,invalid\r\n");
        }
        else if (module_vehicle_phototube_calibration_save_to_flash() != FALSE)
        {
            tools_printf("{ptcal}save,ok\r\n");
        }
        else
        {
            tools_printf("{ptcal}save,fail\r\n");
        }
    }
    else if (command == MODULE_VEHICLE_PHOTOTUBE_COMMAND_LOAD)
    {
        if (module_vehicle_phototube_calibration_load_from_flash() != FALSE)
        {
            tools_printf("{ptcal}load,ok\r\n");
        }
        else
        {
            tools_printf("{ptcal}load,empty\r\n");
        }
    }
    else if (command == MODULE_VEHICLE_PHOTOTUBE_COMMAND_ERASE)
    {
        if (module_vehicle_phototube_calibration_erase_flash() != FALSE)
        {
            tools_printf("{ptcal}erase,ok\r\n");
        }
        else
        {
            tools_printf("{ptcal}erase,fail\r\n");
        }
    }
    else if (command == MODULE_VEHICLE_PHOTOTUBE_COMMAND_FLASH)
    {
        module_vehicle_phototube_flash_record_t record;
        boolean valid = FALSE;

        if (module_vehicle_phototube_calibration_flash_record_read(&record) != FALSE)
        {
            valid = module_vehicle_phototube_calibration_flash_record_validate(&record);
        }
        tools_printf("{ptcal}flash,%u,samples,%u,addr,%u,len,%u\r\n",
                     (unsigned int)((valid != FALSE) ? 1u : 0u),
                     (unsigned int)((valid != FALSE) ? record.sample_count : 0u),
                     (unsigned int)module_vehicle_phototube_flash_address_get(),
                     (unsigned int)sizeof(record));
    }
    else if (command == MODULE_VEHICLE_PHOTOTUBE_COMMAND_USAGE)
    {
        tools_printf("{ptcal}usage: ptcal drive [0.4..1.0]|drive status|drive abort|auto|start|stop|show|abort|save|load|flash|erase\r\n");
    }
}

static void module_vehicle_phototube_line_update(void)
{
    float32 position;
    float32 error;
    float32 dt_s;
    float32 correction_mm_s;
    float32 left_speed_mm_s;
    float32 right_speed_mm_s;
    float32 min_speed_mm_s;
    uint32 line_sum;

    module_vehicle_phototube_runtime.line_valid =
        module_vehicle_phototube_line_calculate(&position, &error, &line_sum);
    module_vehicle_phototube_runtime.line_position = position;
    module_vehicle_phototube_runtime.line_error = error;
    module_vehicle_phototube_runtime.line_sum = line_sum;
    module_vehicle_phototube_runtime.line_active_count =
        module_vehicle_phototube_line_active_count_get();
    dt_s = module_vehicle_phototube_line_frame_dt_get();

    if (module_vehicle_phototube_runtime.line_mode == MODULE_VEHICLE_PHOTOTUBE_LINE_RUN)
    {
        if (module_vehicle_phototube_runtime.line_launch_state
            != MODULE_VEHICLE_PHOTOTUBE_LAUNCH_OFF)
        {
            if (vehicle_control_test_launch_active_get() != FALSE)
            {
                module_vehicle_phototube_runtime.line_launch_state =
                    MODULE_VEHICLE_PHOTOTUBE_LAUNCH_ACTIVE;
                module_vehicle_phototube_line_print();
                return;
            }
            if (module_vehicle_phototube_runtime.line_launch_state
                == MODULE_VEHICLE_PHOTOTUBE_LAUNCH_WAIT_START)
            {
                module_vehicle_phototube_line_print();
                return;
            }

            module_vehicle_phototube_runtime.line_launch_state =
                MODULE_VEHICLE_PHOTOTUBE_LAUNCH_OFF;
            module_vehicle_phototube_line_control_reset();
            module_vehicle_phototube_line_lost_reset();
            module_vehicle_phototube_runtime.line_follow_blend = 0.0f;
            tools_printf("{ptline}launch_done,follow_blend,0.050\r\n");
        }

        if (module_vehicle_phototube_runtime.maneuver_state
            == MODULE_VEHICLE_PHOTOTUBE_MANEUVER_FOLLOW)
        {
            module_vehicle_phototube_line_heading_anchor_update(dt_s);
            if ((module_vehicle_phototube_runtime.element_detection_enabled != FALSE)
                && (module_vehicle_phototube_line_element_detected(dt_s) != FALSE))
            {
                module_vehicle_phototube_line_maneuver_start();
            }
            else if ((module_vehicle_phototube_runtime.element_detection_enabled != FALSE)
                     && (module_vehicle_phototube_runtime.element_candidate_count > 0u))
            {
                module_vehicle_phototube_line_lost_reset();
                module_vehicle_phototube_runtime.line_correction_mm_s = 0.0f;
                module_vehicle_phototube_runtime.line_left_speed_mm_s =
                    module_vehicle_phototube_runtime.line_base_speed_mm_s;
                module_vehicle_phototube_runtime.line_right_speed_mm_s =
                    module_vehicle_phototube_runtime.line_base_speed_mm_s;
                if (module_vehicle_phototube_control_heading_target_post(
                        module_vehicle_phototube_runtime.line_base_speed_mm_s,
                        module_vehicle_phototube_runtime.element_entry_heading_rad) == FALSE)
                {
                    tools_printf("{ptturn}candidate_control_failed\r\n");
                    module_vehicle_phototube_line_stop();
                }
                module_vehicle_phototube_line_print();
                return;
            }
        }

        if (module_vehicle_phototube_runtime.maneuver_state
            == MODULE_VEHICLE_PHOTOTUBE_MANEUVER_HEADING)
        {
            /* Intentional element maneuvers can leave the line for the whole arc. */
            module_vehicle_phototube_line_lost_reset();
            if (module_vehicle_phototube_line_maneuver_update(dt_s) == FALSE)
            {
                module_vehicle_phototube_line_print();
                return;
            }
        }

        if (module_vehicle_phototube_runtime.maneuver_state
            == MODULE_VEHICLE_PHOTOTUBE_MANEUVER_FOLLOW)
        {
            if (module_vehicle_phototube_line_lost_update(dt_s) != FALSE)
            {
                module_vehicle_phototube_line_print();
                return;
            }
        }

        if (module_vehicle_phototube_runtime.line_valid != FALSE)
        {
            correction_mm_s = module_vehicle_phototube_line_correction_calculate(error);
            if (module_vehicle_phototube_runtime.line_follow_blend < 1.0f)
            {
                if ((dt_s > 0.0f) && (MODULE_VEHICLE_PHOTOTUBE_LINE_REACQUIRE_BLEND_S > 0.0f))
                {
                    module_vehicle_phototube_runtime.line_follow_blend +=
                        dt_s / MODULE_VEHICLE_PHOTOTUBE_LINE_REACQUIRE_BLEND_S;
                }
                module_vehicle_phototube_runtime.line_follow_blend =
                    module_vehicle_phototube_clamp_f32(
                        module_vehicle_phototube_runtime.line_follow_blend,
                        0.0f,
                        1.0f);
                correction_mm_s *= module_vehicle_phototube_runtime.line_follow_blend;
            }

            left_speed_mm_s = module_vehicle_phototube_runtime.line_base_speed_mm_s - correction_mm_s;
            right_speed_mm_s = module_vehicle_phototube_runtime.line_base_speed_mm_s + correction_mm_s;
            min_speed_mm_s = module_vehicle_phototube_runtime.line_min_speed_mm_s;
            if (min_speed_mm_s > module_vehicle_phototube_runtime.line_base_speed_mm_s)
            {
                min_speed_mm_s = module_vehicle_phototube_runtime.line_base_speed_mm_s;
            }
            if (left_speed_mm_s < min_speed_mm_s)
            {
                left_speed_mm_s = min_speed_mm_s;
            }
            if (right_speed_mm_s < min_speed_mm_s)
            {
                right_speed_mm_s = min_speed_mm_s;
            }
            correction_mm_s = (right_speed_mm_s - left_speed_mm_s) * 0.5f;
        }
        else
        {
            correction_mm_s = 0.0f;
            left_speed_mm_s = module_vehicle_phototube_runtime.line_base_speed_mm_s;
            right_speed_mm_s = module_vehicle_phototube_runtime.line_base_speed_mm_s;
        }

        module_vehicle_phototube_runtime.line_correction_mm_s = correction_mm_s;
        module_vehicle_phototube_runtime.line_left_speed_mm_s = left_speed_mm_s;
        module_vehicle_phototube_runtime.line_right_speed_mm_s = right_speed_mm_s;
        (void)module_vehicle_phototube_control_speed_target_post(left_speed_mm_s,
                                                                 right_speed_mm_s);
    }

    module_vehicle_phototube_line_print();
}

static void module_vehicle_phototube_line_print(void)
{
    uint16 normalized[16u];
    uint16 raw[16u];
    uint16 adc[16u];

    module_vehicle_phototube_runtime.line_print_count++;
    if (module_vehicle_phototube_runtime.line_print_count
        < module_vehicle_phototube_runtime.line_print_divider)
    {
        return;
    }
    module_vehicle_phototube_runtime.line_print_count = 0u;

    module_vehicle_phototube_normalized_values_get(normalized);
    tools_printf("{ptline}%.1f,%.1f,%.1f,%u,%u,%.0f,%.0f,%.0f,%.3f,%.3f\r\n",
                 (double)module_vehicle_phototube_runtime.line_position,
                 (double)module_vehicle_phototube_runtime.line_error,
                 (double)module_vehicle_phototube_runtime.line_filtered_error,
                 (unsigned int)module_vehicle_phototube_runtime.line_sum,
                 (unsigned int)module_vehicle_phototube_runtime.line_valid,
                 (double)module_vehicle_phototube_runtime.line_base_speed_mm_s,
                 (double)module_vehicle_phototube_runtime.line_left_speed_mm_s,
                 (double)module_vehicle_phototube_runtime.line_right_speed_mm_s,
                 (double)module_vehicle_phototube_runtime.line_correction_mm_s,
                 (double)module_vehicle_phototube_runtime.line_kp_mm_s);
    if (module_vehicle_phototube_runtime.line_detail_enabled == FALSE)
    {
        return;
    }

    module_vehicle_phototube_values_get(raw);
    module_vehicle_phototube_adc_values_get(adc);
    tools_printf("{ptraw}%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                 (unsigned int)raw[0u],
                 (unsigned int)raw[1u],
                 (unsigned int)raw[2u],
                 (unsigned int)raw[3u],
                 (unsigned int)raw[4u],
                 (unsigned int)raw[5u],
                 (unsigned int)raw[6u],
                 (unsigned int)raw[7u],
                 (unsigned int)raw[8u],
                 (unsigned int)raw[9u],
                 (unsigned int)raw[10u],
                 (unsigned int)raw[11u],
                 (unsigned int)raw[12u],
                 (unsigned int)raw[13u],
                 (unsigned int)raw[14u],
                 (unsigned int)raw[15u]);
    tools_printf("{ptadc}%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                 (unsigned int)adc[0u],
                 (unsigned int)adc[1u],
                 (unsigned int)adc[2u],
                 (unsigned int)adc[3u],
                 (unsigned int)adc[4u],
                 (unsigned int)adc[5u],
                 (unsigned int)adc[6u],
                 (unsigned int)adc[7u],
                 (unsigned int)adc[8u],
                 (unsigned int)adc[9u],
                 (unsigned int)adc[10u],
                 (unsigned int)adc[11u],
                 (unsigned int)adc[12u],
                 (unsigned int)adc[13u],
                 (unsigned int)adc[14u],
                 (unsigned int)adc[15u]);
    tools_printf("{ptnorm}%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                 (unsigned int)normalized[0u],
                 (unsigned int)normalized[1u],
                 (unsigned int)normalized[2u],
                 (unsigned int)normalized[3u],
                 (unsigned int)normalized[4u],
                 (unsigned int)normalized[5u],
                 (unsigned int)normalized[6u],
                 (unsigned int)normalized[7u],
                 (unsigned int)normalized[8u],
                 (unsigned int)normalized[9u],
                 (unsigned int)normalized[10u],
                 (unsigned int)normalized[11u],
                 (unsigned int)normalized[12u],
                 (unsigned int)normalized[13u],
                 (unsigned int)normalized[14u],
                 (unsigned int)normalized[15u]);
}

static void module_vehicle_phototube_correction_process(void)
{
    const module_vehicle_pose_fusion_observation_t* pose;
    module_vehicle_phototube_line_observation_t line_observation;
    vehicle_path_replay_target_t replay_target;
    vehicle_path_projection_t projection;
    float32 theta_rad;
    float32 cos_theta;
    float32 sin_theta;
    float32 line_world_x_mm = 0.0f;
    float32 line_world_y_mm = 0.0f;
    float32 center_index_float;
    float32 error_x_mm = 0.0f;
    float32 error_y_mm = 0.0f;
    float32 correction_x_mm = 0.0f;
    float32 correction_y_mm = 0.0f;
    boolean applied = FALSE;
    boolean correction_runtime_allowed;

    module_vehicle_phototube_correction_event_miss();

    if (module_vehicle_phototube_runtime.drive_cal_state
        != MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_IDLE)
    {
        return;
    }

    correction_runtime_allowed =
        module_vehicle_phototube_correction_runtime_allowed();

    if (((module_vehicle_phototube_runtime.correction_enabled == FALSE)
         || (correction_runtime_allowed == FALSE)
         || (device_phototube_is_enabled() == FALSE))
        && (module_vehicle_phototube_runtime.correction_print_enabled == FALSE))
    {
        module_vehicle_phototube_correction_gate_reset();
        return;
    }

    (void)memset(&projection, 0, sizeof(projection));
    module_vehicle_phototube_runtime.correction_shape_pass = FALSE;
    module_vehicle_phototube_runtime.correction_gate_pass = FALSE;
    module_vehicle_phototube_runtime.correction_gate_target_yaw_rate_deg_s = 0.0f;
    module_vehicle_phototube_runtime.correction_gate_heading_delta_deg = 0.0f;
    if (module_vehicle_phototube_line_observation_get(&line_observation) == FALSE)
    {
        module_vehicle_phototube_correction_gate_reset();
        module_vehicle_phototube_correction_print(&line_observation,
                                                  0.0f,
                                                  0.0f,
                                                  &projection,
                                                  0.0f,
                                                  0.0f,
                                                  0.0f,
                                                  0.0f,
                                                  FALSE);
        return;
    }

    module_vehicle_phototube_runtime.line_observation = line_observation;
    module_vehicle_phototube_runtime.correction_shape_pass = TRUE;
    if ((module_vehicle_phototube_runtime.correction_sum_max > 0u)
        && (line_observation.line_sum > module_vehicle_phototube_runtime.correction_sum_max))
    {
        module_vehicle_phototube_runtime.correction_shape_pass = FALSE;
    }
    if ((module_vehicle_phototube_runtime.correction_active_count_max > 0u)
        && (line_observation.active_count
            > module_vehicle_phototube_runtime.correction_active_count_max))
    {
        module_vehicle_phototube_runtime.correction_shape_pass = FALSE;
    }
    if ((module_vehicle_phototube_runtime.correction_active_width_max_mm > 0.0f)
        && (line_observation.active_width_mm
            > module_vehicle_phototube_runtime.correction_active_width_max_mm))
    {
        module_vehicle_phototube_runtime.correction_shape_pass = FALSE;
    }
    if (line_observation.confidence < module_vehicle_phototube_runtime.correction_min_confidence)
    {
        module_vehicle_phototube_correction_gate_reset();
        module_vehicle_phototube_correction_print(&line_observation,
                                                  0.0f,
                                                  0.0f,
                                                  &projection,
                                                  0.0f,
                                                  0.0f,
                                                  0.0f,
                                                  0.0f,
                                                  FALSE);
        return;
    }

    if (module_vehicle_path_replay_target_get(&replay_target) == FALSE)
    {
        module_vehicle_phototube_correction_gate_reset();
        module_vehicle_phototube_correction_print(&line_observation,
                                                  0.0f,
                                                  0.0f,
                                                  &projection,
                                                  0.0f,
                                                  0.0f,
                                                  0.0f,
                                                  0.0f,
                                                  FALSE);
        return;
    }

    pose = module_vehicle_pose_fusion_observation_get();
    theta_rad = pose->theta_rad;
    cos_theta = cosf(theta_rad);
    sin_theta = sinf(theta_rad);
    line_world_x_mm = pose->x_mm
                    + (cos_theta * line_observation.line_body_x_mm)
                    - (sin_theta * line_observation.line_body_y_mm);
    line_world_y_mm = pose->y_mm
                    + (sin_theta * line_observation.line_body_x_mm)
                    + (cos_theta * line_observation.line_body_y_mm);

    /* Projection stays anchored to the vehicle turning-center path index.
     * The sensor offset is used only for the physical line world position. */
    center_index_float = replay_target.base_index_float;

    if (module_vehicle_path_project_local(line_world_x_mm,
                                          line_world_y_mm,
                                          center_index_float,
                                          module_vehicle_phototube_runtime.correction_window_mm,
                                          &projection) == FALSE)
    {
        module_vehicle_phototube_correction_gate_reset();
        module_vehicle_phototube_correction_print(&line_observation,
                                                  line_world_x_mm,
                                                  line_world_y_mm,
                                                  &projection,
                                                  0.0f,
                                                  0.0f,
                                                  0.0f,
                                                  0.0f,
                                                  FALSE);
        return;
    }

    error_x_mm = projection.point.x_mm - line_world_x_mm;
    error_y_mm = projection.point.y_mm - line_world_y_mm;
    if ((projection.distance_mm <= module_vehicle_phototube_runtime.correction_reject_distance_mm)
        && (module_vehicle_phototube_runtime.correction_shape_pass != FALSE))
    {
        if (correction_runtime_allowed != FALSE)
        {
            module_vehicle_phototube_runtime.correction_gate_pass =
                module_vehicle_phototube_correction_gate_update(&replay_target);
        }
        else
        {
            module_vehicle_phototube_correction_gate_reset();
        }
        correction_x_mm = error_x_mm
                        * module_vehicle_phototube_runtime.correction_gain
                        * line_observation.confidence;
        correction_y_mm = error_y_mm
                        * module_vehicle_phototube_runtime.correction_gain
                        * line_observation.confidence;
        (void)module_vehicle_phototube_vector_limit(&correction_x_mm,
                                                    &correction_y_mm,
                                                    module_vehicle_phototube_runtime.correction_max_step_mm);
        if ((module_vehicle_phototube_runtime.correction_enabled != FALSE)
            && (correction_runtime_allowed != FALSE)
            && (device_phototube_is_enabled() != FALSE)
            && (module_vehicle_phototube_runtime.correction_gate_pass != FALSE))
        {
            module_vehicle_pose_fusion_correct_xy(correction_x_mm, correction_y_mm);
            applied = TRUE;
            module_vehicle_phototube_correction_event_apply(&projection,
                                                             line_observation.confidence,
                                                             correction_x_mm,
                                                             correction_y_mm);
        }
    }
    else
    {
        module_vehicle_phototube_correction_gate_reset();
    }

    module_vehicle_phototube_correction_print(&line_observation,
                                              line_world_x_mm,
                                              line_world_y_mm,
                                              &projection,
                                              error_x_mm,
                                              error_y_mm,
                                              correction_x_mm,
                                              correction_y_mm,
                                              applied);
}

static void module_vehicle_phototube_correction_print(
    const module_vehicle_phototube_line_observation_t* line_observation,
    float32 line_world_x_mm,
    float32 line_world_y_mm,
    const vehicle_path_projection_t* projection,
    float32 error_x_mm,
    float32 error_y_mm,
    float32 correction_x_mm,
    float32 correction_y_mm,
    boolean applied)
{
    float32 projection_index = 0.0f;
    float32 projection_x_mm = 0.0f;
    float32 projection_y_mm = 0.0f;
    float32 projection_distance_mm = 0.0f;
    float32 line_body_x_mm = 0.0f;
    float32 line_body_y_mm = 0.0f;
    float32 pose_x_mm = 0.0f;
    float32 pose_y_mm = 0.0f;
    float32 active_width_mm = 0.0f;
    float32 confidence = 0.0f;
    uint32 line_sum = 0u;
    uint32 active_count = 0u;
    boolean line_valid = FALSE;
    boolean projection_valid = FALSE;
    const module_vehicle_pose_fusion_observation_t* pose;

    if (module_vehicle_phototube_runtime.correction_print_enabled == FALSE)
    {
        return;
    }

    module_vehicle_phototube_runtime.correction_print_count++;
    if (module_vehicle_phototube_runtime.correction_print_count
        < module_vehicle_phototube_runtime.correction_print_divider)
    {
        return;
    }
    module_vehicle_phototube_runtime.correction_print_count = 0u;

    if (line_observation != NULL_PTR)
    {
        line_valid = line_observation->valid;
        line_body_x_mm = line_observation->line_body_x_mm;
        line_body_y_mm = line_observation->line_body_y_mm;
        confidence = line_observation->confidence;
        line_sum = line_observation->line_sum;
        active_count = line_observation->active_count;
        active_width_mm = line_observation->active_width_mm;
    }

    pose = module_vehicle_pose_fusion_observation_get();
    if (pose != NULL_PTR)
    {
        pose_x_mm = pose->x_mm;
        pose_y_mm = pose->y_mm;
    }

    if ((projection != NULL_PTR) && (projection->valid != FALSE))
    {
        projection_valid = TRUE;
        projection_index = projection->index_float;
        projection_x_mm = projection->point.x_mm;
        projection_y_mm = projection->point.y_mm;
        projection_distance_mm = projection->distance_mm;
    }

    tools_printf("{ptcorr}%u,%u,%u,%u,%.2f,%.3f,%u,%.1f,%.1f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.3f,%.3f,%u,%u,%u,%.1f,%.1f,%u,%.1f,%u,%.1f,%.1f,%.1f,%u,%u,%u,%u\r\n",
                 (unsigned int)((module_vehicle_phototube_runtime.correction_enabled != FALSE) ? 1u : 0u),
                 (unsigned int)((applied != FALSE) ? 1u : 0u),
                 (unsigned int)((line_valid != FALSE) ? 1u : 0u),
                 (unsigned int)((projection_valid != FALSE) ? 1u : 0u),
                 (double)line_body_y_mm,
                 (double)confidence,
                 (unsigned int)line_sum,
                 (double)line_world_x_mm,
                 (double)line_world_y_mm,
                 (double)projection_index,
                 (double)projection_x_mm,
                 (double)projection_y_mm,
                 (double)projection_distance_mm,
                 (double)error_x_mm,
                 (double)error_y_mm,
                 (double)correction_x_mm,
                 (double)correction_y_mm,
                 (unsigned int)((module_vehicle_phototube_runtime.correction_gate_enabled != FALSE) ? 1u : 0u),
                 (unsigned int)((module_vehicle_phototube_runtime.correction_gate_pass != FALSE) ? 1u : 0u),
                 (unsigned int)module_vehicle_phototube_runtime.correction_gate_stable_count,
                 (double)module_vehicle_phototube_runtime.correction_gate_target_yaw_rate_deg_s,
                 (double)module_vehicle_phototube_runtime.correction_gate_heading_delta_deg,
                 (unsigned int)active_count,
                 (double)active_width_mm,
                 (unsigned int)((module_vehicle_phototube_runtime.correction_shape_pass != FALSE) ? 1u : 0u),
                 (double)line_body_x_mm,
                 (double)pose_x_mm,
                 (double)pose_y_mm,
                 (unsigned int)((module_vehicle_phototube_runtime.correction_monitor_enabled != FALSE) ? 1u : 0u),
                 (unsigned int)((device_phototube_is_enabled() != FALSE) ? 1u : 0u),
                 (unsigned int)((module_vehicle_phototube_correction_runtime_allowed() != FALSE) ? 1u : 0u),
                 (unsigned int)((module_vehicle_pose_fusion_replay_path_straight_get() != FALSE) ? 1u : 0u));
}

static void module_vehicle_phototube_correction_show(void)
{
    tools_printf("{ptcorr}enable,%u,monitor,%u,print,%u,div,%u,sensor,%.1f,spacing,%.2f,sign,%.0f,gain,%.3f,step,%.2f,window,%.1f,reject,%.1f,conf,%.3f,threshold,%u,summax,%u,countmax,%u,widthmax,%.1f,gate,%u,yaw,%.1f,heading,%.1f,stable,%u,gatecnt,%u,power,%u,actual,%u,allow,%u\r\n",
                 (unsigned int)((module_vehicle_phototube_runtime.correction_enabled != FALSE) ? 1u : 0u),
                 (unsigned int)((module_vehicle_phototube_runtime.correction_monitor_enabled != FALSE) ? 1u : 0u),
                 (unsigned int)((module_vehicle_phototube_runtime.correction_print_enabled != FALSE) ? 1u : 0u),
                 (unsigned int)module_vehicle_phototube_runtime.correction_print_divider,
                 (double)module_vehicle_phototube_runtime.correction_sensor_x_mm,
                 (double)module_vehicle_phototube_runtime.correction_sensor_spacing_mm,
                 (double)module_vehicle_phototube_runtime.correction_sensor_y_sign,
                 (double)module_vehicle_phototube_runtime.correction_gain,
                 (double)module_vehicle_phototube_runtime.correction_max_step_mm,
                 (double)module_vehicle_phototube_runtime.correction_window_mm,
                 (double)module_vehicle_phototube_runtime.correction_reject_distance_mm,
                 (double)module_vehicle_phototube_runtime.correction_min_confidence,
                 (unsigned int)module_vehicle_phototube_runtime.correction_strength_threshold,
                 (unsigned int)module_vehicle_phototube_runtime.correction_sum_max,
                 (unsigned int)module_vehicle_phototube_runtime.correction_active_count_max,
                 (double)module_vehicle_phototube_runtime.correction_active_width_max_mm,
                 (unsigned int)((module_vehicle_phototube_runtime.correction_gate_enabled != FALSE) ? 1u : 0u),
                 (double)(module_vehicle_phototube_runtime.correction_gate_yaw_rate_max_rad_s
                          * MODULE_VEHICLE_PHOTOTUBE_RAD_TO_DEG),
                 (double)(module_vehicle_phototube_runtime.correction_gate_heading_delta_max_rad
                          * MODULE_VEHICLE_PHOTOTUBE_RAD_TO_DEG),
                 (unsigned int)module_vehicle_phototube_runtime.correction_gate_stable_required_count,
                 (unsigned int)module_vehicle_phototube_runtime.correction_gate_stable_count,
                 (unsigned int)((module_vehicle_phototube_runtime.power_user_enabled != FALSE) ? 1u : 0u),
                 (unsigned int)((device_phototube_is_enabled() != FALSE) ? 1u : 0u),
                 (unsigned int)((module_vehicle_phototube_correction_runtime_allowed() != FALSE) ? 1u : 0u));
}

void module_vehicle_phototube_line_stop(void)
{
    if (module_vehicle_phototube_runtime.drive_cal_state
        != MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_IDLE)
    {
        module_vehicle_phototube_drive_calibration_abort("vstop");
    }
    module_vehicle_phototube_runtime.line_lost_drive_stop_pending = FALSE;
    module_vehicle_phototube_runtime.line_lost_suction_pending = FALSE;
    module_vehicle_phototube_runtime.line_mode = MODULE_VEHICLE_PHOTOTUBE_LINE_OFF;
    module_vehicle_phototube_runtime.line_launch_state =
        MODULE_VEHICLE_PHOTOTUBE_LAUNCH_OFF;
    module_vehicle_phototube_runtime.line_print_count = 0u;
    module_vehicle_phototube_line_control_reset();
    module_vehicle_phototube_line_maneuver_reset(FALSE);
    module_vehicle_phototube_line_lost_reset();
    (void)module_vehicle_phototube_control_speed_target_post(0.0f, 0.0f);
    (void)module_vehicle_phototube_control_speed_test_enable_post(FALSE);
    (void)module_vehicle_phototube_control_suction_post(0u);
    module_vehicle_phototube_power_update();
}

static void module_vehicle_phototube_correction_event_miss(void)
{
    if (module_vehicle_phototube_runtime.correction_event_active == FALSE)
    {
        return;
    }
    module_vehicle_phototube_runtime.correction_event_release_count++;
    if (module_vehicle_phototube_runtime.correction_event_release_count
        >= MODULE_VEHICLE_PHOTOTUBE_CORR_EVENT_RELEASE_COUNT)
    {
        module_vehicle_phototube_correction_event_finish();
    }
}

static void module_vehicle_phototube_correction_event_apply(
    const vehicle_path_projection_t* projection,
    float32 confidence,
    float32 correction_x_mm,
    float32 correction_y_mm)
{
    float32 step_mm = sqrtf((correction_x_mm * correction_x_mm)
                            + (correction_y_mm * correction_y_mm));

    if (module_vehicle_phototube_runtime.correction_event_active == FALSE)
    {
        module_vehicle_phototube_runtime.correction_event_active = TRUE;
        module_vehicle_phototube_runtime.correction_event_sample_count = 0u;
        module_vehicle_phototube_runtime.correction_event_sum_x_mm = 0.0f;
        module_vehicle_phototube_runtime.correction_event_sum_y_mm = 0.0f;
        module_vehicle_phototube_runtime.correction_event_peak_step_mm = 0.0f;
        module_vehicle_phototube_runtime.correction_event_max_confidence = 0.0f;
        module_vehicle_phototube_runtime.correction_event_start_index = projection->index_float;
        module_vehicle_phototube_runtime.correction_event_start_x_mm = projection->point.x_mm;
        module_vehicle_phototube_runtime.correction_event_start_y_mm = projection->point.y_mm;
    }
    module_vehicle_phototube_runtime.correction_event_release_count = 0u;
    module_vehicle_phototube_runtime.correction_event_sample_count++;
    module_vehicle_phototube_runtime.correction_event_end_index = projection->index_float;
    module_vehicle_phototube_runtime.correction_event_end_x_mm = projection->point.x_mm;
    module_vehicle_phototube_runtime.correction_event_end_y_mm = projection->point.y_mm;
    module_vehicle_phototube_runtime.correction_event_sum_x_mm += correction_x_mm;
    module_vehicle_phototube_runtime.correction_event_sum_y_mm += correction_y_mm;
    if (step_mm > module_vehicle_phototube_runtime.correction_event_peak_step_mm)
    {
        module_vehicle_phototube_runtime.correction_event_peak_step_mm = step_mm;
    }
    if (confidence > module_vehicle_phototube_runtime.correction_event_max_confidence)
    {
        module_vehicle_phototube_runtime.correction_event_max_confidence = confidence;
    }
}

static void module_vehicle_phototube_correction_event_finish(void)
{
    if (module_vehicle_phototube_runtime.correction_event_active == FALSE)
    {
        return;
    }
    tools_printf("{ptcorrseg}%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%u,%.3f\r\n",
                 (double)module_vehicle_phototube_runtime.correction_event_start_index,
                 (double)module_vehicle_phototube_runtime.correction_event_end_index,
                 (double)module_vehicle_phototube_runtime.correction_event_start_x_mm,
                 (double)module_vehicle_phototube_runtime.correction_event_start_y_mm,
                 (double)module_vehicle_phototube_runtime.correction_event_end_x_mm,
                 (double)module_vehicle_phototube_runtime.correction_event_end_y_mm,
                 (double)module_vehicle_phototube_runtime.correction_event_sum_x_mm,
                 (double)module_vehicle_phototube_runtime.correction_event_sum_y_mm,
                 (double)module_vehicle_phototube_runtime.correction_event_peak_step_mm,
                 (unsigned int)module_vehicle_phototube_runtime.correction_event_sample_count,
                 (double)module_vehicle_phototube_runtime.correction_event_max_confidence);
    module_vehicle_phototube_runtime.correction_event_active = FALSE;
    module_vehicle_phototube_runtime.correction_event_release_count = 0u;
}

static void module_vehicle_phototube_line_maneuver_reset(boolean reset_event_index)
{
    module_vehicle_phototube_runtime.maneuver_state =
        MODULE_VEHICLE_PHOTOTUBE_MANEUVER_FOLLOW;
    module_vehicle_phototube_runtime.maneuver_action =
        MODULE_VEHICLE_PHOTOTUBE_TURN_EVALUATE;
    module_vehicle_phototube_runtime.maneuver_entry_heading_rad = 0.0f;
    module_vehicle_phototube_runtime.maneuver_final_heading_rad = 0.0f;
    module_vehicle_phototube_runtime.maneuver_target_heading_rad = 0.0f;
    module_vehicle_phototube_runtime.maneuver_entry_distance_mm = 0.0f;
    module_vehicle_phototube_runtime.maneuver_last_distance_mm = 0.0f;
    module_vehicle_phototube_runtime.maneuver_elapsed_s = 0.0f;
    module_vehicle_phototube_runtime.element_candidate_s = 0.0f;
    module_vehicle_phototube_runtime.element_candidate_count = 0u;
    module_vehicle_phototube_runtime.stable_heading_rad = 0.0f;
    module_vehicle_phototube_runtime.stable_heading_s = 0.0f;
    module_vehicle_phototube_runtime.stable_heading_age_s = 0.0f;
    module_vehicle_phototube_runtime.stable_heading_valid = FALSE;
    module_vehicle_phototube_runtime.element_entry_heading_rad = 0.0f;
    module_vehicle_phototube_runtime.element_entry_heading_valid = FALSE;
    module_vehicle_phototube_runtime.normal_confidence_s = 0.0f;
    module_vehicle_phototube_runtime.normal_confidence_distance_mm = 0.0f;
    module_vehicle_phototube_runtime.normal_confidence_count = 0u;
    module_vehicle_phototube_runtime.line_follow_blend = 1.0f;
    module_vehicle_phototube_runtime.line_last_tick = 0u;
    module_vehicle_phototube_runtime.line_last_tick_valid = FALSE;
    if (reset_event_index != FALSE)
    {
        module_vehicle_phototube_runtime.turn_event_index = 0u;
    }
}

static float32 module_vehicle_phototube_line_frame_dt_get(void)
{
    uint64 current_tick = sysTick_getTick(SYSTICK1);
    uint64 elapsed_tick;
    uint64 elapsed_us;
    float32 dt_s = 0.0f;

    if (module_vehicle_phototube_runtime.line_last_tick_valid != FALSE)
    {
        elapsed_tick = current_tick - module_vehicle_phototube_runtime.line_last_tick;
        elapsed_us = sysTick_ticksToMicroseconds(SYSTICK1, elapsed_tick);
        dt_s = (float32)elapsed_us * MODULE_VEHICLE_PHOTOTUBE_US_TO_S;
        dt_s = module_vehicle_phototube_clamp_f32(dt_s, 0.0f, 0.05f);
    }
    module_vehicle_phototube_runtime.line_last_tick = current_tick;
    module_vehicle_phototube_runtime.line_last_tick_valid = TRUE;
    return dt_s;
}

static uint32 module_vehicle_phototube_line_active_count_get(void)
{
    uint32 index;
    uint32 count = 0u;

    for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
    {
        if (module_vehicle_phototube_normalized_value_get(index)
            >= MODULE_VEHICLE_PHOTOTUBE_LINE_ACTIVE_THRESHOLD)
        {
            count++;
        }
    }
    return count;
}

static void module_vehicle_phototube_line_heading_anchor_update(float32 dt_s)
{
    const module_vehicle_pose_fusion_observation_t* pose =
        module_vehicle_pose_fusion_observation_get();
    float32 abs_error = module_vehicle_phototube_runtime.line_error;
    boolean centered;

    if (abs_error < 0.0f)
    {
        abs_error = -abs_error;
    }
    centered = ((module_vehicle_phototube_runtime.line_valid != FALSE)
                && (module_vehicle_phototube_runtime.line_sum
                    < MODULE_VEHICLE_PHOTOTUBE_LINE_ELEMENT_SUM_MIN)
                && (abs_error
                    <= MODULE_VEHICLE_PHOTOTUBE_LINE_HEADING_ANCHOR_ERROR_MAX))
                   ? TRUE
                   : FALSE;

    if (centered != FALSE)
    {
        module_vehicle_phototube_runtime.stable_heading_s += dt_s;
        module_vehicle_phototube_runtime.stable_heading_age_s = 0.0f;
        if (module_vehicle_phototube_runtime.stable_heading_s
            >= MODULE_VEHICLE_PHOTOTUBE_LINE_HEADING_ANCHOR_STABLE_S)
        {
            module_vehicle_phototube_runtime.stable_heading_s =
                MODULE_VEHICLE_PHOTOTUBE_LINE_HEADING_ANCHOR_STABLE_S;
            module_vehicle_phototube_runtime.stable_heading_rad = pose->theta_rad;
            module_vehicle_phototube_runtime.stable_heading_valid = TRUE;
        }
    }
    else
    {
        module_vehicle_phototube_runtime.stable_heading_s -= dt_s * 2.0f;
        if (module_vehicle_phototube_runtime.stable_heading_s < 0.0f)
        {
            module_vehicle_phototube_runtime.stable_heading_s = 0.0f;
        }
        module_vehicle_phototube_runtime.stable_heading_age_s += dt_s;
        if (module_vehicle_phototube_runtime.stable_heading_age_s
            > MODULE_VEHICLE_PHOTOTUBE_LINE_HEADING_ANCHOR_MAX_AGE_S)
        {
            module_vehicle_phototube_runtime.stable_heading_valid = FALSE;
        }
    }
}

static boolean module_vehicle_phototube_line_element_detected(float32 dt_s)
{
    const module_vehicle_pose_fusion_observation_t* pose =
        module_vehicle_pose_fusion_observation_get();
    boolean broad_white =
        (module_vehicle_phototube_runtime.line_sum
         >= MODULE_VEHICLE_PHOTOTUBE_LINE_ELEMENT_SUM_MIN)
            ? TRUE
            : FALSE;

    if (broad_white != FALSE)
    {
        if (module_vehicle_phototube_runtime.element_candidate_count == 0u)
        {
            module_vehicle_phototube_runtime.element_entry_heading_rad =
                (module_vehicle_phototube_runtime.stable_heading_valid != FALSE)
                    ? module_vehicle_phototube_runtime.stable_heading_rad
                    : pose->theta_rad;
            module_vehicle_phototube_runtime.element_entry_heading_valid = TRUE;
            module_vehicle_phototube_line_control_reset();
        }
        module_vehicle_phototube_runtime.element_candidate_s += dt_s;
        module_vehicle_phototube_runtime.element_candidate_count++;
    }
    else
    {
        if (module_vehicle_phototube_runtime.element_candidate_count > 0u)
        {
            module_vehicle_phototube_runtime.line_follow_blend = 0.0f;
            module_vehicle_phototube_line_control_reset();
        }
        module_vehicle_phototube_runtime.element_candidate_s = 0.0f;
        module_vehicle_phototube_runtime.element_candidate_count = 0u;
        module_vehicle_phototube_runtime.element_entry_heading_valid = FALSE;
    }
    if ((broad_white != FALSE)
        && (module_vehicle_phototube_runtime.element_guard_enabled == FALSE))
    {
        return TRUE;
    }
    return (((module_vehicle_phototube_runtime.element_candidate_s
              >= MODULE_VEHICLE_PHOTOTUBE_LINE_ELEMENT_CONFIRM_S)
             && (module_vehicle_phototube_runtime.element_candidate_count
                 >= MODULE_VEHICLE_PHOTOTUBE_LINE_ELEMENT_CONFIRM_COUNT))
                ? TRUE
                : FALSE);
}

static boolean module_vehicle_phototube_line_normal_detected(void)
{
    return ((module_vehicle_phototube_runtime.line_valid != FALSE)
            && (module_vehicle_phototube_runtime.line_sum
                >= MODULE_VEHICLE_PHOTOTUBE_LINE_NORMAL_SUM_MIN)
            && (module_vehicle_phototube_runtime.line_sum
                <= MODULE_VEHICLE_PHOTOTUBE_LINE_NORMAL_SUM_MAX)
            && (module_vehicle_phototube_runtime.line_active_count
                >= MODULE_VEHICLE_PHOTOTUBE_LINE_NORMAL_ACTIVE_MIN)
            && (module_vehicle_phototube_runtime.line_active_count
                <= MODULE_VEHICLE_PHOTOTUBE_LINE_NORMAL_ACTIVE_MAX))
               ? TRUE
               : FALSE;
}

static void module_vehicle_phototube_line_lost_reset(void)
{
    const module_vehicle_pose_fusion_observation_t* pose =
        module_vehicle_pose_fusion_observation_get();

    module_vehicle_phototube_runtime.line_lost_confidence_s = 0.0f;
    module_vehicle_phototube_runtime.line_lost_confidence_distance_mm = 0.0f;
    module_vehicle_phototube_runtime.line_lost_last_distance_mm = pose->distance_mm;
    module_vehicle_phototube_runtime.line_lost_distance_initialized = TRUE;
}

static boolean module_vehicle_phototube_line_lost_update(float32 dt_s)
{
    const module_vehicle_pose_fusion_observation_t* pose =
        module_vehicle_pose_fusion_observation_get();
    float32 step_mm = 0.0f;
    boolean line_observed =
        ((module_vehicle_phototube_runtime.line_valid != FALSE)
         && (module_vehicle_phototube_runtime.line_sum
             >= MODULE_VEHICLE_PHOTOTUBE_LINE_LOST_SUM_MIN))
            ? TRUE
            : FALSE;

    if (module_vehicle_phototube_runtime.line_lost_distance_initialized != FALSE)
    {
        step_mm = pose->distance_mm
                  - module_vehicle_phototube_runtime.line_lost_last_distance_mm;
        if (step_mm < 0.0f)
        {
            step_mm = -step_mm;
        }
    }
    else
    {
        module_vehicle_phototube_runtime.line_lost_distance_initialized = TRUE;
    }
    module_vehicle_phototube_runtime.line_lost_last_distance_mm = pose->distance_mm;

    if (line_observed == FALSE)
    {
        module_vehicle_phototube_runtime.line_lost_confidence_s += dt_s;
        module_vehicle_phototube_runtime.line_lost_confidence_distance_mm += step_mm;
    }
    else
    {
        module_vehicle_phototube_runtime.line_lost_confidence_s -=
            dt_s * MODULE_VEHICLE_PHOTOTUBE_LINE_LOST_RECOVERY_RATIO;
        module_vehicle_phototube_runtime.line_lost_confidence_distance_mm -=
            step_mm * MODULE_VEHICLE_PHOTOTUBE_LINE_LOST_RECOVERY_RATIO;
    }

    module_vehicle_phototube_runtime.line_lost_confidence_s =
        module_vehicle_phototube_clamp_f32(
            module_vehicle_phototube_runtime.line_lost_confidence_s,
            0.0f,
            MODULE_VEHICLE_PHOTOTUBE_LINE_LOST_TIMEOUT_S);
    module_vehicle_phototube_runtime.line_lost_confidence_distance_mm =
        module_vehicle_phototube_clamp_f32(
            module_vehicle_phototube_runtime.line_lost_confidence_distance_mm,
            0.0f,
            MODULE_VEHICLE_PHOTOTUBE_LINE_LOST_DISTANCE_MM);

    if ((module_vehicle_phototube_runtime.line_lost_confidence_s
         >= MODULE_VEHICLE_PHOTOTUBE_LINE_LOST_TIMEOUT_S)
        || (module_vehicle_phototube_runtime.line_lost_confidence_distance_mm
            >= MODULE_VEHICLE_PHOTOTUBE_LINE_LOST_DISTANCE_MM))
    {
        module_vehicle_phototube_line_lost_stop();
        return TRUE;
    }
    return FALSE;
}

static void module_vehicle_phototube_line_lost_stop(void)
{
    uint32 suction_duty = (vehicle_control_auto_suction_enabled_get() != FALSE)
                              ? VEHICLE_CONTROL_DEFAULT_RUN_SUCTION_DUTY
                              : 0u;
    boolean stop_posted;

    tools_printf("{ptline}line_lost,time,%.3f,distance,%.1f,sum,%u\r\n",
                 (double)module_vehicle_phototube_runtime.line_lost_confidence_s,
                 (double)module_vehicle_phototube_runtime.line_lost_confidence_distance_mm,
                 (unsigned int)module_vehicle_phototube_runtime.line_sum);
    module_vehicle_phototube_runtime.line_mode = MODULE_VEHICLE_PHOTOTUBE_LINE_OFF;
    module_vehicle_phototube_runtime.line_launch_state =
        MODULE_VEHICLE_PHOTOTUBE_LAUNCH_OFF;
    module_vehicle_phototube_runtime.line_print_count = 0u;
    module_vehicle_phototube_line_control_reset();
    module_vehicle_phototube_line_maneuver_reset(FALSE);
    stop_posted = module_vehicle_phototube_control_drive_stop_keep_suction_post(suction_duty);
    module_vehicle_phototube_runtime.line_lost_drive_stop_pending =
        (stop_posted != FALSE) ? FALSE : TRUE;
    module_vehicle_phototube_runtime.line_lost_suction_pending = TRUE;
    module_vehicle_phototube_runtime.line_lost_stop_tick = sysTick_getTick(SYSTICK1);
}

static void module_vehicle_phototube_line_lost_suction_update(void)
{
    uint64 elapsed_tick;
    uint64 elapsed_us;

    if (module_vehicle_phototube_runtime.line_lost_suction_pending == FALSE)
    {
        return;
    }

    if (module_vehicle_phototube_runtime.line_lost_drive_stop_pending != FALSE)
    {
        uint32 suction_duty = (vehicle_control_auto_suction_enabled_get() != FALSE)
                                  ? VEHICLE_CONTROL_DEFAULT_RUN_SUCTION_DUTY
                                  : 0u;

        if (module_vehicle_phototube_control_drive_stop_keep_suction_post(suction_duty) == FALSE)
        {
            return;
        }
        module_vehicle_phototube_runtime.line_lost_drive_stop_pending = FALSE;
        module_vehicle_phototube_runtime.line_lost_stop_tick = sysTick_getTick(SYSTICK1);
    }

    elapsed_tick = sysTick_getTick(SYSTICK1)
                   - module_vehicle_phototube_runtime.line_lost_stop_tick;
    elapsed_us = sysTick_ticksToMicroseconds(SYSTICK1, elapsed_tick);
    if (elapsed_us < MODULE_VEHICLE_PHOTOTUBE_LINE_LOST_SUCTION_DELAY_US)
    {
        return;
    }

    if (module_vehicle_phototube_control_suction_post(0u) != FALSE)
    {
        module_vehicle_phototube_runtime.line_lost_suction_pending = FALSE;
        tools_printf("{ptline}line_lost_suction_off\r\n");
    }
}

static void module_vehicle_phototube_line_maneuver_start(void)
{
    const module_vehicle_pose_fusion_observation_t* pose =
        module_vehicle_pose_fusion_observation_get();
    uint32 event_index = module_vehicle_phototube_runtime.turn_event_index;
    module_vehicle_phototube_turn_action_t action =
        module_vehicle_phototube_turn_action_get(event_index);
    float32 target_delta_rad = 0.0f;

    if (action == MODULE_VEHICLE_PHOTOTUBE_TURN_LEFT)
    {
        target_delta_rad = MODULE_VEHICLE_PHOTOTUBE_LINE_TURN_ANGLE_RAD;
    }
    else if (action == MODULE_VEHICLE_PHOTOTUBE_TURN_RIGHT)
    {
        target_delta_rad = -MODULE_VEHICLE_PHOTOTUBE_LINE_TURN_ANGLE_RAD;
    }

    module_vehicle_phototube_runtime.maneuver_state =
        MODULE_VEHICLE_PHOTOTUBE_MANEUVER_HEADING;
    module_vehicle_phototube_runtime.maneuver_action = action;
    module_vehicle_phototube_runtime.maneuver_entry_heading_rad =
        (module_vehicle_phototube_runtime.element_entry_heading_valid != FALSE)
            ? module_vehicle_phototube_runtime.element_entry_heading_rad
            : pose->theta_rad;
    module_vehicle_phototube_runtime.maneuver_final_heading_rad =
        module_vehicle_phototube_wrap_pi(
            module_vehicle_phototube_runtime.maneuver_entry_heading_rad
            + target_delta_rad);
    module_vehicle_phototube_runtime.maneuver_target_heading_rad =
        module_vehicle_phototube_runtime.maneuver_entry_heading_rad;
    module_vehicle_phototube_runtime.maneuver_entry_distance_mm = pose->distance_mm;
    module_vehicle_phototube_runtime.maneuver_last_distance_mm = pose->distance_mm;
    module_vehicle_phototube_runtime.maneuver_elapsed_s = 0.0f;
    module_vehicle_phototube_runtime.element_candidate_s = 0.0f;
    module_vehicle_phototube_runtime.element_candidate_count = 0u;
    module_vehicle_phototube_runtime.normal_confidence_s = 0.0f;
    module_vehicle_phototube_runtime.normal_confidence_distance_mm = 0.0f;
    module_vehicle_phototube_runtime.normal_confidence_count = 0u;
    module_vehicle_phototube_runtime.stable_heading_valid = FALSE;
    module_vehicle_phototube_runtime.turn_event_index++;
    module_vehicle_phototube_line_control_reset();
    tools_printf("{ptturn}enter,%u,%u,%s,sum,%u,active,%u,heading,%.2f,base,%.2f,distance,%.1f,radius,%.1f,arc,%.1f\r\n",
                 (unsigned int)event_index,
                 (unsigned int)action,
                 module_vehicle_phototube_turn_action_name(action),
                 (unsigned int)module_vehicle_phototube_runtime.line_sum,
                 (unsigned int)module_vehicle_phototube_runtime.line_active_count,
                 (double)(pose->theta_rad * MODULE_VEHICLE_PHOTOTUBE_RAD_TO_DEG),
                 (double)(module_vehicle_phototube_runtime.maneuver_entry_heading_rad
                          * MODULE_VEHICLE_PHOTOTUBE_RAD_TO_DEG),
                 (double)pose->distance_mm,
                 (double)module_vehicle_phototube_runtime.turn_radius_mm,
                 (double)(module_vehicle_phototube_runtime.turn_radius_mm
                          * MODULE_VEHICLE_PHOTOTUBE_LINE_TURN_ANGLE_RAD));
}

static boolean module_vehicle_phototube_line_maneuver_update(float32 dt_s)
{
    const module_vehicle_pose_fusion_observation_t* pose =
        module_vehicle_pose_fusion_observation_get();
    float32 traveled_mm = pose->distance_mm
                          - module_vehicle_phototube_runtime.maneuver_entry_distance_mm;
    float32 step_mm = pose->distance_mm
                      - module_vehicle_phototube_runtime.maneuver_last_distance_mm;
    float32 target_delta_rad = 0.0f;
    float32 heading_error_rad;
    boolean normal_line;

    if (traveled_mm < 0.0f)
    {
        traveled_mm = -traveled_mm;
    }
    if (step_mm < 0.0f)
    {
        step_mm = -step_mm;
    }
    module_vehicle_phototube_runtime.maneuver_last_distance_mm = pose->distance_mm;
    module_vehicle_phototube_runtime.maneuver_elapsed_s += dt_s;

    if ((module_vehicle_phototube_runtime.maneuver_action
         == MODULE_VEHICLE_PHOTOTUBE_TURN_LEFT)
        || (module_vehicle_phototube_runtime.maneuver_action
            == MODULE_VEHICLE_PHOTOTUBE_TURN_RIGHT))
    {
        target_delta_rad = traveled_mm / module_vehicle_phototube_runtime.turn_radius_mm;
        target_delta_rad = module_vehicle_phototube_clamp_f32(
            target_delta_rad,
            0.0f,
            MODULE_VEHICLE_PHOTOTUBE_LINE_TURN_ANGLE_RAD);
        if (module_vehicle_phototube_runtime.maneuver_action
            == MODULE_VEHICLE_PHOTOTUBE_TURN_RIGHT)
        {
            target_delta_rad = -target_delta_rad;
        }
    }
    module_vehicle_phototube_runtime.maneuver_target_heading_rad =
        module_vehicle_phototube_wrap_pi(
            module_vehicle_phototube_runtime.maneuver_entry_heading_rad
            + target_delta_rad);

    if (module_vehicle_phototube_control_heading_target_post(
            module_vehicle_phototube_runtime.line_base_speed_mm_s,
            module_vehicle_phototube_runtime.maneuver_target_heading_rad) == FALSE)
    {
        tools_printf("{ptturn}control_failed\r\n");
        module_vehicle_phototube_line_stop();
        return FALSE;
    }

    normal_line = module_vehicle_phototube_line_normal_detected();
    if (normal_line != FALSE)
    {
        module_vehicle_phototube_runtime.normal_confidence_s += dt_s;
        module_vehicle_phototube_runtime.normal_confidence_distance_mm += step_mm;
        if (module_vehicle_phototube_runtime.normal_confidence_count
            < MODULE_VEHICLE_PHOTOTUBE_LINE_REACQUIRE_STABLE_COUNT)
        {
            module_vehicle_phototube_runtime.normal_confidence_count++;
        }
    }
    else
    {
        module_vehicle_phototube_runtime.normal_confidence_s -= dt_s * 2.0f;
        module_vehicle_phototube_runtime.normal_confidence_distance_mm -= step_mm * 2.0f;
        if (module_vehicle_phototube_runtime.normal_confidence_count > 2u)
        {
            module_vehicle_phototube_runtime.normal_confidence_count -= 2u;
        }
        else
        {
            module_vehicle_phototube_runtime.normal_confidence_count = 0u;
        }
    }
    module_vehicle_phototube_runtime.normal_confidence_s =
        module_vehicle_phototube_clamp_f32(
            module_vehicle_phototube_runtime.normal_confidence_s,
            0.0f,
            MODULE_VEHICLE_PHOTOTUBE_LINE_REACQUIRE_STABLE_S);
    module_vehicle_phototube_runtime.normal_confidence_distance_mm =
        module_vehicle_phototube_clamp_f32(
            module_vehicle_phototube_runtime.normal_confidence_distance_mm,
            0.0f,
            MODULE_VEHICLE_PHOTOTUBE_LINE_REACQUIRE_STABLE_DISTANCE_MM);

    heading_error_rad = module_vehicle_phototube_wrap_pi(
        module_vehicle_phototube_runtime.maneuver_final_heading_rad - pose->theta_rad);
    if (heading_error_rad < 0.0f)
    {
        heading_error_rad = -heading_error_rad;
    }
    if ((normal_line != FALSE)
        && (traveled_mm >= MODULE_VEHICLE_PHOTOTUBE_LINE_REACQUIRE_MIN_DISTANCE_MM)
        && (heading_error_rad <= MODULE_VEHICLE_PHOTOTUBE_LINE_REACQUIRE_ANGLE_RAD)
        && (module_vehicle_phototube_runtime.normal_confidence_s
            >= MODULE_VEHICLE_PHOTOTUBE_LINE_REACQUIRE_STABLE_S)
        && (module_vehicle_phototube_runtime.normal_confidence_distance_mm
            >= MODULE_VEHICLE_PHOTOTUBE_LINE_REACQUIRE_STABLE_DISTANCE_MM)
        && (module_vehicle_phototube_runtime.normal_confidence_count
            >= MODULE_VEHICLE_PHOTOTUBE_LINE_REACQUIRE_STABLE_COUNT))
    {
        tools_printf("{ptturn}exit,%u,%s,distance,%.1f,angle_error,%.2f\r\n",
                     (unsigned int)(module_vehicle_phototube_runtime.turn_event_index - 1u),
                     module_vehicle_phototube_turn_action_name(
                         module_vehicle_phototube_runtime.maneuver_action),
                     (double)traveled_mm,
                     (double)(heading_error_rad * MODULE_VEHICLE_PHOTOTUBE_RAD_TO_DEG));
        module_vehicle_phototube_runtime.maneuver_state =
            MODULE_VEHICLE_PHOTOTUBE_MANEUVER_FOLLOW;
        module_vehicle_phototube_runtime.element_candidate_s = 0.0f;
        module_vehicle_phototube_runtime.element_candidate_count = 0u;
        module_vehicle_phototube_runtime.line_follow_blend = 0.0f;
        module_vehicle_phototube_line_control_reset();
        return TRUE;
    }

    if ((module_vehicle_phototube_runtime.maneuver_elapsed_s
         >= MODULE_VEHICLE_PHOTOTUBE_LINE_MANEUVER_TIMEOUT_S)
        || (traveled_mm >= MODULE_VEHICLE_PHOTOTUBE_LINE_MANEUVER_MAX_DISTANCE_MM))
    {
        tools_printf("{ptturn}timeout,%u,%s,distance,%.1f,elapsed,%.3f,angle_error,%.2f\r\n",
                     (unsigned int)(module_vehicle_phototube_runtime.turn_event_index - 1u),
                     module_vehicle_phototube_turn_action_name(
                         module_vehicle_phototube_runtime.maneuver_action),
                     (double)traveled_mm,
                     (double)module_vehicle_phototube_runtime.maneuver_elapsed_s,
                     (double)(heading_error_rad * MODULE_VEHICLE_PHOTOTUBE_RAD_TO_DEG));
        module_vehicle_phototube_line_stop();
    }
    return FALSE;
}

static module_vehicle_phototube_turn_action_t module_vehicle_phototube_turn_action_get(uint32 index)
{
    if (index >= module_vehicle_phototube_runtime.turn_table_count)
    {
        return MODULE_VEHICLE_PHOTOTUBE_TURN_EVALUATE;
    }
    return module_vehicle_phototube_runtime.turn_table[index];
}

static boolean module_vehicle_phototube_turn_action_parse(
    uint8* text,
    module_vehicle_phototube_turn_action_t* action)
{
    if ((text == NULL_PTR) || (action == NULL_PTR))
    {
        return FALSE;
    }
    if ((module_vehicle_phototube_command_is(text, "0") != FALSE)
        || (module_vehicle_phototube_command_is(text, "l") != FALSE)
        || (module_vehicle_phototube_command_is(text, "left") != FALSE))
    {
        *action = MODULE_VEHICLE_PHOTOTUBE_TURN_LEFT;
        return TRUE;
    }
    if ((module_vehicle_phototube_command_is(text, "1") != FALSE)
        || (module_vehicle_phototube_command_is(text, "r") != FALSE)
        || (module_vehicle_phototube_command_is(text, "right") != FALSE))
    {
        *action = MODULE_VEHICLE_PHOTOTUBE_TURN_RIGHT;
        return TRUE;
    }
    if ((module_vehicle_phototube_command_is(text, "2") != FALSE)
        || (module_vehicle_phototube_command_is(text, "e") != FALSE)
        || (module_vehicle_phototube_command_is(text, "eval") != FALSE))
    {
        *action = MODULE_VEHICLE_PHOTOTUBE_TURN_EVALUATE;
        return TRUE;
    }
    if ((module_vehicle_phototube_command_is(text, "3") != FALSE)
        || (module_vehicle_phototube_command_is(text, "s") != FALSE)
        || (module_vehicle_phototube_command_is(text, "straight") != FALSE))
    {
        *action = MODULE_VEHICLE_PHOTOTUBE_TURN_STRAIGHT;
        return TRUE;
    }
    return FALSE;
}

static const char* module_vehicle_phototube_turn_action_name(
    module_vehicle_phototube_turn_action_t action)
{
    switch (action)
    {
        case MODULE_VEHICLE_PHOTOTUBE_TURN_LEFT:
            return "left";
        case MODULE_VEHICLE_PHOTOTUBE_TURN_RIGHT:
            return "right";
        case MODULE_VEHICLE_PHOTOTUBE_TURN_STRAIGHT:
            return "straight";
        case MODULE_VEHICLE_PHOTOTUBE_TURN_EVALUATE:
        default:
            return "eval";
    }
}

static void module_vehicle_phototube_power_update(void)
{
    device_phototube_set_enabled(module_vehicle_phototube_power_needed());
}

static boolean module_vehicle_phototube_power_needed(void)
{
    if (module_vehicle_phototube_runtime.power_user_enabled == FALSE)
    {
        return FALSE;
    }
    if (module_vehicle_phototube_runtime.line_mode != MODULE_VEHICLE_PHOTOTUBE_LINE_OFF)
    {
        return TRUE;
    }
    if (module_vehicle_phototube_runtime.line_lost_suction_pending != FALSE)
    {
        return TRUE;
    }
    if ((module_vehicle_phototube_runtime.calibration_active != FALSE)
        || (module_vehicle_phototube_runtime.auto_state != MODULE_VEHICLE_PHOTOTUBE_AUTO_IDLE))
    {
        return TRUE;
    }
    if (module_vehicle_phototube_runtime.correction_enabled != FALSE)
    {
        return TRUE;
    }
    if (module_vehicle_phototube_runtime.drive_cal_state
        != MODULE_VEHICLE_PHOTOTUBE_DRIVE_CAL_IDLE)
    {
        return TRUE;
    }
    if (module_vehicle_phototube_runtime.correction_monitor_enabled != FALSE)
    {
        return TRUE;
    }

    return FALSE;
}

static boolean module_vehicle_phototube_correction_runtime_allowed(void)
{
    return ((module_vehicle_phototube_runtime.correction_enabled != FALSE)
            && (vehicle_control_phototube_correction_allowed() != FALSE))
               ? TRUE
               : FALSE;
}

static void module_vehicle_phototube_power_show(void)
{
    tools_printf("{ptpower}user,%u,actual,%u,need,%u,line,%u,cal,%u,corr,%u,allow,%u\r\n",
                 (unsigned int)((module_vehicle_phototube_runtime.power_user_enabled != FALSE) ? 1u : 0u),
                 (unsigned int)((device_phototube_is_enabled() != FALSE) ? 1u : 0u),
                 (unsigned int)((module_vehicle_phototube_power_needed() != FALSE) ? 1u : 0u),
                 (unsigned int)module_vehicle_phototube_runtime.line_mode,
                 (unsigned int)(((module_vehicle_phototube_runtime.calibration_active != FALSE)
                                 || (module_vehicle_phototube_runtime.auto_state
                                     != MODULE_VEHICLE_PHOTOTUBE_AUTO_IDLE)) ? 1u : 0u),
                 (unsigned int)((module_vehicle_phototube_runtime.correction_enabled != FALSE) ? 1u : 0u),
                 (unsigned int)((module_vehicle_phototube_correction_runtime_allowed() != FALSE) ? 1u : 0u));
}

static boolean module_vehicle_phototube_control_speed_test_enable_post(boolean enable)
{
    vehicle_control_command_t command;
    boolean result;

    command.type = VEHICLE_CONTROL_COMMAND_SPEED_TEST_ENABLE;
    command.enable = (enable != FALSE) ? TRUE : FALSE;
    command.speed_mm_s = 0.0f;
    command.theta_rad = 0.0f;
    command.left_speed_mm_s = 0.0f;
    command.right_speed_mm_s = 0.0f;
    command.step_deg = 0.0f;
    command.left_duty = 0;
    command.right_duty = 0;
    command.suction_duty = 0u;
    result = vehicle_control_command_post(&command);
    if (result == FALSE)
    {
        tools_printf("{ptline}ctrl_queue_full,%u\r\n", (unsigned int)command.type);
    }
    return result;
}

static boolean module_vehicle_phototube_control_speed_test_stop_post(void)
{
    vehicle_control_command_t command;
    boolean result;

    command.type = VEHICLE_CONTROL_COMMAND_SPEED_TEST_STOP;
    command.enable = FALSE;
    command.speed_mm_s = 0.0f;
    command.theta_rad = 0.0f;
    command.left_speed_mm_s = 0.0f;
    command.right_speed_mm_s = 0.0f;
    command.step_deg = 0.0f;
    command.left_duty = 0;
    command.right_duty = 0;
    command.suction_duty = 0u;
    result = vehicle_control_command_post(&command);
    if (result == FALSE)
    {
        tools_printf("{ptcaldrive}ctrl_queue_full,%u\r\n", (unsigned int)command.type);
    }
    return result;
}

static boolean module_vehicle_phototube_control_suction_post(uint32 duty)
{
    vehicle_control_command_t command;
    boolean result;

    command.type = VEHICLE_CONTROL_COMMAND_SET_SUCTION;
    command.enable = FALSE;
    command.speed_mm_s = 0.0f;
    command.theta_rad = 0.0f;
    command.left_speed_mm_s = 0.0f;
    command.right_speed_mm_s = 0.0f;
    command.step_deg = 0.0f;
    command.left_duty = 0;
    command.right_duty = 0;
    command.suction_duty = duty;
    result = vehicle_control_command_post(&command);
    if (result == FALSE)
    {
        tools_printf("{ptline}ctrl_queue_full,%u\r\n", (unsigned int)command.type);
    }
    return result;
}

static boolean module_vehicle_phototube_control_drive_stop_keep_suction_post(uint32 duty)
{
    vehicle_control_command_t command;
    boolean result;

    command.type = VEHICLE_CONTROL_COMMAND_DRIVE_STOP_KEEP_SUCTION;
    command.enable = FALSE;
    command.speed_mm_s = 0.0f;
    command.theta_rad = 0.0f;
    command.left_speed_mm_s = 0.0f;
    command.right_speed_mm_s = 0.0f;
    command.step_deg = 0.0f;
    command.left_duty = 0;
    command.right_duty = 0;
    command.suction_duty = duty;
    result = vehicle_control_command_post(&command);
    if (result == FALSE)
    {
        tools_printf("{ptline}ctrl_queue_full,%u\r\n", (unsigned int)command.type);
    }
    return result;
}

static boolean module_vehicle_phototube_control_heading_target_post(float32 speed_mm_s,
                                                                    float32 theta_rad)
{
    vehicle_control_command_t command;
    boolean result;

    command.type = VEHICLE_CONTROL_COMMAND_SPEED_TEST_HEADING_TARGET;
    command.enable = TRUE;
    command.speed_mm_s = speed_mm_s;
    command.theta_rad = theta_rad;
    command.left_speed_mm_s = speed_mm_s;
    command.right_speed_mm_s = speed_mm_s;
    command.step_deg = 0.0f;
    command.left_duty = 0;
    command.right_duty = 0;
    command.suction_duty = 0u;
    result = vehicle_control_command_post(&command);
    if (result == FALSE)
    {
        tools_printf("{ptline}ctrl_queue_full,%u\r\n", (unsigned int)command.type);
    }
    return result;
}

static boolean module_vehicle_phototube_control_speed_target_post(float32 left_speed_mm_s,
                                                                  float32 right_speed_mm_s)
{
    vehicle_control_command_t command;
    boolean result;

    command.type = VEHICLE_CONTROL_COMMAND_SPEED_TEST_TARGET;
    command.enable = TRUE;
    command.speed_mm_s = 0.0f;
    command.theta_rad = 0.0f;
    command.left_speed_mm_s = left_speed_mm_s;
    command.right_speed_mm_s = right_speed_mm_s;
    command.step_deg = 0.0f;
    command.left_duty = 0;
    command.right_duty = 0;
    command.suction_duty = 0u;
    result = vehicle_control_command_post(&command);
    if (result == FALSE)
    {
        tools_printf("{ptline}ctrl_queue_full,%u\r\n", (unsigned int)command.type);
    }
    return result;
}

static void module_vehicle_phototube_line_control_reset(void)
{
    module_vehicle_phototube_runtime.line_filter_initialized = FALSE;
    module_vehicle_phototube_runtime.line_filtered_error = 0.0f;
    module_vehicle_phototube_runtime.line_last_filtered_error = 0.0f;
    module_vehicle_phototube_runtime.line_correction_mm_s = 0.0f;
    module_vehicle_phototube_runtime.line_left_speed_mm_s = 0.0f;
    module_vehicle_phototube_runtime.line_right_speed_mm_s = 0.0f;
}

static float32 module_vehicle_phototube_line_correction_calculate(float32 error)
{
    float32 filtered_error;
    float32 derivative_error;
    float32 abs_error;
    float32 edge_ratio;
    float32 boosted_kp;
    float32 correction_mm_s;
    float32 center_position =
        ((float32)MODULE_VEHICLE_PHOTOTUBE_COUNT - 1.0f)
        * MODULE_VEHICLE_PHOTOTUBE_LINE_POSITION_STEP
        * 0.5f;

    if (module_vehicle_phototube_runtime.line_filter_initialized == FALSE)
    {
        module_vehicle_phototube_runtime.line_filtered_error = error;
        module_vehicle_phototube_runtime.line_last_filtered_error = error;
        module_vehicle_phototube_runtime.line_filter_initialized = TRUE;
    }
    else
    {
        module_vehicle_phototube_runtime.line_last_filtered_error =
            module_vehicle_phototube_runtime.line_filtered_error;
        module_vehicle_phototube_runtime.line_filtered_error +=
            module_vehicle_phototube_runtime.line_filter_alpha
            * (error - module_vehicle_phototube_runtime.line_filtered_error);
    }

    filtered_error = module_vehicle_phototube_runtime.line_filtered_error;
    derivative_error = filtered_error - module_vehicle_phototube_runtime.line_last_filtered_error;
    abs_error = (filtered_error >= 0.0f) ? filtered_error : -filtered_error;
    edge_ratio = (center_position > 0.0f) ? (abs_error / center_position) : 0.0f;
    edge_ratio = module_vehicle_phototube_clamp_f32(edge_ratio, 0.0f, 1.0f);
    boosted_kp = module_vehicle_phototube_runtime.line_kp_mm_s
                 * (1.0f + (module_vehicle_phototube_runtime.line_edge_boost * edge_ratio));

    correction_mm_s = module_vehicle_phototube_runtime.line_direction
                      * ((boosted_kp * filtered_error)
                         + (module_vehicle_phototube_runtime.line_kd_mm_s * derivative_error));
    correction_mm_s = module_vehicle_phototube_clamp_f32(correction_mm_s,
                                                         -module_vehicle_phototube_runtime.line_diff_limit_mm_s,
                                                         module_vehicle_phototube_runtime.line_diff_limit_mm_s);
    return correction_mm_s;
}

static boolean module_vehicle_phototube_line_calculate(float32* position,
                                                       float32* error,
                                                       uint32* line_sum)
{
    uint32 index;
    uint32 raw_sum = 0u;
    uint32 weighted_sum_q15 = 0u;
    float32 weighted_sum = 0.0f;
    float32 center_position =
        ((float32)MODULE_VEHICLE_PHOTOTUBE_COUNT - 1.0f)
        * MODULE_VEHICLE_PHOTOTUBE_LINE_POSITION_STEP
        * 0.5f;

    if ((position == NULL_PTR) || (error == NULL_PTR) || (line_sum == NULL_PTR))
    {
        return FALSE;
    }

    for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
    {
        uint16 normalized = module_vehicle_phototube_normalized_value_get(index);
        uint32 weight_q15 = (module_vehicle_phototube_runtime.line_weight_enabled != FALSE)
                            ? module_vehicle_phototube_line_weight_q15[index]
                            : MODULE_VEHICLE_PHOTOTUBE_WEIGHT_Q15_BASE;
        uint32 weighted_normalized =
            ((uint32)normalized * weight_q15) / MODULE_VEHICLE_PHOTOTUBE_WEIGHT_Q15_BASE;
        float32 sensor_position = (float32)index * MODULE_VEHICLE_PHOTOTUBE_LINE_POSITION_STEP;

        raw_sum += normalized;
        weighted_sum_q15 += weighted_normalized;
        weighted_sum += sensor_position * (float32)weighted_normalized;
    }

    *line_sum = raw_sum;
    if ((raw_sum < MODULE_VEHICLE_PHOTOTUBE_LINE_MIN_SUM) || (weighted_sum_q15 == 0u))
    {
        *position = module_vehicle_phototube_runtime.line_position;
        *error = module_vehicle_phototube_runtime.line_error;
        return FALSE;
    }

    *position = weighted_sum / (float32)weighted_sum_q15;
    *error = *position - center_position;
    return TRUE;
}

static boolean module_vehicle_phototube_line_observation_calculate(
    module_vehicle_phototube_line_observation_t* observation)
{
    uint32 index;
    uint32 raw_sum = 0u;
    uint32 active_count = 0u;
    uint16 normalized;
    float32 sensor_y_mm;
    float32 min_active_y_mm = 0.0f;
    float32 max_active_y_mm = 0.0f;
    float32 weighted_y_sum = 0.0f;
    float32 confidence;

    if (observation == NULL_PTR)
    {
        return FALSE;
    }

    observation->valid = FALSE;
    observation->line_body_x_mm = module_vehicle_phototube_runtime.correction_sensor_x_mm;
    observation->line_body_y_mm = 0.0f;
    observation->confidence = 0.0f;
    observation->line_sum = 0u;
    observation->active_count = 0u;
    observation->active_width_mm = 0.0f;

    if (module_vehicle_phototube_runtime.calibration.valid == FALSE)
    {
        return FALSE;
    }

    for (index = 0u; index < MODULE_VEHICLE_PHOTOTUBE_COUNT; index++)
    {
        normalized = module_vehicle_phototube_normalized_value_get(index);
        if (normalized < module_vehicle_phototube_runtime.correction_strength_threshold)
        {
            continue;
        }

        sensor_y_mm = (((float32)index - 7.5f)
                       * module_vehicle_phototube_runtime.correction_sensor_spacing_mm
                       * module_vehicle_phototube_runtime.correction_sensor_y_sign);

        raw_sum += normalized;
        if (active_count == 0u)
        {
            min_active_y_mm = sensor_y_mm;
            max_active_y_mm = sensor_y_mm;
        }
        else
        {
            if (sensor_y_mm < min_active_y_mm)
            {
                min_active_y_mm = sensor_y_mm;
            }
            if (sensor_y_mm > max_active_y_mm)
            {
                max_active_y_mm = sensor_y_mm;
            }
        }
        active_count++;
        weighted_y_sum += sensor_y_mm * (float32)normalized;
    }

    observation->line_sum = raw_sum;
    observation->active_count = active_count;
    observation->active_width_mm = max_active_y_mm - min_active_y_mm;
    if (raw_sum < MODULE_VEHICLE_PHOTOTUBE_LINE_MIN_SUM)
    {
        return FALSE;
    }

    confidence = (float32)raw_sum / MODULE_VEHICLE_PHOTOTUBE_CORR_CONFIDENCE_FULL_SUM;
    confidence = module_vehicle_phototube_clamp_f32(confidence, 0.0f, 1.0f);

    observation->line_body_y_mm = weighted_y_sum / (float32)raw_sum;
    observation->confidence = confidence;
    observation->valid = TRUE;
    return TRUE;
}

static float32 module_vehicle_phototube_vector_limit(float32* x_mm, float32* y_mm, float32 limit_mm)
{
    float32 length_mm;
    float32 scale;

    if ((x_mm == NULL_PTR) || (y_mm == NULL_PTR))
    {
        return 0.0f;
    }

    if (limit_mm <= 0.0f)
    {
        *x_mm = 0.0f;
        *y_mm = 0.0f;
        return 0.0f;
    }

    length_mm = sqrtf((*x_mm * *x_mm) + (*y_mm * *y_mm));
    if ((length_mm > limit_mm) && (length_mm > 0.001f))
    {
        scale = limit_mm / length_mm;
        *x_mm *= scale;
        *y_mm *= scale;
        length_mm = limit_mm;
    }

    return length_mm;
}

static float32 module_vehicle_phototube_wrap_pi(float32 angle_rad)
{
    while (angle_rad > MODULE_VEHICLE_PHOTOTUBE_PI)
    {
        angle_rad -= 2.0f * MODULE_VEHICLE_PHOTOTUBE_PI;
    }

    while (angle_rad < -MODULE_VEHICLE_PHOTOTUBE_PI)
    {
        angle_rad += 2.0f * MODULE_VEHICLE_PHOTOTUBE_PI;
    }

    return angle_rad;
}

static boolean module_vehicle_phototube_correction_gate_update(
    const vehicle_path_replay_target_t* replay_target)
{
    const module_vehicle_pose_fusion_observation_t* pose;
    vehicle_path_projection_t sensor_projection;
    float32 sensor_world_x_mm;
    float32 sensor_world_y_mm;
    float32 sensor_index_float;

    if (replay_target == NULL_PTR)
    {
        module_vehicle_phototube_correction_gate_reset();
        return FALSE;
    }

    /* ptzone is uploaded against the turning-center path. Project the actual
     * front-mounted sensor back onto that path before applying the exclusion. */
    pose = module_vehicle_pose_fusion_observation_get();
    sensor_world_x_mm = pose->x_mm
                      + (cosf(pose->theta_rad)
                         * module_vehicle_phototube_runtime.correction_sensor_x_mm);
    sensor_world_y_mm = pose->y_mm
                      + (sinf(pose->theta_rad)
                         * module_vehicle_phototube_runtime.correction_sensor_x_mm);
    sensor_index_float = replay_target->base_index_float;
    if (module_vehicle_path_project_local(sensor_world_x_mm,
                                          sensor_world_y_mm,
                                          replay_target->base_index_float,
                                          module_vehicle_phototube_runtime.correction_window_mm,
                                          &sensor_projection) != FALSE)
    {
        sensor_index_float = sensor_projection.index_float;
    }

    if (module_vehicle_path_phototube_correction_allowed(
            (uint32)(sensor_index_float + 0.5f)) == FALSE)
    {
        module_vehicle_phototube_correction_gate_reset();
        return FALSE;
    }

    /* ptzone remains the only path-segment exclusion. Do not delay valid
     * corrections for geometric straight-section, yaw, or heading checks. */
    module_vehicle_phototube_runtime.correction_gate_stable_count =
        module_vehicle_phototube_runtime.correction_gate_stable_required_count;
    return TRUE;
}

static void module_vehicle_phototube_correction_gate_reset(void)
{
    module_vehicle_phototube_runtime.correction_gate_stable_count = 0u;
    module_vehicle_phototube_runtime.correction_gate_pass = FALSE;
    module_vehicle_phototube_runtime.correction_gate_target_yaw_rate_deg_s = 0.0f;
    module_vehicle_phototube_runtime.correction_gate_heading_delta_deg = 0.0f;
}

static boolean module_vehicle_phototube_command_is(uint8* argument, const char* command)
{
    return (boolean)(strcmp((const char*)argument, command) == 0);
}

static float32 module_vehicle_phototube_command_float_get(uint8* text)
{
    float32 value = 0.0f;
    float32 fraction = 0.1f;
    boolean negative = FALSE;
    boolean decimal = FALSE;
    uint32 index = 0u;

    if (text == NULL_PTR)
    {
        return 0.0f;
    }

    if (text[index] == (uint8)'-')
    {
        negative = TRUE;
        index++;
    }

    while ((((text[index] >= (uint8)'0') && (text[index] <= (uint8)'9'))
            || (text[index] == (uint8)'.')))
    {
        if (text[index] == (uint8)'.')
        {
            decimal = TRUE;
            index++;
            continue;
        }

        if (decimal == FALSE)
        {
            value = (value * 10.0f) + (float32)(text[index] - (uint8)'0');
        }
        else
        {
            value += (float32)(text[index] - (uint8)'0') * fraction;
            fraction *= 0.1f;
        }
        index++;
    }

    return (negative != FALSE) ? -value : value;
}

static uint32 module_vehicle_phototube_command_uint_get(uint8* text)
{
    uint32 value = 0u;
    uint32 index = 0u;

    if (text == NULL_PTR)
    {
        return 0u;
    }

    while ((text[index] >= (uint8)'0') && (text[index] <= (uint8)'9'))
    {
        value = (value * 10u) + (uint32)(text[index] - (uint8)'0');
        index++;
    }

    return value;
}

static uint16 module_vehicle_phototube_normalized_value_get(uint32 index)
{
    const module_vehicle_phototube_calibration_t* calibration =
        &module_vehicle_phototube_runtime.calibration;
    sint32 raw_value;
    sint32 blue_value;
    sint32 white_value;
    sint32 numerator;
    sint32 denominator;
    sint32 normalized;

    if (index >= MODULE_VEHICLE_PHOTOTUBE_COUNT)
    {
        return 0u;
    }

    if (calibration->valid == FALSE)
    {
        return 0u;
    }

    raw_value = (sint32)module_vehicle_phototube_runtime.value[index];
    blue_value = (sint32)calibration->blue_value[index];
    white_value = (sint32)calibration->white_value[index];
    denominator = white_value - blue_value;
    if ((denominator > -((sint32)MODULE_VEHICLE_PHOTOTUBE_MIN_SPAN))
        && (denominator < (sint32)MODULE_VEHICLE_PHOTOTUBE_MIN_SPAN))
    {
        return 0u;
    }

    numerator = raw_value - blue_value;
    normalized = (numerator * (sint32)MODULE_VEHICLE_PHOTOTUBE_NORM_MAX) / denominator;
    if (normalized < 0)
    {
        normalized = 0;
    }
    if (normalized > (sint32)MODULE_VEHICLE_PHOTOTUBE_NORM_MAX)
    {
        normalized = MODULE_VEHICLE_PHOTOTUBE_NORM_MAX;
    }

    return (uint16)normalized;
}

static uint16 module_vehicle_phototube_clamp_u16(sint32 value)
{
    if (value < 0)
    {
        return 0u;
    }
    if (value > (sint32)MODULE_VEHICLE_PHOTOTUBE_ADC_MAX)
    {
        return MODULE_VEHICLE_PHOTOTUBE_ADC_MAX;
    }

    return (uint16)value;
}

static float32 module_vehicle_phototube_clamp_f32(float32 value, float32 min_value, float32 max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}
