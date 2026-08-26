/**
 * @file app_task_vehicle.c
 * @brief Vehicle top-level record and replay task.
 */

#include "../../../../inc/app/app_task/app_task_asynchronous/app_task_vehicle.h"

#include "../../../../config/app/app_task/app_task_asynchronous/app_task_vehicle_cfg.h"
#include "../../../../inc/app/module/module_vehicle_control/module_vehicle_control.h"
#include "../../../../inc/app/module/module_vehicle_encoder/module_vehicle_encoder.h"
#include "../../../../inc/app/module/module_vehicle_esc/module_vehicle_esc.h"
#include "../../../../inc/app/module/module_vehicle_gyro/module_vehicle_gyro.h"
#include "../../../../inc/app/module/module_vehicle_path/module_vehicle_path.h"
#include "../../../../inc/app/module/module_vehicle_phototube/module_vehicle_phototube.h"
#include "../../../../inc/app/module/module_vehicle_pose_fusion/module_vehicle_pose_fusion.h"
#include "../../../../inc/app/service/service_storage/service_storage.h"
#include "../../../../inc/device/device_imu/device_imu.h"
#include "../../../../inc/device/device_key/device_key.h"
#include "../../../../inc/device/device_led/device_led.h"
#include "../../../../inc/middleware/sysTick/sysTick.h"
#include "../../../../inc/middleware/tools/tools_host/tools_host.h"
#include "../../../../inc/middleware/tools/tools_print/tools_print.h"

#include <string.h>

#define APP_TASK_ASYNCHRONOUS_VEHICLE_SERVICE_PERIOD_US (1000u)
#define APP_TASK_ASYNCHRONOUS_VEHICLE_US_TO_S (0.000001f)
#define APP_TASK_ASYNCHRONOUS_VEHICLE_RAD_TO_DEG (57.29577951308232f)
#define APP_TASK_ASYNCHRONOUS_VEHICLE_KEY_SERVICE_DIVIDER ((uint32)1u)
#define APP_TASK_ASYNCHRONOUS_VEHICLE_STARTUP_LEFT_TEST_ENABLE (0u)
#define APP_TASK_ASYNCHRONOUS_VEHICLE_STARTUP_LEFT_TEST_SPEED_MM_S (500.0f)
#define APP_TASK_ASYNCHRONOUS_VEHICLE_STARTUP_LEFT_TEST_TIME_S (1.0f)
#define APP_TASK_ASYNCHRONOUS_VEHICLE_ENCODER_TIMEOUT_S (0.050f)
#define APP_TASK_ASYNCHRONOUS_VEHICLE_IMU_TIMEOUT_S (0.100f)
#define APP_TASK_ASYNCHRONOUS_VEHICLE_REPLAY_MODE_TIMEOUT_S (0.020f)
#define APP_TASK_ASYNCHRONOUS_VEHICLE_PATH_BINARY_VERSION (1u)
#define APP_TASK_ASYNCHRONOUS_VEHICLE_PATH_BINARY_TYPE_BLOCK (1u)
#define APP_TASK_ASYNCHRONOUS_VEHICLE_PATH_BINARY_HEADER_LENGTH (8u)
#define APP_TASK_ASYNCHRONOUS_VEHICLE_PATH_BINARY_PAYLOAD_HEADER_LENGTH (12u)
#define APP_TASK_ASYNCHRONOUS_VEHICLE_PATH_BINARY_POINT_LENGTH (16u)
#define APP_TASK_ASYNCHRONOUS_VEHICLE_PATH_BINARY_CRC_LENGTH (4u)
#define APP_TASK_ASYNCHRONOUS_VEHICLE_PATH_BINARY_POINT_MAX_COUNT (32u)
#define APP_TASK_ASYNCHRONOUS_VEHICLE_SENSOR_WATCHDOG_STEP_MAX_S (0.010f)

typedef enum
{
    APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_IDLE = 0,
    APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_RECORD_DELAY = 1,
    APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_RECORDING = 2,
    APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_PATH_SAVING = 3,
    APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_PATH_SAVED = 4,
    APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_PATH_SAVE_FAILED = 5,
    APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_PATH_LOADING = 6,
    APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_REPLAY_SUCTION_DELAY = 7,
    APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_REPLAY_DRIVE_DELAY = 8,
    APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_REPLAYING = 9,
    APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_REPLAY_FINISH_DRIVE_HOLD = 10,
    APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_REPLAY_FINISH_SUCTION_HOLD = 11,
    APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_REPLAY_FINISHED = 12,
    APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_PATH_LOAD_FAILED = 13,
    APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_ERROR = 14,
    APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_PATH_PLANNING = 15,
    APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_PATH_PLAN_FAILED = 16,
} app_task_asynchronous_vehicle_status_t;

typedef struct
{
    app_task_asynchronous_vehicle_status_t status;
    boolean record_active;
    boolean replay_active;
    boolean path_loaded;
    boolean path_saved;
    boolean mode_blocked;
    uint32 last_point_cnt;
    float32 timer_s;
} app_task_asynchronous_vehicle_runtime_t;

typedef struct
{
    boolean record_active;
} app_task_asynchronous_vehicle_fast_state_t;

typedef struct
{
    boolean armed;
    boolean fault_latched;
    uint32 left_encoder_sample_count;
    uint32 right_encoder_sample_count;
    uint32 imu_raw_update_count[VEHICLE_GYRO_IMU_COUNT];
    float32 left_encoder_stale_s;
    float32 right_encoder_stale_s;
    float32 imu_stale_s[VEHICLE_GYRO_IMU_COUNT];
} app_task_asynchronous_vehicle_sensor_watchdog_t;

typedef struct
{
    boolean active;
    boolean binary_protocol;
    boolean ready_sent;
    uint32 session_id;
    uint32 expected_checksum;
    uint32 expected_point_cnt;
} app_task_asynchronous_vehicle_path_upload_t;

static volatile app_task_asynchronous_vehicle_runtime_t app_task_asynchronous_vehicle_runtime;
static volatile app_task_asynchronous_vehicle_fast_state_t app_task_asynchronous_vehicle_fast_state;
static app_task_asynchronous_vehicle_sensor_watchdog_t app_task_asynchronous_vehicle_sensor_watchdog;
static app_task_asynchronous_vehicle_path_upload_t app_task_asynchronous_vehicle_path_upload;
static boolean app_task_asynchronous_vehicle_imu_startup_indicator_initialized;
static boolean app_task_asynchronous_vehicle_imu_startup_ready;

static void app_task_asynchronous_vehicle_key_process(void);
static void app_task_asynchronous_vehicle_timer_process(float32 dt_s);
static void app_task_asynchronous_vehicle_startup_left_test_process(float32 dt_s);
static void app_task_asynchronous_vehicle_sensor_watchdog_reset(void);
static void app_task_asynchronous_vehicle_sensor_watchdog_arm(void);
static void app_task_asynchronous_vehicle_sensor_watchdog_process(float32 dt_s);
static boolean app_task_asynchronous_vehicle_sensor_watchdog_active(void);
static void app_task_asynchronous_vehicle_sensor_fault_stop(const char* reason);
static void app_task_asynchronous_vehicle_fast_state_set(boolean record_active);
static boolean app_task_asynchronous_vehicle_record_prepare(void);
static boolean app_task_asynchronous_vehicle_record_start(void);
static boolean app_task_asynchronous_vehicle_record_stop_save(void);
static boolean app_task_asynchronous_vehicle_record_suction_set(boolean enable);
static boolean app_task_asynchronous_vehicle_replay_load_start(void);
static boolean app_task_asynchronous_vehicle_replay_drive_start(void);
static boolean app_task_asynchronous_vehicle_replay_path_start(void);
static void app_task_asynchronous_vehicle_stop(void);
static void app_task_asynchronous_vehicle_drive_stop_keep_suction(void);
static void app_task_asynchronous_vehicle_path_diag_print(const char* event);
static void app_task_asynchronous_vehicle_record_full_check(void);
static void app_task_asynchronous_vehicle_record_save_check(void);
static void app_task_asynchronous_vehicle_record_plan_check(void);
static void app_task_asynchronous_vehicle_replay_load_check(void);
static void app_task_asynchronous_vehicle_replay_finish_check(void);
static boolean app_task_asynchronous_vehicle_control_command_post(
    const vehicle_control_command_t* command);
static const app_task_asynchronous_vehicle_cfg_t* app_task_asynchronous_vehicle_cfg_get_local(void);
static boolean app_task_asynchronous_vehicle_command_word_is(const uint8* text, const char* word);
static float32 app_task_asynchronous_vehicle_command_float_get(const uint8* text);
static boolean app_task_asynchronous_vehicle_command_uint32_get(const uint8* text, uint32* value);
static vehicle_path_marker_kind_t app_task_asynchronous_vehicle_path_marker_kind_get(const uint8* text);
static uint16 app_task_asynchronous_vehicle_uint16_le_get(const uint8* data);
static uint32 app_task_asynchronous_vehicle_uint32_le_get(const uint8* data);
static float32 app_task_asynchronous_vehicle_float32_le_get(const uint8* data);
static uint32 app_task_asynchronous_vehicle_crc32(const uint8* data, uint16 length);
static void app_task_asynchronous_vehicle_path_upload_reset(void);
static void app_task_asynchronous_vehicle_path_upload_ready_check(void);
static void app_task_asynchronous_vehicle_path_ahead_print(void);
static void app_task_asynchronous_vehicle_path_speed_print(void);
static void app_task_asynchronous_vehicle_imu_startup_indicator_process(void);

void app_task_asynchronous_vehicle_init(void)
{
    app_task_asynchronous_vehicle_runtime.status = APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_IDLE;
    app_task_asynchronous_vehicle_runtime.record_active = FALSE;
    app_task_asynchronous_vehicle_runtime.replay_active = FALSE;
    app_task_asynchronous_vehicle_runtime.path_loaded = FALSE;
    app_task_asynchronous_vehicle_runtime.path_saved = FALSE;
    app_task_asynchronous_vehicle_runtime.mode_blocked = FALSE;
    app_task_asynchronous_vehicle_runtime.last_point_cnt = 0u;
    app_task_asynchronous_vehicle_runtime.timer_s = 0.0f;
    app_task_asynchronous_vehicle_fast_state_set(FALSE);
    app_task_asynchronous_vehicle_sensor_watchdog_reset();
    app_task_asynchronous_vehicle_path_upload_reset();
    app_task_asynchronous_vehicle_imu_startup_indicator_initialized = FALSE;
    app_task_asynchronous_vehicle_imu_startup_ready = FALSE;
    device_key_clearAllFlag();
    app_task_asynchronous_vehicle_stop();
    app_task_asynchronous_vehicle_imu_startup_indicator_process();
    device_led_set_state(DEVICE_LED_2, DEVICE_LED_DARK);
}

void app_task_asynchronous_vehicle_run(void)
{
    app_task_asynchronous_vehicle_fast_run();
    app_task_asynchronous_vehicle_service_run();
}

void app_task_asynchronous_vehicle_fast_run(void)
{
    if (app_task_asynchronous_vehicle_fast_state.record_active != FALSE)
    {
        (void)module_vehicle_path_update();
    }
}

void app_task_asynchronous_vehicle_service_run(void)
{
    static boolean initialized = FALSE;
    static uint64 last_tick = 0u;
    static uint32 key_service_divider = 0u;
    uint64 current_tick;
    uint64 elapsed_tick;
    uint64 elapsed_us;
    uint32 period_tick;
    float32 dt_s;

    current_tick = sysTick_getTick(SYSTICK1);
    period_tick = sysTick_getTicksFromMicroseconds(SYSTICK1,
                                                   APP_TASK_ASYNCHRONOUS_VEHICLE_SERVICE_PERIOD_US);
    if (period_tick == 0u)
    {
        period_tick = 1u;
    }

    if (initialized == FALSE)
    {
        initialized = TRUE;
        last_tick = current_tick;
        return;
    }

    elapsed_tick = current_tick - last_tick;
    if (elapsed_tick < period_tick)
    {
        return;
    }

    last_tick = current_tick;
    elapsed_us = sysTick_ticksToMicroseconds(SYSTICK1, elapsed_tick);
    dt_s = (float32)elapsed_us * APP_TASK_ASYNCHRONOUS_VEHICLE_US_TO_S;

    tools_host_process();
    app_task_asynchronous_vehicle_imu_startup_indicator_process();
    module_vehicle_path_run();
    app_task_asynchronous_vehicle_path_upload_ready_check();
    app_task_asynchronous_vehicle_startup_left_test_process(dt_s);
    app_task_asynchronous_vehicle_timer_process(dt_s);
    app_task_asynchronous_vehicle_sensor_watchdog_process(dt_s);
    app_task_asynchronous_vehicle_record_save_check();
    app_task_asynchronous_vehicle_record_plan_check();
    app_task_asynchronous_vehicle_replay_load_check();
    module_vehicle_encoder_angle_print_process();
    module_vehicle_gyro_print_process();
    vehicle_control_speed_test_print_process();
    vehicle_control_angle_test_print_process();
    vehicle_control_replay_debug_print_process();
    app_task_asynchronous_vehicle_record_full_check();
    app_task_asynchronous_vehicle_replay_finish_check();

    key_service_divider++;
    if (key_service_divider >= APP_TASK_ASYNCHRONOUS_VEHICLE_KEY_SERVICE_DIVIDER)
    {
        key_service_divider = 0u;
        app_task_asynchronous_vehicle_key_process();
    }
}

void app_task_asynchronous_vehicle_path_command(uint8 argc, uint8* argv[])
{
    boolean result = FALSE;

    if (argc < 2u)
    {
        tools_printf("{pathcmd}usage\r\n");
        return;
    }

    if ((app_task_asynchronous_vehicle_command_word_is(argv[1], "rec") != FALSE)
        || (app_task_asynchronous_vehicle_command_word_is(argv[1], "record") != FALSE))
    {
        if ((argc >= 3u)
            && ((app_task_asynchronous_vehicle_command_word_is(argv[2], "stop") != FALSE)
                || (app_task_asynchronous_vehicle_command_word_is(argv[2], "end") != FALSE)
                || (app_task_asynchronous_vehicle_command_word_is(argv[2], "off") != FALSE)))
        {
            result = app_task_asynchronous_vehicle_record_stop_save();
            tools_printf("{pathcmd}record_stop,%u,%u\r\n",
                         (unsigned int)result,
                         (unsigned int)app_task_asynchronous_vehicle_runtime.last_point_cnt);
            return;
        }

        result = app_task_asynchronous_vehicle_record_prepare();
        tools_printf("{pathcmd}record_prepare,%u\r\n", (unsigned int)result);
        return;
    }

    if ((app_task_asynchronous_vehicle_command_word_is(argv[1], "replay") != FALSE)
        || (app_task_asynchronous_vehicle_command_word_is(argv[1], "play") != FALSE))
    {
        if ((argc >= 3u)
            && ((app_task_asynchronous_vehicle_command_word_is(argv[2], "stop") != FALSE)
                || (app_task_asynchronous_vehicle_command_word_is(argv[2], "end") != FALSE)
                || (app_task_asynchronous_vehicle_command_word_is(argv[2], "off") != FALSE)))
        {
            app_task_asynchronous_vehicle_stop();
            app_task_asynchronous_vehicle_runtime.status = APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_REPLAY_FINISHED;
            device_led_set_state(DEVICE_LED_2, DEVICE_LED_DARK);
            tools_printf("{pathcmd}replay_stop,%u\r\n",
                         (unsigned int)app_task_asynchronous_vehicle_runtime.last_point_cnt);
            return;
        }

        result = app_task_asynchronous_vehicle_replay_load_start();
        tools_printf("{pathcmd}replay_load,%u\r\n", (unsigned int)result);
        return;
    }

    if (app_task_asynchronous_vehicle_command_word_is(argv[1], "upload") != FALSE)
    {
        uint32 received_cnt = module_vehicle_path_import_received_count_get();
        uint32 expected_cnt = module_vehicle_path_import_expected_count_get();

        if ((argc < 3u)
            || (app_task_asynchronous_vehicle_command_word_is(argv[2], "status") != FALSE)
            || (app_task_asynchronous_vehicle_command_word_is(argv[2], "show") != FALSE))
        {
            tools_printf("{pathupload}status,%u,%u,%u,%u,%u\r\n",
                         (unsigned int)(((module_vehicle_path_import_active_get() != FALSE)
                                      || (module_vehicle_path_import_preparing_get() != FALSE)) ? 1u : 0u),
                         (unsigned int)received_cnt,
                         (unsigned int)expected_cnt,
                         (unsigned int)module_vehicle_path_flash_point_capacity_get(),
                         (unsigned int)((module_vehicle_path_save_is_busy() != FALSE) ? 1u : 0u));
            return;
        }

        if ((app_task_asynchronous_vehicle_command_word_is(argv[2], "begin") != FALSE)
            && (argc >= 4u))
        {
            float32 count_value = app_task_asynchronous_vehicle_command_float_get(argv[3]);
            uint32 point_cnt = (count_value >= 0.0f) ? (uint32)count_value : 0u;
            result = module_vehicle_path_import_begin(point_cnt);
            app_task_asynchronous_vehicle_path_upload_reset();
            if (result != FALSE)
            {
                app_task_asynchronous_vehicle_path_upload.active = TRUE;
                app_task_asynchronous_vehicle_path_upload.binary_protocol = FALSE;
                app_task_asynchronous_vehicle_path_upload.expected_point_cnt = point_cnt;
                tools_printf("{pathupload}preparing,0,%u,%u\r\n",
                             (unsigned int)point_cnt,
                             (unsigned int)module_vehicle_path_flash_point_capacity_get());
            }
            else
            {
                tools_printf("{pathupload}begin,0,%u,%u\r\n",
                             (unsigned int)point_cnt,
                             (unsigned int)module_vehicle_path_flash_point_capacity_get());
            }
            return;
        }

        if ((app_task_asynchronous_vehicle_command_word_is(argv[2], "begin2") != FALSE)
            && (argc >= 6u))
        {
            uint32 point_cnt = 0u;
            uint32 session_id = 0u;
            uint32 expected_checksum = 0u;
            boolean arguments_valid =
                app_task_asynchronous_vehicle_command_uint32_get(argv[3], &point_cnt);
            arguments_valid = (boolean)(arguments_valid
                && app_task_asynchronous_vehicle_command_uint32_get(argv[4], &session_id));
            arguments_valid = (boolean)(arguments_valid
                && app_task_asynchronous_vehicle_command_uint32_get(argv[5], &expected_checksum));

            if ((app_task_asynchronous_vehicle_path_upload.active != FALSE)
                && (module_vehicle_path_import_active_get() == FALSE)
                && (module_vehicle_path_import_preparing_get() == FALSE))
            {
                app_task_asynchronous_vehicle_path_upload_reset();
            }

            if ((arguments_valid != FALSE)
                && (app_task_asynchronous_vehicle_path_upload.active != FALSE)
                && (app_task_asynchronous_vehicle_path_upload.session_id == session_id)
                && (app_task_asynchronous_vehicle_path_upload.expected_point_cnt == point_cnt)
                && (app_task_asynchronous_vehicle_path_upload.expected_checksum == expected_checksum))
            {
                result = TRUE;
            }
            else if ((arguments_valid != FALSE)
                     && (app_task_asynchronous_vehicle_path_upload.active == FALSE))
            {
                result = module_vehicle_path_import_begin(point_cnt);
                if (result != FALSE)
                {
                    app_task_asynchronous_vehicle_path_upload.active = TRUE;
                    app_task_asynchronous_vehicle_path_upload.binary_protocol = TRUE;
                    app_task_asynchronous_vehicle_path_upload.ready_sent = FALSE;
                    app_task_asynchronous_vehicle_path_upload.session_id = session_id;
                    app_task_asynchronous_vehicle_path_upload.expected_checksum = expected_checksum;
                    app_task_asynchronous_vehicle_path_upload.expected_point_cnt = point_cnt;
                }
            }

            if (result == FALSE)
            {
                tools_printf("{pathupload}begin2,0,%u,%u,%u\r\n",
                             (unsigned int)session_id,
                             (unsigned int)point_cnt,
                             (unsigned int)module_vehicle_path_flash_point_capacity_get());
            }
            else if (module_vehicle_path_import_active_get() != FALSE)
            {
                app_task_asynchronous_vehicle_path_upload.ready_sent = TRUE;
                tools_printf("{pathupload}begin2,1,%u,%u,%u\r\n",
                             (unsigned int)session_id,
                             (unsigned int)point_cnt,
                             (unsigned int)module_vehicle_path_flash_point_capacity_get());
            }
            else
            {
                tools_printf("{pathupload}preparing,%u,%u,%u\r\n",
                             (unsigned int)session_id,
                             (unsigned int)point_cnt,
                             (unsigned int)module_vehicle_path_flash_point_capacity_get());
            }
            return;
        }

        if ((app_task_asynchronous_vehicle_command_word_is(argv[2], "point") != FALSE)
            && (argc >= 8u))
        {
            vehicle_path_point_t point;
            float32 index_value = app_task_asynchronous_vehicle_command_float_get(argv[3]);
            uint32 index_cnt = (index_value >= 0.0f) ? (uint32)index_value : 0xFFFFFFFFu;
            point.x_mm = app_task_asynchronous_vehicle_command_float_get(argv[4]);
            point.y_mm = app_task_asynchronous_vehicle_command_float_get(argv[5]);
            point.theta_rad = app_task_asynchronous_vehicle_command_float_get(argv[6]);
            point.speed_mm_s = app_task_asynchronous_vehicle_command_float_get(argv[7]);
            result = module_vehicle_path_import_point(index_cnt, &point);
            tools_printf("{pathupload}point,%u,%u,%u\r\n",
                         (unsigned int)result,
                         (unsigned int)index_cnt,
                         (unsigned int)module_vehicle_path_import_received_count_get());
            return;
        }

        if ((app_task_asynchronous_vehicle_command_word_is(argv[2], "marker") != FALSE)
            && (argc >= 5u))
        {
            float32 index_value = app_task_asynchronous_vehicle_command_float_get(argv[3]);
            uint32 index_cnt = (index_value >= 0.0f) ? (uint32)index_value : 0xFFFFFFFFu;
            vehicle_path_marker_kind_t kind =
                app_task_asynchronous_vehicle_path_marker_kind_get(argv[4]);
            const char* kind_name = (kind == VEHICLE_PATH_MARKER_KIND_TURN_OUT) ? "out" : "in";

            result = module_vehicle_path_import_marker(kind, index_cnt);
            tools_printf("{pathupload}marker,%u,%u,%s\r\n",
                         (unsigned int)result,
                         (unsigned int)index_cnt,
                         kind_name);
            return;
        }

        if ((app_task_asynchronous_vehicle_command_word_is(argv[2], "ptzone") != FALSE)
            && (argc >= 5u))
        {
            float32 start_value = app_task_asynchronous_vehicle_command_float_get(argv[3]);
            float32 end_value = app_task_asynchronous_vehicle_command_float_get(argv[4]);
            uint32 start_index_cnt = (start_value >= 0.0f) ? (uint32)start_value : 0xFFFFFFFFu;
            uint32 end_index_cnt = (end_value >= 0.0f) ? (uint32)end_value : 0xFFFFFFFFu;

            result = module_vehicle_path_import_phototube_zone(start_index_cnt, end_index_cnt);
            tools_printf("{pathupload}ptzone,%u,%u,%u\r\n",
                         (unsigned int)result,
                         (unsigned int)start_index_cnt,
                         (unsigned int)end_index_cnt);
            return;
        }

        if (app_task_asynchronous_vehicle_command_word_is(argv[2], "commit") != FALSE)
        {
            result = module_vehicle_path_import_commit();
            tools_printf("{pathupload}commit,%u,%u,%u\r\n",
                         (unsigned int)result,
                         (unsigned int)module_vehicle_path_import_received_count_get(),
                         (unsigned int)module_vehicle_path_import_expected_count_get());
            return;
        }

        if ((app_task_asynchronous_vehicle_command_word_is(argv[2], "abort") != FALSE)
            || (app_task_asynchronous_vehicle_command_word_is(argv[2], "cancel") != FALSE))
        {
            module_vehicle_path_import_abort();
            app_task_asynchronous_vehicle_path_upload_reset();
            tools_printf("{pathupload}abort\r\n");
            return;
        }

        if ((app_task_asynchronous_vehicle_command_word_is(argv[2], "commit2") != FALSE)
            && (argc >= 5u))
        {
            uint32 session_id = 0u;
            uint32 expected_checksum = 0u;
            uint32 actual_checksum = module_vehicle_path_import_checksum_get();
            uint32 actual_count = module_vehicle_path_import_received_count_get();
            uint32 expected_count = module_vehicle_path_import_expected_count_get();
            boolean arguments_valid =
                app_task_asynchronous_vehicle_command_uint32_get(argv[3], &session_id);
            arguments_valid = (boolean)(arguments_valid
                && app_task_asynchronous_vehicle_command_uint32_get(argv[4], &expected_checksum));

            if ((arguments_valid != FALSE)
                && (app_task_asynchronous_vehicle_path_upload.active != FALSE)
                && (app_task_asynchronous_vehicle_path_upload.session_id == session_id)
                && (app_task_asynchronous_vehicle_path_upload.expected_checksum == expected_checksum)
                && (actual_count == expected_count)
                && (actual_count == app_task_asynchronous_vehicle_path_upload.expected_point_cnt)
                && (actual_checksum == expected_checksum))
            {
                result = module_vehicle_path_import_commit();
            }

            tools_printf("{pathupload}commit2,%u,%u,%u,%u,%u\r\n",
                         (unsigned int)result,
                         (unsigned int)session_id,
                         (unsigned int)actual_count,
                         (unsigned int)expected_count,
                         (unsigned int)actual_checksum);
            return;
        }

        tools_printf("{pathupload}usage:begin <count>|begin2 <count> <session> <checksum>|point <i> <x> <y> <theta> <speed>|marker <i> in|out|ptzone <start> <end>|commit|commit2 <session> <checksum>|abort|status\r\n");
        return;
    }

    if ((app_task_asynchronous_vehicle_command_word_is(argv[1], "load") != FALSE)
        || (app_task_asynchronous_vehicle_command_word_is(argv[1], "use") != FALSE))
    {
        if ((argc >= 3u)
            && ((app_task_asynchronous_vehicle_command_word_is(argv[2], "planned") != FALSE)
                || (app_task_asynchronous_vehicle_command_word_is(argv[2], "plan") != FALSE)))
        {
            result = module_vehicle_path_load_planned();
            tools_printf("{pathcmd}load_planned,%u\r\n", (unsigned int)result);
            return;
        }

        if ((argc >= 3u)
            && ((app_task_asynchronous_vehicle_command_word_is(argv[2], "raw") != FALSE)
                || (app_task_asynchronous_vehicle_command_word_is(argv[2], "record") != FALSE)))
        {
            result = module_vehicle_path_load_raw();
            tools_printf("{pathcmd}load_raw,%u\r\n", (unsigned int)result);
            return;
        }

        tools_printf("{pathcmd}load_usage\r\n");
        return;
    }

    if (app_task_asynchronous_vehicle_command_word_is(argv[1], "plan") != FALSE)
    {
        result = module_vehicle_path_plan();
        tools_printf("{pathcmd}plan,%u\r\n", (unsigned int)result);
        return;
    }

    if (app_task_asynchronous_vehicle_command_word_is(argv[1], "speed") != FALSE)
    {
        if ((argc < 3u)
            || (app_task_asynchronous_vehicle_command_word_is(argv[2], "show") != FALSE)
            || (app_task_asynchronous_vehicle_command_word_is(argv[2], "status") != FALSE))
        {
            app_task_asynchronous_vehicle_path_speed_print();
            return;
        }

        if ((app_task_asynchronous_vehicle_command_word_is(argv[2], "fixed") != FALSE)
            || (app_task_asynchronous_vehicle_command_word_is(argv[2], "off") != FALSE))
        {
            (void)module_vehicle_path_replay_planned_speed_enable(FALSE);
            app_task_asynchronous_vehicle_path_speed_print();
            return;
        }

        if ((app_task_asynchronous_vehicle_command_word_is(argv[2], "planned") != FALSE)
            || (app_task_asynchronous_vehicle_command_word_is(argv[2], "plan") != FALSE)
            || (app_task_asynchronous_vehicle_command_word_is(argv[2], "on") != FALSE))
        {
            result = module_vehicle_path_replay_planned_speed_enable(TRUE);
            if (result == FALSE)
            {
                tools_printf("{pathspeed}planned_unavailable\r\n");
            }
            app_task_asynchronous_vehicle_path_speed_print();
            return;
        }

        tools_printf("{pathspeed}usage:fixed|planned|show\r\n");
        return;
    }

    if (app_task_asynchronous_vehicle_command_word_is(argv[1], "slip") != FALSE)
    {
        if ((argc >= 3u)
            && ((app_task_asynchronous_vehicle_command_word_is(argv[2], "on") != FALSE)
                || (app_task_asynchronous_vehicle_command_word_is(argv[2], "enable") != FALSE)))
        {
            module_vehicle_pose_fusion_replay_correction_enable_set(TRUE);
        }
        else if ((argc >= 3u)
                 && ((app_task_asynchronous_vehicle_command_word_is(argv[2], "off") != FALSE)
                     || (app_task_asynchronous_vehicle_command_word_is(argv[2], "disable") != FALSE)))
        {
            module_vehicle_pose_fusion_replay_correction_enable_set(FALSE);
        }
        else if ((argc >= 4u)
                 && (app_task_asynchronous_vehicle_command_word_is(argv[2], "full") != FALSE
                     || app_task_asynchronous_vehicle_command_word_is(argv[2], "weak") != FALSE
                     || app_task_asynchronous_vehicle_command_word_is(argv[2], "weakgain") != FALSE
                     || app_task_asynchronous_vehicle_command_word_is(argv[2], "gain") != FALSE
                     || app_task_asynchronous_vehicle_command_word_is(argv[2], "release") != FALSE
                     || app_task_asynchronous_vehicle_command_word_is(argv[2], "wheelbase") != FALSE
                     || app_task_asynchronous_vehicle_command_word_is(argv[2], "minturn") != FALSE
                     || app_task_asynchronous_vehicle_command_word_is(argv[2], "maxturn") != FALSE
                     || app_task_asynchronous_vehicle_command_word_is(argv[2], "maxreplay") != FALSE))
        {
            uint8 parameter = MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_FULL_RATIO;
            if (app_task_asynchronous_vehicle_command_word_is(argv[2], "weak") != FALSE) parameter = MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_WEAK_RATIO;
            else if (app_task_asynchronous_vehicle_command_word_is(argv[2], "weakgain") != FALSE) parameter = MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_WEAK_GAIN;
            else if (app_task_asynchronous_vehicle_command_word_is(argv[2], "gain") != FALSE) parameter = MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_GAIN;
            else if (app_task_asynchronous_vehicle_command_word_is(argv[2], "release") != FALSE) parameter = MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_RELEASE;
            else if (app_task_asynchronous_vehicle_command_word_is(argv[2], "wheelbase") != FALSE) parameter = MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_WHEEL_BASE;
            else if (app_task_asynchronous_vehicle_command_word_is(argv[2], "minturn") != FALSE) parameter = MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_MIN_TURN;
            else if (app_task_asynchronous_vehicle_command_word_is(argv[2], "maxturn") != FALSE) parameter = MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_MAX_TURN;
            else if (app_task_asynchronous_vehicle_command_word_is(argv[2], "maxreplay") != FALSE) parameter = MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_MAX_REPLAY;
            if (module_vehicle_pose_fusion_slip_parameter_set(
                    parameter, app_task_asynchronous_vehicle_command_float_get(argv[3])) == FALSE)
            {
                tools_printf("{vslipcfg}invalid,%s,%.3f\r\n", argv[2],
                             (double)app_task_asynchronous_vehicle_command_float_get(argv[3]));
                return;
            }
        }
        else if ((argc >= 3u)
                 && (app_task_asynchronous_vehicle_command_word_is(argv[2], "status") == FALSE)
                 && (app_task_asynchronous_vehicle_command_word_is(argv[2], "show") == FALSE))
        {
            tools_printf("{vslipcfg}usage:on|off|status|full <0-1>|weak <0-1>|weakgain <0-1>|gain <0-2>|release <0-1>|wheelbase <mm>|minturn <rad>|maxturn <mm>|maxreplay <mm>\r\n");
            return;
        }

        tools_printf("{vslipcfg}enable,%u,active,%u,full,%.3f,weak,%.3f,weakgain,%.3f,gain,%.3f,release,%.5f,wheelbase,%.1f,minturn,%.3f,maxturn,%.1f,maxreplay,%.1f\r\n",
                     (unsigned int)((module_vehicle_pose_fusion_replay_correction_enable_get() != FALSE) ? 1u : 0u),
                     (unsigned int)((module_vehicle_pose_fusion_replay_correction_get()->active != FALSE) ? 1u : 0u),
                     (double)module_vehicle_pose_fusion_slip_parameter_get(MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_FULL_RATIO),
                     (double)module_vehicle_pose_fusion_slip_parameter_get(MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_WEAK_RATIO),
                     (double)module_vehicle_pose_fusion_slip_parameter_get(MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_WEAK_GAIN),
                     (double)module_vehicle_pose_fusion_slip_parameter_get(MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_GAIN),
                     (double)module_vehicle_pose_fusion_slip_parameter_get(MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_RELEASE),
                     (double)module_vehicle_pose_fusion_slip_parameter_get(MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_WHEEL_BASE),
                     (double)module_vehicle_pose_fusion_slip_parameter_get(MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_MIN_TURN),
                     (double)module_vehicle_pose_fusion_slip_parameter_get(MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_MAX_TURN),
                     (double)module_vehicle_pose_fusion_slip_parameter_get(MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_MAX_REPLAY));
        return;
    }

    if ((app_task_asynchronous_vehicle_command_word_is(argv[1], "speedlookahead") != FALSE)
        || (app_task_asynchronous_vehicle_command_word_is(argv[1], "speedpreview") != FALSE)
        || (app_task_asynchronous_vehicle_command_word_is(argv[1], "preview") != FALSE))
    {
        if ((argc < 3u)
            || (app_task_asynchronous_vehicle_command_word_is(argv[2], "show") != FALSE)
            || (app_task_asynchronous_vehicle_command_word_is(argv[2], "status") != FALSE))
        {
            app_task_asynchronous_vehicle_path_speed_print();
            return;
        }

        if ((app_task_asynchronous_vehicle_command_word_is(argv[2], "reset") != FALSE)
            || (app_task_asynchronous_vehicle_command_word_is(argv[2], "default") != FALSE))
        {
            module_vehicle_path_replay_speed_preview_distance_set(
                VEHICLE_PATH_DEFAULT_REPLAY_SPEED_PREVIEW_DISTANCE_MM);
        }
        else
        {
            module_vehicle_path_replay_speed_preview_distance_set(
                app_task_asynchronous_vehicle_command_float_get(argv[2]));
        }
        app_task_asynchronous_vehicle_path_speed_print();
        return;
    }

    if ((app_task_asynchronous_vehicle_command_word_is(argv[1], "mark") != FALSE)
        || (app_task_asynchronous_vehicle_command_word_is(argv[1], "marker") != FALSE))
    {
        vehicle_path_marker_kind_t kind;

        if ((argc < 3u)
            || (app_task_asynchronous_vehicle_command_word_is(argv[2], "show") != FALSE)
            || (app_task_asynchronous_vehicle_command_word_is(argv[2], "list") != FALSE)
            || (app_task_asynchronous_vehicle_command_word_is(argv[2], "status") != FALSE))
        {
            module_vehicle_path_marker_print();
            return;
        }

        if ((app_task_asynchronous_vehicle_command_word_is(argv[2], "on") != FALSE)
            || (app_task_asynchronous_vehicle_command_word_is(argv[2], "enable") != FALSE))
        {
            module_vehicle_path_marker_enable(TRUE);
            return;
        }

        if ((app_task_asynchronous_vehicle_command_word_is(argv[2], "off") != FALSE)
            || (app_task_asynchronous_vehicle_command_word_is(argv[2], "disable") != FALSE))
        {
            module_vehicle_path_marker_enable(FALSE);
            return;
        }

        if (app_task_asynchronous_vehicle_command_word_is(argv[2], "clear") != FALSE)
        {
            module_vehicle_path_marker_clear();
            return;
        }

        if ((app_task_asynchronous_vehicle_command_word_is(argv[2], "add") != FALSE)
            && (argc >= 4u))
        {
            kind = app_task_asynchronous_vehicle_path_marker_kind_get(argv[3]);
            result = module_vehicle_path_marker_add(kind);
            tools_printf("{pathcmd}mark_add,%u\r\n", (unsigned int)result);
            return;
        }

        if ((app_task_asynchronous_vehicle_command_word_is(argv[2], "set") != FALSE)
            && (argc >= 5u))
        {
            float32 index_value;

            kind = app_task_asynchronous_vehicle_path_marker_kind_get(argv[3]);
            index_value = app_task_asynchronous_vehicle_command_float_get(argv[4]);
            result = (index_value >= 0.0f)
                   ? module_vehicle_path_marker_set(kind, (uint32)index_value)
                   : FALSE;
            tools_printf("{pathcmd}mark_set,%u\r\n", (unsigned int)result);
            return;
        }

        kind = app_task_asynchronous_vehicle_path_marker_kind_get(argv[2]);
        if (kind != VEHICLE_PATH_MARKER_KIND_NONE)
        {
            result = module_vehicle_path_marker_add(kind);
            tools_printf("{pathcmd}mark_add,%u\r\n", (unsigned int)result);
            return;
        }

        tools_printf("{vmark}usage:add in|out,set in|out <index>,show,clear,on,off\r\n");
        return;
    }

    if ((app_task_asynchronous_vehicle_command_word_is(argv[1], "ahead") != FALSE)
        || (app_task_asynchronous_vehicle_command_word_is(argv[1], "look") != FALSE)
        || (app_task_asynchronous_vehicle_command_word_is(argv[1], "lookahead") != FALSE)
        || (app_task_asynchronous_vehicle_command_word_is(argv[1], "tangent") != FALSE)
        || (app_task_asynchronous_vehicle_command_word_is(argv[1], "tan") != FALSE))
    {
        if ((argc < 3u)
            || (app_task_asynchronous_vehicle_command_word_is(argv[2], "show") != FALSE)
            || (app_task_asynchronous_vehicle_command_word_is(argv[2], "status") != FALSE)
            || (app_task_asynchronous_vehicle_command_word_is(argv[2], "s") != FALSE))
        {
            app_task_asynchronous_vehicle_path_ahead_print();
            return;
        }

        if ((app_task_asynchronous_vehicle_command_word_is(argv[2], "reset") != FALSE)
            || (app_task_asynchronous_vehicle_command_word_is(argv[2], "default") != FALSE))
        {
            module_vehicle_path_replay_ahead_reset();
            app_task_asynchronous_vehicle_path_ahead_print();
            return;
        }

        if ((app_task_asynchronous_vehicle_command_word_is(argv[1], "ahead") != FALSE)
            && (argc >= 4u))
        {
            module_vehicle_path_replay_ahead_set(
                app_task_asynchronous_vehicle_command_float_get(argv[2]),
                app_task_asynchronous_vehicle_command_float_get(argv[3]));
            app_task_asynchronous_vehicle_path_ahead_print();
            return;
        }

        if ((app_task_asynchronous_vehicle_command_word_is(argv[1], "look") != FALSE)
            || (app_task_asynchronous_vehicle_command_word_is(argv[1], "lookahead") != FALSE))
        {
            module_vehicle_path_replay_lookahead_set(
                app_task_asynchronous_vehicle_command_float_get(argv[2]));
            app_task_asynchronous_vehicle_path_ahead_print();
            return;
        }

        if ((app_task_asynchronous_vehicle_command_word_is(argv[1], "tangent") != FALSE)
            || (app_task_asynchronous_vehicle_command_word_is(argv[1], "tan") != FALSE))
        {
            module_vehicle_path_replay_tangent_set(
                app_task_asynchronous_vehicle_command_float_get(argv[2]));
            app_task_asynchronous_vehicle_path_ahead_print();
            return;
        }

        tools_printf("{pathahead}usage\r\n");
        return;
    }

    if ((app_task_asynchronous_vehicle_command_word_is(argv[1], "dump") != FALSE)
        || (app_task_asynchronous_vehicle_command_word_is(argv[1], "print") != FALSE))
    {
        if ((argc >= 3u)
            && ((app_task_asynchronous_vehicle_command_word_is(argv[2], "raw") != FALSE)
                || (app_task_asynchronous_vehicle_command_word_is(argv[2], "record") != FALSE)))
        {
            result = module_vehicle_path_dump_raw();
        }
        else if ((argc >= 3u)
                 && ((app_task_asynchronous_vehicle_command_word_is(argv[2], "planned") != FALSE)
                     || (app_task_asynchronous_vehicle_command_word_is(argv[2], "plan") != FALSE)))
        {
            result = module_vehicle_path_dump_planned();
        }
        else
        {
            result = module_vehicle_path_dump();
        }

        tools_printf("{pathcmd}dump,%u\r\n", (unsigned int)result);
        return;
    }

    if ((app_task_asynchronous_vehicle_command_word_is(argv[1], "stop") != FALSE)
        || (app_task_asynchronous_vehicle_command_word_is(argv[1], "off") != FALSE))
    {
        app_task_asynchronous_vehicle_stop();
        device_led_set_state(DEVICE_LED_2, DEVICE_LED_DARK);
        tools_printf("{pathcmd}stop\r\n");
        return;
    }

    tools_printf("{pathcmd}unknown\r\n");
}

void app_task_asynchronous_vehicle_path_binary_frame(const uint8* frame, uint16 length)
{
    uint32 session_id = 0u;
    uint32 start_index = 0u;
    uint32 expected_crc = 0u;
    uint32 actual_crc = 0u;
    uint16 payload_length = 0u;
    uint16 expected_length = 0u;
    uint8 point_count = 0u;
    uint8 point_index;
    boolean result = FALSE;

    if ((frame != NULL_PTR) && (length >= 24u))
    {
        payload_length = app_task_asynchronous_vehicle_uint16_le_get(&frame[6]);
        expected_length = (uint16)(APP_TASK_ASYNCHRONOUS_VEHICLE_PATH_BINARY_HEADER_LENGTH
                          + payload_length
                          + APP_TASK_ASYNCHRONOUS_VEHICLE_PATH_BINARY_CRC_LENGTH);
        session_id = app_task_asynchronous_vehicle_uint32_le_get(&frame[8]);
        start_index = app_task_asynchronous_vehicle_uint32_le_get(&frame[12]);
        point_count = frame[16];
        expected_crc = app_task_asynchronous_vehicle_uint32_le_get(&frame[length - 4u]);
        actual_crc = app_task_asynchronous_vehicle_crc32(frame, (uint16)(length - 4u));

        if ((frame[0] == 0x00u) && (frame[1] == 0xA5u)
            && (frame[2] == 0x5Au) && (frame[3] == 0xC3u)
            && (frame[4] == APP_TASK_ASYNCHRONOUS_VEHICLE_PATH_BINARY_VERSION)
            && (frame[5] == APP_TASK_ASYNCHRONOUS_VEHICLE_PATH_BINARY_TYPE_BLOCK)
            && (length == expected_length)
            && (point_count > 0u)
            && (point_count <= APP_TASK_ASYNCHRONOUS_VEHICLE_PATH_BINARY_POINT_MAX_COUNT)
            && (payload_length == (uint16)(APP_TASK_ASYNCHRONOUS_VEHICLE_PATH_BINARY_PAYLOAD_HEADER_LENGTH
                                 + ((uint16)point_count
                                    * APP_TASK_ASYNCHRONOUS_VEHICLE_PATH_BINARY_POINT_LENGTH)))
            && (expected_crc == actual_crc)
            && (app_task_asynchronous_vehicle_path_upload.active != FALSE)
            && (app_task_asynchronous_vehicle_path_upload.session_id == session_id)
            && (module_vehicle_path_import_active_get() != FALSE)
            && (start_index <= module_vehicle_path_import_received_count_get())
            && ((start_index + (uint32)point_count)
                <= app_task_asynchronous_vehicle_path_upload.expected_point_cnt))
        {
            result = TRUE;
            for (point_index = 0u; point_index < point_count; point_index++)
            {
                uint16 point_offset = (uint16)(20u
                    + ((uint16)point_index * APP_TASK_ASYNCHRONOUS_VEHICLE_PATH_BINARY_POINT_LENGTH));
                vehicle_path_point_t point;

                point.x_mm = app_task_asynchronous_vehicle_float32_le_get(&frame[point_offset]);
                point.y_mm = app_task_asynchronous_vehicle_float32_le_get(&frame[point_offset + 4u]);
                point.theta_rad = app_task_asynchronous_vehicle_float32_le_get(&frame[point_offset + 8u]);
                point.speed_mm_s = app_task_asynchronous_vehicle_float32_le_get(&frame[point_offset + 12u]);
                if (module_vehicle_path_import_point(start_index + (uint32)point_index, &point) == FALSE)
                {
                    result = FALSE;
                    break;
                }
            }
        }
    }

    tools_printf("{pathupload}block,%u,%u,%u,%u,%u,%u\r\n",
                 (unsigned int)result,
                 (unsigned int)session_id,
                 (unsigned int)start_index,
                 (unsigned int)point_count,
                 (unsigned int)module_vehicle_path_import_received_count_get(),
                 (unsigned int)actual_crc);
}

void app_task_asynchronous_vehicle_stop_command(uint8 argc, uint8* argv[])
{
    (void)argc;
    (void)argv;

    app_task_asynchronous_vehicle_stop();
    device_led_set_state(DEVICE_LED_2, DEVICE_LED_DARK);
    tools_printf("{vstop}ok\r\n");
}

static void app_task_asynchronous_vehicle_key_process(void)
{
    static boolean record_key_long_handled = FALSE;
    static boolean replay_key_long_handled = FALSE;
    device_key_flag_t record_key_flag;
    device_key_flag_t replay_key_flag;

    device_key_scanner();

    record_key_flag = device_key_getFlag(DEVICE_KEY_1);
    replay_key_flag = device_key_getFlag(DEVICE_KEY_2);

    if (device_key_getState(DEVICE_KEY_1) == DEVICE_KEY_NOTPRESS)
    {
        record_key_long_handled = FALSE;
    }
    if (device_key_getState(DEVICE_KEY_2) == DEVICE_KEY_NOTPRESS)
    {
        replay_key_long_handled = FALSE;
    }

    if (record_key_flag == DEVICE_KEY_LONG_PRESS_FLAG)
    {
        if (record_key_long_handled == FALSE)
        {
            if (app_task_asynchronous_vehicle_runtime.status == APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_RECORDING)
            {
                boolean result = module_vehicle_path_marker_add(VEHICLE_PATH_MARKER_KIND_TURN_IN);

                tools_printf("{key}mark_in,%u\r\n", (unsigned int)result);
            }
            else if (app_task_asynchronous_vehicle_runtime.status != APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_RECORD_DELAY)
            {
                boolean result = app_task_asynchronous_vehicle_record_prepare();

                tools_printf("{key}record_prepare,%u,%u\r\n",
                             (unsigned int)result,
                             (unsigned int)app_task_asynchronous_vehicle_runtime.status);
            }

            record_key_long_handled = TRUE;
        }

        device_key_clearFlag(DEVICE_KEY_1);
    }
    else if (record_key_flag == DEVICE_KEY_SHORT_PRESS_FLAG)
    {
        if ((app_task_asynchronous_vehicle_runtime.status == APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_RECORD_DELAY)
            || (app_task_asynchronous_vehicle_runtime.status == APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_RECORDING))
        {
            boolean result = app_task_asynchronous_vehicle_record_stop_save();

            tools_printf("{key}record_stop,%u,%u\r\n",
                         (unsigned int)result,
                         (unsigned int)app_task_asynchronous_vehicle_runtime.last_point_cnt);
        }
        else
        {
            boolean result = app_task_asynchronous_vehicle_record_prepare();

            tools_printf("{key}record_prepare,%u,%u\r\n",
                         (unsigned int)result,
                         (unsigned int)app_task_asynchronous_vehicle_runtime.status);
        }

        device_key_clearFlag(DEVICE_KEY_1);
    }

    if (replay_key_flag == DEVICE_KEY_LONG_PRESS_FLAG)
    {
        if ((replay_key_long_handled == FALSE)
            && (app_task_asynchronous_vehicle_runtime.status == APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_RECORDING))
        {
            boolean result = module_vehicle_path_marker_add(VEHICLE_PATH_MARKER_KIND_TURN_OUT);

            tools_printf("{key}mark_out,%u\r\n", (unsigned int)result);
            replay_key_long_handled = TRUE;
        }

        device_key_clearFlag(DEVICE_KEY_2);
    }
    else if (replay_key_flag == DEVICE_KEY_SHORT_PRESS_FLAG)
    {
        if ((app_task_asynchronous_vehicle_runtime.status == APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_IDLE)
            || (app_task_asynchronous_vehicle_runtime.status == APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_PATH_SAVED)
            || (app_task_asynchronous_vehicle_runtime.status == APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_REPLAY_FINISHED))
        {
            (void)app_task_asynchronous_vehicle_replay_load_start();
        }

        device_key_clearFlag(DEVICE_KEY_2);
    }
}

static void app_task_asynchronous_vehicle_timer_process(float32 dt_s)
{
    const app_task_asynchronous_vehicle_cfg_t* cfg = app_task_asynchronous_vehicle_cfg_get_local();

    switch (app_task_asynchronous_vehicle_runtime.status)
    {
        case APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_RECORD_DELAY:
            app_task_asynchronous_vehicle_runtime.timer_s += dt_s;
            if (app_task_asynchronous_vehicle_runtime.timer_s >= cfg->record_start_delay_s)
            {
                (void)app_task_asynchronous_vehicle_record_start();
            }
            break;

        case APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_REPLAY_SUCTION_DELAY:
            app_task_asynchronous_vehicle_runtime.timer_s += dt_s;
            if (app_task_asynchronous_vehicle_runtime.timer_s >= cfg->replay_suction_delay_s)
            {
                vehicle_control_command_t command;

                command.type = VEHICLE_CONTROL_COMMAND_SET_SUCTION;
                command.enable = FALSE;
                command.speed_mm_s = 0.0f;
                command.theta_rad = 0.0f;
                command.left_speed_mm_s = 0.0f;
                command.right_speed_mm_s = 0.0f;
                command.step_deg = 0.0f;
                command.left_duty = 0;
                command.right_duty = 0;
                if (vehicle_control_auto_suction_enabled_get() != FALSE)
                {
                    command.suction_duty = cfg->suction_run_duty;
                    (void)app_task_asynchronous_vehicle_control_command_post(&command);
                    tools_printf("{pathinfo}replay_suction_on,%u\r\n", (unsigned int)cfg->suction_run_duty);
                }
                else
                {
                    command.suction_duty = 0u;
                    (void)app_task_asynchronous_vehicle_control_command_post(&command);
                    tools_printf("{pathinfo}replay_suction_off\r\n");
                }
                app_task_asynchronous_vehicle_runtime.timer_s = 0.0f;
                app_task_asynchronous_vehicle_runtime.status =
                    APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_REPLAY_DRIVE_DELAY;
            }
            break;

        case APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_REPLAY_DRIVE_DELAY:
            app_task_asynchronous_vehicle_runtime.timer_s += dt_s;
            if (app_task_asynchronous_vehicle_runtime.timer_s >= cfg->replay_drive_delay_s)
            {
                (void)app_task_asynchronous_vehicle_replay_drive_start();
            }
            break;

        case APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_REPLAY_FINISH_DRIVE_HOLD:
            app_task_asynchronous_vehicle_runtime.timer_s += dt_s;
            if (app_task_asynchronous_vehicle_runtime.timer_s >= cfg->replay_finish_drive_hold_s)
            {
                app_task_asynchronous_vehicle_path_diag_print("replay_drive_off");
                app_task_asynchronous_vehicle_drive_stop_keep_suction();
                app_task_asynchronous_vehicle_runtime.timer_s = 0.0f;
                tools_printf("{pathinfo}replay_drive_off\r\n");
                if (vehicle_control_auto_suction_enabled_get() != FALSE)
                {
                    app_task_asynchronous_vehicle_runtime.status =
                        APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_REPLAY_FINISH_SUCTION_HOLD;
                }
                else
                {
                    app_task_asynchronous_vehicle_stop();
                    app_task_asynchronous_vehicle_runtime.status = APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_REPLAY_FINISHED;
                    device_led_set_state(DEVICE_LED_2, DEVICE_LED_DARK);
                    tools_printf("{pathinfo}replay_finished\r\n");
                }
            }
            break;

        case APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_REPLAYING:
            if (app_task_asynchronous_vehicle_runtime.replay_active != FALSE)
            {
                const vehicle_control_state_t* control_state = vehicle_control_state_get();

                if (control_state->mode == VEHICLE_CONTROL_MODE_REPLAY)
                {
                    app_task_asynchronous_vehicle_runtime.timer_s = 0.0f;
                }
                else
                {
                    app_task_asynchronous_vehicle_runtime.timer_s += dt_s;
                    if (app_task_asynchronous_vehicle_runtime.timer_s
                        >= APP_TASK_ASYNCHRONOUS_VEHICLE_REPLAY_MODE_TIMEOUT_S)
                    {
                        app_task_asynchronous_vehicle_path_diag_print("replay_mode_lost");
                        app_task_asynchronous_vehicle_stop();
                        app_task_asynchronous_vehicle_runtime.status =
                            APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_ERROR;
                        app_task_asynchronous_vehicle_runtime.mode_blocked = TRUE;
                        device_led_set_state(DEVICE_LED_2, DEVICE_LED_DARK);
                        tools_printf("{pathinfo}replay_mode_lost\r\n");
                    }
                }
            }
            break;

        case APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_REPLAY_FINISH_SUCTION_HOLD:
            app_task_asynchronous_vehicle_runtime.timer_s += dt_s;
            if (app_task_asynchronous_vehicle_runtime.timer_s >= cfg->replay_finish_suction_hold_s)
            {
                app_task_asynchronous_vehicle_stop();
                app_task_asynchronous_vehicle_runtime.status = APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_REPLAY_FINISHED;
                device_led_set_state(DEVICE_LED_2, DEVICE_LED_DARK);
                tools_printf("{pathinfo}replay_finished\r\n");
            }
            break;

        default:
            break;
    }
}

static void app_task_asynchronous_vehicle_startup_left_test_process(float32 dt_s)
{
#if (APP_TASK_ASYNCHRONOUS_VEHICLE_STARTUP_LEFT_TEST_ENABLE != 0u)
    static boolean initialized = FALSE;
    static boolean running = FALSE;
    static boolean finished = FALSE;
    static float32 timer_s = 0.0f;

    if (finished != FALSE)
    {
        return;
    }

    if (app_task_asynchronous_vehicle_runtime.status != APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_IDLE)
    {
        if (running != FALSE)
        {
            vehicle_control_command_t command;

            command.type = VEHICLE_CONTROL_COMMAND_SPEED_TEST_ENABLE;
            command.enable = FALSE;
            command.speed_mm_s = 0.0f;
            command.theta_rad = 0.0f;
            command.left_speed_mm_s = 0.0f;
            command.right_speed_mm_s = 0.0f;
            command.step_deg = 0.0f;
            command.left_duty = 0;
            command.right_duty = 0;
            command.suction_duty = 0u;
            (void)app_task_asynchronous_vehicle_control_command_post(&command);
            running = FALSE;
        }

        finished = TRUE;
        tools_printf("{vstart}cancel\r\n");
        return;
    }

    if (initialized == FALSE)
    {
        vehicle_control_command_t command;

        initialized = TRUE;
        running = TRUE;
        timer_s = 0.0f;
        command.type = VEHICLE_CONTROL_COMMAND_SPEED_TEST_TARGET;
        command.enable = TRUE;
        command.speed_mm_s = 0.0f;
        command.theta_rad = 0.0f;
        command.left_speed_mm_s = APP_TASK_ASYNCHRONOUS_VEHICLE_STARTUP_LEFT_TEST_SPEED_MM_S;
        command.right_speed_mm_s = 0.0f;
        command.step_deg = 0.0f;
        command.left_duty = 0;
        command.right_duty = 0;
        command.suction_duty = 0u;
        (void)app_task_asynchronous_vehicle_control_command_post(&command);
        tools_printf("{vstart}left,%.3f,%.3f\r\n",
                     (double)APP_TASK_ASYNCHRONOUS_VEHICLE_STARTUP_LEFT_TEST_SPEED_MM_S,
                     (double)APP_TASK_ASYNCHRONOUS_VEHICLE_STARTUP_LEFT_TEST_TIME_S);
        return;
    }

    if (running == FALSE)
    {
        return;
    }

    timer_s += dt_s;
    if (timer_s >= APP_TASK_ASYNCHRONOUS_VEHICLE_STARTUP_LEFT_TEST_TIME_S)
    {
        vehicle_control_command_t command;

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
        (void)app_task_asynchronous_vehicle_control_command_post(&command);
        running = FALSE;
        finished = TRUE;
        tools_printf("{vstart}stop\r\n");
    }
#else
    (void)dt_s;
#endif
}

static void app_task_asynchronous_vehicle_sensor_watchdog_reset(void)
{
    uint32 index;

    app_task_asynchronous_vehicle_sensor_watchdog.armed = FALSE;
    app_task_asynchronous_vehicle_sensor_watchdog.left_encoder_sample_count = 0u;
    app_task_asynchronous_vehicle_sensor_watchdog.right_encoder_sample_count = 0u;
    app_task_asynchronous_vehicle_sensor_watchdog.left_encoder_stale_s = 0.0f;
    app_task_asynchronous_vehicle_sensor_watchdog.right_encoder_stale_s = 0.0f;
    for (index = 0u; index < VEHICLE_GYRO_IMU_COUNT; index++)
    {
        app_task_asynchronous_vehicle_sensor_watchdog.imu_raw_update_count[index] = 0u;
        app_task_asynchronous_vehicle_sensor_watchdog.imu_stale_s[index] = 0.0f;
    }
}

static void app_task_asynchronous_vehicle_sensor_watchdog_arm(void)
{
    const module_vehicle_encoder_observation_t* encoder_observation =
        module_vehicle_encoder_observation_get();
    const vehicle_gyro_cfg_t* gyro_cfg = vehicle_gyro_cfg_get();
    const device_imu_runtime_t* imu_runtime = device_imu_runtime_table_get();
    uint32 index;

    app_task_asynchronous_vehicle_sensor_watchdog.left_encoder_sample_count =
        encoder_observation->left_raw_sample_count;
    app_task_asynchronous_vehicle_sensor_watchdog.right_encoder_sample_count =
        encoder_observation->right_raw_sample_count;
    app_task_asynchronous_vehicle_sensor_watchdog.left_encoder_stale_s = 0.0f;
    app_task_asynchronous_vehicle_sensor_watchdog.right_encoder_stale_s = 0.0f;
    for (index = 0u; index < VEHICLE_GYRO_IMU_COUNT; index++)
    {
        device_imu_id_t imu_id = gyro_cfg->imu[index].imu_id;

        app_task_asynchronous_vehicle_sensor_watchdog.imu_raw_update_count[index] =
            imu_runtime[imu_id].imu_raw_update_count;
        app_task_asynchronous_vehicle_sensor_watchdog.imu_stale_s[index] = 0.0f;
    }
    app_task_asynchronous_vehicle_sensor_watchdog.armed = TRUE;
}

static void app_task_asynchronous_vehicle_sensor_watchdog_process(float32 dt_s)
{
    const module_vehicle_encoder_observation_t* encoder_observation;
    const vehicle_gyro_cfg_t* gyro_cfg;
    const device_imu_runtime_t* imu_runtime;
    uint32 index;
    float32 watchdog_step_s;

    if (app_task_asynchronous_vehicle_sensor_watchdog_active() == FALSE)
    {
        app_task_asynchronous_vehicle_sensor_watchdog_reset();
        app_task_asynchronous_vehicle_sensor_watchdog.fault_latched = FALSE;
        return;
    }

    if ((dt_s <= 0.0f) || (app_task_asynchronous_vehicle_sensor_watchdog.fault_latched != FALSE))
    {
        return;
    }

    watchdog_step_s = (dt_s > APP_TASK_ASYNCHRONOUS_VEHICLE_SENSOR_WATCHDOG_STEP_MAX_S)
                    ? APP_TASK_ASYNCHRONOUS_VEHICLE_SENSOR_WATCHDOG_STEP_MAX_S
                    : dt_s;

    if (app_task_asynchronous_vehicle_sensor_watchdog.armed == FALSE)
    {
        app_task_asynchronous_vehicle_sensor_watchdog_arm();
        return;
    }

    encoder_observation = module_vehicle_encoder_observation_get();
    if (encoder_observation->left_raw_sample_count
        != app_task_asynchronous_vehicle_sensor_watchdog.left_encoder_sample_count)
    {
        app_task_asynchronous_vehicle_sensor_watchdog.left_encoder_sample_count =
            encoder_observation->left_raw_sample_count;
        app_task_asynchronous_vehicle_sensor_watchdog.left_encoder_stale_s = 0.0f;
    }
    else
    {
        app_task_asynchronous_vehicle_sensor_watchdog.left_encoder_stale_s += watchdog_step_s;
        if (app_task_asynchronous_vehicle_sensor_watchdog.left_encoder_stale_s
            >= APP_TASK_ASYNCHRONOUS_VEHICLE_ENCODER_TIMEOUT_S)
        {
            app_task_asynchronous_vehicle_sensor_fault_stop("encoder_left");
            return;
        }
    }

    if (encoder_observation->right_raw_sample_count
        != app_task_asynchronous_vehicle_sensor_watchdog.right_encoder_sample_count)
    {
        app_task_asynchronous_vehicle_sensor_watchdog.right_encoder_sample_count =
            encoder_observation->right_raw_sample_count;
        app_task_asynchronous_vehicle_sensor_watchdog.right_encoder_stale_s = 0.0f;
    }
    else
    {
        app_task_asynchronous_vehicle_sensor_watchdog.right_encoder_stale_s += watchdog_step_s;
        if (app_task_asynchronous_vehicle_sensor_watchdog.right_encoder_stale_s
            >= APP_TASK_ASYNCHRONOUS_VEHICLE_ENCODER_TIMEOUT_S)
        {
            app_task_asynchronous_vehicle_sensor_fault_stop("encoder_right");
            return;
        }
    }

    gyro_cfg = vehicle_gyro_cfg_get();
    imu_runtime = device_imu_runtime_table_get();
    for (index = 0u; index < VEHICLE_GYRO_IMU_COUNT; index++)
    {
        device_imu_id_t imu_id = gyro_cfg->imu[index].imu_id;

        if (imu_runtime[imu_id].imu_raw_update_count
            != app_task_asynchronous_vehicle_sensor_watchdog.imu_raw_update_count[index])
        {
            app_task_asynchronous_vehicle_sensor_watchdog.imu_raw_update_count[index] =
                imu_runtime[imu_id].imu_raw_update_count;
            app_task_asynchronous_vehicle_sensor_watchdog.imu_stale_s[index] = 0.0f;
        }
        else
        {
            app_task_asynchronous_vehicle_sensor_watchdog.imu_stale_s[index] += watchdog_step_s;
            if (app_task_asynchronous_vehicle_sensor_watchdog.imu_stale_s[index]
                >= APP_TASK_ASYNCHRONOUS_VEHICLE_IMU_TIMEOUT_S)
            {
                app_task_asynchronous_vehicle_sensor_fault_stop((index == 0u) ? "imu_1" : "imu_2");
                return;
            }
        }
    }
}

static boolean app_task_asynchronous_vehicle_sensor_watchdog_active(void)
{
    const vehicle_control_state_t* control_state = vehicle_control_state_get();

    if (app_task_asynchronous_vehicle_runtime.record_active != FALSE)
    {
        return TRUE;
    }

    if (app_task_asynchronous_vehicle_runtime.replay_active != FALSE)
    {
        return TRUE;
    }

    if ((app_task_asynchronous_vehicle_runtime.status == APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_REPLAY_SUCTION_DELAY)
        || (app_task_asynchronous_vehicle_runtime.status == APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_REPLAY_DRIVE_DELAY)
        || (app_task_asynchronous_vehicle_runtime.status == APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_REPLAYING))
    {
        return TRUE;
    }

    if ((control_state->enabled != FALSE) || (vehicle_control_speed_test_is_enabled() != FALSE))
    {
        return TRUE;
    }

    return FALSE;
}

static void app_task_asynchronous_vehicle_sensor_fault_stop(const char* reason)
{
    if (reason == NULL_PTR)
    {
        reason = "unknown";
    }

    app_task_asynchronous_vehicle_sensor_watchdog.fault_latched = TRUE;
    app_task_asynchronous_vehicle_path_diag_print("sensor_fault");
    app_task_asynchronous_vehicle_stop();
    app_task_asynchronous_vehicle_sensor_watchdog.armed = FALSE;
    device_led_set_state(DEVICE_LED_2, DEVICE_LED_DARK);
    tools_printf("{safety}sensor_lost,%s\r\n", reason);
}

static void app_task_asynchronous_vehicle_fast_state_set(boolean record_active)
{
    app_task_asynchronous_vehicle_fast_state.record_active = record_active;
}

static void app_task_asynchronous_vehicle_imu_startup_indicator_process(void)
{
    boolean startup_ready = module_vehicle_gyro_startup_ready_get();

    if ((app_task_asynchronous_vehicle_imu_startup_indicator_initialized != FALSE)
        && (app_task_asynchronous_vehicle_imu_startup_ready == startup_ready))
    {
        return;
    }

    app_task_asynchronous_vehicle_imu_startup_indicator_initialized = TRUE;
    app_task_asynchronous_vehicle_imu_startup_ready = startup_ready;
    device_led_set_state(DEVICE_LED_1,
                         (startup_ready != FALSE) ? DEVICE_LED_DARK : DEVICE_LED_LIGHT);
}

static boolean app_task_asynchronous_vehicle_record_prepare(void)
{
    float32 current_heading = module_vehicle_pose_fusion_heading_get();
    const module_vehicle_phototube_calibration_t* phototube_calibration =
        module_vehicle_phototube_calibration_get();

    if (module_vehicle_path_job_is_busy() != FALSE)
    {
        tools_printf("{pathinfo}path_job_busy\r\n");
        return FALSE;
    }

    app_task_asynchronous_vehicle_stop();
    if (app_task_asynchronous_vehicle_record_suction_set(TRUE) == FALSE)
    {
        return FALSE;
    }

    module_vehicle_path_clear();
    module_vehicle_encoder_reset_baseline();
    module_vehicle_pose_fusion_reset(0.0f, 0.0f, current_heading);
    (void)service_storage_erase(SERVICE_STORAGE_1);
    if ((phototube_calibration != NULL_PTR) && (phototube_calibration->valid != FALSE))
    {
        if (module_vehicle_phototube_calibration_flash_save() != FALSE)
        {
            tools_printf("{ptcal}preserve,ok\r\n");
        }
        else
        {
            tools_printf("{ptcal}preserve,fail\r\n");
        }
    }

    app_task_asynchronous_vehicle_runtime.record_active = FALSE;
    app_task_asynchronous_vehicle_runtime.replay_active = FALSE;
    app_task_asynchronous_vehicle_runtime.path_loaded = FALSE;
    app_task_asynchronous_vehicle_runtime.path_saved = FALSE;
    app_task_asynchronous_vehicle_runtime.mode_blocked = FALSE;
    app_task_asynchronous_vehicle_runtime.last_point_cnt = 0u;
    app_task_asynchronous_vehicle_runtime.timer_s = 0.0f;
    app_task_asynchronous_vehicle_runtime.status = APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_RECORD_DELAY;
    app_task_asynchronous_vehicle_fast_state_set(FALSE);
    tools_printf("{pathinfo}record_prepare\r\n");
    return TRUE;
}

static boolean app_task_asynchronous_vehicle_record_start(void)
{
    if (module_vehicle_path_start() == FALSE)
    {
        app_task_asynchronous_vehicle_stop();
        app_task_asynchronous_vehicle_runtime.status = APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_ERROR;
        app_task_asynchronous_vehicle_runtime.mode_blocked = TRUE;
        tools_printf("{pathinfo}record_start_failed\r\n");
        return FALSE;
    }

    app_task_asynchronous_vehicle_runtime.status = APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_RECORDING;
    app_task_asynchronous_vehicle_runtime.record_active = TRUE;
    app_task_asynchronous_vehicle_runtime.replay_active = FALSE;
    app_task_asynchronous_vehicle_runtime.path_loaded = FALSE;
    app_task_asynchronous_vehicle_runtime.path_saved = FALSE;
    app_task_asynchronous_vehicle_runtime.mode_blocked = FALSE;
    app_task_asynchronous_vehicle_runtime.last_point_cnt = module_vehicle_path_state_get()->point_cnt;
    app_task_asynchronous_vehicle_fast_state_set(TRUE);
    tools_printf("{pathinfo}record_start\r\n");
    return TRUE;
}

static boolean app_task_asynchronous_vehicle_record_stop_save(void)
{
    boolean result = FALSE;

    if (app_task_asynchronous_vehicle_runtime.status == APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_RECORD_DELAY)
    {
        app_task_asynchronous_vehicle_stop();
        tools_printf("{pathinfo}record_cancel\r\n");
        return FALSE;
    }

    app_task_asynchronous_vehicle_fast_state_set(FALSE);

    if (app_task_asynchronous_vehicle_runtime.record_active != FALSE)
    {
        module_vehicle_path_stop();
    }

    (void)app_task_asynchronous_vehicle_record_suction_set(FALSE);

    app_task_asynchronous_vehicle_runtime.record_active = FALSE;
    app_task_asynchronous_vehicle_runtime.last_point_cnt = module_vehicle_path_state_get()->point_cnt;
    app_task_asynchronous_vehicle_path_diag_print("record_stop");

    if (app_task_asynchronous_vehicle_runtime.last_point_cnt >= vehicle_path_cfg_get()->min_valid_point_cnt)
    {
        result = module_vehicle_path_save_close_request();
    }

    app_task_asynchronous_vehicle_runtime.path_saved = result;
    app_task_asynchronous_vehicle_runtime.mode_blocked = (result == FALSE) ? TRUE : FALSE;
    app_task_asynchronous_vehicle_runtime.status = (result != FALSE)
                                               ? APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_PATH_SAVING
                                               : APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_PATH_SAVE_FAILED;
    tools_printf("{pathinfo}record_stop,%u\r\n",
                 (unsigned int)app_task_asynchronous_vehicle_runtime.last_point_cnt);
    return result;
}

static boolean app_task_asynchronous_vehicle_record_suction_set(boolean enable)
{
    const app_task_asynchronous_vehicle_cfg_t* cfg = app_task_asynchronous_vehicle_cfg_get_local();
    vehicle_control_command_t command;
    uint32 duty = cfg->suction_stop_duty;

    if ((enable != FALSE) && (vehicle_control_auto_suction_enabled_get() != FALSE))
    {
        duty = cfg->record_suction_duty;
    }

    command.type = VEHICLE_CONTROL_COMMAND_SET_SUCTION;
    command.enable = enable;
    command.speed_mm_s = 0.0f;
    command.theta_rad = 0.0f;
    command.left_speed_mm_s = 0.0f;
    command.right_speed_mm_s = 0.0f;
    command.step_deg = 0.0f;
    command.left_duty = 0;
    command.right_duty = 0;
    command.suction_duty = duty;

    if (app_task_asynchronous_vehicle_control_command_post(&command) == FALSE)
    {
        tools_printf("{pathinfo}record_suction_failed\r\n");
        return FALSE;
    }

    if ((enable != FALSE) && (duty > 0u))
    {
        tools_printf("{pathinfo}record_suction_on,%u\r\n", (unsigned int)duty);
    }
    else
    {
        tools_printf("{pathinfo}record_suction_off\r\n");
    }

    return TRUE;
}

static boolean app_task_asynchronous_vehicle_replay_load_start(void)
{
    const vehicle_path_state_t* path_state;

    if (module_vehicle_path_job_is_busy() != FALSE)
    {
        tools_printf("{pathinfo}path_job_busy\r\n");
        return FALSE;
    }

    app_task_asynchronous_vehicle_stop();
    device_led_set_state(DEVICE_LED_2, DEVICE_LED_LIGHT);

    path_state = module_vehicle_path_state_get();
    if (path_state->point_cnt < vehicle_path_cfg_get()->min_valid_point_cnt)
    {
        if (module_vehicle_path_load() == FALSE)
        {
            app_task_asynchronous_vehicle_runtime.path_loaded = FALSE;
            app_task_asynchronous_vehicle_runtime.mode_blocked = TRUE;
            app_task_asynchronous_vehicle_runtime.status = APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_PATH_LOAD_FAILED;
            device_led_set_state(DEVICE_LED_2, DEVICE_LED_DARK);
            tools_printf("{pathinfo}replay_load_failed\r\n");
            return FALSE;
        }

        path_state = module_vehicle_path_state_get();
    }

    app_task_asynchronous_vehicle_runtime.record_active = FALSE;
    app_task_asynchronous_vehicle_runtime.replay_active = FALSE;
    app_task_asynchronous_vehicle_runtime.path_loaded = FALSE;
    app_task_asynchronous_vehicle_runtime.mode_blocked = FALSE;
    app_task_asynchronous_vehicle_fast_state_set(FALSE);
    app_task_asynchronous_vehicle_runtime.timer_s = 0.0f;
    app_task_asynchronous_vehicle_runtime.status = APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_PATH_LOADING;
    tools_printf("{pathinfo}replay_load,%u\r\n", (unsigned int)path_state->point_cnt);
    return TRUE;
}

static boolean app_task_asynchronous_vehicle_replay_drive_start(void)
{
    const app_task_asynchronous_vehicle_cfg_t* cfg = app_task_asynchronous_vehicle_cfg_get_local();
    vehicle_control_command_t command;

    command.type = VEHICLE_CONTROL_COMMAND_REPLAY_START;
    command.enable = TRUE;
    command.speed_mm_s = 0.0f;
    command.theta_rad = 0.0f;
    command.left_speed_mm_s = 0.0f;
    command.right_speed_mm_s = 0.0f;
    command.step_deg = 0.0f;
    command.left_duty = 0;
    command.right_duty = 0;
    command.suction_duty = (vehicle_control_auto_suction_enabled_get() != FALSE)
                           ? cfg->suction_run_duty
                           : 0u;
    if (app_task_asynchronous_vehicle_control_command_post(&command) == FALSE)
    {
        return FALSE;
    }

    return app_task_asynchronous_vehicle_replay_path_start();
}

static boolean app_task_asynchronous_vehicle_replay_path_start(void)
{
    const vehicle_path_state_t* path_state = module_vehicle_path_state_get();

    if (path_state->point_cnt < vehicle_path_cfg_get()->min_valid_point_cnt)
    {
        app_task_asynchronous_vehicle_stop();
        app_task_asynchronous_vehicle_runtime.status = APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_ERROR;
        app_task_asynchronous_vehicle_runtime.mode_blocked = TRUE;
        tools_printf("{pathinfo}replay_start_failed\r\n");
        return FALSE;
    }

    app_task_asynchronous_vehicle_path_diag_print("replay_start");
    app_task_asynchronous_vehicle_runtime.status = APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_REPLAYING;
    app_task_asynchronous_vehicle_runtime.replay_active = TRUE;
    app_task_asynchronous_vehicle_runtime.path_loaded = TRUE;
    app_task_asynchronous_vehicle_runtime.mode_blocked = FALSE;
    app_task_asynchronous_vehicle_runtime.timer_s = 0.0f;
    tools_printf("{pathinfo}replay_drive_start\r\n");
    return TRUE;
}

static void app_task_asynchronous_vehicle_stop(void)
{
    const app_task_asynchronous_vehicle_cfg_t* cfg = app_task_asynchronous_vehicle_cfg_get_local();
    vehicle_control_command_t command;

    module_vehicle_phototube_line_stop();
    module_vehicle_path_stop();

    command.type = VEHICLE_CONTROL_COMMAND_STOP_ALL;
    command.enable = FALSE;
    command.speed_mm_s = cfg->drive_stop_speed_mm_s;
    command.theta_rad = 0.0f;
    command.left_speed_mm_s = 0.0f;
    command.right_speed_mm_s = 0.0f;
    command.step_deg = 0.0f;
    command.left_duty = 0;
    command.right_duty = 0;
    command.suction_duty = cfg->suction_stop_duty;
    (void)app_task_asynchronous_vehicle_control_command_post(&command);

    /* Fail-safe immediate drive stop if the CPU1 encoder frame is no longer running. */
    (void)module_vehicle_esc_stop(VEHICLE_ESC_ROLE_LEFT_DRIVE);
    (void)module_vehicle_esc_stop(VEHICLE_ESC_ROLE_RIGHT_DRIVE);
    (void)module_vehicle_esc_stop(VEHICLE_ESC_ROLE_SUCTION);
    app_task_asynchronous_vehicle_runtime.record_active = FALSE;
    app_task_asynchronous_vehicle_runtime.replay_active = FALSE;
    app_task_asynchronous_vehicle_fast_state_set(FALSE);

    if ((app_task_asynchronous_vehicle_runtime.status != APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_PATH_SAVING)
        && (app_task_asynchronous_vehicle_runtime.status != APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_PATH_SAVED)
        && (app_task_asynchronous_vehicle_runtime.status != APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_PATH_SAVE_FAILED)
        && (app_task_asynchronous_vehicle_runtime.status != APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_PATH_LOAD_FAILED)
        && (app_task_asynchronous_vehicle_runtime.status != APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_ERROR))
    {
        app_task_asynchronous_vehicle_runtime.status = APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_IDLE;
    }
}

static void app_task_asynchronous_vehicle_drive_stop_keep_suction(void)
{
    const app_task_asynchronous_vehicle_cfg_t* cfg = app_task_asynchronous_vehicle_cfg_get_local();
    vehicle_control_command_t command;

    command.type = VEHICLE_CONTROL_COMMAND_DRIVE_STOP_KEEP_SUCTION;
    command.enable = FALSE;
    command.speed_mm_s = 0.0f;
    command.theta_rad = 0.0f;
    command.left_speed_mm_s = 0.0f;
    command.right_speed_mm_s = 0.0f;
    command.step_deg = 0.0f;
    command.left_duty = 0;
    command.right_duty = 0;
    command.suction_duty = (vehicle_control_auto_suction_enabled_get() != FALSE)
                           ? cfg->suction_run_duty
                           : 0u;
    (void)app_task_asynchronous_vehicle_control_command_post(&command);
    app_task_asynchronous_vehicle_runtime.replay_active = FALSE;
    app_task_asynchronous_vehicle_fast_state_set(FALSE);
}

static void app_task_asynchronous_vehicle_path_diag_print(const char* event)
{
    const vehicle_path_state_t* path_state = module_vehicle_path_state_get();
    const module_vehicle_encoder_observation_t* encoder_observation =
        module_vehicle_encoder_observation_get();
    const module_vehicle_pose_fusion_observation_t* pose_observation =
        module_vehicle_pose_fusion_observation_get();

    if (event == NULL_PTR)
    {
        event = "unknown";
    }

    tools_printf("{pathdiag}%s,%u,%.3f,%.3f,%.3f,%.3f,%ld,%ld,%.3f,%.3f,%.3f,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
                 event,
                 (unsigned int)path_state->point_cnt,
                 (double)path_state->last_distance_mm,
                 (double)encoder_observation->distance_mm,
                 (double)encoder_observation->left_total_distance_mm,
                 (double)encoder_observation->right_total_distance_mm,
                 (long)encoder_observation->left_total_count,
                 (long)encoder_observation->right_total_count,
                 (double)pose_observation->x_mm,
                 (double)pose_observation->y_mm,
                 (double)(pose_observation->theta_accum_rad
                          * APP_TASK_ASYNCHRONOUS_VEHICLE_RAD_TO_DEG),
                 (unsigned long)pose_observation->update_count,
                 (unsigned long)encoder_observation->update_count,
                 (unsigned long)encoder_observation->left_sample_count,
                 (unsigned long)encoder_observation->right_sample_count,
                 (unsigned long)encoder_observation->left_raw_sample_count,
                 (unsigned long)encoder_observation->right_raw_sample_count);
}

static void app_task_asynchronous_vehicle_record_full_check(void)
{
    const vehicle_path_state_t* path_state = module_vehicle_path_state_get();

    if (app_task_asynchronous_vehicle_runtime.record_active == FALSE)
    {
        return;
    }

    app_task_asynchronous_vehicle_runtime.last_point_cnt = path_state->point_cnt;

    if (path_state->status == VEHICLE_PATH_STATUS_FULL)
    {
        (void)app_task_asynchronous_vehicle_record_stop_save();
        tools_printf("{pathinfo}record_full\r\n");
    }
}

static void app_task_asynchronous_vehicle_record_save_check(void)
{
    if (app_task_asynchronous_vehicle_runtime.status != APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_PATH_SAVING)
    {
        return;
    }

    if (module_vehicle_path_save_is_busy() != FALSE)
    {
        return;
    }

    if (module_vehicle_path_save_is_failed() != FALSE)
    {
        app_task_asynchronous_vehicle_runtime.path_saved = FALSE;
        app_task_asynchronous_vehicle_runtime.mode_blocked = TRUE;
        app_task_asynchronous_vehicle_runtime.last_point_cnt = module_vehicle_path_state_get()->point_cnt;
        app_task_asynchronous_vehicle_runtime.status = APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_PATH_SAVE_FAILED;
        tools_printf("{pathinfo}record_save_failed,%u\r\n",
                     (unsigned int)app_task_asynchronous_vehicle_runtime.last_point_cnt);
        return;
    }

    app_task_asynchronous_vehicle_runtime.path_saved = TRUE;
    app_task_asynchronous_vehicle_runtime.mode_blocked = FALSE;
    app_task_asynchronous_vehicle_runtime.last_point_cnt = module_vehicle_path_state_get()->point_cnt;
    tools_printf("{pathinfo}record_saved,%u\r\n",
                 (unsigned int)app_task_asynchronous_vehicle_runtime.last_point_cnt);
#if VEHICLE_PATH_DEFAULT_AUTO_PLAN_ENABLE
    if (module_vehicle_path_plan() != FALSE)
    {
        app_task_asynchronous_vehicle_runtime.status = APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_PATH_PLANNING;
        tools_printf("{pathinfo}record_plan_queued,%u\r\n",
                     (unsigned int)app_task_asynchronous_vehicle_runtime.last_point_cnt);
    }
    else
    {
        app_task_asynchronous_vehicle_runtime.status = APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_PATH_PLAN_FAILED;
        app_task_asynchronous_vehicle_runtime.mode_blocked = TRUE;
        tools_printf("{pathinfo}record_plan_queue_failed\r\n");
    }
#else
    app_task_asynchronous_vehicle_runtime.status = APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_PATH_SAVED;
    app_task_asynchronous_vehicle_runtime.path_loaded = TRUE;
    tools_printf("{pathinfo}record_ready_raw,%u\r\n",
                 (unsigned int)app_task_asynchronous_vehicle_runtime.last_point_cnt);
#endif
}

static void app_task_asynchronous_vehicle_record_plan_check(void)
{
    boolean success;

    if (app_task_asynchronous_vehicle_runtime.status != APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_PATH_PLANNING)
    {
        return;
    }
    if (module_vehicle_path_job_is_busy() != FALSE)
    {
        return;
    }
    if ((module_vehicle_path_plan_last_result_get(&success) == FALSE) || (success == FALSE))
    {
        app_task_asynchronous_vehicle_runtime.status = APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_PATH_PLAN_FAILED;
        app_task_asynchronous_vehicle_runtime.mode_blocked = TRUE;
        tools_printf("{pathinfo}record_plan_failed\r\n");
        return;
    }

    app_task_asynchronous_vehicle_runtime.status = APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_PATH_SAVED;
    app_task_asynchronous_vehicle_runtime.path_loaded = TRUE;
    app_task_asynchronous_vehicle_runtime.mode_blocked = FALSE;
    app_task_asynchronous_vehicle_runtime.last_point_cnt = module_vehicle_path_state_get()->point_cnt;
    tools_printf("{pathinfo}record_plan_success,%u\r\n",
                 (unsigned int)app_task_asynchronous_vehicle_runtime.last_point_cnt);
}

static void app_task_asynchronous_vehicle_replay_load_check(void)
{
    const vehicle_path_state_t* path_state = module_vehicle_path_state_get();

    if (app_task_asynchronous_vehicle_runtime.status != APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_PATH_LOADING)
    {
        return;
    }

    if (module_vehicle_path_load_is_busy() != FALSE)
    {
        return;
    }

    if ((module_vehicle_path_load_is_failed() != FALSE)
        || (path_state->point_cnt < vehicle_path_cfg_get()->min_valid_point_cnt))
    {
        app_task_asynchronous_vehicle_runtime.path_loaded = FALSE;
        app_task_asynchronous_vehicle_runtime.mode_blocked = TRUE;
        app_task_asynchronous_vehicle_runtime.status = APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_PATH_LOAD_FAILED;
        device_led_set_state(DEVICE_LED_2, DEVICE_LED_DARK);
        tools_printf("{pathinfo}replay_load_failed,%u\r\n", (unsigned int)path_state->point_cnt);
        return;
    }

    app_task_asynchronous_vehicle_runtime.last_point_cnt = path_state->point_cnt;
    app_task_asynchronous_vehicle_runtime.path_loaded = TRUE;
    app_task_asynchronous_vehicle_runtime.timer_s = 0.0f;
    app_task_asynchronous_vehicle_runtime.status = (vehicle_control_auto_suction_enabled_get() != FALSE)
                                                  ? APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_REPLAY_SUCTION_DELAY
                                                  : APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_REPLAY_DRIVE_DELAY;
    tools_printf("{pathinfo}replay_start,%u\r\n", (unsigned int)path_state->point_cnt);
    if (vehicle_control_auto_suction_enabled_get() == FALSE)
    {
        tools_printf("{pathinfo}replay_suction_off\r\n");
    }
}

static void app_task_asynchronous_vehicle_replay_finish_check(void)
{
    const vehicle_path_state_t* path_state = module_vehicle_path_state_get();

    app_task_asynchronous_vehicle_runtime.last_point_cnt = path_state->point_cnt;

    if (app_task_asynchronous_vehicle_runtime.replay_active == FALSE)
    {
        return;
    }

    if (path_state->status == VEHICLE_PATH_STATUS_FINISHED)
    {
        app_task_asynchronous_vehicle_runtime.replay_active = FALSE;
        app_task_asynchronous_vehicle_runtime.timer_s = 0.0f;
        app_task_asynchronous_vehicle_runtime.status =
            APP_TASK_ASYNCHRONOUS_VEHICLE_STATUS_REPLAY_FINISH_DRIVE_HOLD;
        app_task_asynchronous_vehicle_path_diag_print("replay_path_end");
        tools_printf("{pathinfo}replay_path_end\r\n");
    }
}

static void app_task_asynchronous_vehicle_path_ahead_print(void)
{
    float32 lookahead_mm;
    float32 tangent_mm;

    module_vehicle_path_replay_ahead_get(&lookahead_mm, &tangent_mm);
    tools_printf("{pathahead}look,%.1f,tangent,%.1f\r\n",
                 (double)lookahead_mm,
                 (double)tangent_mm);
}

static void app_task_asynchronous_vehicle_path_speed_print(void)
{
    boolean active = module_vehicle_path_replay_planned_speed_active_get();
    boolean available = module_vehicle_path_replay_planned_speed_available_get();
    boolean requested = module_vehicle_path_replay_planned_speed_requested_get();
    const char* mode = "fixed";

    if (active != FALSE)
    {
        mode = "planned";
    }
    else if (requested != FALSE)
    {
        mode = "planned_pending";
    }

    tools_printf("{pathspeed}mode,%s,available,%u,fixed,%.1f,preview,%.1f\r\n",
                 mode,
                 (unsigned int)((available != FALSE) ? 1u : 0u),
                 (double)module_vehicle_path_replay_speed_get(),
                 (double)module_vehicle_path_replay_speed_preview_distance_get());
}

static vehicle_path_marker_kind_t app_task_asynchronous_vehicle_path_marker_kind_get(const uint8* text)
{
    if ((app_task_asynchronous_vehicle_command_word_is(text, "in") != FALSE)
        || (app_task_asynchronous_vehicle_command_word_is(text, "turnin") != FALSE)
        || (app_task_asynchronous_vehicle_command_word_is(text, "enter") != FALSE))
    {
        return VEHICLE_PATH_MARKER_KIND_TURN_IN;
    }

    if ((app_task_asynchronous_vehicle_command_word_is(text, "out") != FALSE)
        || (app_task_asynchronous_vehicle_command_word_is(text, "turnout") != FALSE)
        || (app_task_asynchronous_vehicle_command_word_is(text, "exit") != FALSE))
    {
        return VEHICLE_PATH_MARKER_KIND_TURN_OUT;
    }

    return VEHICLE_PATH_MARKER_KIND_NONE;
}

static float32 app_task_asynchronous_vehicle_command_float_get(const uint8* text)
{
    float32 value = 0.0f;
    float32 fraction = 0.1f;
    sint32 sign = 1;
    uint32 index = 0u;
    boolean fraction_part = FALSE;

    if (text == NULL_PTR)
    {
        return 0.0f;
    }

    if (text[index] == (uint8)'-')
    {
        sign = -1;
        index++;
    }
    else if (text[index] == (uint8)'+')
    {
        index++;
    }

    while (text[index] != (uint8)'\0')
    {
        if (text[index] == (uint8)'.')
        {
            fraction_part = TRUE;
            index++;
            continue;
        }

        if ((text[index] < (uint8)'0') || (text[index] > (uint8)'9'))
        {
            break;
        }

        if (fraction_part == FALSE)
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

    return (sign < 0) ? -value : value;
}

static boolean app_task_asynchronous_vehicle_command_uint32_get(const uint8* text, uint32* value)
{
    uint32 result = 0u;
    uint32 index = 0u;

    if ((text == NULL_PTR) || (value == NULL_PTR) || (text[0] == (uint8)'\0'))
    {
        return FALSE;
    }

    while (text[index] != (uint8)'\0')
    {
        uint32 digit;

        if ((text[index] < (uint8)'0') || (text[index] > (uint8)'9'))
        {
            return FALSE;
        }
        digit = (uint32)(text[index] - (uint8)'0');
        if (result > ((0xFFFFFFFFu - digit) / 10u))
        {
            return FALSE;
        }
        result = (result * 10u) + digit;
        index++;
    }

    *value = result;
    return TRUE;
}

static uint16 app_task_asynchronous_vehicle_uint16_le_get(const uint8* data)
{
    return (uint16)((uint16)data[0] | ((uint16)data[1] << 8u));
}

static uint32 app_task_asynchronous_vehicle_uint32_le_get(const uint8* data)
{
    return (uint32)data[0]
         | ((uint32)data[1] << 8u)
         | ((uint32)data[2] << 16u)
         | ((uint32)data[3] << 24u);
}

static float32 app_task_asynchronous_vehicle_float32_le_get(const uint8* data)
{
    uint32 bits = app_task_asynchronous_vehicle_uint32_le_get(data);
    float32 value;

    (void)memcpy(&value, &bits, sizeof(value));
    return value;
}

static uint32 app_task_asynchronous_vehicle_crc32(const uint8* data, uint16 length)
{
    uint32 crc = 0xFFFFFFFFu;
    uint16 index;

    if (data == NULL_PTR)
    {
        return 0u;
    }

    for (index = 0u; index < length; index++)
    {
        uint8 bit;

        crc ^= (uint32)data[index];
        for (bit = 0u; bit < 8u; bit++)
        {
            crc = ((crc & 1u) != 0u) ? ((crc >> 1u) ^ 0xEDB88320u) : (crc >> 1u);
        }
    }

    return crc ^ 0xFFFFFFFFu;
}

static void app_task_asynchronous_vehicle_path_upload_reset(void)
{
    app_task_asynchronous_vehicle_path_upload.active = FALSE;
    app_task_asynchronous_vehicle_path_upload.binary_protocol = FALSE;
    app_task_asynchronous_vehicle_path_upload.ready_sent = FALSE;
    app_task_asynchronous_vehicle_path_upload.session_id = 0u;
    app_task_asynchronous_vehicle_path_upload.expected_checksum = 0u;
    app_task_asynchronous_vehicle_path_upload.expected_point_cnt = 0u;
}

static void app_task_asynchronous_vehicle_path_upload_ready_check(void)
{
    if ((app_task_asynchronous_vehicle_path_upload.active == FALSE)
        || (app_task_asynchronous_vehicle_path_upload.ready_sent != FALSE)
        || (module_vehicle_path_import_active_get() == FALSE))
    {
        return;
    }

    app_task_asynchronous_vehicle_path_upload.ready_sent = TRUE;
    if (app_task_asynchronous_vehicle_path_upload.binary_protocol != FALSE)
    {
        tools_printf("{pathupload}begin2,1,%u,%u,%u\r\n",
                     (unsigned int)app_task_asynchronous_vehicle_path_upload.session_id,
                     (unsigned int)app_task_asynchronous_vehicle_path_upload.expected_point_cnt,
                     (unsigned int)module_vehicle_path_flash_point_capacity_get());
    }
    else
    {
        tools_printf("{pathupload}begin,1,%u,%u\r\n",
                     (unsigned int)app_task_asynchronous_vehicle_path_upload.expected_point_cnt,
                     (unsigned int)module_vehicle_path_flash_point_capacity_get());
    }
}

static boolean app_task_asynchronous_vehicle_control_command_post(
    const vehicle_control_command_t* command)
{
    boolean result = vehicle_control_command_post(command);

    if (result == FALSE)
    {
        uint32 command_type = (command != NULL_PTR) ? (uint32)command->type : 0u;
        tools_printf("{vctrl}queue_full,%u\r\n", (unsigned int)command_type);
    }

    return result;
}

static const app_task_asynchronous_vehicle_cfg_t* app_task_asynchronous_vehicle_cfg_get_local(void)
{
    return app_task_asynchronous_vehicle_cfg_get();
}

static boolean app_task_asynchronous_vehicle_command_word_is(const uint8* text, const char* word)
{
    uint32 index = 0u;
    uint8 text_ch;
    uint8 word_ch;

    if ((text == NULL_PTR) || (word == NULL_PTR))
    {
        return FALSE;
    }

    while ((text[index] != (uint8)'\0') && (word[index] != '\0'))
    {
        text_ch = text[index];
        word_ch = (uint8)word[index];

        if ((text_ch >= (uint8)'A') && (text_ch <= (uint8)'Z'))
        {
            text_ch = (uint8)(text_ch + ((uint8)'a' - (uint8)'A'));
        }

        if ((word_ch >= (uint8)'A') && (word_ch <= (uint8)'Z'))
        {
            word_ch = (uint8)(word_ch + ((uint8)'a' - (uint8)'A'));
        }

        if (text_ch != word_ch)
        {
            return FALSE;
        }

        index++;
    }

    return ((text[index] == (uint8)'\0') && (word[index] == '\0')) ? TRUE : FALSE;
}
