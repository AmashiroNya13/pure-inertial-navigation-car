/**
 * @file module_vehicle_gyro.c
 * @brief 车体航向观测模块实现。
 */

#include "../../../../inc/app/module/module_vehicle_gyro/module_vehicle_gyro.h"
#include "../../../../inc/device/device_imu/device_imu.h"
#include "../../../../inc/middleware/sysTick/sysTick.h"
#include "../../../../inc/middleware/tools/tools_print/tools_print.h"

#include <math.h>

#define MODULE_VEHICLE_GYRO_PI            (3.14159265358979323846f)
#define MODULE_VEHICLE_GYRO_DEG_TO_RAD    (MODULE_VEHICLE_GYRO_PI / 180.0f)
#define MODULE_VEHICLE_GYRO_RAD_TO_DEG    (180.0f / MODULE_VEHICLE_GYRO_PI)
#define MODULE_VEHICLE_GYRO_PRINT_ENABLE (0u)
#define MODULE_VEHICLE_GYRO_PRINT_PERIOD_MS ((uint32)100u)
#define MODULE_VEHICLE_GYRO_STARTUP_WAIT_MS ((uint32)17000u)
#define MODULE_VEHICLE_GYRO_STARTUP_PRINT_MS ((uint32)500u)
#define MODULE_VEHICLE_GYRO_STATIC_ACC_REF_LSB (4096.0f)
#define MODULE_VEHICLE_GYRO_STATIC_ACC_TOL_LSB (500.0f)
#define MODULE_VEHICLE_GYRO_ACCEL_MM_S2_PER_LSB (9806.65f / MODULE_VEHICLE_GYRO_STATIC_ACC_REF_LSB)
#define MODULE_VEHICLE_GYRO_STATIC_GYRO_XY_DPS (0.8f)
#define MODULE_VEHICLE_GYRO_STATIC_GYRO_Z_DPS  (0.25f)
#define MODULE_VEHICLE_GYRO_STATIC_CONFIRM_COUNT ((uint32)100u)
#define MODULE_VEHICLE_GYRO_STATIC_EXIT_COUNT ((uint32)5u)
#define MODULE_VEHICLE_GYRO_SFLP_YAW_SMOOTH_ALPHA (0.05f)
#define MODULE_VEHICLE_GYRO_SFLP_YAW_SPIKE_RAD (20.0f * MODULE_VEHICLE_GYRO_DEG_TO_RAD)
#define MODULE_VEHICLE_GYRO_DYNAMIC_BIAS_ALPHA (0.002f)
#define MODULE_VEHICLE_GYRO_DYNAMIC_BIAS_LIMIT_DPS (1.0f)
#define MODULE_VEHICLE_GYRO_MIN_STATIC_DRIFT_MS ((uint32)500u)
#define MODULE_VEHICLE_GYRO_STATIC_DRIFT_UPDATE_MS ((uint32)2000u)
#define MODULE_VEHICLE_GYRO_GRAVITY_ALPHA (0.02f)
#define MODULE_VEHICLE_GYRO_GRAVITY_MIN_NORM_LSB (1000.0f)
#define MODULE_VEHICLE_GYRO_VECTOR_MIN_NORM (0.0001f)

#define MODULE_VEHICLE_GYRO_NOTCH_COUNT ((uint32)3u)
#define MODULE_VEHICLE_GYRO_NOTCH_TABLE_COUNT ((uint32)12u)
#define MODULE_VEHICLE_GYRO_NOTCH_SAMPLE_HZ (1920.0f)
#define MODULE_VEHICLE_GYRO_NOTCH_MIN_HZ (1.0f)
#define MODULE_VEHICLE_GYRO_NOTCH_MAX_HZ ((MODULE_VEHICLE_GYRO_NOTCH_SAMPLE_HZ * 0.5f) - 1.0f)
#define MODULE_VEHICLE_GYRO_NOTCH_DEFAULT_Q (8.0f)
#define MODULE_VEHICLE_GYRO_LPF_CUTOFF_HZ (400.0f)
#define MODULE_VEHICLE_GYRO_LPF_Q (0.70710678f)

typedef struct
{
    float32 b0;
    float32 b1;
    float32 b2;
    float32 a1;
    float32 a2;
    float32 x1;
    float32 x2;
    float32 y1;
    float32 y2;
    float32 frequency_hz;
    float32 q;
    boolean enabled;
    boolean ready;
} module_vehicle_gyro_notch_t;

typedef struct
{
    uint32 suction_duty;
    float32 frequency_hz[MODULE_VEHICLE_GYRO_NOTCH_COUNT];
    float32 q[MODULE_VEHICLE_GYRO_NOTCH_COUNT];
    boolean enabled[MODULE_VEHICLE_GYRO_NOTCH_COUNT];
} module_vehicle_gyro_notch_profile_t;

typedef struct
{
    module_vehicle_gyro_observation_t observation;
    float32 imu_sflp_yaw_last_rad[VEHICLE_GYRO_IMU_COUNT];
    float32 imu_sflp_yaw_raw_rad[VEHICLE_GYRO_IMU_COUNT];
    float32 imu_sflp_yaw_accum_rad[VEHICLE_GYRO_IMU_COUNT];
    boolean imu_sflp_yaw_ready[VEHICLE_GYRO_IMU_COUNT];
    boolean imu_sflp_valid[VEHICLE_GYRO_IMU_COUNT];
    boolean imu_sflp_yaw_tick_ready[VEHICLE_GYRO_IMU_COUNT];
    boolean imu_sflp_yaw_smooth_ready[VEHICLE_GYRO_IMU_COUNT];
    boolean imu_sflp_static_active[VEHICLE_GYRO_IMU_COUNT];
    uint64 imu_sflp_yaw_last_tick[VEHICLE_GYRO_IMU_COUNT];
    uint64 imu_sflp_static_start_tick[VEHICLE_GYRO_IMU_COUNT];
    uint64 imu_sflp_static_end_tick[VEHICLE_GYRO_IMU_COUNT];
    float32 imu_sflp_static_start_yaw_rad[VEHICLE_GYRO_IMU_COUNT];
    float32 imu_sflp_static_end_yaw_rad[VEHICLE_GYRO_IMU_COUNT];
    module_vehicle_gyro_notch_t gyro_z_lpf;
    module_vehicle_gyro_notch_t gyro_z_notch[MODULE_VEHICLE_GYRO_NOTCH_COUNT];
    boolean startup_wait_done;
    boolean startup_wait_timer_started;
    boolean startup_wait_print_started;
    uint64 startup_wait_start_tick;
    uint64 startup_wait_print_tick;
    uint32 startup_wait_elapsed_ms;
    uint32 static_confirm_count;
    uint32 static_exit_count;
    boolean static_detected;
    boolean static_ready;
    float32 static_check_accel_lsb[VEHICLE_GYRO_IMU_COUNT];
    module_vehicle_gyro_vector3_t static_check_gyro_dps[VEHICLE_GYRO_IMU_COUNT];
    float32 imu_sflp_yaw_smooth_rad[VEHICLE_GYRO_IMU_COUNT];
    float32 dynamic_bias_dps[VEHICLE_GYRO_IMU_COUNT];
    module_vehicle_gyro_vector3_t gravity_unit;
    module_vehicle_gyro_vector3_t gravity_static_sum_raw;
    uint32 gravity_static_count;
    boolean gravity_ready;
    boolean gravity_static_active;
    uint64 process_last_tick;
    boolean process_tick_ready;
    uint64 print_last_tick;
    boolean print_tick_ready;
    boolean print_enabled;
    uint32 imu_raw_last_update_count[VEHICLE_GYRO_IMU_COUNT];
    uint32 imu_sflp_last_update_count[VEHICLE_GYRO_IMU_COUNT];
    uint32 suction_duty;
    uint32 notch_profile_index;
    boolean notch_auto_enabled;
} module_vehicle_gyro_runtime_t;

static module_vehicle_gyro_runtime_t module_vehicle_gyro_runtime;

static module_vehicle_gyro_notch_profile_t module_vehicle_gyro_notch_table[MODULE_VEHICLE_GYRO_NOTCH_TABLE_COUNT] =
{
    {0u,     {0.0f,   0.0f,   0.0f},   {8.0f, 8.0f, 8.0f}, {FALSE, FALSE, FALSE}},
    {2000u,  {10.0f,  20.0f,  30.0f},  {8.0f, 8.0f, 8.0f}, {FALSE, FALSE, FALSE}},
    {3000u,  {32.0f,  64.0f,  96.0f},  {8.0f, 8.0f, 8.0f}, {FALSE, FALSE, FALSE}},
    {4000u,  {40.0f,  80.0f,  120.0f}, {8.0f, 8.0f, 8.0f}, {FALSE, FALSE, FALSE}},
    {5000u,  {33.0f,  66.0f,  99.0f},  {8.0f, 8.0f, 8.0f}, {FALSE, FALSE, FALSE}},
    {6000u,  {70.0f,  140.0f, 210.0f}, {8.0f, 8.0f, 8.0f}, {FALSE, FALSE, FALSE}},
    {7000u,  {75.0f,  150.0f, 225.0f}, {8.0f, 8.0f, 8.0f}, {FALSE, FALSE, FALSE}},
    {8000u,  {60.0f,  135.0f, 260.0f}, {8.0f, 8.0f, 8.0f}, {FALSE, FALSE, FALSE}},
    {9000u,  {55.0f,  110.0f, 260.0f}, {8.0f, 8.0f, 8.0f}, {FALSE, FALSE, FALSE}},
    {10000u, {100.0f, 200.0f, 260.0f}, {8.0f, 8.0f, 8.0f}, {FALSE, FALSE, FALSE}},
    {11000u, {105.0f, 210.0f, 260.0f}, {8.0f, 8.0f, 8.0f}, {FALSE, FALSE, FALSE}},
    {12000u, {105.0f, 210.0f, 315.0f}, {8.0f, 8.0f, 8.0f}, {FALSE, FALSE, FALSE}},
};

static void module_vehicle_gyro_samples_read(module_vehicle_gyro_sample_t sample[VEHICLE_GYRO_IMU_COUNT]);
static boolean module_vehicle_gyro_raw_frame_ready(void);
static void module_vehicle_gyro_raw_frame_mark(void);
static boolean module_vehicle_gyro_sflp_frame_ready(void);
static void module_vehicle_gyro_sflp_frame_mark(void);
static void module_vehicle_gyro_raw_vector_get(const uint8* buffer, sint16 raw[VEHICLE_GYRO_AXIS_COUNT]);
static uint32 module_vehicle_gyro_timestamp_get(const uint8* timestamp_buffer);
static float32 module_vehicle_gyro_axis_convert(const sint16 raw[VEHICLE_GYRO_AXIS_COUNT],
                                                const vehicle_gyro_axis_map_t* map,
                                                float32 scale);
static void module_vehicle_gyro_observation_reset(float32 heading_rad);
static boolean module_vehicle_gyro_startup_wait_update(uint32 raw_valid_mask);
static void module_vehicle_gyro_startup_wait_print(uint32 raw_valid_mask,
                                                   uint32 sflp_valid_mask,
                                                   const device_imu_runtime_t* imu_runtime,
                                                   uint32 state,
                                                   uint32 elapsed_ms);
static void module_vehicle_gyro_startup_wait_finish(uint32 raw_valid_mask,
                                                    uint32 sflp_valid_mask,
                                                    const device_imu_runtime_t* imu_runtime,
                                                    uint64 current_tick);
static boolean module_vehicle_gyro_static_state_update(const module_vehicle_gyro_sample_t sample[VEHICLE_GYRO_IMU_COUNT],
                                                       const module_vehicle_gyro_vector3_t gyro_sample_dps[VEHICLE_GYRO_IMU_COUNT]);
static boolean module_vehicle_gyro_static_detect(const module_vehicle_gyro_sample_t sample[VEHICLE_GYRO_IMU_COUNT],
                                                 const module_vehicle_gyro_vector3_t gyro_sample_dps[VEHICLE_GYRO_IMU_COUNT]);
static float32 module_vehicle_gyro_accel_magnitude_get(const sint16 accel_raw[VEHICLE_GYRO_AXIS_COUNT]);
static float32 module_vehicle_gyro_sflp_yaw_smooth_update(uint32 index, float32 yaw_rad);
static void module_vehicle_gyro_sflp_static_drift_update(uint32 index);
static void module_vehicle_gyro_fused_accel_update(const module_vehicle_gyro_sample_t sample[VEHICLE_GYRO_IMU_COUNT]);
static void module_vehicle_gyro_linear_accel_update(void);
static void module_vehicle_gyro_fused_rate_update(const module_vehicle_gyro_vector3_t gyro_sample_dps[VEHICLE_GYRO_IMU_COUNT]);
static float32 module_vehicle_gyro_process_time_update(void);
static void module_vehicle_gyro_gravity_estimate_update(boolean static_ready);
static void module_vehicle_gyro_gravity_static_estimate_apply(void);
static float32 module_vehicle_gyro_yaw_rate_from_gravity_axis_get(void);
static void module_vehicle_gyro_vector_zero(module_vehicle_gyro_vector3_t* value);
static boolean module_vehicle_gyro_vector_normalize(module_vehicle_gyro_vector3_t* value);
static void module_vehicle_gyro_notch_reset(module_vehicle_gyro_notch_t* filter);
static void module_vehicle_gyro_notch_configure(module_vehicle_gyro_notch_t* filter,
                                                float32 frequency_hz,
                                                float32 q,
                                                boolean enabled);
static float32 module_vehicle_gyro_notch_update(module_vehicle_gyro_notch_t* filter, float32 input);
static float32 module_vehicle_gyro_notch_chain_update(float32 input);
static void module_vehicle_gyro_lpf_configure(module_vehicle_gyro_notch_t* filter,
                                              float32 cutoff_hz,
                                              float32 q);
static float32 module_vehicle_gyro_lpf_update(module_vehicle_gyro_notch_t* filter, float32 input);
static uint32 module_vehicle_gyro_notch_profile_find(uint32 suction_duty);
static void module_vehicle_gyro_notch_profile_apply(uint32 profile_index);
static float32 module_vehicle_gyro_command_float_get(const uint8* text);
static boolean module_vehicle_gyro_command_word_is(const uint8* text, const char* word);
static boolean module_vehicle_gyro_sflp_heading_update(boolean static_ready);
static void module_vehicle_gyro_quaternion_from_yaw(float32 yaw_rad, module_vehicle_gyro_quaternion_t* q);
static float32 module_vehicle_gyro_quaternion_dot(const module_vehicle_gyro_quaternion_t* q_a,
                                                  const module_vehicle_gyro_quaternion_t* q_b);
static void module_vehicle_gyro_quaternion_normalize(module_vehicle_gyro_quaternion_t* q);
static void module_vehicle_gyro_quaternion_fuse(const module_vehicle_gyro_quaternion_t* q_a,
                                                const module_vehicle_gyro_quaternion_t* q_b,
                                                module_vehicle_gyro_quaternion_t* q_out);

void module_vehicle_gyro_init(void)
{
    uint32 index;

    module_vehicle_gyro_observation_reset(0.0f);
    for (index = 0u; index < VEHICLE_GYRO_IMU_COUNT; index++)
    {
        module_vehicle_gyro_runtime.imu_sflp_yaw_last_rad[index] = 0.0f;
        module_vehicle_gyro_runtime.imu_sflp_yaw_raw_rad[index] = 0.0f;
        module_vehicle_gyro_runtime.imu_sflp_yaw_accum_rad[index] = 0.0f;
        module_vehicle_gyro_runtime.imu_sflp_yaw_ready[index] = FALSE;
        module_vehicle_gyro_runtime.imu_sflp_valid[index] = FALSE;
        module_vehicle_gyro_runtime.imu_sflp_yaw_tick_ready[index] = FALSE;
        module_vehicle_gyro_runtime.imu_sflp_yaw_smooth_ready[index] = FALSE;
        module_vehicle_gyro_runtime.imu_sflp_static_active[index] = FALSE;
        module_vehicle_gyro_runtime.imu_sflp_yaw_last_tick[index] = 0u;
        module_vehicle_gyro_runtime.imu_sflp_static_start_tick[index] = 0u;
        module_vehicle_gyro_runtime.imu_sflp_static_end_tick[index] = 0u;
        module_vehicle_gyro_runtime.imu_sflp_static_start_yaw_rad[index] = 0.0f;
        module_vehicle_gyro_runtime.imu_sflp_static_end_yaw_rad[index] = 0.0f;
        module_vehicle_gyro_runtime.static_check_accel_lsb[index] = 0.0f;
        module_vehicle_gyro_vector_zero(&module_vehicle_gyro_runtime.static_check_gyro_dps[index]);
        module_vehicle_gyro_runtime.imu_sflp_yaw_smooth_rad[index] = 0.0f;
        module_vehicle_gyro_runtime.dynamic_bias_dps[index] = 0.0f;
    }
    for (index = 0u; index < MODULE_VEHICLE_GYRO_NOTCH_COUNT; index++)
    {
        module_vehicle_gyro_notch_reset(&module_vehicle_gyro_runtime.gyro_z_notch[index]);
    }
    module_vehicle_gyro_lpf_configure(&module_vehicle_gyro_runtime.gyro_z_lpf,
                                      MODULE_VEHICLE_GYRO_LPF_CUTOFF_HZ,
                                      MODULE_VEHICLE_GYRO_LPF_Q);
    module_vehicle_gyro_runtime.startup_wait_done = FALSE;
    module_vehicle_gyro_runtime.startup_wait_timer_started = FALSE;
    module_vehicle_gyro_runtime.startup_wait_print_started = FALSE;
    module_vehicle_gyro_runtime.startup_wait_start_tick = 0u;
    module_vehicle_gyro_runtime.startup_wait_print_tick = 0u;
    module_vehicle_gyro_runtime.startup_wait_elapsed_ms = 0u;
    module_vehicle_gyro_runtime.static_confirm_count = 0u;
    module_vehicle_gyro_runtime.static_exit_count = 0u;
    module_vehicle_gyro_runtime.static_detected = FALSE;
    module_vehicle_gyro_runtime.static_ready = FALSE;
    module_vehicle_gyro_runtime.gravity_unit.x = 0.0f;
    module_vehicle_gyro_runtime.gravity_unit.y = 0.0f;
    module_vehicle_gyro_runtime.gravity_unit.z = 1.0f;
    module_vehicle_gyro_vector_zero(&module_vehicle_gyro_runtime.gravity_static_sum_raw);
    module_vehicle_gyro_runtime.gravity_static_count = 0u;
    module_vehicle_gyro_runtime.gravity_ready = FALSE;
    module_vehicle_gyro_runtime.gravity_static_active = FALSE;
    module_vehicle_gyro_runtime.process_last_tick = 0u;
    module_vehicle_gyro_runtime.process_tick_ready = FALSE;
    module_vehicle_gyro_runtime.print_last_tick = 0u;
    module_vehicle_gyro_runtime.print_tick_ready = FALSE;
    module_vehicle_gyro_runtime.print_enabled =
        (MODULE_VEHICLE_GYRO_PRINT_ENABLE != 0u) ? TRUE : FALSE;
    module_vehicle_gyro_runtime.suction_duty = 0u;
    module_vehicle_gyro_runtime.notch_profile_index = 0u;
    module_vehicle_gyro_runtime.notch_auto_enabled = FALSE;
    module_vehicle_gyro_notch_profile_apply(0u);
}

void module_vehicle_gyro_reset_heading(float32 heading_rad)
{
    uint32 index;

    module_vehicle_gyro_observation_reset(heading_rad);
    for (index = 0u; index < VEHICLE_GYRO_IMU_COUNT; index++)
    {
        module_vehicle_gyro_runtime.imu_sflp_yaw_last_rad[index] = 0.0f;
        module_vehicle_gyro_runtime.imu_sflp_yaw_raw_rad[index] = heading_rad;
        module_vehicle_gyro_runtime.imu_sflp_yaw_accum_rad[index] = heading_rad;
        module_vehicle_gyro_runtime.imu_sflp_yaw_ready[index] = FALSE;
        module_vehicle_gyro_runtime.imu_sflp_valid[index] = FALSE;
        module_vehicle_gyro_runtime.imu_sflp_yaw_tick_ready[index] = FALSE;
        module_vehicle_gyro_runtime.imu_sflp_yaw_smooth_ready[index] = FALSE;
        module_vehicle_gyro_runtime.imu_sflp_static_active[index] = FALSE;
        module_vehicle_gyro_runtime.imu_sflp_yaw_last_tick[index] = 0u;
        module_vehicle_gyro_runtime.imu_sflp_static_start_tick[index] = 0u;
        module_vehicle_gyro_runtime.imu_sflp_static_end_tick[index] = 0u;
        module_vehicle_gyro_runtime.imu_sflp_static_start_yaw_rad[index] = heading_rad;
        module_vehicle_gyro_runtime.imu_sflp_static_end_yaw_rad[index] = heading_rad;
        module_vehicle_gyro_runtime.static_check_accel_lsb[index] = 0.0f;
        module_vehicle_gyro_vector_zero(&module_vehicle_gyro_runtime.static_check_gyro_dps[index]);
        module_vehicle_gyro_runtime.imu_sflp_yaw_smooth_rad[index] = heading_rad;
        module_vehicle_gyro_runtime.dynamic_bias_dps[index] = 0.0f;
        module_vehicle_gyro_runtime.imu_raw_last_update_count[index] = 0u;
        module_vehicle_gyro_runtime.imu_sflp_last_update_count[index] = 0u;
    }
    module_vehicle_gyro_runtime.static_confirm_count = 0u;
    module_vehicle_gyro_runtime.static_exit_count = 0u;
    module_vehicle_gyro_runtime.static_detected = FALSE;
    module_vehicle_gyro_runtime.static_ready = FALSE;
    module_vehicle_gyro_runtime.gravity_unit.x = 0.0f;
    module_vehicle_gyro_runtime.gravity_unit.y = 0.0f;
    module_vehicle_gyro_runtime.gravity_unit.z = 1.0f;
    module_vehicle_gyro_vector_zero(&module_vehicle_gyro_runtime.gravity_static_sum_raw);
    module_vehicle_gyro_runtime.gravity_static_count = 0u;
    module_vehicle_gyro_runtime.gravity_ready = FALSE;
    module_vehicle_gyro_runtime.gravity_static_active = FALSE;
    module_vehicle_gyro_runtime.process_last_tick = 0u;
    module_vehicle_gyro_runtime.process_tick_ready = FALSE;
    module_vehicle_gyro_lpf_configure(&module_vehicle_gyro_runtime.gyro_z_lpf,
                                      MODULE_VEHICLE_GYRO_LPF_CUTOFF_HZ,
                                      MODULE_VEHICLE_GYRO_LPF_Q);
}

void module_vehicle_gyro_update(float32 fallback_dt_s)
{
    module_vehicle_gyro_sample_t sample[VEHICLE_GYRO_IMU_COUNT];

    (void)fallback_dt_s;
    if (module_vehicle_gyro_raw_frame_ready() == FALSE)
    {
        return;
    }

    module_vehicle_gyro_samples_read(sample);
    module_vehicle_gyro_raw_frame_mark();
    module_vehicle_gyro_update_from_samples(sample, 0.0f);
}

void module_vehicle_gyro_update_from_samples(const module_vehicle_gyro_sample_t sample[VEHICLE_GYRO_IMU_COUNT],
                                             float32 fallback_dt_s)
{
    module_vehicle_gyro_vector3_t gyro_sample_dps[VEHICLE_GYRO_IMU_COUNT];
    boolean static_ready;
    uint32 index;
    uint32 valid_count = 0u;
    uint32 raw_valid_mask = 0u;

    (void)fallback_dt_s;
    for (index = 0u; index < VEHICLE_GYRO_IMU_COUNT; index++)
    {
        if (sample[index].valid == FALSE)
        {
            continue;
        }
        gyro_sample_dps[index] = sample[index].gyro_dps;
        valid_count++;
        raw_valid_mask |= ((uint32)1u << index);
    }

    if (valid_count < VEHICLE_GYRO_IMU_COUNT)
    {
        module_vehicle_gyro_runtime.observation.imu_dt_s = 0.0f;
        if (module_vehicle_gyro_runtime.startup_wait_done == FALSE)
        {
            (void)module_vehicle_gyro_startup_wait_update(raw_valid_mask);
        }
        return;
    }

    module_vehicle_gyro_runtime.observation.fused_timestamp = sample[0].timestamp;
    module_vehicle_gyro_fused_accel_update(sample);
    module_vehicle_gyro_fused_rate_update(gyro_sample_dps);
    (void)module_vehicle_gyro_process_time_update();

    /* Publish yaw rate with the same sign convention as theta_accum_rad. */
    module_vehicle_gyro_runtime.observation.gyro_z_rad_s =
        -module_vehicle_gyro_runtime.observation.gyro_z_raw_rad_s;

    if (module_vehicle_gyro_startup_wait_update(raw_valid_mask) == FALSE)
    {
        return;
    }

    static_ready = module_vehicle_gyro_static_state_update(sample, gyro_sample_dps);
    module_vehicle_gyro_gravity_estimate_update(static_ready);
    module_vehicle_gyro_linear_accel_update();
    (void)module_vehicle_gyro_sflp_heading_update(static_ready);
}

const module_vehicle_gyro_observation_t* module_vehicle_gyro_observation_get(void)
{
    return &module_vehicle_gyro_runtime.observation;
}

float32 module_vehicle_gyro_heading_get(void)
{
    return module_vehicle_gyro_runtime.observation.theta_smooth_rad;
}

float32 module_vehicle_gyro_yaw_rate_get(void)
{
    return module_vehicle_gyro_runtime.observation.gyro_z_rad_s;
}

boolean module_vehicle_gyro_startup_ready_get(void)
{
    return module_vehicle_gyro_runtime.startup_wait_done;
}

void module_vehicle_gyro_print_process(void)
{
    uint64 current_tick;
    uint32 print_period_tick;

    if ((module_vehicle_gyro_runtime.print_enabled == FALSE)
        || (module_vehicle_gyro_runtime.startup_wait_done == FALSE))
    {
        module_vehicle_gyro_runtime.print_tick_ready = FALSE;
        return;
    }

    current_tick = sysTick_getTick(SYSTICK1);
    print_period_tick = sysTick_getTicksFromMilliseconds(SYSTICK1,
                                                         MODULE_VEHICLE_GYRO_PRINT_PERIOD_MS);
    if ((module_vehicle_gyro_runtime.print_tick_ready != FALSE)
        && ((current_tick - module_vehicle_gyro_runtime.print_last_tick) < (uint64)print_period_tick))
    {
        return;
    }

    module_vehicle_gyro_runtime.print_tick_ready = TRUE;
    module_vehicle_gyro_runtime.print_last_tick = current_tick;
    tools_printf("{imu}%.3f,%.3f,%.3f,%lu\r\n",
                 (double)(module_vehicle_gyro_runtime.observation.theta_accum_rad
                          * MODULE_VEHICLE_GYRO_RAD_TO_DEG),
                 (double)(module_vehicle_gyro_runtime.imu_sflp_yaw_accum_rad[0]
                          * MODULE_VEHICLE_GYRO_RAD_TO_DEG),
                  (double)(module_vehicle_gyro_runtime.imu_sflp_yaw_accum_rad[1]
                           * MODULE_VEHICLE_GYRO_RAD_TO_DEG),
                  (unsigned long)module_vehicle_gyro_runtime.static_ready);
}

void module_vehicle_gyro_suction_duty_notify(uint32 suction_duty)
{
    uint32 profile_index;

    module_vehicle_gyro_runtime.suction_duty = suction_duty;
    if (module_vehicle_gyro_runtime.notch_auto_enabled == FALSE)
    {
        return;
    }

    profile_index = module_vehicle_gyro_notch_profile_find(suction_duty);
    if (profile_index != module_vehicle_gyro_runtime.notch_profile_index)
    {
        module_vehicle_gyro_notch_profile_apply(profile_index);
    }
}

void module_vehicle_gyro_print_command(uint8 argc, uint8* argv[])
{
    if ((argc < 2u)
        || (module_vehicle_gyro_command_word_is(argv[1], "show") != FALSE)
        || (module_vehicle_gyro_command_word_is(argv[1], "status") != FALSE))
    {
        tools_printf("{gprint}enable,%lu,period_ms,%lu\r\n",
                     (unsigned long)module_vehicle_gyro_runtime.print_enabled,
                     (unsigned long)MODULE_VEHICLE_GYRO_PRINT_PERIOD_MS);
        return;
    }

    if ((module_vehicle_gyro_command_word_is(argv[1], "on") != FALSE)
        || (module_vehicle_gyro_command_word_is(argv[1], "enable") != FALSE)
        || (module_vehicle_gyro_command_word_is(argv[1], "1") != FALSE))
    {
        module_vehicle_gyro_runtime.print_enabled = TRUE;
        module_vehicle_gyro_runtime.print_tick_ready = FALSE;
        tools_printf("{gprint}enable,1,period_ms,%lu\r\n",
                     (unsigned long)MODULE_VEHICLE_GYRO_PRINT_PERIOD_MS);
        return;
    }

    if ((module_vehicle_gyro_command_word_is(argv[1], "off") != FALSE)
        || (module_vehicle_gyro_command_word_is(argv[1], "disable") != FALSE)
        || (module_vehicle_gyro_command_word_is(argv[1], "0") != FALSE))
    {
        module_vehicle_gyro_runtime.print_enabled = FALSE;
        module_vehicle_gyro_runtime.print_tick_ready = FALSE;
        tools_printf("{gprint}enable,0,period_ms,%lu\r\n",
                     (unsigned long)MODULE_VEHICLE_GYRO_PRINT_PERIOD_MS);
        return;
    }

    tools_printf("{gprint}usage:on,off,show\r\n");
}

void module_vehicle_gyro_notch_command(uint8 argc, uint8* argv[])
{
    uint32 profile_index;
    uint32 notch_index;
    float32 frequency_hz;
    float32 q;
    boolean enabled;

    if (argc < 2u)
    {
        tools_printf("{gnotch}auto,%lu,profile,%lu,duty,%lu\r\n",
                     (unsigned long)module_vehicle_gyro_runtime.notch_auto_enabled,
                     (unsigned long)module_vehicle_gyro_runtime.notch_profile_index,
                     (unsigned long)module_vehicle_gyro_runtime.suction_duty);
        return;
    }

    if (module_vehicle_gyro_command_word_is(argv[1], "auto") != FALSE)
    {
        if ((argc >= 3u) && (module_vehicle_gyro_command_word_is(argv[2], "off") == FALSE)
            && (module_vehicle_gyro_command_word_is(argv[2], "0") == FALSE))
        {
            module_vehicle_gyro_runtime.notch_auto_enabled = TRUE;
            module_vehicle_gyro_notch_profile_apply(
                module_vehicle_gyro_notch_profile_find(module_vehicle_gyro_runtime.suction_duty));
        }
        else
        {
            module_vehicle_gyro_runtime.notch_auto_enabled = FALSE;
            module_vehicle_gyro_notch_profile_apply(0u);
        }
        tools_printf("{gnotch}auto,%lu\r\n",
                     (unsigned long)module_vehicle_gyro_runtime.notch_auto_enabled);
        return;
    }

    if (module_vehicle_gyro_command_word_is(argv[1], "list") != FALSE)
    {
        for (profile_index = 0u; profile_index < MODULE_VEHICLE_GYRO_NOTCH_TABLE_COUNT; profile_index++)
        {
            tools_printf("{gnotch}%lu,%lu,%.3f,%.3f,%lu,%.3f,%.3f,%lu,%.3f,%.3f,%lu\r\n",
                         (unsigned long)profile_index,
                         (unsigned long)module_vehicle_gyro_notch_table[profile_index].suction_duty,
                         (double)module_vehicle_gyro_notch_table[profile_index].frequency_hz[0],
                         (double)module_vehicle_gyro_notch_table[profile_index].q[0],
                         (unsigned long)module_vehicle_gyro_notch_table[profile_index].enabled[0],
                         (double)module_vehicle_gyro_notch_table[profile_index].frequency_hz[1],
                         (double)module_vehicle_gyro_notch_table[profile_index].q[1],
                         (unsigned long)module_vehicle_gyro_notch_table[profile_index].enabled[1],
                         (double)module_vehicle_gyro_notch_table[profile_index].frequency_hz[2],
                         (double)module_vehicle_gyro_notch_table[profile_index].q[2],
                         (unsigned long)module_vehicle_gyro_notch_table[profile_index].enabled[2]);
        }
        return;
    }

    if ((module_vehicle_gyro_command_word_is(argv[1], "use") != FALSE) && (argc >= 3u))
    {
        profile_index = (uint32)(module_vehicle_gyro_command_float_get(argv[2]) + 0.5f);
        if (profile_index < MODULE_VEHICLE_GYRO_NOTCH_TABLE_COUNT)
        {
            module_vehicle_gyro_runtime.notch_auto_enabled = FALSE;
            module_vehicle_gyro_notch_profile_apply(profile_index);
            tools_printf("{gnotch}use,%lu\r\n", (unsigned long)profile_index);
        }
        return;
    }

    if ((module_vehicle_gyro_command_word_is(argv[1], "set") != FALSE) && (argc >= 7u))
    {
        profile_index = (uint32)(module_vehicle_gyro_command_float_get(argv[2]) + 0.5f);
        notch_index = (uint32)(module_vehicle_gyro_command_float_get(argv[3]) + 0.5f);
        if ((profile_index >= MODULE_VEHICLE_GYRO_NOTCH_TABLE_COUNT)
            || (notch_index >= MODULE_VEHICLE_GYRO_NOTCH_COUNT))
        {
            return;
        }

        frequency_hz = module_vehicle_gyro_command_float_get(argv[4]);
        q = module_vehicle_gyro_command_float_get(argv[5]);
        enabled = (module_vehicle_gyro_command_float_get(argv[6]) > 0.5f) ? TRUE : FALSE;
        module_vehicle_gyro_notch_table[profile_index].frequency_hz[notch_index] = frequency_hz;
        module_vehicle_gyro_notch_table[profile_index].q[notch_index] = q;
        module_vehicle_gyro_notch_table[profile_index].enabled[notch_index] = enabled;

        if (profile_index == module_vehicle_gyro_runtime.notch_profile_index)
        {
            module_vehicle_gyro_notch_profile_apply(profile_index);
        }
        tools_printf("{gnotch}set,%lu,%lu,%.3f,%.3f,%lu\r\n",
                     (unsigned long)profile_index,
                     (unsigned long)notch_index,
                     (double)frequency_hz,
                     (double)q,
                     (unsigned long)enabled);
    }
}

static void module_vehicle_gyro_samples_read(module_vehicle_gyro_sample_t sample[VEHICLE_GYRO_IMU_COUNT])
{
    const vehicle_gyro_cfg_t* gyro_cfg = vehicle_gyro_cfg_get();
    device_imu_runtime_t* imu_runtime = device_imu_runtime_table_get();
    uint32 index;

    for (index = 0u; index < VEHICLE_GYRO_IMU_COUNT; index++)
    {
        const vehicle_gyro_imu_cfg_t* imu_cfg = &gyro_cfg->imu[index];
        sint16 gyro_raw[VEHICLE_GYRO_AXIS_COUNT];

        sample[index].accel_raw[VEHICLE_GYRO_AXIS_X] = 0;
        sample[index].accel_raw[VEHICLE_GYRO_AXIS_Y] = 0;
        sample[index].accel_raw[VEHICLE_GYRO_AXIS_Z] = 0;
        sample[index].timestamp = 0u;
        sample[index].gyro_dps.x = 0.0f;
        sample[index].gyro_dps.y = 0.0f;
        sample[index].gyro_dps.z = 0.0f;
        sample[index].valid = FALSE;

        module_vehicle_gyro_raw_vector_get(imu_runtime[imu_cfg->imu_id].imu_accelerometer_buffer,
                                           sample[index].accel_raw);
        module_vehicle_gyro_raw_vector_get(imu_runtime[imu_cfg->imu_id].imu_gyroscope_buffer,
                                           gyro_raw);
        sample[index].timestamp =
            module_vehicle_gyro_timestamp_get(imu_runtime[imu_cfg->imu_id].imu_timestamp_buffer);

        if ((sample[index].timestamp == 0u)
            && (sample[index].accel_raw[VEHICLE_GYRO_AXIS_X] == 0)
            && (sample[index].accel_raw[VEHICLE_GYRO_AXIS_Y] == 0)
            && (sample[index].accel_raw[VEHICLE_GYRO_AXIS_Z] == 0)
            && (gyro_raw[VEHICLE_GYRO_AXIS_X] == 0)
            && (gyro_raw[VEHICLE_GYRO_AXIS_Y] == 0)
            && (gyro_raw[VEHICLE_GYRO_AXIS_Z] == 0))
        {
            continue;
        }

        sample[index].gyro_dps.x = module_vehicle_gyro_axis_convert(gyro_raw,
                                                                    &imu_cfg->gyro_map[VEHICLE_GYRO_AXIS_X],
                                                                    gyro_cfg->gyro_scale_dps_per_lsb)
                                 - imu_cfg->gyro_bias_dps[VEHICLE_GYRO_AXIS_X];
        sample[index].gyro_dps.y = module_vehicle_gyro_axis_convert(gyro_raw,
                                                                    &imu_cfg->gyro_map[VEHICLE_GYRO_AXIS_Y],
                                                                    gyro_cfg->gyro_scale_dps_per_lsb)
                                 - imu_cfg->gyro_bias_dps[VEHICLE_GYRO_AXIS_Y];
        sample[index].gyro_dps.z = module_vehicle_gyro_axis_convert(gyro_raw,
                                                                    &imu_cfg->gyro_map[VEHICLE_GYRO_AXIS_Z],
                                                                    gyro_cfg->gyro_scale_dps_per_lsb)
                                 - imu_cfg->gyro_bias_dps[VEHICLE_GYRO_AXIS_Z];
        sample[index].valid = TRUE;
    }
}

static void module_vehicle_gyro_observation_reset(float32 heading_rad)
{
    heading_rad = algorithm_attitude_wrap_pi(heading_rad);

    module_vehicle_gyro_quaternion_from_yaw(heading_rad, &module_vehicle_gyro_runtime.observation.attitude_q);
    module_vehicle_gyro_runtime.observation.fused_accel_raw.x = 0.0f;
    module_vehicle_gyro_runtime.observation.fused_accel_raw.y = 0.0f;
    module_vehicle_gyro_runtime.observation.fused_accel_raw.z = 0.0f;
    module_vehicle_gyro_runtime.observation.fused_gyro_dps.x = 0.0f;
    module_vehicle_gyro_runtime.observation.fused_gyro_dps.y = 0.0f;
    module_vehicle_gyro_runtime.observation.fused_gyro_dps.z = 0.0f;
    module_vehicle_gyro_runtime.observation.fused_gyro_rad_s.x = 0.0f;
    module_vehicle_gyro_runtime.observation.fused_gyro_rad_s.y = 0.0f;
    module_vehicle_gyro_runtime.observation.fused_gyro_rad_s.z = 0.0f;
    module_vehicle_gyro_runtime.observation.theta_rad = heading_rad;
    module_vehicle_gyro_runtime.observation.theta_smooth_rad = heading_rad;
    module_vehicle_gyro_runtime.observation.theta_accum_rad = heading_rad;
    module_vehicle_gyro_runtime.observation.theta_smooth_accum_rad = heading_rad;
    module_vehicle_gyro_runtime.observation.gyro_z_raw_rad_s = 0.0f;
    module_vehicle_gyro_runtime.observation.gyro_z_rad_s = 0.0f;
    module_vehicle_gyro_runtime.observation.gyro_integral_rad = heading_rad;
    module_vehicle_gyro_runtime.observation.imu_dt_s = 0.0f;
    module_vehicle_gyro_runtime.observation.fused_timestamp = 0u;
}

static boolean module_vehicle_gyro_startup_wait_update(uint32 raw_valid_mask)
{
    const vehicle_gyro_cfg_t* gyro_cfg = vehicle_gyro_cfg_get();
    device_imu_runtime_t* imu_runtime = device_imu_runtime_table_get();
    uint64 current_tick;
    uint32 index;
    uint32 sflp_valid_mask = 0u;
    boolean sflp_ready = TRUE;

    if (module_vehicle_gyro_runtime.startup_wait_done != FALSE)
    {
        return TRUE;
    }

    for (index = 0u; index < VEHICLE_GYRO_IMU_COUNT; index++)
    {
        const vehicle_gyro_imu_cfg_t* imu_cfg = &gyro_cfg->imu[index];

        if (imu_runtime[imu_cfg->imu_id].imu_sflp_game_valid == FALSE)
        {
            sflp_ready = FALSE;
        }
        else
        {
            sflp_valid_mask |= ((uint32)1u << index);
        }
    }

    current_tick = sysTick_getTick(SYSTICK1);
    if (module_vehicle_gyro_runtime.startup_wait_timer_started == FALSE)
    {
        module_vehicle_gyro_runtime.startup_wait_start_tick = current_tick;
        module_vehicle_gyro_runtime.startup_wait_timer_started = TRUE;
    }

    module_vehicle_gyro_runtime.startup_wait_elapsed_ms =
        (uint32)sysTick_ticksToMilliseconds(SYSTICK1,
                                            current_tick
                                          - module_vehicle_gyro_runtime.startup_wait_start_tick);

    if (module_vehicle_gyro_runtime.startup_wait_elapsed_ms >= MODULE_VEHICLE_GYRO_STARTUP_WAIT_MS)
    {
        module_vehicle_gyro_startup_wait_finish(raw_valid_mask,
                                                sflp_valid_mask,
                                                imu_runtime,
                                                current_tick);
        return TRUE;
    }

    if (sflp_ready == FALSE)
    {
        module_vehicle_gyro_startup_wait_print(raw_valid_mask,
                                               sflp_valid_mask,
                                               imu_runtime,
                                               1u,
                                               module_vehicle_gyro_runtime.startup_wait_elapsed_ms);
        return FALSE;
    }

    module_vehicle_gyro_startup_wait_print(raw_valid_mask,
                                           sflp_valid_mask,
                                           imu_runtime,
                                           2u,
                                           module_vehicle_gyro_runtime.startup_wait_elapsed_ms);
    return FALSE;
}

static void module_vehicle_gyro_startup_wait_finish(uint32 raw_valid_mask,
                                                    uint32 sflp_valid_mask,
                                                    const device_imu_runtime_t* imu_runtime,
                                                    uint64 current_tick)
{
    uint32 index;

    module_vehicle_gyro_runtime.startup_wait_done = TRUE;
    module_vehicle_gyro_runtime.observation.theta_rad = 0.0f;
    module_vehicle_gyro_runtime.observation.theta_smooth_rad = 0.0f;
    module_vehicle_gyro_runtime.observation.theta_accum_rad = 0.0f;
    module_vehicle_gyro_runtime.observation.theta_smooth_accum_rad = 0.0f;
    module_vehicle_gyro_vector_zero(&module_vehicle_gyro_runtime.gravity_static_sum_raw);
    module_vehicle_gyro_runtime.gravity_static_count = 0u;
    module_vehicle_gyro_runtime.gravity_static_active = FALSE;
    module_vehicle_gyro_runtime.process_last_tick = current_tick;
    module_vehicle_gyro_runtime.process_tick_ready = FALSE;
    module_vehicle_gyro_quaternion_from_yaw(0.0f, &module_vehicle_gyro_runtime.observation.attitude_q);
    for (index = 0u; index < VEHICLE_GYRO_IMU_COUNT; index++)
    {
        module_vehicle_gyro_runtime.imu_sflp_yaw_last_rad[index] = 0.0f;
        module_vehicle_gyro_runtime.imu_sflp_yaw_raw_rad[index] = 0.0f;
        module_vehicle_gyro_runtime.imu_sflp_yaw_accum_rad[index] = 0.0f;
        module_vehicle_gyro_runtime.imu_sflp_yaw_ready[index] = FALSE;
        module_vehicle_gyro_runtime.imu_sflp_valid[index] = FALSE;
        module_vehicle_gyro_runtime.imu_sflp_yaw_tick_ready[index] = FALSE;
        module_vehicle_gyro_runtime.imu_sflp_yaw_smooth_ready[index] = FALSE;
        module_vehicle_gyro_runtime.imu_sflp_static_active[index] = FALSE;
        module_vehicle_gyro_runtime.imu_sflp_yaw_last_tick[index] = 0u;
        module_vehicle_gyro_runtime.imu_sflp_static_start_tick[index] = 0u;
        module_vehicle_gyro_runtime.imu_sflp_static_end_tick[index] = 0u;
        module_vehicle_gyro_runtime.imu_sflp_static_start_yaw_rad[index] = 0.0f;
        module_vehicle_gyro_runtime.imu_sflp_static_end_yaw_rad[index] = 0.0f;
        module_vehicle_gyro_runtime.imu_sflp_yaw_smooth_rad[index] = 0.0f;
    }
    module_vehicle_gyro_runtime.static_confirm_count = 0u;
    module_vehicle_gyro_runtime.static_exit_count = 0u;
    module_vehicle_gyro_runtime.static_detected = FALSE;
    module_vehicle_gyro_runtime.static_ready = FALSE;
    module_vehicle_gyro_startup_wait_print(raw_valid_mask,
                                           sflp_valid_mask,
                                           imu_runtime,
                                           3u,
                                           module_vehicle_gyro_runtime.startup_wait_elapsed_ms);
}

static boolean module_vehicle_gyro_static_state_update(const module_vehicle_gyro_sample_t sample[VEHICLE_GYRO_IMU_COUNT],
                                                       const module_vehicle_gyro_vector3_t gyro_sample_dps[VEHICLE_GYRO_IMU_COUNT])
{
    if (module_vehicle_gyro_static_detect(sample, gyro_sample_dps) == FALSE)
    {
        module_vehicle_gyro_runtime.static_detected = FALSE;
        if (module_vehicle_gyro_runtime.static_ready != FALSE)
        {
            module_vehicle_gyro_runtime.static_exit_count++;
            if (module_vehicle_gyro_runtime.static_exit_count < MODULE_VEHICLE_GYRO_STATIC_EXIT_COUNT)
            {
                return TRUE;
            }
        }

        module_vehicle_gyro_runtime.static_confirm_count = 0u;
        module_vehicle_gyro_runtime.static_exit_count = 0u;
        module_vehicle_gyro_runtime.static_ready = FALSE;
        return FALSE;
    }

    module_vehicle_gyro_runtime.static_detected = TRUE;
    module_vehicle_gyro_runtime.static_exit_count = 0u;
    if (module_vehicle_gyro_runtime.static_confirm_count < MODULE_VEHICLE_GYRO_STATIC_CONFIRM_COUNT)
    {
        module_vehicle_gyro_runtime.static_confirm_count++;
    }

    if (module_vehicle_gyro_runtime.static_confirm_count >= MODULE_VEHICLE_GYRO_STATIC_CONFIRM_COUNT)
    {
        module_vehicle_gyro_runtime.static_ready = TRUE;
    }

    return module_vehicle_gyro_runtime.static_ready;
}

static boolean module_vehicle_gyro_static_detect(const module_vehicle_gyro_sample_t sample[VEHICLE_GYRO_IMU_COUNT],
                                                 const module_vehicle_gyro_vector3_t gyro_sample_dps[VEHICLE_GYRO_IMU_COUNT])
{
    uint32 index;

    for (index = 0u; index < VEHICLE_GYRO_IMU_COUNT; index++)
    {
        float32 accel_magnitude = module_vehicle_gyro_accel_magnitude_get(sample[index].accel_raw);
        module_vehicle_gyro_runtime.static_check_accel_lsb[index] = accel_magnitude;
        module_vehicle_gyro_runtime.static_check_gyro_dps[index] = gyro_sample_dps[index];
        if ((fabsf(accel_magnitude - MODULE_VEHICLE_GYRO_STATIC_ACC_REF_LSB)
             > MODULE_VEHICLE_GYRO_STATIC_ACC_TOL_LSB)
            || (fabsf(gyro_sample_dps[index].x) > MODULE_VEHICLE_GYRO_STATIC_GYRO_XY_DPS)
            || (fabsf(gyro_sample_dps[index].y) > MODULE_VEHICLE_GYRO_STATIC_GYRO_XY_DPS)
            || (fabsf(gyro_sample_dps[index].z) > MODULE_VEHICLE_GYRO_STATIC_GYRO_Z_DPS))
        {
            return FALSE;
        }
    }

    return TRUE;
}

static float32 module_vehicle_gyro_accel_magnitude_get(const sint16 accel_raw[VEHICLE_GYRO_AXIS_COUNT])
{
    float32 x = (float32)accel_raw[VEHICLE_GYRO_AXIS_X];
    float32 y = (float32)accel_raw[VEHICLE_GYRO_AXIS_Y];
    float32 z = (float32)accel_raw[VEHICLE_GYRO_AXIS_Z];

    return sqrtf((x * x) + (y * y) + (z * z));
}

static float32 module_vehicle_gyro_sflp_yaw_smooth_update(uint32 index, float32 yaw_rad)
{
    if (module_vehicle_gyro_runtime.imu_sflp_yaw_smooth_ready[index] == FALSE)
    {
        module_vehicle_gyro_runtime.imu_sflp_yaw_smooth_rad[index] = yaw_rad;
        module_vehicle_gyro_runtime.imu_sflp_yaw_smooth_ready[index] = TRUE;
    }
    else
    {
        float32 yaw_delta_rad =
            algorithm_attitude_wrap_pi(yaw_rad - module_vehicle_gyro_runtime.imu_sflp_yaw_smooth_rad[index]);

        if (fabsf(yaw_delta_rad) <= MODULE_VEHICLE_GYRO_SFLP_YAW_SPIKE_RAD)
        {
            module_vehicle_gyro_runtime.imu_sflp_yaw_smooth_rad[index] =
                algorithm_attitude_wrap_pi(module_vehicle_gyro_runtime.imu_sflp_yaw_smooth_rad[index]
                                         + (MODULE_VEHICLE_GYRO_SFLP_YAW_SMOOTH_ALPHA * yaw_delta_rad));
        }
    }

    return module_vehicle_gyro_runtime.imu_sflp_yaw_smooth_rad[index];
}

static void module_vehicle_gyro_sflp_static_drift_update(uint32 index)
{
    uint32 static_ms =
        (uint32)sysTick_ticksToMilliseconds(SYSTICK1,
                                            module_vehicle_gyro_runtime.imu_sflp_static_end_tick[index]
                                          - module_vehicle_gyro_runtime.imu_sflp_static_start_tick[index]);

    if (static_ms >= MODULE_VEHICLE_GYRO_MIN_STATIC_DRIFT_MS)
    {
        float32 static_dt_s = (float32)static_ms * 0.001f;
        float32 static_drift_dps =
            algorithm_attitude_wrap_pi(module_vehicle_gyro_runtime.imu_sflp_static_end_yaw_rad[index]
                                     - module_vehicle_gyro_runtime.imu_sflp_static_start_yaw_rad[index])
            * MODULE_VEHICLE_GYRO_RAD_TO_DEG
            / static_dt_s;

        module_vehicle_gyro_runtime.dynamic_bias_dps[index] +=
            MODULE_VEHICLE_GYRO_DYNAMIC_BIAS_ALPHA
            * (static_drift_dps - module_vehicle_gyro_runtime.dynamic_bias_dps[index]);
        module_vehicle_gyro_runtime.dynamic_bias_dps[index] =
            algorithm_attitude_clamp(module_vehicle_gyro_runtime.dynamic_bias_dps[index],
                                     -MODULE_VEHICLE_GYRO_DYNAMIC_BIAS_LIMIT_DPS,
                                     MODULE_VEHICLE_GYRO_DYNAMIC_BIAS_LIMIT_DPS);
    }
}

static boolean module_vehicle_gyro_raw_frame_ready(void)
{
    const vehicle_gyro_cfg_t* gyro_cfg = vehicle_gyro_cfg_get();
    device_imu_runtime_t* imu_runtime = device_imu_runtime_table_get();
    uint32 index;
    boolean any_changed = FALSE;

    for (index = 0u; index < VEHICLE_GYRO_IMU_COUNT; index++)
    {
        const vehicle_gyro_imu_cfg_t* imu_cfg = &gyro_cfg->imu[index];

        if (imu_runtime[imu_cfg->imu_id].imu_raw_update_count
            == module_vehicle_gyro_runtime.imu_raw_last_update_count[index])
        {
            if (module_vehicle_gyro_runtime.startup_wait_done == FALSE)
            {
                continue;
            }
            return FALSE;
        }

        any_changed = TRUE;
    }

    return any_changed;
}

static void module_vehicle_gyro_raw_frame_mark(void)
{
    const vehicle_gyro_cfg_t* gyro_cfg = vehicle_gyro_cfg_get();
    device_imu_runtime_t* imu_runtime = device_imu_runtime_table_get();
    uint32 index;

    for (index = 0u; index < VEHICLE_GYRO_IMU_COUNT; index++)
    {
        const vehicle_gyro_imu_cfg_t* imu_cfg = &gyro_cfg->imu[index];

        module_vehicle_gyro_runtime.imu_raw_last_update_count[index] =
            imu_runtime[imu_cfg->imu_id].imu_raw_update_count;
    }
}

static boolean module_vehicle_gyro_sflp_frame_ready(void)
{
    const vehicle_gyro_cfg_t* gyro_cfg = vehicle_gyro_cfg_get();
    device_imu_runtime_t* imu_runtime = device_imu_runtime_table_get();
    uint32 index;

    for (index = 0u; index < VEHICLE_GYRO_IMU_COUNT; index++)
    {
        const vehicle_gyro_imu_cfg_t* imu_cfg = &gyro_cfg->imu[index];

        if (imu_runtime[imu_cfg->imu_id].imu_sflp_game_update_count
            == module_vehicle_gyro_runtime.imu_sflp_last_update_count[index])
        {
            return FALSE;
        }
    }

    return TRUE;
}

static void module_vehicle_gyro_sflp_frame_mark(void)
{
    const vehicle_gyro_cfg_t* gyro_cfg = vehicle_gyro_cfg_get();
    device_imu_runtime_t* imu_runtime = device_imu_runtime_table_get();
    uint32 index;

    for (index = 0u; index < VEHICLE_GYRO_IMU_COUNT; index++)
    {
        const vehicle_gyro_imu_cfg_t* imu_cfg = &gyro_cfg->imu[index];

        module_vehicle_gyro_runtime.imu_sflp_last_update_count[index] =
            imu_runtime[imu_cfg->imu_id].imu_sflp_game_update_count;
    }
}

static void module_vehicle_gyro_startup_wait_print(uint32 raw_valid_mask,
                                                   uint32 sflp_valid_mask,
                                                   const device_imu_runtime_t* imu_runtime,
                                                   uint32 state,
                                                   uint32 elapsed_ms)
{
    uint64 current_tick = sysTick_getTick(SYSTICK1);
    uint32 print_period_tick = sysTick_getTicksFromMilliseconds(SYSTICK1,
                                                                MODULE_VEHICLE_GYRO_STARTUP_PRINT_MS);

    if ((module_vehicle_gyro_runtime.startup_wait_print_started != FALSE)
        && ((current_tick - module_vehicle_gyro_runtime.startup_wait_print_tick) < (uint64)print_period_tick))
    {
        return;
    }

    module_vehicle_gyro_runtime.startup_wait_print_started = TRUE;
    module_vehicle_gyro_runtime.startup_wait_print_tick = current_tick;
    (void)imu_runtime;
    tools_printf("{imustart}%lu,%lu,%lu,%lu,%lu\r\n",
                 (unsigned long)elapsed_ms,
                 (unsigned long)MODULE_VEHICLE_GYRO_STARTUP_WAIT_MS,
                 (unsigned long)raw_valid_mask,
                 (unsigned long)sflp_valid_mask,
                 (unsigned long)state);
}

static void module_vehicle_gyro_fused_accel_update(const module_vehicle_gyro_sample_t sample[VEHICLE_GYRO_IMU_COUNT])
{
    const vehicle_gyro_cfg_t* cfg = vehicle_gyro_cfg_get();
    module_vehicle_gyro_vector3_t accel[VEHICLE_GYRO_IMU_COUNT];
    uint32 index;

    for (index = 0u; index < VEHICLE_GYRO_IMU_COUNT; index++)
    {
        accel[index].x = module_vehicle_gyro_axis_convert(sample[index].accel_raw,
                                                          &cfg->imu[index].accel_map[VEHICLE_GYRO_AXIS_X],
                                                          1.0f);
        accel[index].y = module_vehicle_gyro_axis_convert(sample[index].accel_raw,
                                                          &cfg->imu[index].accel_map[VEHICLE_GYRO_AXIS_Y],
                                                          1.0f);
        accel[index].z = module_vehicle_gyro_axis_convert(sample[index].accel_raw,
                                                          &cfg->imu[index].accel_map[VEHICLE_GYRO_AXIS_Z],
                                                          1.0f);
    }
    module_vehicle_gyro_runtime.observation.fused_accel_raw.x =
        (accel[0].x + accel[1].x) * 0.5f;
    module_vehicle_gyro_runtime.observation.fused_accel_raw.y =
        (accel[0].y + accel[1].y) * 0.5f;
    module_vehicle_gyro_runtime.observation.fused_accel_raw.z =
        (accel[0].z + accel[1].z) * 0.5f;
}

static void module_vehicle_gyro_linear_accel_update(void)
{
    module_vehicle_gyro_vector3_t* linear =
        &module_vehicle_gyro_runtime.observation.linear_accel_mm_s2;

    module_vehicle_gyro_runtime.observation.gravity_ready =
        module_vehicle_gyro_runtime.gravity_ready;
    if (module_vehicle_gyro_runtime.gravity_ready == FALSE)
    {
        module_vehicle_gyro_vector_zero(linear);
        return;
    }

    linear->x = (module_vehicle_gyro_runtime.observation.fused_accel_raw.x
               - module_vehicle_gyro_runtime.gravity_unit.x
                 * MODULE_VEHICLE_GYRO_STATIC_ACC_REF_LSB)
              * MODULE_VEHICLE_GYRO_ACCEL_MM_S2_PER_LSB;
    linear->y = (module_vehicle_gyro_runtime.observation.fused_accel_raw.y
               - module_vehicle_gyro_runtime.gravity_unit.y
                 * MODULE_VEHICLE_GYRO_STATIC_ACC_REF_LSB)
              * MODULE_VEHICLE_GYRO_ACCEL_MM_S2_PER_LSB;
    linear->z = (module_vehicle_gyro_runtime.observation.fused_accel_raw.z
               - module_vehicle_gyro_runtime.gravity_unit.z
                 * MODULE_VEHICLE_GYRO_STATIC_ACC_REF_LSB)
              * MODULE_VEHICLE_GYRO_ACCEL_MM_S2_PER_LSB;
}

static void module_vehicle_gyro_fused_rate_update(const module_vehicle_gyro_vector3_t gyro_sample_dps[VEHICLE_GYRO_IMU_COUNT])
{
    module_vehicle_gyro_runtime.observation.fused_gyro_dps.x = (gyro_sample_dps[0].x + gyro_sample_dps[1].x) * 0.5f;
    module_vehicle_gyro_runtime.observation.fused_gyro_dps.y = (gyro_sample_dps[0].y + gyro_sample_dps[1].y) * 0.5f;
    module_vehicle_gyro_runtime.observation.fused_gyro_dps.z = (gyro_sample_dps[0].z + gyro_sample_dps[1].z) * 0.5f;

    module_vehicle_gyro_runtime.observation.fused_gyro_rad_s.x =
        module_vehicle_gyro_runtime.observation.fused_gyro_dps.x * MODULE_VEHICLE_GYRO_DEG_TO_RAD;
    module_vehicle_gyro_runtime.observation.fused_gyro_rad_s.y =
        module_vehicle_gyro_runtime.observation.fused_gyro_dps.y * MODULE_VEHICLE_GYRO_DEG_TO_RAD;
    module_vehicle_gyro_runtime.observation.fused_gyro_rad_s.z =
        module_vehicle_gyro_runtime.observation.fused_gyro_dps.z * MODULE_VEHICLE_GYRO_DEG_TO_RAD;
    module_vehicle_gyro_runtime.observation.gyro_z_raw_rad_s =
        module_vehicle_gyro_yaw_rate_from_gravity_axis_get();
}

static float32 module_vehicle_gyro_process_time_update(void)
{
    uint64 current_tick = sysTick_getTick(SYSTICK1);
    uint64 frequency_hz;
    uint64 delta_tick;
    float32 dt_s = 0.0f;

    if (module_vehicle_gyro_runtime.process_tick_ready == FALSE)
    {
        module_vehicle_gyro_runtime.process_last_tick = current_tick;
        module_vehicle_gyro_runtime.process_tick_ready = TRUE;
        module_vehicle_gyro_runtime.observation.imu_dt_s = 0.0f;
        return 0.0f;
    }

    delta_tick = current_tick - module_vehicle_gyro_runtime.process_last_tick;
    module_vehicle_gyro_runtime.process_last_tick = current_tick;
    frequency_hz = sysTick_getFrequencyHz(SYSTICK1);
    if (frequency_hz > 0u)
    {
        dt_s = (float32)((double)delta_tick / (double)frequency_hz);
    }

    module_vehicle_gyro_runtime.observation.imu_dt_s = dt_s;
    return dt_s;
}

static void module_vehicle_gyro_gravity_estimate_update(boolean static_ready)
{
    if (static_ready != FALSE)
    {
        module_vehicle_gyro_runtime.gravity_static_active = TRUE;
        module_vehicle_gyro_runtime.gravity_static_sum_raw.x +=
            module_vehicle_gyro_runtime.observation.fused_accel_raw.x;
        module_vehicle_gyro_runtime.gravity_static_sum_raw.y +=
            module_vehicle_gyro_runtime.observation.fused_accel_raw.y;
        module_vehicle_gyro_runtime.gravity_static_sum_raw.z +=
            module_vehicle_gyro_runtime.observation.fused_accel_raw.z;
        module_vehicle_gyro_runtime.gravity_static_count++;
        return;
    }

    if (module_vehicle_gyro_runtime.gravity_static_active != FALSE)
    {
        module_vehicle_gyro_gravity_static_estimate_apply();
        module_vehicle_gyro_runtime.gravity_static_active = FALSE;
        module_vehicle_gyro_vector_zero(&module_vehicle_gyro_runtime.gravity_static_sum_raw);
        module_vehicle_gyro_runtime.gravity_static_count = 0u;
    }
}

static void module_vehicle_gyro_gravity_static_estimate_apply(void)
{
    module_vehicle_gyro_vector3_t gravity_sample;
    float32 gravity_norm;

    if (module_vehicle_gyro_runtime.gravity_static_count == 0u)
    {
        return;
    }

    gravity_sample.x =
        module_vehicle_gyro_runtime.gravity_static_sum_raw.x
        / (float32)module_vehicle_gyro_runtime.gravity_static_count;
    gravity_sample.y =
        module_vehicle_gyro_runtime.gravity_static_sum_raw.y
        / (float32)module_vehicle_gyro_runtime.gravity_static_count;
    gravity_sample.z =
        module_vehicle_gyro_runtime.gravity_static_sum_raw.z
        / (float32)module_vehicle_gyro_runtime.gravity_static_count;

    gravity_norm = sqrtf((gravity_sample.x * gravity_sample.x)
                       + (gravity_sample.y * gravity_sample.y)
                       + (gravity_sample.z * gravity_sample.z));
    if (gravity_norm < MODULE_VEHICLE_GYRO_GRAVITY_MIN_NORM_LSB)
    {
        return;
    }

    if (module_vehicle_gyro_vector_normalize(&gravity_sample) == FALSE)
    {
        return;
    }
    if (gravity_sample.z < 0.0f)
    {
        gravity_sample.x = -gravity_sample.x;
        gravity_sample.y = -gravity_sample.y;
        gravity_sample.z = -gravity_sample.z;
    }

    if (module_vehicle_gyro_runtime.gravity_ready == FALSE)
    {
        module_vehicle_gyro_runtime.gravity_unit = gravity_sample;
        module_vehicle_gyro_runtime.gravity_ready = TRUE;
        return;
    }

    module_vehicle_gyro_runtime.gravity_unit.x +=
        MODULE_VEHICLE_GYRO_GRAVITY_ALPHA
        * (gravity_sample.x - module_vehicle_gyro_runtime.gravity_unit.x);
    module_vehicle_gyro_runtime.gravity_unit.y +=
        MODULE_VEHICLE_GYRO_GRAVITY_ALPHA
        * (gravity_sample.y - module_vehicle_gyro_runtime.gravity_unit.y);
    module_vehicle_gyro_runtime.gravity_unit.z +=
        MODULE_VEHICLE_GYRO_GRAVITY_ALPHA
        * (gravity_sample.z - module_vehicle_gyro_runtime.gravity_unit.z);
    (void)module_vehicle_gyro_vector_normalize(&module_vehicle_gyro_runtime.gravity_unit);
}

static float32 module_vehicle_gyro_yaw_rate_from_gravity_axis_get(void)
{
    if (module_vehicle_gyro_runtime.gravity_ready == FALSE)
    {
        return module_vehicle_gyro_runtime.observation.fused_gyro_rad_s.z;
    }

    return (module_vehicle_gyro_runtime.observation.fused_gyro_rad_s.x * module_vehicle_gyro_runtime.gravity_unit.x)
         + (module_vehicle_gyro_runtime.observation.fused_gyro_rad_s.y * module_vehicle_gyro_runtime.gravity_unit.y)
         + (module_vehicle_gyro_runtime.observation.fused_gyro_rad_s.z * module_vehicle_gyro_runtime.gravity_unit.z);
}

static void module_vehicle_gyro_vector_zero(module_vehicle_gyro_vector3_t* value)
{
    value->x = 0.0f;
    value->y = 0.0f;
    value->z = 0.0f;
}

static boolean module_vehicle_gyro_vector_normalize(module_vehicle_gyro_vector3_t* value)
{
    float32 norm = sqrtf((value->x * value->x) + (value->y * value->y) + (value->z * value->z));

    if (norm < MODULE_VEHICLE_GYRO_VECTOR_MIN_NORM)
    {
        return FALSE;
    }

    value->x /= norm;
    value->y /= norm;
    value->z /= norm;
    return TRUE;
}

static void module_vehicle_gyro_notch_reset(module_vehicle_gyro_notch_t* filter)
{
    filter->b0 = 1.0f;
    filter->b1 = 0.0f;
    filter->b2 = 0.0f;
    filter->a1 = 0.0f;
    filter->a2 = 0.0f;
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
    filter->frequency_hz = 0.0f;
    filter->q = MODULE_VEHICLE_GYRO_NOTCH_DEFAULT_Q;
    filter->enabled = FALSE;
    filter->ready = FALSE;
}

static void module_vehicle_gyro_notch_configure(module_vehicle_gyro_notch_t* filter,
                                                float32 frequency_hz,
                                                float32 q,
                                                boolean enabled)
{
    float32 omega;
    float32 omega_sin;
    float32 omega_cos;
    float32 alpha;
    float32 a0;

    if ((enabled == FALSE)
        || (frequency_hz < MODULE_VEHICLE_GYRO_NOTCH_MIN_HZ)
        || (frequency_hz > MODULE_VEHICLE_GYRO_NOTCH_MAX_HZ)
        || (q <= 0.1f))
    {
        module_vehicle_gyro_notch_reset(filter);
        filter->frequency_hz = frequency_hz;
        filter->q = q;
        return;
    }

    omega = 2.0f * MODULE_VEHICLE_GYRO_PI * frequency_hz / MODULE_VEHICLE_GYRO_NOTCH_SAMPLE_HZ;
    omega_sin = sinf(omega);
    omega_cos = cosf(omega);
    alpha = omega_sin / (2.0f * q);
    a0 = 1.0f + alpha;

    filter->b0 = 1.0f / a0;
    filter->b1 = (-2.0f * omega_cos) / a0;
    filter->b2 = 1.0f / a0;
    filter->a1 = (-2.0f * omega_cos) / a0;
    filter->a2 = (1.0f - alpha) / a0;
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
    filter->frequency_hz = frequency_hz;
    filter->q = q;
    filter->enabled = TRUE;
    filter->ready = FALSE;
}

static float32 module_vehicle_gyro_notch_update(module_vehicle_gyro_notch_t* filter, float32 input)
{
    float32 output;

    if (filter->enabled == FALSE)
    {
        return input;
    }

    if (filter->ready == FALSE)
    {
        filter->x1 = input;
        filter->x2 = input;
        filter->y1 = input;
        filter->y2 = input;
        filter->ready = TRUE;
        return input;
    }

    output = (filter->b0 * input)
           + (filter->b1 * filter->x1)
           + (filter->b2 * filter->x2)
           - (filter->a1 * filter->y1)
           - (filter->a2 * filter->y2);

    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;

    return output;
}

static float32 module_vehicle_gyro_notch_chain_update(float32 input)
{
    float32 output = input;
    uint32 index;

    for (index = 0u; index < MODULE_VEHICLE_GYRO_NOTCH_COUNT; index++)
    {
        output = module_vehicle_gyro_notch_update(&module_vehicle_gyro_runtime.gyro_z_notch[index], output);
    }

    return output;
}

static void module_vehicle_gyro_lpf_configure(module_vehicle_gyro_notch_t* filter,
                                              float32 cutoff_hz,
                                              float32 q)
{
    float32 omega;
    float32 omega_sin;
    float32 omega_cos;
    float32 alpha;
    float32 a0;

    if ((cutoff_hz < MODULE_VEHICLE_GYRO_NOTCH_MIN_HZ)
        || (cutoff_hz > MODULE_VEHICLE_GYRO_NOTCH_MAX_HZ)
        || (q <= 0.1f))
    {
        module_vehicle_gyro_notch_reset(filter);
        filter->frequency_hz = cutoff_hz;
        filter->q = q;
        return;
    }

    omega = 2.0f * MODULE_VEHICLE_GYRO_PI * cutoff_hz / MODULE_VEHICLE_GYRO_NOTCH_SAMPLE_HZ;
    omega_sin = sinf(omega);
    omega_cos = cosf(omega);
    alpha = omega_sin / (2.0f * q);
    a0 = 1.0f + alpha;

    filter->b0 = ((1.0f - omega_cos) * 0.5f) / a0;
    filter->b1 = (1.0f - omega_cos) / a0;
    filter->b2 = ((1.0f - omega_cos) * 0.5f) / a0;
    filter->a1 = (-2.0f * omega_cos) / a0;
    filter->a2 = (1.0f - alpha) / a0;
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
    filter->frequency_hz = cutoff_hz;
    filter->q = q;
    filter->enabled = TRUE;
    filter->ready = FALSE;
}

static float32 module_vehicle_gyro_lpf_update(module_vehicle_gyro_notch_t* filter, float32 input)
{
    return module_vehicle_gyro_notch_update(filter, input);
}

static uint32 module_vehicle_gyro_notch_profile_find(uint32 suction_duty)
{
    uint32 index;
    uint32 best_index = 0u;
    uint32 best_delta = 0xffffffffu;

    for (index = 0u; index < MODULE_VEHICLE_GYRO_NOTCH_TABLE_COUNT; index++)
    {
        uint32 table_duty = module_vehicle_gyro_notch_table[index].suction_duty;
        uint32 delta = (suction_duty > table_duty) ? (suction_duty - table_duty) : (table_duty - suction_duty);

        if (delta < best_delta)
        {
            best_delta = delta;
            best_index = index;
        }
    }

    return best_index;
}

static void module_vehicle_gyro_notch_profile_apply(uint32 profile_index)
{
    uint32 index;

    if (profile_index >= MODULE_VEHICLE_GYRO_NOTCH_TABLE_COUNT)
    {
        return;
    }

    module_vehicle_gyro_runtime.notch_profile_index = profile_index;
    for (index = 0u; index < MODULE_VEHICLE_GYRO_NOTCH_COUNT; index++)
    {
        module_vehicle_gyro_notch_configure(&module_vehicle_gyro_runtime.gyro_z_notch[index],
                                            module_vehicle_gyro_notch_table[profile_index].frequency_hz[index],
                                            module_vehicle_gyro_notch_table[profile_index].q[index],
                                            module_vehicle_gyro_notch_table[profile_index].enabled[index]);
    }
}

static boolean module_vehicle_gyro_sflp_heading_update(boolean static_ready)
{
    module_vehicle_gyro_quaternion_t q[VEHICLE_GYRO_IMU_COUNT];
    float32 yaw_rad[VEHICLE_GYRO_IMU_COUNT];
    float32 yaw_average_rad;
    uint64 current_tick = sysTick_getTick(SYSTICK1);
    uint32 index;

    if (module_vehicle_gyro_sflp_frame_ready() == FALSE)
    {
        return TRUE;
    }

    for (index = 0u; index < VEHICLE_GYRO_IMU_COUNT; index++)
    {
        const vehicle_gyro_cfg_t* gyro_cfg = vehicle_gyro_cfg_get();
        const vehicle_gyro_imu_cfg_t* imu_cfg = &gyro_cfg->imu[index];
        float32 q_raw[4];
        float32 yaw_smooth_rad;

        module_vehicle_gyro_runtime.imu_sflp_valid[index] =
            device_imu_sflp_game_get(imu_cfg->imu_id, q_raw, &yaw_rad[index]);
        if (module_vehicle_gyro_runtime.imu_sflp_valid[index] == FALSE)
        {
            return FALSE;
        }
        module_vehicle_gyro_runtime.imu_sflp_yaw_raw_rad[index] = yaw_rad[index];

        q[index].w = q_raw[0];
        q[index].x = q_raw[1];
        q[index].y = q_raw[2];
        q[index].z = q_raw[3];
        module_vehicle_gyro_quaternion_normalize(&q[index]);
        yaw_smooth_rad = module_vehicle_gyro_sflp_yaw_smooth_update(index, yaw_rad[index]);

        if (module_vehicle_gyro_runtime.imu_sflp_yaw_ready[index] == FALSE)
        {
            module_vehicle_gyro_runtime.imu_sflp_yaw_last_rad[index] = yaw_rad[index];
            module_vehicle_gyro_runtime.imu_sflp_yaw_last_tick[index] = current_tick;
            module_vehicle_gyro_runtime.imu_sflp_yaw_ready[index] = TRUE;
            module_vehicle_gyro_runtime.imu_sflp_yaw_tick_ready[index] = TRUE;
        }
        else
        {
            float32 yaw_delta_rad =
                algorithm_attitude_wrap_pi(yaw_rad[index] - module_vehicle_gyro_runtime.imu_sflp_yaw_last_rad[index]);
            float32 dt_s = 0.0f;

            if (module_vehicle_gyro_runtime.imu_sflp_yaw_tick_ready[index] != FALSE)
            {
                dt_s = (float32)sysTick_ticksToMicroseconds(SYSTICK1,
                                                            current_tick
                                                          - module_vehicle_gyro_runtime.imu_sflp_yaw_last_tick[index])
                     * 0.000001f;
            }

            if (static_ready == FALSE)
            {
                if (module_vehicle_gyro_runtime.imu_sflp_static_active[index] != FALSE)
                {
                    module_vehicle_gyro_sflp_static_drift_update(index);
                    module_vehicle_gyro_runtime.imu_sflp_yaw_last_rad[index] =
                        module_vehicle_gyro_runtime.imu_sflp_static_end_yaw_rad[index];
                    module_vehicle_gyro_runtime.imu_sflp_yaw_last_tick[index] =
                        module_vehicle_gyro_runtime.imu_sflp_static_end_tick[index];
                    yaw_delta_rad =
                        algorithm_attitude_wrap_pi(yaw_rad[index]
                                                 - module_vehicle_gyro_runtime.imu_sflp_yaw_last_rad[index]);
                    if (module_vehicle_gyro_runtime.imu_sflp_yaw_tick_ready[index] != FALSE)
                    {
                        dt_s = (float32)sysTick_ticksToMicroseconds(SYSTICK1,
                                                                    current_tick
                                                                  - module_vehicle_gyro_runtime.imu_sflp_yaw_last_tick[index])
                             * 0.000001f;
                    }
                }
                module_vehicle_gyro_runtime.imu_sflp_static_active[index] = FALSE;
                yaw_delta_rad -= module_vehicle_gyro_runtime.dynamic_bias_dps[index]
                               * dt_s
                               * MODULE_VEHICLE_GYRO_DEG_TO_RAD;
                module_vehicle_gyro_runtime.imu_sflp_yaw_accum_rad[index] +=
                    yaw_delta_rad;
            }
            else
            {
                if (module_vehicle_gyro_runtime.imu_sflp_static_active[index] == FALSE)
                {
                    module_vehicle_gyro_runtime.imu_sflp_static_active[index] = TRUE;
                    module_vehicle_gyro_runtime.imu_sflp_static_start_tick[index] = current_tick;
                    module_vehicle_gyro_runtime.imu_sflp_static_end_tick[index] = current_tick;
                    module_vehicle_gyro_runtime.imu_sflp_static_start_yaw_rad[index] = yaw_smooth_rad;
                    module_vehicle_gyro_runtime.imu_sflp_static_end_yaw_rad[index] = yaw_smooth_rad;
                }
                else
                {
                    if (module_vehicle_gyro_runtime.static_detected != FALSE)
                    {
                        uint32 static_ms;
                        module_vehicle_gyro_runtime.imu_sflp_static_end_tick[index] = current_tick;
                        module_vehicle_gyro_runtime.imu_sflp_static_end_yaw_rad[index] = yaw_smooth_rad;
                        static_ms =
                            (uint32)sysTick_ticksToMilliseconds(SYSTICK1,
                                                                module_vehicle_gyro_runtime.imu_sflp_static_end_tick[index]
                                                              - module_vehicle_gyro_runtime.imu_sflp_static_start_tick[index]);
                        if (static_ms >= MODULE_VEHICLE_GYRO_STATIC_DRIFT_UPDATE_MS)
                        {
                            module_vehicle_gyro_sflp_static_drift_update(index);
                            module_vehicle_gyro_runtime.imu_sflp_static_start_tick[index] = current_tick;
                            module_vehicle_gyro_runtime.imu_sflp_static_end_tick[index] = current_tick;
                            module_vehicle_gyro_runtime.imu_sflp_static_start_yaw_rad[index] = yaw_smooth_rad;
                            module_vehicle_gyro_runtime.imu_sflp_static_end_yaw_rad[index] = yaw_smooth_rad;
                        }
                    }
                }
            }
            module_vehicle_gyro_runtime.imu_sflp_yaw_last_rad[index] = yaw_rad[index];
            module_vehicle_gyro_runtime.imu_sflp_yaw_last_tick[index] = current_tick;
        }
    }

    module_vehicle_gyro_quaternion_fuse(&q[0], &q[1], &module_vehicle_gyro_runtime.observation.attitude_q);
    yaw_average_rad = (module_vehicle_gyro_runtime.imu_sflp_yaw_accum_rad[0]
                     + module_vehicle_gyro_runtime.imu_sflp_yaw_accum_rad[1]) * 0.5f;
    module_vehicle_gyro_runtime.observation.theta_accum_rad = yaw_average_rad;
    module_vehicle_gyro_runtime.observation.theta_smooth_accum_rad = yaw_average_rad;
    module_vehicle_gyro_runtime.observation.theta_rad = algorithm_attitude_wrap_pi(yaw_average_rad);
    module_vehicle_gyro_runtime.observation.theta_smooth_rad =
        module_vehicle_gyro_runtime.observation.theta_rad;
    module_vehicle_gyro_sflp_frame_mark();

    return TRUE;
}

static void module_vehicle_gyro_quaternion_from_yaw(float32 yaw_rad, module_vehicle_gyro_quaternion_t* q)
{
    yaw_rad = algorithm_attitude_wrap_pi(yaw_rad);
    q->w = cosf(yaw_rad * 0.5f);
    q->x = 0.0f;
    q->y = 0.0f;
    q->z = sinf(yaw_rad * 0.5f);
}

static float32 module_vehicle_gyro_quaternion_dot(const module_vehicle_gyro_quaternion_t* q_a,
                                                  const module_vehicle_gyro_quaternion_t* q_b)
{
    return (q_a->w * q_b->w)
         + (q_a->x * q_b->x)
         + (q_a->y * q_b->y)
         + (q_a->z * q_b->z);
}

static void module_vehicle_gyro_quaternion_normalize(module_vehicle_gyro_quaternion_t* q)
{
    float32 norm = sqrtf((q->w * q->w) + (q->x * q->x) + (q->y * q->y) + (q->z * q->z));

    if (norm <= 0.0f)
    {
        q->w = 1.0f;
        q->x = 0.0f;
        q->y = 0.0f;
        q->z = 0.0f;
        return;
    }

    q->w /= norm;
    q->x /= norm;
    q->y /= norm;
    q->z /= norm;
}

static void module_vehicle_gyro_quaternion_fuse(const module_vehicle_gyro_quaternion_t* q_a,
                                                const module_vehicle_gyro_quaternion_t* q_b,
                                                module_vehicle_gyro_quaternion_t* q_out)
{
    float32 q_b_sign = 1.0f;

    if (module_vehicle_gyro_quaternion_dot(q_a, q_b) < 0.0f)
    {
        q_b_sign = -1.0f;
    }

    q_out->w = q_a->w + (q_b_sign * q_b->w);
    q_out->x = q_a->x + (q_b_sign * q_b->x);
    q_out->y = q_a->y + (q_b_sign * q_b->y);
    q_out->z = q_a->z + (q_b_sign * q_b->z);
    module_vehicle_gyro_quaternion_normalize(q_out);
}

static void module_vehicle_gyro_raw_vector_get(const uint8* buffer, sint16 raw[VEHICLE_GYRO_AXIS_COUNT])
{
    raw[VEHICLE_GYRO_AXIS_X] = (sint16)((((uint16)buffer[1]) << 8u) | ((uint16)buffer[0]));
    raw[VEHICLE_GYRO_AXIS_Y] = (sint16)((((uint16)buffer[3]) << 8u) | ((uint16)buffer[2]));
    raw[VEHICLE_GYRO_AXIS_Z] = (sint16)((((uint16)buffer[5]) << 8u) | ((uint16)buffer[4]));
}

static uint32 module_vehicle_gyro_timestamp_get(const uint8* timestamp_buffer)
{
    return (((uint32)timestamp_buffer[3]) << 24u)
         | (((uint32)timestamp_buffer[2]) << 16u)
         | (((uint32)timestamp_buffer[1]) << 8u)
         | ((uint32)timestamp_buffer[0]);
}

static float32 module_vehicle_gyro_axis_convert(const sint16 raw[VEHICLE_GYRO_AXIS_COUNT],
                                                const vehicle_gyro_axis_map_t* map,
                                                float32 scale)
{
    return (float32)raw[map->source_axis] * (float32)map->sign * scale;
}

static float32 module_vehicle_gyro_command_float_get(const uint8* text)
{
    float32 value = 0.0f;
    float32 fraction = 0.1f;
    sint32 sign = 1;
    boolean fractional = FALSE;

    if (text == NULL_PTR)
    {
        return 0.0f;
    }

    if (*text == (uint8)'-')
    {
        sign = -1;
        text++;
    }

    while (*text != (uint8)'\0')
    {
        if (*text == (uint8)'.')
        {
            fractional = TRUE;
            text++;
            continue;
        }

        if ((*text >= (uint8)'0') && (*text <= (uint8)'9'))
        {
            if (fractional == FALSE)
            {
                value = (value * 10.0f) + (float32)(*text - (uint8)'0');
            }
            else
            {
                value += (float32)(*text - (uint8)'0') * fraction;
                fraction *= 0.1f;
            }
        }
        text++;
    }

    return value * (float32)sign;
}

static boolean module_vehicle_gyro_command_word_is(const uint8* text, const char* word)
{
    if ((text == NULL_PTR) || (word == NULL_PTR))
    {
        return FALSE;
    }

    while ((*text != (uint8)'\0') && (*word != '\0'))
    {
        uint8 lhs = *text;
        uint8 rhs = (uint8)*word;

        if ((lhs >= (uint8)'A') && (lhs <= (uint8)'Z'))
        {
            lhs = (uint8)(lhs + ((uint8)'a' - (uint8)'A'));
        }
        if ((rhs >= (uint8)'A') && (rhs <= (uint8)'Z'))
        {
            rhs = (uint8)(rhs + ((uint8)'a' - (uint8)'A'));
        }

        if (lhs != rhs)
        {
            return FALSE;
        }
        text++;
        word++;
    }

    return ((*text == (uint8)'\0') && (*word == '\0')) ? TRUE : FALSE;
}
