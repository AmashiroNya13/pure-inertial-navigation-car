/**
 * @file module_vehicle_encoder.c
 * @brief Vehicle odometry module implementation.
 */

#include "../../../../inc/app/module/module_vehicle_encoder/module_vehicle_encoder.h"
#include "../../../../config/device/device_magnetic_encoder/device_magnetic_encoder_cfg.h"
#include "../../../../config/device/device_magnetic_encoder/device_magnetic_encoder_register.h"
#include "../../../../inc/middleware/algorithm/algorithm_attitude.h"
#include "../../../../inc/middleware/algorithm/algorithm_odometry.h"
#include "../../../../inc/middleware/tools/tools_print/tools_print.h"

#define MODULE_VEHICLE_ENCODER_RAD_TO_DEG (57.29577951308232f)
#define MODULE_VEHICLE_ENCODER_ANGLE_PRINT_ENABLE (0u)
#define MODULE_VEHICLE_ENCODER_ANGLE_PRINT_PERIOD_MS ((uint32)100u)
#define MODULE_VEHICLE_ENCODER_ZERO_RAW_GUARD_COUNT ((uint16)32u)
#define MODULE_VEHICLE_ENCODER_WHEEL_BASE_MM (140.0f)

typedef struct
{
    float32 output;
    boolean ready;
} module_vehicle_encoder_lowpass_t;

typedef struct
{
    uint16 last_raw;
    uint16 raw_angle;
    sint16 raw_delta_count;
    boolean ready;
    float32 raw_speed_mm_s;
    float32 sample_dt_s;
    float32 speed_mm_s;
    sint16 last_valid_delta_count;
    boolean delta_gate_ready;
    uint32 invalid_sample_count;
    uint32 invalid_streak_count;
    uint16 rejected_raw_angle;
    module_vehicle_encoder_lowpass_t speed_lpf;
} module_vehicle_encoder_wheel_runtime_t;

typedef struct
{
    module_vehicle_encoder_observation_t observation;
    module_vehicle_encoder_wheel_runtime_t left_wheel;
    module_vehicle_encoder_wheel_runtime_t right_wheel;
} module_vehicle_encoder_runtime_t;

static module_vehicle_encoder_runtime_t module_vehicle_encoder_runtime;
static float32 module_vehicle_encoder_wheel_circumference_mm;

static boolean module_vehicle_encoder_wheel_update(const vehicle_encoder_wheel_cfg_t* wheel_cfg,
                                                   module_vehicle_encoder_wheel_runtime_t* wheel_runtime,
                                                   float32 dt_s,
                                                   sint16* delta_count,
                                                   float32* distance_mm,
                                                   float32* speed_mm_s);
static void module_vehicle_encoder_apply_wheel_sample(boolean is_left,
                                                      sint16 delta_count,
                                                      float32 distance_mm,
                                                      float32 speed_mm_s);
static void module_vehicle_encoder_sync_wheel_diag(boolean is_left);
static boolean module_vehicle_encoder_raw_sample_valid(const module_vehicle_encoder_wheel_runtime_t* wheel_runtime,
                                                       uint16 raw_frame,
                                                       uint16 raw_angle,
                                                       sint16 raw_delta_count,
                                                       sint16 max_delta_count);
static void module_vehicle_encoder_delta_gate_reset(module_vehicle_encoder_wheel_runtime_t* wheel_runtime);
static sint32 module_vehicle_encoder_delta_gate_limit_get(
    const module_vehicle_encoder_wheel_runtime_t* wheel_runtime);
static boolean module_vehicle_encoder_delta_gate_jump_detected(
    const module_vehicle_encoder_wheel_runtime_t* wheel_runtime,
    sint16 delta_count);
static sint16 module_vehicle_encoder_delta_direction_apply(const vehicle_encoder_wheel_cfg_t* wheel_cfg,
                                                           sint16 delta_count);
static void module_vehicle_encoder_rejected_baseline_advance(
    const vehicle_encoder_wheel_cfg_t* wheel_cfg,
    module_vehicle_encoder_wheel_runtime_t* wheel_runtime);
static void module_vehicle_encoder_sample_reject(module_vehicle_encoder_wheel_runtime_t* wheel_runtime,
                                                 uint16 raw_angle,
                                                 sint16 raw_delta_count,
                                                 sint16* delta_count,
                                                 float32* distance_mm,
                                                 float32* speed_mm_s);
static float32 module_vehicle_encoder_dt_get(float32 fallback_dt_s);
static void module_vehicle_encoder_lowpass_reset(module_vehicle_encoder_lowpass_t* filter);
static float32 module_vehicle_encoder_lowpass_update(module_vehicle_encoder_lowpass_t* filter,
                                                     float32 input);
static uint16 module_vehicle_encoder_raw_frame_from_buffer(device_magnetic_encoder_id_t encoder_id);
static uint16 module_vehicle_encoder_raw_angle_from_buffer(device_magnetic_encoder_id_t encoder_id);

void module_vehicle_encoder_init(void)
{
    module_vehicle_encoder_wheel_circumference_mm =
        vehicle_encoder_cfg_get()->wheel_circumference_mm;
    module_vehicle_encoder_reset();
    module_vehicle_encoder_runtime.left_wheel.last_raw = 0u;
    module_vehicle_encoder_runtime.right_wheel.last_raw = 0u;
    module_vehicle_encoder_runtime.left_wheel.ready = FALSE;
    module_vehicle_encoder_runtime.right_wheel.ready = FALSE;
    module_vehicle_encoder_runtime.left_wheel.speed_mm_s = 0.0f;
    module_vehicle_encoder_runtime.right_wheel.speed_mm_s = 0.0f;
    module_vehicle_encoder_runtime.left_wheel.invalid_sample_count = 0u;
    module_vehicle_encoder_runtime.right_wheel.invalid_sample_count = 0u;
    module_vehicle_encoder_runtime.left_wheel.invalid_streak_count = 0u;
    module_vehicle_encoder_runtime.right_wheel.invalid_streak_count = 0u;
    module_vehicle_encoder_runtime.left_wheel.rejected_raw_angle = 0u;
    module_vehicle_encoder_runtime.right_wheel.rejected_raw_angle = 0u;
    module_vehicle_encoder_delta_gate_reset(&module_vehicle_encoder_runtime.left_wheel);
    module_vehicle_encoder_delta_gate_reset(&module_vehicle_encoder_runtime.right_wheel);
    module_vehicle_encoder_lowpass_reset(&module_vehicle_encoder_runtime.left_wheel.speed_lpf);
    module_vehicle_encoder_lowpass_reset(&module_vehicle_encoder_runtime.right_wheel.speed_lpf);
}

void module_vehicle_encoder_reset(void)
{
    module_vehicle_encoder_runtime.observation.distance_mm = 0.0f;
    module_vehicle_encoder_runtime.observation.left_speed_mm_s = 0.0f;
    module_vehicle_encoder_runtime.observation.right_speed_mm_s = 0.0f;
    module_vehicle_encoder_runtime.observation.speed_mm_s = 0.0f;
    module_vehicle_encoder_runtime.observation.left_delta_count = 0;
    module_vehicle_encoder_runtime.observation.right_delta_count = 0;
    module_vehicle_encoder_runtime.observation.left_distance_mm = 0.0f;
    module_vehicle_encoder_runtime.observation.right_distance_mm = 0.0f;
    module_vehicle_encoder_runtime.observation.left_total_distance_mm = 0.0f;
    module_vehicle_encoder_runtime.observation.right_total_distance_mm = 0.0f;
    module_vehicle_encoder_runtime.observation.left_total_count = 0;
    module_vehicle_encoder_runtime.observation.right_total_count = 0;
    module_vehicle_encoder_runtime.observation.left_raw_speed_mm_s = 0.0f;
    module_vehicle_encoder_runtime.observation.right_raw_speed_mm_s = 0.0f;
    module_vehicle_encoder_runtime.observation.left_sample_dt_s = 0.0f;
    module_vehicle_encoder_runtime.observation.right_sample_dt_s = 0.0f;
    module_vehicle_encoder_runtime.observation.left_raw_delta_count = 0;
    module_vehicle_encoder_runtime.observation.right_raw_delta_count = 0;
    module_vehicle_encoder_runtime.observation.left_raw_angle = 0u;
    module_vehicle_encoder_runtime.observation.right_raw_angle = 0u;
    module_vehicle_encoder_runtime.observation.wheel_distance_delta_mm = 0.0f;
    module_vehicle_encoder_runtime.observation.theta_accum_rad = 0.0f;
    module_vehicle_encoder_runtime.observation.theta_rad = 0.0f;
    module_vehicle_encoder_runtime.observation.left_sample_count = 0u;
    module_vehicle_encoder_runtime.observation.right_sample_count = 0u;
    module_vehicle_encoder_runtime.observation.left_raw_sample_count = 0u;
    module_vehicle_encoder_runtime.observation.right_raw_sample_count = 0u;
    module_vehicle_encoder_runtime.observation.update_count = 0u;
    module_vehicle_encoder_runtime.observation.encoder_drop_count = 0u;
    module_vehicle_encoder_runtime.observation.left_invalid_sample_count = 0u;
    module_vehicle_encoder_runtime.observation.right_invalid_sample_count = 0u;
    module_vehicle_encoder_runtime.observation.left_invalid_streak_count = 0u;
    module_vehicle_encoder_runtime.observation.right_invalid_streak_count = 0u;
    module_vehicle_encoder_runtime.observation.left_rejected_raw_angle = 0u;
    module_vehicle_encoder_runtime.observation.right_rejected_raw_angle = 0u;
    module_vehicle_encoder_runtime.left_wheel.ready = FALSE;
    module_vehicle_encoder_runtime.right_wheel.ready = FALSE;
    module_vehicle_encoder_runtime.left_wheel.raw_angle = 0u;
    module_vehicle_encoder_runtime.right_wheel.raw_angle = 0u;
    module_vehicle_encoder_runtime.left_wheel.raw_delta_count = 0;
    module_vehicle_encoder_runtime.right_wheel.raw_delta_count = 0;
    module_vehicle_encoder_runtime.left_wheel.raw_speed_mm_s = 0.0f;
    module_vehicle_encoder_runtime.right_wheel.raw_speed_mm_s = 0.0f;
    module_vehicle_encoder_runtime.left_wheel.sample_dt_s = 0.0f;
    module_vehicle_encoder_runtime.right_wheel.sample_dt_s = 0.0f;
    module_vehicle_encoder_runtime.left_wheel.speed_mm_s = 0.0f;
    module_vehicle_encoder_runtime.right_wheel.speed_mm_s = 0.0f;
    module_vehicle_encoder_runtime.left_wheel.invalid_sample_count = 0u;
    module_vehicle_encoder_runtime.right_wheel.invalid_sample_count = 0u;
    module_vehicle_encoder_runtime.left_wheel.invalid_streak_count = 0u;
    module_vehicle_encoder_runtime.right_wheel.invalid_streak_count = 0u;
    module_vehicle_encoder_runtime.left_wheel.rejected_raw_angle = 0u;
    module_vehicle_encoder_runtime.right_wheel.rejected_raw_angle = 0u;
    module_vehicle_encoder_delta_gate_reset(&module_vehicle_encoder_runtime.left_wheel);
    module_vehicle_encoder_delta_gate_reset(&module_vehicle_encoder_runtime.right_wheel);
    module_vehicle_encoder_lowpass_reset(&module_vehicle_encoder_runtime.left_wheel.speed_lpf);
    module_vehicle_encoder_lowpass_reset(&module_vehicle_encoder_runtime.right_wheel.speed_lpf);
}

void module_vehicle_encoder_reset_baseline(void)
{
    const vehicle_encoder_cfg_t* encoder_cfg = vehicle_encoder_cfg_get();

    module_vehicle_encoder_reset();

    module_vehicle_encoder_runtime.left_wheel.last_raw =
        module_vehicle_encoder_raw_angle_from_buffer(encoder_cfg->left_encoder.encoder_id);
    module_vehicle_encoder_runtime.right_wheel.last_raw =
        module_vehicle_encoder_raw_angle_from_buffer(encoder_cfg->right_encoder.encoder_id);
    module_vehicle_encoder_runtime.left_wheel.ready = TRUE;
    module_vehicle_encoder_runtime.right_wheel.ready = TRUE;
}

void module_vehicle_encoder_sample(device_magnetic_encoder_id_t encoder_id,
                                   float32 fallback_dt_s)
{
    const vehicle_encoder_cfg_t* encoder_cfg = vehicle_encoder_cfg_get();
    float32 dt_s = module_vehicle_encoder_dt_get(fallback_dt_s);
    float32 distance_mm = 0.0f;
    float32 speed_mm_s = 0.0f;
    sint16 delta_count = 0;
    boolean valid = FALSE;
    boolean is_left = FALSE;

    if (encoder_id == encoder_cfg->left_encoder.encoder_id)
    {
        module_vehicle_encoder_runtime.observation.left_raw_sample_count++;
        valid = module_vehicle_encoder_wheel_update(&encoder_cfg->left_encoder,
                                                    &module_vehicle_encoder_runtime.left_wheel,
                                                    dt_s,
                                                    &delta_count,
                                                    &distance_mm,
                                                    &speed_mm_s);
        is_left = TRUE;
    }
    else if (encoder_id == encoder_cfg->right_encoder.encoder_id)
    {
        module_vehicle_encoder_runtime.observation.right_raw_sample_count++;
        valid = module_vehicle_encoder_wheel_update(&encoder_cfg->right_encoder,
                                                    &module_vehicle_encoder_runtime.right_wheel,
                                                    dt_s,
                                                    &delta_count,
                                                    &distance_mm,
                                                    &speed_mm_s);
        is_left = FALSE;
    }
    else
    {
        return;
    }

    if (valid != FALSE)
    {
        module_vehicle_encoder_apply_wheel_sample(is_left,
                                                  delta_count,
                                                  distance_mm,
                                                  speed_mm_s);
    }
    else
    {
        module_vehicle_encoder_sync_wheel_diag(is_left);
    }
}

void module_vehicle_encoder_update(float32 fallback_dt_s)
{
    const vehicle_encoder_cfg_t* encoder_cfg = vehicle_encoder_cfg_get();
    float32 dt_s = module_vehicle_encoder_dt_get(fallback_dt_s);
    float32 left_distance_mm = 0.0f;
    float32 right_distance_mm = 0.0f;
    float32 left_speed_mm_s = 0.0f;
    float32 right_speed_mm_s = 0.0f;
    sint16 left_delta_count = 0;
    sint16 right_delta_count = 0;
    boolean left_valid;
    boolean right_valid;

    module_vehicle_encoder_runtime.observation.left_raw_sample_count++;
    module_vehicle_encoder_runtime.observation.right_raw_sample_count++;
    left_valid = module_vehicle_encoder_wheel_update(&encoder_cfg->left_encoder,
                                                     &module_vehicle_encoder_runtime.left_wheel,
                                                     dt_s,
                                                     &left_delta_count,
                                                     &left_distance_mm,
                                                     &left_speed_mm_s);
    right_valid = module_vehicle_encoder_wheel_update(&encoder_cfg->right_encoder,
                                                      &module_vehicle_encoder_runtime.right_wheel,
                                                      dt_s,
                                                      &right_delta_count,
                                                      &right_distance_mm,
                                                      &right_speed_mm_s);

    module_vehicle_encoder_runtime.observation.left_delta_count = left_delta_count;
    module_vehicle_encoder_runtime.observation.right_delta_count = right_delta_count;
    module_vehicle_encoder_runtime.observation.left_raw_delta_count =
        module_vehicle_encoder_runtime.left_wheel.raw_delta_count;
    module_vehicle_encoder_runtime.observation.right_raw_delta_count =
        module_vehicle_encoder_runtime.right_wheel.raw_delta_count;
    module_vehicle_encoder_runtime.observation.left_raw_angle =
        module_vehicle_encoder_runtime.left_wheel.raw_angle;
    module_vehicle_encoder_runtime.observation.right_raw_angle =
        module_vehicle_encoder_runtime.right_wheel.raw_angle;
    module_vehicle_encoder_runtime.observation.left_raw_speed_mm_s =
        module_vehicle_encoder_runtime.left_wheel.raw_speed_mm_s;
    module_vehicle_encoder_runtime.observation.right_raw_speed_mm_s =
        module_vehicle_encoder_runtime.right_wheel.raw_speed_mm_s;
    module_vehicle_encoder_runtime.observation.left_sample_dt_s =
        module_vehicle_encoder_runtime.left_wheel.sample_dt_s;
    module_vehicle_encoder_runtime.observation.right_sample_dt_s =
        module_vehicle_encoder_runtime.right_wheel.sample_dt_s;
    module_vehicle_encoder_sync_wheel_diag(TRUE);
    module_vehicle_encoder_sync_wheel_diag(FALSE);
    module_vehicle_encoder_runtime.observation.left_distance_mm = left_distance_mm;
    module_vehicle_encoder_runtime.observation.right_distance_mm = right_distance_mm;
    module_vehicle_encoder_runtime.observation.left_speed_mm_s = left_speed_mm_s;
    module_vehicle_encoder_runtime.observation.right_speed_mm_s = right_speed_mm_s;

    if ((left_valid != FALSE) && (right_valid != FALSE))
    {
        float32 center_distance_mm = (left_distance_mm + right_distance_mm) * 0.5f;

        module_vehicle_encoder_runtime.observation.speed_mm_s = (left_speed_mm_s + right_speed_mm_s) * 0.5f;
        module_vehicle_encoder_runtime.observation.distance_mm += center_distance_mm;
        module_vehicle_encoder_runtime.observation.left_total_distance_mm += left_distance_mm;
        module_vehicle_encoder_runtime.observation.right_total_distance_mm += right_distance_mm;
        module_vehicle_encoder_runtime.observation.left_total_count += (sint32)left_delta_count;
        module_vehicle_encoder_runtime.observation.right_total_count += (sint32)right_delta_count;
        module_vehicle_encoder_runtime.observation.wheel_distance_delta_mm =
            module_vehicle_encoder_runtime.observation.right_total_distance_mm
            - module_vehicle_encoder_runtime.observation.left_total_distance_mm;
        module_vehicle_encoder_runtime.observation.theta_accum_rad =
            module_vehicle_encoder_runtime.observation.wheel_distance_delta_mm
            / MODULE_VEHICLE_ENCODER_WHEEL_BASE_MM;
        module_vehicle_encoder_runtime.observation.theta_rad =
            algorithm_attitude_wrap_pi(module_vehicle_encoder_runtime.observation.theta_accum_rad);
        module_vehicle_encoder_runtime.observation.update_count++;
    }
}

const module_vehicle_encoder_observation_t* module_vehicle_encoder_observation_get(void)
{
    return &module_vehicle_encoder_runtime.observation;
}

float32 module_vehicle_encoder_wheel_circumference_get(void)
{
    return module_vehicle_encoder_wheel_circumference_mm;
}

void module_vehicle_encoder_wheel_circumference_set(float32 circumference_mm)
{
    module_vehicle_encoder_wheel_circumference_mm = circumference_mm;
}

void module_vehicle_encoder_angle_print_process(void)
{
    static uint32 print_divider = 0u;

#if (MODULE_VEHICLE_ENCODER_ANGLE_PRINT_ENABLE == 0u)
    (void)print_divider;
    return;
#else
    print_divider++;
    if (print_divider < MODULE_VEHICLE_ENCODER_ANGLE_PRINT_PERIOD_MS)
    {
        return;
    }
    print_divider = 0u;

    tools_printf("{encang}%.3f,%.3f,%.3f,%.3f\r\n",
                 (double)module_vehicle_encoder_runtime.observation.left_total_distance_mm,
                 (double)module_vehicle_encoder_runtime.observation.right_total_distance_mm,
                  (double)module_vehicle_encoder_runtime.observation.wheel_distance_delta_mm,
                  (double)(module_vehicle_encoder_runtime.observation.theta_accum_rad
                           * MODULE_VEHICLE_ENCODER_RAD_TO_DEG));
#endif
}

static boolean module_vehicle_encoder_wheel_update(const vehicle_encoder_wheel_cfg_t* wheel_cfg,
                                                   module_vehicle_encoder_wheel_runtime_t* wheel_runtime,
                                                   float32 dt_s,
                                                   sint16* delta_count,
                                                   float32* distance_mm,
                                                   float32* speed_mm_s)
{
    const vehicle_encoder_cfg_t* encoder_cfg = vehicle_encoder_cfg_get();
    uint16 raw_frame = module_vehicle_encoder_raw_frame_from_buffer(wheel_cfg->encoder_id);
    uint16 raw_angle = (uint16)(raw_frame & DEVICE_MAGNETIC_ENCODER_REGISTER_DATA_MASK);
    sint16 delta;
    sint16 directed_delta;
    float32 instant_speed_mm_s;

    wheel_runtime->raw_angle = raw_angle;
    wheel_runtime->sample_dt_s = dt_s;
    if (wheel_runtime->ready == FALSE)
    {
        module_vehicle_encoder_rejected_baseline_advance(wheel_cfg, wheel_runtime);
        wheel_runtime->ready = TRUE;
        module_vehicle_encoder_delta_gate_reset(wheel_runtime);
        module_vehicle_encoder_lowpass_reset(&wheel_runtime->speed_lpf);
        wheel_runtime->raw_delta_count = 0;
        wheel_runtime->raw_speed_mm_s = 0.0f;
        wheel_runtime->speed_mm_s = 0.0f;
        *delta_count = 0;
        *distance_mm = 0.0f;
        *speed_mm_s = wheel_runtime->speed_mm_s;
        return FALSE;
    }

    delta = algorithm_odometry_raw_diff_wrap(raw_angle,
                                             wheel_runtime->last_raw,
                                             VEHICLE_ENCODER_RAW_COUNT_PER_REV,
                                             VEHICLE_ENCODER_RAW_HALF_COUNT);
    if (module_vehicle_encoder_raw_sample_valid(wheel_runtime,
                                                raw_frame,
                                                raw_angle,
                                                delta,
                                                encoder_cfg->encoder_max_delta_count)
        == FALSE)
    {
        module_vehicle_encoder_rejected_baseline_advance(wheel_cfg, wheel_runtime);
        module_vehicle_encoder_sample_reject(wheel_runtime,
                                             raw_angle,
                                             delta,
                                             delta_count,
                                             distance_mm,
                                             speed_mm_s);
        return FALSE;
    }

    wheel_runtime->raw_delta_count = delta;
    directed_delta = module_vehicle_encoder_delta_direction_apply(wheel_cfg, delta);

    if (module_vehicle_encoder_delta_gate_jump_detected(wheel_runtime,
                                                        directed_delta)
        != FALSE)
    {
        wheel_runtime->last_raw = raw_angle;
        module_vehicle_encoder_sample_reject(wheel_runtime,
                                             raw_angle,
                                             delta,
                                             delta_count,
                                             distance_mm,
                                             speed_mm_s);
        return FALSE;
    }

    wheel_runtime->last_raw = raw_angle;
    wheel_runtime->last_valid_delta_count = directed_delta;
    wheel_runtime->delta_gate_ready = TRUE;
    wheel_runtime->invalid_streak_count = 0u;

    *distance_mm = algorithm_odometry_distance_from_count(directed_delta,
                                                        module_vehicle_encoder_wheel_circumference_mm,
                                                          encoder_cfg->encoder_counts_per_rev);
    instant_speed_mm_s = *distance_mm / dt_s;

    wheel_runtime->raw_speed_mm_s = instant_speed_mm_s;
    wheel_runtime->speed_mm_s =
        module_vehicle_encoder_lowpass_update(&wheel_runtime->speed_lpf, instant_speed_mm_s);

    *delta_count = directed_delta;
    *speed_mm_s = wheel_runtime->speed_mm_s;

    return TRUE;
}

static void module_vehicle_encoder_apply_wheel_sample(boolean is_left,
                                                      sint16 delta_count,
                                                      float32 distance_mm,
                                                      float32 speed_mm_s)
{
    module_vehicle_encoder_observation_t* observation = &module_vehicle_encoder_runtime.observation;

    if (is_left != FALSE)
    {
        observation->left_raw_angle = module_vehicle_encoder_runtime.left_wheel.raw_angle;
        observation->left_raw_delta_count = module_vehicle_encoder_runtime.left_wheel.raw_delta_count;
        observation->left_raw_speed_mm_s = module_vehicle_encoder_runtime.left_wheel.raw_speed_mm_s;
        observation->left_sample_dt_s = module_vehicle_encoder_runtime.left_wheel.sample_dt_s;
        observation->left_delta_count = delta_count;
        observation->left_distance_mm = distance_mm;
        observation->left_speed_mm_s = speed_mm_s;
        observation->left_total_distance_mm += distance_mm;
        observation->left_total_count += (sint32)delta_count;
        observation->left_sample_count++;
    }
    else
    {
        observation->right_raw_angle = module_vehicle_encoder_runtime.right_wheel.raw_angle;
        observation->right_raw_delta_count = module_vehicle_encoder_runtime.right_wheel.raw_delta_count;
        observation->right_raw_speed_mm_s = module_vehicle_encoder_runtime.right_wheel.raw_speed_mm_s;
        observation->right_sample_dt_s = module_vehicle_encoder_runtime.right_wheel.sample_dt_s;
        observation->right_delta_count = delta_count;
        observation->right_distance_mm = distance_mm;
        observation->right_speed_mm_s = speed_mm_s;
        observation->right_total_distance_mm += distance_mm;
        observation->right_total_count += (sint32)delta_count;
        observation->right_sample_count++;
    }

    observation->speed_mm_s = (observation->left_speed_mm_s + observation->right_speed_mm_s) * 0.5f;
    observation->distance_mm =
        (observation->left_total_distance_mm + observation->right_total_distance_mm) * 0.5f;
    observation->wheel_distance_delta_mm =
        observation->right_total_distance_mm - observation->left_total_distance_mm;
    observation->theta_accum_rad =
        observation->wheel_distance_delta_mm / MODULE_VEHICLE_ENCODER_WHEEL_BASE_MM;
    observation->theta_rad = algorithm_attitude_wrap_pi(observation->theta_accum_rad);
    observation->update_count++;
    module_vehicle_encoder_sync_wheel_diag(is_left);
}

static void module_vehicle_encoder_sync_wheel_diag(boolean is_left)
{
    module_vehicle_encoder_observation_t* observation = &module_vehicle_encoder_runtime.observation;
    const module_vehicle_encoder_wheel_runtime_t* wheel_runtime;

    if (is_left != FALSE)
    {
        wheel_runtime = &module_vehicle_encoder_runtime.left_wheel;
        observation->left_invalid_sample_count = wheel_runtime->invalid_sample_count;
        observation->left_invalid_streak_count = wheel_runtime->invalid_streak_count;
        observation->left_rejected_raw_angle = wheel_runtime->rejected_raw_angle;
    }
    else
    {
        wheel_runtime = &module_vehicle_encoder_runtime.right_wheel;
        observation->right_invalid_sample_count = wheel_runtime->invalid_sample_count;
        observation->right_invalid_streak_count = wheel_runtime->invalid_streak_count;
        observation->right_rejected_raw_angle = wheel_runtime->rejected_raw_angle;
    }
}

static boolean module_vehicle_encoder_raw_sample_valid(const module_vehicle_encoder_wheel_runtime_t* wheel_runtime,
                                                       uint16 raw_frame,
                                                       uint16 raw_angle,
                                                       sint16 raw_delta_count,
                                                       sint16 max_delta_count)
{
    device_magnetic_encoder_frame_t frame;
    sint32 delta_abs = (sint32)raw_delta_count;
    uint16 zero_guard_high =
        (uint16)(VEHICLE_ENCODER_RAW_COUNT_PER_REV - MODULE_VEHICLE_ENCODER_ZERO_RAW_GUARD_COUNT);

    frame.U = raw_frame;
    if (frame.B.error_flag != 0u)
    {
        return FALSE;
    }

    if (delta_abs < 0)
    {
        delta_abs = -delta_abs;
    }

    if ((max_delta_count > 0) && (delta_abs > (sint32)max_delta_count))
    {
        return FALSE;
    }

    if ((raw_angle == 0u)
        && (wheel_runtime->last_raw > MODULE_VEHICLE_ENCODER_ZERO_RAW_GUARD_COUNT)
        && (wheel_runtime->last_raw < zero_guard_high))
    {
        return FALSE;
    }

    return TRUE;
}

static void module_vehicle_encoder_delta_gate_reset(module_vehicle_encoder_wheel_runtime_t* wheel_runtime)
{
    wheel_runtime->last_valid_delta_count = 0;
    wheel_runtime->delta_gate_ready = FALSE;
}

static sint32 module_vehicle_encoder_delta_gate_limit_get(
    const module_vehicle_encoder_wheel_runtime_t* wheel_runtime)
{
    const vehicle_encoder_cfg_t* encoder_cfg = vehicle_encoder_cfg_get();
    sint32 delta_abs = (sint32)wheel_runtime->last_valid_delta_count;
    sint32 proportional_limit;
    sint32 minimum_limit = (sint32)encoder_cfg->encoder_delta_gate_min_count;

    if (delta_abs < 0)
    {
        delta_abs = -delta_abs;
    }
    proportional_limit = (sint32)((float32)delta_abs * encoder_cfg->encoder_delta_gate_ratio);

    return proportional_limit > minimum_limit ? proportional_limit : minimum_limit;
}

static boolean module_vehicle_encoder_delta_gate_jump_detected(
    const module_vehicle_encoder_wheel_runtime_t* wheel_runtime,
    sint16 delta_count)
{
    sint32 difference;

    if (wheel_runtime->delta_gate_ready == FALSE)
    {
        return FALSE;
    }

    difference = (sint32)delta_count - (sint32)wheel_runtime->last_valid_delta_count;
    if (difference < 0)
    {
        difference = -difference;
    }

    return difference > module_vehicle_encoder_delta_gate_limit_get(wheel_runtime);
}

static sint16 module_vehicle_encoder_delta_direction_apply(const vehicle_encoder_wheel_cfg_t* wheel_cfg,
                                                           sint16 delta_count)
{
    return wheel_cfg->direction < 0 ? (sint16)(-delta_count) : delta_count;
}

static void module_vehicle_encoder_rejected_baseline_advance(
    const vehicle_encoder_wheel_cfg_t* wheel_cfg,
    module_vehicle_encoder_wheel_runtime_t* wheel_runtime)
{
    sint32 predicted_raw;
    sint32 expected_raw_delta;

    if (wheel_runtime->delta_gate_ready == FALSE)
    {
        wheel_runtime->last_raw = wheel_runtime->raw_angle;
        return;
    }

    expected_raw_delta = wheel_cfg->direction < 0
                             ? -(sint32)wheel_runtime->last_valid_delta_count
                             : (sint32)wheel_runtime->last_valid_delta_count;
    predicted_raw = (sint32)wheel_runtime->last_raw + expected_raw_delta;
    if (predicted_raw < 0)
    {
        predicted_raw += (sint32)VEHICLE_ENCODER_RAW_COUNT_PER_REV;
    }
    else if (predicted_raw >= (sint32)VEHICLE_ENCODER_RAW_COUNT_PER_REV)
    {
        predicted_raw -= (sint32)VEHICLE_ENCODER_RAW_COUNT_PER_REV;
    }
    wheel_runtime->last_raw = (uint16)predicted_raw;
}

static void module_vehicle_encoder_sample_reject(module_vehicle_encoder_wheel_runtime_t* wheel_runtime,
                                                 uint16 raw_angle,
                                                 sint16 raw_delta_count,
                                                 sint16* delta_count,
                                                 float32* distance_mm,
                                                 float32* speed_mm_s)
{
    wheel_runtime->raw_angle = raw_angle;
    wheel_runtime->raw_delta_count = raw_delta_count;
    wheel_runtime->raw_speed_mm_s = 0.0f;
    wheel_runtime->invalid_sample_count++;
    wheel_runtime->invalid_streak_count++;
    wheel_runtime->rejected_raw_angle = raw_angle;
    module_vehicle_encoder_runtime.observation.encoder_drop_count++;
    *delta_count = 0;
    *distance_mm = 0.0f;
    *speed_mm_s = wheel_runtime->speed_mm_s;
}

static float32 module_vehicle_encoder_dt_get(float32 fallback_dt_s)
{
    if (fallback_dt_s > 0.0f)
    {
        return fallback_dt_s;
    }

    return 1.0f / vehicle_encoder_cfg_get()->update_frequency_hz;
}

static void module_vehicle_encoder_lowpass_reset(module_vehicle_encoder_lowpass_t* filter)
{
    filter->output = 0.0f;
    filter->ready = FALSE;
}

static float32 module_vehicle_encoder_lowpass_update(module_vehicle_encoder_lowpass_t* filter,
                                                     float32 input)
{
    float32 alpha = vehicle_encoder_cfg_get()->encoder_iir_alpha;

    if (filter->ready == FALSE)
    {
        filter->output = input;
        filter->ready = TRUE;
        return input;
    }

    if (alpha < 0.0f)
    {
        alpha = 0.0f;
    }
    else if (alpha > 1.0f)
    {
        alpha = 1.0f;
    }

    filter->output += alpha * (input - filter->output);

    return filter->output;
}

static uint16 module_vehicle_encoder_raw_frame_from_buffer(device_magnetic_encoder_id_t encoder_id)
{
    device_magnetic_encoder_runtime_t* magnetic_encoder_runtime = device_magnetic_encoder_runtime_table_get();

    return magnetic_encoder_runtime[encoder_id].dma_receive_buffer;
}

static uint16 module_vehicle_encoder_raw_angle_from_buffer(device_magnetic_encoder_id_t encoder_id)
{
    return (uint16)(module_vehicle_encoder_raw_frame_from_buffer(encoder_id)
                    & DEVICE_MAGNETIC_ENCODER_REGISTER_DATA_MASK);
}
