/**
 * @file module_vehicle_pose_fusion.c
 * @brief 车体位姿融合模块实现。
 */

#include "../../../../inc/app/module/module_vehicle_pose_fusion/module_vehicle_pose_fusion.h"

#include "../../../../inc/app/module/module_vehicle_encoder/module_vehicle_encoder.h"
#include "../../../../inc/app/module/module_vehicle_gyro/module_vehicle_gyro.h"
#include "../../../../inc/device/device_int_flash/device_int_flash.h"
#include "../../../../inc/driver/driver_flash/driver_flash.h"
#include "../../../../inc/middleware/algorithm/algorithm_attitude.h"
#include "../../../../inc/middleware/algorithm/algorithm_odometry.h"

#include <math.h>

#define MODULE_VEHICLE_POSE_FUSION_FLASH_MAGIC   ((uint32)0x504F5345u)
#define MODULE_VEHICLE_POSE_FUSION_FLASH_PAGE_LEN ((uint32)IFXFLASH_PFLASH_PAGE_LENGTH)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_BUCKET_COUNT ((uint32)15u)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_MIN_VALID_BUCKET_COUNT ((uint32)10u)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_MIN_CORRECTION_BUCKET_COUNT ((uint32)14u)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_BUCKET_TIME_S (0.010f)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_MIN_SPEED_MM_S (2100.0f)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_EFFECTIVE_WHEEL_BASE_MM (140.0f)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_MIN_ENCODER_TURN_RAD (0.12f)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_FULL_REALIZATION_RATIO (0.84f)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_WEAK_REALIZATION_RATIO (0.90f)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_WEAK_CORRECTION_GAIN (0.40f)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_CORRECTION_GAIN (1.00f)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_RELEASE_MM_PER_TRAVEL_MM (0.002f)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_MAX_TURN_MM (30.0f)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_MAX_REPLAY_MM (300.0f)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_TURN_RELEASE_S (0.10f)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_MIN_BUCKET_DIFF_MM (0.08f)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_STRAIGHT_CONFIRM_S (0.010f)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_STRAIGHT_MIN_SPEED_MM_S (500.0f)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_STRAIGHT_MAX_YAW_RATE_RAD_S (1.50f)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_STRAIGHT_MAX_DIFF_RATE_MM_S (250.0f)

#define MODULE_VEHICLE_POSE_FUSION_SLIP_REJECT_NONE ((uint8)0u)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_REJECT_WINDOW_SHORT ((uint8)1u)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_REJECT_VALID_SHORT ((uint8)2u)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_REJECT_TURN_SMALL ((uint8)3u)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_REJECT_RATIO_OK ((uint8)4u)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_REJECT_EXCESS_INVALID ((uint8)5u)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_REJECT_CORRECTION_LIMIT ((uint8)6u)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_REJECT_CORRECTION_QUALITY ((uint8)7u)

typedef struct
{
    float32 encoder_diff_mm;
    float32 imu_diff_mm;
    float32 excess_mm;
    float32 excess_x_mm;
    float32 excess_y_mm;
    float32 speed_time_mm;
    float32 duration_s;
    sint8 turn_sign;
    boolean valid;
} module_vehicle_pose_fusion_slip_bucket_t;

typedef struct
{
    uint32 magic;
    float32 x_mm;
    float32 y_mm;
    uint32 padding[5];
} module_vehicle_pose_fusion_flash_record_t;

typedef struct
{
    module_vehicle_pose_fusion_observation_t observation;  /**< 对外发布的车体融合位姿观测。*/
    float32 theta_offset_rad;                            /**< Encoder heading offset after pose reset, unit: rad. */
    uint32 last_encoder_update_count;                      /**< 上一次处理的编码器更新计数，单位：次。*/
    float32 last_left_total_distance_mm;
    float32 last_right_total_distance_mm;
    float32 last_gyro_theta_accum_rad;
    module_vehicle_pose_fusion_slip_bucket_t
        slip_buckets[MODULE_VEHICLE_POSE_FUSION_SLIP_BUCKET_COUNT];
    module_vehicle_pose_fusion_slip_bucket_t slip_bucket;
    uint32 slip_bucket_count;
    uint32 slip_bucket_head;
    float32 slip_turn_release_timer_s;
    float32 slip_straight_timer_s;
    boolean replay_path_straight;
    float32 slip_turn_correction_mm;
    float32 slip_pending_x_mm;
    float32 slip_pending_y_mm;
    boolean slip_event_latched;
    sint8 slip_event_turn_sign;
    boolean replay_correction_enabled;
    module_vehicle_pose_fusion_replay_correction_t replay_correction;
} module_vehicle_pose_fusion_runtime_t;

static module_vehicle_pose_fusion_runtime_t module_vehicle_pose_fusion_runtime;
static struct
{
    float32 full_realization_ratio;
    float32 weak_realization_ratio;
    float32 weak_correction_gain;
    float32 correction_gain;
    float32 release_mm_per_travel_mm;
    float32 effective_wheel_base_mm;
    float32 min_encoder_turn_rad;
    float32 max_turn_correction_mm;
    float32 max_replay_correction_mm;
} module_vehicle_pose_fusion_slip_config;

static uint32 module_vehicle_pose_fusion_flash_address_get(void);
static void module_vehicle_pose_fusion_flash_record_pack(const module_vehicle_pose_fusion_flash_record_t* record,
                                                         uint32 (*word_l)[4],
                                                         uint32 (*word_u)[4]);
static void module_vehicle_pose_fusion_replay_correction_reset(boolean active);
static void module_vehicle_pose_fusion_slip_observe(float32 left_step_distance_mm,
                                                    float32 right_step_distance_mm,
                                                    float32 gyro_step_rad,
                                                    float32 heading_rad,
                                                    float32 speed_mm_s,
                                                    float32 yaw_rate_rad_s,
                                                    float32 wheel_diff_rate_mm_s,
                                                    float32 dt_s);
static void module_vehicle_pose_fusion_slip_bucket_finish(void);
static void module_vehicle_pose_fusion_slip_window_evaluate(void);
static void module_vehicle_pose_fusion_slip_pending_apply(float32 traveled_mm);

/**
 * @brief 初始化车体位姿融合模块。
 * @param[in] void 无参数。
 * @return void
 */
void module_vehicle_pose_fusion_init(void)
{
    const module_vehicle_pose_fusion_cfg_t* cfg = module_vehicle_pose_fusion_cfg_get();

    module_vehicle_pose_fusion_runtime.replay_correction_enabled = TRUE;
    module_vehicle_pose_fusion_slip_config.full_realization_ratio =
        MODULE_VEHICLE_POSE_FUSION_SLIP_FULL_REALIZATION_RATIO;
    module_vehicle_pose_fusion_slip_config.weak_realization_ratio =
        MODULE_VEHICLE_POSE_FUSION_SLIP_WEAK_REALIZATION_RATIO;
    module_vehicle_pose_fusion_slip_config.weak_correction_gain =
        MODULE_VEHICLE_POSE_FUSION_SLIP_WEAK_CORRECTION_GAIN;
    module_vehicle_pose_fusion_slip_config.correction_gain =
        MODULE_VEHICLE_POSE_FUSION_SLIP_CORRECTION_GAIN;
    module_vehicle_pose_fusion_slip_config.release_mm_per_travel_mm =
        MODULE_VEHICLE_POSE_FUSION_SLIP_RELEASE_MM_PER_TRAVEL_MM;
    module_vehicle_pose_fusion_slip_config.effective_wheel_base_mm =
        MODULE_VEHICLE_POSE_FUSION_SLIP_EFFECTIVE_WHEEL_BASE_MM;
    module_vehicle_pose_fusion_slip_config.min_encoder_turn_rad =
        MODULE_VEHICLE_POSE_FUSION_SLIP_MIN_ENCODER_TURN_RAD;
    module_vehicle_pose_fusion_slip_config.max_turn_correction_mm =
        MODULE_VEHICLE_POSE_FUSION_SLIP_MAX_TURN_MM;
    module_vehicle_pose_fusion_slip_config.max_replay_correction_mm =
        MODULE_VEHICLE_POSE_FUSION_SLIP_MAX_REPLAY_MM;
    module_vehicle_pose_fusion_reset(cfg->init_x_mm, cfg->init_y_mm, cfg->init_theta_rad);
}

/**
 * @brief 重置车体融合位姿。
 * @param[in] x_mm 初始 X 坐标，单位：毫米。
 * @param[in] y_mm 初始 Y 坐标，单位：毫米。
 * @param[in] theta_rad 初始航向角，单位：弧度。
 * @return void
 */
void module_vehicle_pose_fusion_reset(float32 x_mm, float32 y_mm, float32 theta_rad)
{
    const module_vehicle_encoder_observation_t* encoder_observation = module_vehicle_encoder_observation_get();

    module_vehicle_pose_fusion_runtime.observation.x_mm = x_mm;
    module_vehicle_pose_fusion_runtime.observation.y_mm = y_mm;
    module_vehicle_pose_fusion_runtime.observation.theta_rad = algorithm_attitude_wrap_pi(theta_rad);
    module_vehicle_pose_fusion_runtime.observation.theta_accum_rad = theta_rad;
    module_vehicle_pose_fusion_runtime.theta_offset_rad = theta_rad;
    module_vehicle_pose_fusion_runtime.observation.distance_mm = encoder_observation->distance_mm;
    module_vehicle_pose_fusion_runtime.observation.update_count = 0u;
    module_vehicle_pose_fusion_runtime.last_left_total_distance_mm = encoder_observation->left_total_distance_mm;
    module_vehicle_pose_fusion_runtime.last_right_total_distance_mm = encoder_observation->right_total_distance_mm;
    module_vehicle_pose_fusion_runtime.last_encoder_update_count = encoder_observation->update_count;
    module_vehicle_pose_fusion_runtime.last_gyro_theta_accum_rad =
        module_vehicle_gyro_observation_get()->theta_accum_rad;
    module_vehicle_pose_fusion_replay_correction_reset(FALSE);
}

void module_vehicle_pose_fusion_correct_xy(float32 correction_x_mm, float32 correction_y_mm)
{
    module_vehicle_pose_fusion_runtime.observation.x_mm += correction_x_mm;
    module_vehicle_pose_fusion_runtime.observation.y_mm += correction_y_mm;
}

void module_vehicle_pose_fusion_replay_correction_start(void)
{
    module_vehicle_pose_fusion_replay_correction_reset(
        module_vehicle_pose_fusion_runtime.replay_correction_enabled);
}

void module_vehicle_pose_fusion_replay_correction_stop(void)
{
    module_vehicle_pose_fusion_replay_correction_reset(FALSE);
}

void module_vehicle_pose_fusion_replay_correction_enable_set(boolean enable)
{
    module_vehicle_pose_fusion_runtime.replay_correction_enabled =
        (enable != FALSE) ? TRUE : FALSE;
    if (enable == FALSE)
    {
        module_vehicle_pose_fusion_runtime.replay_correction.active = FALSE;
        module_vehicle_pose_fusion_runtime.replay_correction.slip_confirmed = FALSE;
        module_vehicle_pose_fusion_runtime.replay_correction.straight_release_ready = FALSE;
        module_vehicle_pose_fusion_runtime.replay_correction.pending_correction_mm = 0.0f;
        module_vehicle_pose_fusion_runtime.slip_pending_x_mm = 0.0f;
        module_vehicle_pose_fusion_runtime.slip_pending_y_mm = 0.0f;
    }
}

boolean module_vehicle_pose_fusion_replay_correction_enable_get(void)
{
    return module_vehicle_pose_fusion_runtime.replay_correction_enabled;
}

void module_vehicle_pose_fusion_replay_path_straight_set(boolean straight)
{
    module_vehicle_pose_fusion_runtime.replay_path_straight = straight;
    if (straight == FALSE)
    {
        module_vehicle_pose_fusion_runtime.slip_straight_timer_s = 0.0f;
        module_vehicle_pose_fusion_runtime.replay_correction.straight_release_ready = FALSE;
    }
}

boolean module_vehicle_pose_fusion_replay_path_straight_get(void)
{
    return module_vehicle_pose_fusion_runtime.replay_path_straight;
}

const module_vehicle_pose_fusion_replay_correction_t*
module_vehicle_pose_fusion_replay_correction_get(void)
{
    return &module_vehicle_pose_fusion_runtime.replay_correction;
}

boolean module_vehicle_pose_fusion_slip_parameter_set(uint8 parameter, float32 value)
{
    switch (parameter)
    {
        case MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_FULL_RATIO:
            if ((value < 0.0f) || (value > 1.0f)) return FALSE;
            module_vehicle_pose_fusion_slip_config.full_realization_ratio = value;
            return TRUE;
        case MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_WEAK_RATIO:
            if ((value < 0.0f) || (value > 1.0f)) return FALSE;
            module_vehicle_pose_fusion_slip_config.weak_realization_ratio = value;
            return TRUE;
        case MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_WEAK_GAIN:
            if ((value < 0.0f) || (value > 1.0f)) return FALSE;
            module_vehicle_pose_fusion_slip_config.weak_correction_gain = value;
            return TRUE;
        case MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_GAIN:
            if ((value < 0.0f) || (value > 2.0f)) return FALSE;
            module_vehicle_pose_fusion_slip_config.correction_gain = value;
            return TRUE;
        case MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_RELEASE:
            if ((value < 0.0f) || (value > 1.0f)) return FALSE;
            module_vehicle_pose_fusion_slip_config.release_mm_per_travel_mm = value;
            return TRUE;
        case MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_WHEEL_BASE:
            if ((value < 50.0f) || (value > 500.0f)) return FALSE;
            module_vehicle_pose_fusion_slip_config.effective_wheel_base_mm = value;
            return TRUE;
        case MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_MIN_TURN:
            if ((value < 0.0f) || (value > 3.0f)) return FALSE;
            module_vehicle_pose_fusion_slip_config.min_encoder_turn_rad = value;
            return TRUE;
        case MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_MAX_TURN:
            if ((value < 0.0f) || (value > 500.0f)) return FALSE;
            module_vehicle_pose_fusion_slip_config.max_turn_correction_mm = value;
            return TRUE;
        case MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_MAX_REPLAY:
            if ((value < 0.0f) || (value > 2000.0f)) return FALSE;
            module_vehicle_pose_fusion_slip_config.max_replay_correction_mm = value;
            return TRUE;
        default:
            return FALSE;
    }
}

float32 module_vehicle_pose_fusion_slip_parameter_get(uint8 parameter)
{
    switch (parameter)
    {
        case MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_FULL_RATIO:
            return module_vehicle_pose_fusion_slip_config.full_realization_ratio;
        case MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_WEAK_RATIO:
            return module_vehicle_pose_fusion_slip_config.weak_realization_ratio;
        case MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_WEAK_GAIN:
            return module_vehicle_pose_fusion_slip_config.weak_correction_gain;
        case MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_GAIN:
            return module_vehicle_pose_fusion_slip_config.correction_gain;
        case MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_RELEASE:
            return module_vehicle_pose_fusion_slip_config.release_mm_per_travel_mm;
        case MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_WHEEL_BASE:
            return module_vehicle_pose_fusion_slip_config.effective_wheel_base_mm;
        case MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_MIN_TURN:
            return module_vehicle_pose_fusion_slip_config.min_encoder_turn_rad;
        case MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_MAX_TURN:
            return module_vehicle_pose_fusion_slip_config.max_turn_correction_mm;
        case MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_MAX_REPLAY:
            return module_vehicle_pose_fusion_slip_config.max_replay_correction_mm;
        default:
            return 0.0f;
    }
}

/**
 * @brief 根据编码器里程与陀螺仪航向推进一次位姿融合。
 * @param[in] void 无参数。
 * @return void
 */
void module_vehicle_pose_fusion_update(void)
{
    module_vehicle_pose_fusion_calculate();
}

void module_vehicle_pose_fusion_calculate(void)
{
    const module_vehicle_pose_fusion_cfg_t* cfg = module_vehicle_pose_fusion_cfg_get();
    const module_vehicle_encoder_observation_t* encoder_observation = module_vehicle_encoder_observation_get();
    const module_vehicle_gyro_observation_t* gyro_observation = module_vehicle_gyro_observation_get();
    float32 fused_distance_mm = encoder_observation->distance_mm;
    float32 left_weight = cfg->left_distance_weight;
    float32 right_weight = cfg->right_distance_weight;
    float32 weight_sum;
    float32 left_step_distance_mm;
    float32 right_step_distance_mm;
    float32 step_distance_mm;
    float32 gyro_step_rad;
    float32 sample_dt_s;

    if (encoder_observation->update_count == module_vehicle_pose_fusion_runtime.last_encoder_update_count)
    {
        return;
    }

    if (left_weight < 0.0f)
    {
        left_weight = 0.0f;
    }

    if (right_weight < 0.0f)
    {
        right_weight = 0.0f;
    }

    weight_sum = left_weight + right_weight;
    if (weight_sum <= 0.0f)
    {
        left_weight = 0.5f;
        right_weight = 0.5f;
        weight_sum = 1.0f;
    }

    module_vehicle_pose_fusion_runtime.observation.theta_accum_rad =
        module_vehicle_pose_fusion_runtime.theta_offset_rad + gyro_observation->theta_accum_rad;
    module_vehicle_pose_fusion_runtime.observation.theta_rad =
        algorithm_attitude_wrap_pi(module_vehicle_pose_fusion_runtime.observation.theta_accum_rad);
    module_vehicle_pose_fusion_runtime.observation.distance_mm = fused_distance_mm;

    module_vehicle_pose_fusion_runtime.last_encoder_update_count = encoder_observation->update_count;
    left_step_distance_mm = encoder_observation->left_total_distance_mm
                          - module_vehicle_pose_fusion_runtime.last_left_total_distance_mm;
    right_step_distance_mm = encoder_observation->right_total_distance_mm
                           - module_vehicle_pose_fusion_runtime.last_right_total_distance_mm;
    module_vehicle_pose_fusion_runtime.last_left_total_distance_mm =
        encoder_observation->left_total_distance_mm;
    module_vehicle_pose_fusion_runtime.last_right_total_distance_mm =
        encoder_observation->right_total_distance_mm;
    gyro_step_rad = gyro_observation->theta_accum_rad
                  - module_vehicle_pose_fusion_runtime.last_gyro_theta_accum_rad;
    module_vehicle_pose_fusion_runtime.last_gyro_theta_accum_rad =
        gyro_observation->theta_accum_rad;
    step_distance_mm = ((left_step_distance_mm * left_weight)
                      + (right_step_distance_mm * right_weight))
                     / weight_sum;
    algorithm_odometry_integrate_pose(&module_vehicle_pose_fusion_runtime.observation.x_mm,
                                      &module_vehicle_pose_fusion_runtime.observation.y_mm,
                                      &fused_distance_mm,
                                      module_vehicle_pose_fusion_runtime.observation.theta_rad,
                                      step_distance_mm);
    sample_dt_s = 0.5f * (encoder_observation->left_sample_dt_s
                        + encoder_observation->right_sample_dt_s);
    if ((sample_dt_s <= 0.0f) || (sample_dt_s > 0.01f))
    {
        sample_dt_s = gyro_observation->imu_dt_s;
    }
    if ((sample_dt_s <= 0.0f) || (sample_dt_s > 0.01f))
    {
        sample_dt_s = 0.0005208333f;
    }
    module_vehicle_pose_fusion_slip_observe(left_step_distance_mm,
                                            right_step_distance_mm,
                                            gyro_step_rad,
                                            module_vehicle_pose_fusion_runtime.observation.theta_rad,
                                            encoder_observation->speed_mm_s,
                                            gyro_observation->gyro_z_rad_s,
                                            encoder_observation->right_speed_mm_s
                                                - encoder_observation->left_speed_mm_s,
                                            sample_dt_s);
    module_vehicle_pose_fusion_slip_pending_apply(fabsf(step_distance_mm));
    module_vehicle_pose_fusion_runtime.observation.distance_mm =
        encoder_observation->distance_mm
        - module_vehicle_pose_fusion_runtime.replay_correction.total_correction_mm;
    module_vehicle_pose_fusion_runtime.observation.update_count++;
}

static void module_vehicle_pose_fusion_replay_correction_reset(boolean active)
{
    const module_vehicle_encoder_observation_t* encoder_observation =
        module_vehicle_encoder_observation_get();
    uint32 index;

    for (index = 0u; index < MODULE_VEHICLE_POSE_FUSION_SLIP_BUCKET_COUNT; index++)
    {
        module_vehicle_pose_fusion_runtime.slip_buckets[index].valid = FALSE;
    }
    module_vehicle_pose_fusion_runtime.slip_bucket.encoder_diff_mm = 0.0f;
    module_vehicle_pose_fusion_runtime.slip_bucket.imu_diff_mm = 0.0f;
    module_vehicle_pose_fusion_runtime.slip_bucket.excess_mm = 0.0f;
    module_vehicle_pose_fusion_runtime.slip_bucket.excess_x_mm = 0.0f;
    module_vehicle_pose_fusion_runtime.slip_bucket.excess_y_mm = 0.0f;
    module_vehicle_pose_fusion_runtime.slip_bucket.speed_time_mm = 0.0f;
    module_vehicle_pose_fusion_runtime.slip_bucket.duration_s = 0.0f;
    module_vehicle_pose_fusion_runtime.slip_bucket.turn_sign = 0;
    module_vehicle_pose_fusion_runtime.slip_bucket.valid = TRUE;
    module_vehicle_pose_fusion_runtime.slip_bucket_count = 0u;
    module_vehicle_pose_fusion_runtime.slip_bucket_head = 0u;
    module_vehicle_pose_fusion_runtime.slip_turn_release_timer_s = 0.0f;
    module_vehicle_pose_fusion_runtime.slip_straight_timer_s = 0.0f;
    module_vehicle_pose_fusion_runtime.replay_path_straight = FALSE;
    module_vehicle_pose_fusion_runtime.slip_turn_correction_mm = 0.0f;
    module_vehicle_pose_fusion_runtime.slip_pending_x_mm = 0.0f;
    module_vehicle_pose_fusion_runtime.slip_pending_y_mm = 0.0f;
    module_vehicle_pose_fusion_runtime.slip_event_latched = FALSE;
    module_vehicle_pose_fusion_runtime.slip_event_turn_sign = 0;
    module_vehicle_pose_fusion_runtime.replay_correction.active = active;
    module_vehicle_pose_fusion_runtime.replay_correction.slip_confirmed = FALSE;
    module_vehicle_pose_fusion_runtime.replay_correction.straight_release_ready = FALSE;
    module_vehicle_pose_fusion_runtime.replay_correction.slip_window_ready = FALSE;
    module_vehicle_pose_fusion_runtime.replay_correction.slip_window_bucket_count = 0u;
    module_vehicle_pose_fusion_runtime.replay_correction.slip_window_valid_count = 0u;
    module_vehicle_pose_fusion_runtime.replay_correction.slip_reject_reason =
        MODULE_VEHICLE_POSE_FUSION_SLIP_REJECT_WINDOW_SHORT;
    module_vehicle_pose_fusion_runtime.replay_correction.slip_turn_sign = 0;
    module_vehicle_pose_fusion_runtime.replay_correction.encoder_turn_rad = 0.0f;
    module_vehicle_pose_fusion_runtime.replay_correction.imu_turn_rad = 0.0f;
    module_vehicle_pose_fusion_runtime.replay_correction.yaw_realization_ratio = 1.0f;
    module_vehicle_pose_fusion_runtime.replay_correction.slip_window_duration_s = 0.0f;
    module_vehicle_pose_fusion_runtime.replay_correction.pending_correction_mm = 0.0f;
    module_vehicle_pose_fusion_runtime.replay_correction.total_correction_mm = 0.0f;
    module_vehicle_pose_fusion_runtime.replay_correction.correction_x_mm = 0.0f;
    module_vehicle_pose_fusion_runtime.replay_correction.correction_y_mm = 0.0f;
    module_vehicle_pose_fusion_runtime.observation.distance_mm =
        encoder_observation->distance_mm;
    module_vehicle_pose_fusion_runtime.last_gyro_theta_accum_rad =
        module_vehicle_gyro_observation_get()->theta_accum_rad;
}

static void module_vehicle_pose_fusion_slip_observe(float32 left_step_distance_mm,
                                                    float32 right_step_distance_mm,
                                                    float32 gyro_step_rad,
                                                    float32 heading_rad,
                                                    float32 speed_mm_s,
                                                    float32 yaw_rate_rad_s,
                                                    float32 wheel_diff_rate_mm_s,
                                                    float32 dt_s)
{
    module_vehicle_pose_fusion_slip_bucket_t* bucket =
        &module_vehicle_pose_fusion_runtime.slip_bucket;
    float32 encoder_diff_mm;
    float32 imu_diff_mm;
    float32 residual_mm;
    float32 speed_abs_mm_s = fabsf(speed_mm_s);
    float32 yaw_rate_abs_rad_s;
    float32 wheel_diff_rate_abs_mm_s;

    if (module_vehicle_pose_fusion_runtime.replay_correction.active == FALSE)
    {
        return;
    }

    encoder_diff_mm = right_step_distance_mm - left_step_distance_mm;
    imu_diff_mm = module_vehicle_pose_fusion_slip_config.effective_wheel_base_mm * gyro_step_rad;
    yaw_rate_abs_rad_s = fabsf(yaw_rate_rad_s);
    wheel_diff_rate_abs_mm_s = fabsf(wheel_diff_rate_mm_s);
    if ((module_vehicle_pose_fusion_runtime.replay_path_straight != FALSE)
        && (speed_abs_mm_s >= MODULE_VEHICLE_POSE_FUSION_SLIP_STRAIGHT_MIN_SPEED_MM_S)
        && (yaw_rate_abs_rad_s <= MODULE_VEHICLE_POSE_FUSION_SLIP_STRAIGHT_MAX_YAW_RATE_RAD_S)
        && (wheel_diff_rate_abs_mm_s
            <= MODULE_VEHICLE_POSE_FUSION_SLIP_STRAIGHT_MAX_DIFF_RATE_MM_S))
    {
        module_vehicle_pose_fusion_runtime.slip_straight_timer_s += dt_s;
        if (module_vehicle_pose_fusion_runtime.slip_straight_timer_s
            >= MODULE_VEHICLE_POSE_FUSION_SLIP_STRAIGHT_CONFIRM_S)
        {
            module_vehicle_pose_fusion_runtime.replay_correction.straight_release_ready = TRUE;
        }
    }
    else
    {
        module_vehicle_pose_fusion_runtime.slip_straight_timer_s = 0.0f;
        module_vehicle_pose_fusion_runtime.replay_correction.straight_release_ready = FALSE;
    }
    residual_mm = encoder_diff_mm - imu_diff_mm;
    bucket->excess_mm += residual_mm;
    bucket->excess_x_mm += residual_mm * cosf(heading_rad);
    bucket->excess_y_mm += residual_mm * sinf(heading_rad);
    bucket->encoder_diff_mm += encoder_diff_mm;
    bucket->imu_diff_mm += imu_diff_mm;
    bucket->speed_time_mm += speed_abs_mm_s * dt_s;
    bucket->duration_s += dt_s;

    if (bucket->duration_s >= MODULE_VEHICLE_POSE_FUSION_SLIP_BUCKET_TIME_S)
    {
        module_vehicle_pose_fusion_slip_bucket_finish();
    }
}

static void module_vehicle_pose_fusion_slip_bucket_finish(void)
{
    module_vehicle_pose_fusion_slip_bucket_t* bucket =
        &module_vehicle_pose_fusion_runtime.slip_bucket;
    float32 average_speed_mm_s = bucket->speed_time_mm / bucket->duration_s;
    sint8 encoder_sign = (bucket->encoder_diff_mm > 0.0f) ? 1 : -1;

    bucket->turn_sign = encoder_sign;
    bucket->excess_mm *= (float32)encoder_sign;
    bucket->excess_x_mm *= (float32)encoder_sign;
    bucket->excess_y_mm *= (float32)encoder_sign;

    if ((fabsf(bucket->encoder_diff_mm) < MODULE_VEHICLE_POSE_FUSION_SLIP_MIN_BUCKET_DIFF_MM)
        || (average_speed_mm_s < MODULE_VEHICLE_POSE_FUSION_SLIP_MIN_SPEED_MM_S))
    {
        bucket->valid = FALSE;
    }

    if (bucket->valid == FALSE)
    {
        module_vehicle_pose_fusion_runtime.slip_turn_release_timer_s += bucket->duration_s;
        if (module_vehicle_pose_fusion_runtime.slip_turn_release_timer_s
            >= MODULE_VEHICLE_POSE_FUSION_SLIP_TURN_RELEASE_S)
        {
            module_vehicle_pose_fusion_runtime.slip_turn_correction_mm = 0.0f;
            module_vehicle_pose_fusion_runtime.replay_correction.slip_confirmed = FALSE;
            module_vehicle_pose_fusion_runtime.slip_event_latched = FALSE;
            module_vehicle_pose_fusion_runtime.slip_event_turn_sign = 0;
        }
    }
    else
    {
        module_vehicle_pose_fusion_runtime.slip_turn_release_timer_s = 0.0f;
    }

    module_vehicle_pose_fusion_runtime.slip_buckets[
        module_vehicle_pose_fusion_runtime.slip_bucket_head] = *bucket;
    module_vehicle_pose_fusion_runtime.slip_bucket_head++;
    if (module_vehicle_pose_fusion_runtime.slip_bucket_head
        >= MODULE_VEHICLE_POSE_FUSION_SLIP_BUCKET_COUNT)
    {
        module_vehicle_pose_fusion_runtime.slip_bucket_head = 0u;
    }
    if (module_vehicle_pose_fusion_runtime.slip_bucket_count
        < MODULE_VEHICLE_POSE_FUSION_SLIP_BUCKET_COUNT)
    {
        module_vehicle_pose_fusion_runtime.slip_bucket_count++;
    }
    module_vehicle_pose_fusion_runtime.replay_correction.slip_window_bucket_count =
        (uint8)module_vehicle_pose_fusion_runtime.slip_bucket_count;
    if (module_vehicle_pose_fusion_runtime.slip_bucket_count
        >= MODULE_VEHICLE_POSE_FUSION_SLIP_BUCKET_COUNT)
    {
        module_vehicle_pose_fusion_slip_window_evaluate();
    }
    else
    {
        module_vehicle_pose_fusion_runtime.replay_correction.slip_window_ready = FALSE;
        module_vehicle_pose_fusion_runtime.replay_correction.slip_reject_reason =
            MODULE_VEHICLE_POSE_FUSION_SLIP_REJECT_WINDOW_SHORT;
    }

    bucket->encoder_diff_mm = 0.0f;
    bucket->imu_diff_mm = 0.0f;
    bucket->excess_mm = 0.0f;
    bucket->excess_x_mm = 0.0f;
    bucket->excess_y_mm = 0.0f;
    bucket->speed_time_mm = 0.0f;
    bucket->duration_s = 0.0f;
    bucket->turn_sign = 0;
    bucket->valid = TRUE;
}

static void module_vehicle_pose_fusion_slip_window_evaluate(void)
{
    float32 encoder_diff_mm = 0.0f;
    float32 imu_diff_mm = 0.0f;
    float32 excess_mm = 0.0f;
    float32 excess_x_mm = 0.0f;
    float32 excess_y_mm = 0.0f;
    float32 realization_ratio;
    float32 correction_mm;
    float32 correction_scale;
    float32 remaining_turn_mm;
    float32 remaining_replay_mm;
    float32 positive_turn_weight_mm = 0.0f;
    float32 negative_turn_weight_mm = 0.0f;
    float32 window_duration_s = 0.0f;
    sint8 turn_sign;
    uint32 valid_bucket_count = 0u;
    uint32 index;

    for (index = 0u; index < MODULE_VEHICLE_POSE_FUSION_SLIP_BUCKET_COUNT; index++)
    {
        const module_vehicle_pose_fusion_slip_bucket_t* bucket =
            &module_vehicle_pose_fusion_runtime.slip_buckets[index];

        window_duration_s += bucket->duration_s;
        if (bucket->valid == FALSE)
        {
            continue;
        }
        if (bucket->turn_sign > 0)
        {
            positive_turn_weight_mm += fabsf(bucket->encoder_diff_mm);
        }
        else
        {
            negative_turn_weight_mm += fabsf(bucket->encoder_diff_mm);
        }
    }
    turn_sign = (positive_turn_weight_mm >= negative_turn_weight_mm) ? 1 : -1;

    for (index = 0u; index < MODULE_VEHICLE_POSE_FUSION_SLIP_BUCKET_COUNT; index++)
    {
        const module_vehicle_pose_fusion_slip_bucket_t* bucket =
            &module_vehicle_pose_fusion_runtime.slip_buckets[index];

        if ((bucket->valid == FALSE) || (bucket->turn_sign != turn_sign))
        {
            continue;
        }
        valid_bucket_count++;
        encoder_diff_mm += bucket->encoder_diff_mm;
        imu_diff_mm += bucket->imu_diff_mm;
        excess_mm += bucket->excess_mm;
        excess_x_mm += bucket->excess_x_mm;
        excess_y_mm += bucket->excess_y_mm;
    }

    module_vehicle_pose_fusion_runtime.replay_correction.slip_window_valid_count =
        (uint8)valid_bucket_count;
    module_vehicle_pose_fusion_runtime.replay_correction.slip_window_duration_s =
        window_duration_s;
    module_vehicle_pose_fusion_runtime.replay_correction.slip_turn_sign = turn_sign;
    module_vehicle_pose_fusion_runtime.replay_correction.slip_window_ready = FALSE;

    if (valid_bucket_count < MODULE_VEHICLE_POSE_FUSION_SLIP_MIN_VALID_BUCKET_COUNT)
    {
        module_vehicle_pose_fusion_runtime.replay_correction.slip_reject_reason =
            MODULE_VEHICLE_POSE_FUSION_SLIP_REJECT_VALID_SHORT;
        return;
    }

    module_vehicle_pose_fusion_runtime.replay_correction.encoder_turn_rad =
        encoder_diff_mm / module_vehicle_pose_fusion_slip_config.effective_wheel_base_mm;
    module_vehicle_pose_fusion_runtime.replay_correction.imu_turn_rad =
        imu_diff_mm / module_vehicle_pose_fusion_slip_config.effective_wheel_base_mm;
    realization_ratio = (fabsf(encoder_diff_mm) > 0.001f)
                      ? (imu_diff_mm / encoder_diff_mm)
                      : 1.0f;
    module_vehicle_pose_fusion_runtime.replay_correction.yaw_realization_ratio =
        realization_ratio;

    if ((fabsf(module_vehicle_pose_fusion_runtime.replay_correction.encoder_turn_rad)
         < module_vehicle_pose_fusion_slip_config.min_encoder_turn_rad))
    {
        module_vehicle_pose_fusion_runtime.replay_correction.slip_reject_reason =
            MODULE_VEHICLE_POSE_FUSION_SLIP_REJECT_TURN_SMALL;
        return;
    }
    module_vehicle_pose_fusion_runtime.replay_correction.slip_window_ready = TRUE;
    if ((module_vehicle_pose_fusion_runtime.slip_event_turn_sign != 0)
        && (module_vehicle_pose_fusion_runtime.slip_event_turn_sign != turn_sign))
    {
        module_vehicle_pose_fusion_runtime.slip_turn_correction_mm = 0.0f;
        module_vehicle_pose_fusion_runtime.slip_event_latched = FALSE;
        module_vehicle_pose_fusion_runtime.replay_correction.slip_confirmed = FALSE;
    }
    module_vehicle_pose_fusion_runtime.slip_event_turn_sign = turn_sign;
    if (realization_ratio >= module_vehicle_pose_fusion_slip_config.weak_realization_ratio)
    {
        module_vehicle_pose_fusion_runtime.replay_correction.slip_reject_reason =
            MODULE_VEHICLE_POSE_FUSION_SLIP_REJECT_RATIO_OK;
        return;
    }
    if (excess_mm <= 0.0f)
    {
        module_vehicle_pose_fusion_runtime.replay_correction.slip_reject_reason =
            MODULE_VEHICLE_POSE_FUSION_SLIP_REJECT_EXCESS_INVALID;
        return;
    }

    module_vehicle_pose_fusion_runtime.replay_correction.slip_reject_reason =
        MODULE_VEHICLE_POSE_FUSION_SLIP_REJECT_NONE;
    module_vehicle_pose_fusion_runtime.replay_correction.slip_confirmed = TRUE;
    if ((valid_bucket_count
         < MODULE_VEHICLE_POSE_FUSION_SLIP_MIN_CORRECTION_BUCKET_COUNT)
        || (realization_ratio <= 0.0f))
    {
        module_vehicle_pose_fusion_runtime.replay_correction.slip_reject_reason =
            MODULE_VEHICLE_POSE_FUSION_SLIP_REJECT_CORRECTION_QUALITY;
        return;
    }
    if (module_vehicle_pose_fusion_runtime.slip_event_latched != FALSE)
    {
        return;
    }

    correction_mm = 0.5f * module_vehicle_pose_fusion_slip_config.correction_gain
                  * (fabsf(encoder_diff_mm) - fabsf(imu_diff_mm));
    if (realization_ratio >= module_vehicle_pose_fusion_slip_config.full_realization_ratio)
    {
        correction_mm *= module_vehicle_pose_fusion_slip_config.weak_correction_gain;
    }
    if (correction_mm <= 0.0f)
    {
        module_vehicle_pose_fusion_runtime.replay_correction.slip_reject_reason =
            MODULE_VEHICLE_POSE_FUSION_SLIP_REJECT_CORRECTION_LIMIT;
        return;
    }
    remaining_turn_mm = module_vehicle_pose_fusion_slip_config.max_turn_correction_mm
                      - module_vehicle_pose_fusion_runtime.slip_turn_correction_mm;
    remaining_replay_mm = module_vehicle_pose_fusion_slip_config.max_replay_correction_mm
                        - (module_vehicle_pose_fusion_runtime.replay_correction.total_correction_mm
                           + module_vehicle_pose_fusion_runtime.replay_correction.pending_correction_mm);
    if (correction_mm > remaining_turn_mm)
    {
        correction_mm = remaining_turn_mm;
    }
    if (correction_mm > remaining_replay_mm)
    {
        correction_mm = remaining_replay_mm;
    }
    if (correction_mm <= 0.0f)
    {
        module_vehicle_pose_fusion_runtime.replay_correction.slip_reject_reason =
            MODULE_VEHICLE_POSE_FUSION_SLIP_REJECT_CORRECTION_LIMIT;
        return;
    }

    correction_scale = correction_mm / excess_mm;
    module_vehicle_pose_fusion_runtime.slip_pending_x_mm += excess_x_mm * correction_scale;
    module_vehicle_pose_fusion_runtime.slip_pending_y_mm += excess_y_mm * correction_scale;
    module_vehicle_pose_fusion_runtime.replay_correction.pending_correction_mm += correction_mm;
    module_vehicle_pose_fusion_runtime.slip_turn_correction_mm += correction_mm;
    module_vehicle_pose_fusion_runtime.slip_event_latched = TRUE;
}

static void module_vehicle_pose_fusion_slip_pending_apply(float32 traveled_mm)
{
    float32 pending_mm =
        module_vehicle_pose_fusion_runtime.replay_correction.pending_correction_mm;
    float32 apply_mm;
    float32 scale;
    float32 correction_x_mm;
    float32 correction_y_mm;

    if ((module_vehicle_pose_fusion_runtime.replay_correction.active == FALSE)
        || (pending_mm <= 0.0f)
        || (traveled_mm <= 0.0f))
    {
        return;
    }

    apply_mm = traveled_mm * module_vehicle_pose_fusion_slip_config.release_mm_per_travel_mm;
    apply_mm = (pending_mm > apply_mm)
             ? apply_mm
             : pending_mm;
    scale = apply_mm / pending_mm;
    correction_x_mm = module_vehicle_pose_fusion_runtime.slip_pending_x_mm * scale;
    correction_y_mm = module_vehicle_pose_fusion_runtime.slip_pending_y_mm * scale;
    module_vehicle_pose_fusion_runtime.observation.x_mm -= correction_x_mm;
    module_vehicle_pose_fusion_runtime.observation.y_mm -= correction_y_mm;
    module_vehicle_pose_fusion_runtime.slip_pending_x_mm -= correction_x_mm;
    module_vehicle_pose_fusion_runtime.slip_pending_y_mm -= correction_y_mm;
    module_vehicle_pose_fusion_runtime.replay_correction.pending_correction_mm -= apply_mm;
    module_vehicle_pose_fusion_runtime.replay_correction.total_correction_mm += apply_mm;
    module_vehicle_pose_fusion_runtime.replay_correction.correction_x_mm += correction_x_mm;
    module_vehicle_pose_fusion_runtime.replay_correction.correction_y_mm += correction_y_mm;
}

/**
 * @brief 获取当前车体融合位姿观测。
 * @param[in] void 无参数。
 * @return 只读车体融合位姿观测指针。
 */
/* Save current pose to the reserved internal flash page. */
boolean module_vehicle_pose_fusion_save_to_flash(void)
{
    module_vehicle_pose_fusion_flash_record_t record;
    uint32 word_l[4];
    uint32 word_u[4];
    uint32 flash_address = module_vehicle_pose_fusion_flash_address_get();
    uint32 index;

    record.magic = MODULE_VEHICLE_POSE_FUSION_FLASH_MAGIC;
    record.x_mm = module_vehicle_pose_fusion_runtime.observation.x_mm;
    record.y_mm = module_vehicle_pose_fusion_runtime.observation.y_mm;
    for (index = 0u; index < 5u; index++)
    {
        record.padding[index] = 0xFFFFFFFFu;
    }

    module_vehicle_pose_fusion_flash_record_pack(&record, &word_l, &word_u);

    driver_flash_eraseSector(flash_address);

    return device_int_flash_writePFlashPageData(DEVICE_INT_FLASH_1,
                                                flash_address,
                                                &word_l,
                                                &word_u);
}

boolean module_vehicle_pose_fusion_load_from_flash(void)
{
    module_vehicle_pose_fusion_flash_record_t record;

    device_int_flash_read(DEVICE_INT_FLASH_1,
                          module_vehicle_pose_fusion_flash_address_get(),
                          (uint8*)&record,
                          (uint32)sizeof(record));

    if (record.magic != MODULE_VEHICLE_POSE_FUSION_FLASH_MAGIC)
    {
        return FALSE;
    }

    module_vehicle_pose_fusion_runtime.observation.x_mm = record.x_mm;
    module_vehicle_pose_fusion_runtime.observation.y_mm = record.y_mm;
    return TRUE;
}

/* Get current fused vehicle pose observation. */
const module_vehicle_pose_fusion_observation_t* module_vehicle_pose_fusion_observation_get(void)
{
    return &module_vehicle_pose_fusion_runtime.observation;
}

/**
 * @brief 获取当前融合航向角。
 * @param[in] void 无参数。
 * @return 当前融合航向角，单位：弧度。
 */
float32 module_vehicle_pose_fusion_heading_get(void)
{
    return module_vehicle_pose_fusion_runtime.observation.theta_rad;
}

static uint32 module_vehicle_pose_fusion_flash_address_get(void)
{
    device_int_flash_runtime_t* runtime = device_int_flash_runtime_table_get();

    return device_int_flash_start_address_get(DEVICE_INT_FLASH_1)
           + runtime[DEVICE_INT_FLASH_1].flash_runtime.group_capacity
           - MODULE_VEHICLE_POSE_FUSION_FLASH_PAGE_LEN;
}

static void module_vehicle_pose_fusion_flash_record_pack(const module_vehicle_pose_fusion_flash_record_t* record,
                                                         uint32 (*word_l)[4],
                                                         uint32 (*word_u)[4])
{
    const uint8* data = (const uint8*)record;
    uint32 index;

    for (index = 0u; index < 4u; index++)
    {
        (*word_l)[index] = ((uint32)data[index * 8u])
                         | (((uint32)data[(index * 8u) + 1u]) << 8u)
                         | (((uint32)data[(index * 8u) + 2u]) << 16u)
                         | (((uint32)data[(index * 8u) + 3u]) << 24u);

        (*word_u)[index] = ((uint32)data[(index * 8u) + 4u])
                         | (((uint32)data[(index * 8u) + 5u]) << 8u)
                         | (((uint32)data[(index * 8u) + 6u]) << 16u)
                         | (((uint32)data[(index * 8u) + 7u]) << 24u);
    }
}
