/**
 * @file module_vehicle_pose_fusion.h
 * @brief 车体位姿融合模块接口。
 */

#ifndef MAD_CIRCUITS_APP_MODULE_MODULE_VEHICLE_POSE_FUSION_H
#define MAD_CIRCUITS_APP_MODULE_MODULE_VEHICLE_POSE_FUSION_H

#include "Ifx_Types.h"

#include "../../../../config/app/module/module_vehicle_pose_fusion/module_vehicle_pose_fusion_cfg.h"

typedef struct
{
    float32 x_mm;          /**< 当前融合 X 坐标，单位：毫米。*/
    float32 y_mm;          /**< 当前融合 Y 坐标，单位：毫米。*/
    float32 theta_rad;     /**< 当前融合航向角，单位：弧度。*/
    float32 theta_accum_rad; /**< Encoder differential accumulated heading, unit: rad. */
    float32 distance_mm;   /**< 当前融合累计里程，单位：毫米。*/
    uint32 update_count;   /**< 有效位姿融合更新次数，单位：次。*/
} module_vehicle_pose_fusion_observation_t;

typedef struct
{
    boolean active;
    boolean slip_confirmed;
    boolean straight_release_ready;
    boolean slip_window_ready;
    uint8 slip_window_bucket_count;
    uint8 slip_window_valid_count;
    uint8 slip_reject_reason;
    sint8 slip_turn_sign;
    float32 encoder_turn_rad;
    float32 imu_turn_rad;
    float32 yaw_realization_ratio;
    float32 slip_window_duration_s;
    float32 pending_correction_mm;
    float32 total_correction_mm;
    float32 correction_x_mm;
    float32 correction_y_mm;
} module_vehicle_pose_fusion_replay_correction_t;

#define MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_FULL_RATIO ((uint8)0u)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_WEAK_RATIO ((uint8)1u)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_WEAK_GAIN ((uint8)2u)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_GAIN ((uint8)3u)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_RELEASE ((uint8)4u)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_WHEEL_BASE ((uint8)5u)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_MIN_TURN ((uint8)6u)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_MAX_TURN ((uint8)7u)
#define MODULE_VEHICLE_POSE_FUSION_SLIP_PARAM_MAX_REPLAY ((uint8)8u)

/**
 * @brief 初始化车体位姿融合模块。
 * @param[in] void 无参数。
 * @return void
 */
void module_vehicle_pose_fusion_init(void);

/**
 * @brief 重置车体融合位姿。
 * @param[in] x_mm 初始 X 坐标，单位：毫米。
 * @param[in] y_mm 初始 Y 坐标，单位：毫米。
 * @param[in] theta_rad 初始航向角，单位：弧度。
 * @return void
 */
void module_vehicle_pose_fusion_reset(float32 x_mm, float32 y_mm, float32 theta_rad);

void module_vehicle_pose_fusion_correct_xy(float32 correction_x_mm, float32 correction_y_mm);

void module_vehicle_pose_fusion_replay_correction_start(void);

void module_vehicle_pose_fusion_replay_correction_stop(void);

void module_vehicle_pose_fusion_replay_correction_enable_set(boolean enable);

boolean module_vehicle_pose_fusion_slip_parameter_set(uint8 parameter, float32 value);

float32 module_vehicle_pose_fusion_slip_parameter_get(uint8 parameter);

boolean module_vehicle_pose_fusion_replay_correction_enable_get(void);

void module_vehicle_pose_fusion_replay_path_straight_set(boolean straight);

boolean module_vehicle_pose_fusion_replay_path_straight_get(void);

const module_vehicle_pose_fusion_replay_correction_t*
module_vehicle_pose_fusion_replay_correction_get(void);

/**
 * @brief 根据编码器里程与陀螺仪航向推进一次位姿融合。
 * @param[in] void 无参数。
 * @return void
 */
void module_vehicle_pose_fusion_update(void);

/**
 * @brief Calculate current vehicle pose once from encoder distance and gyro heading.
 * @param[in] void No parameter.
 * @return void
 */
void module_vehicle_pose_fusion_calculate(void);

/**
 * @brief Save current fused X/Y coordinate to the reserved internal Flash page.
 * @param[in] void No parameter.
 * @return TRUE if the Flash page write is accepted, otherwise FALSE.
 */
boolean module_vehicle_pose_fusion_save_to_flash(void);

/**
 * @brief Load fused X/Y coordinate from the reserved internal Flash page.
 * @param[in] void No parameter.
 * @return TRUE if a valid pose record is loaded, otherwise FALSE.
 */
boolean module_vehicle_pose_fusion_load_from_flash(void);

/**
 * @brief 获取当前车体融合位姿观测。
 * @param[in] void 无参数。
 * @return 只读车体融合位姿观测指针。
 */
const module_vehicle_pose_fusion_observation_t* module_vehicle_pose_fusion_observation_get(void);

/**
 * @brief 获取当前融合航向角。
 * @param[in] void 无参数。
 * @return 当前融合航向角，单位：弧度。
 */
float32 module_vehicle_pose_fusion_heading_get(void);

#endif
