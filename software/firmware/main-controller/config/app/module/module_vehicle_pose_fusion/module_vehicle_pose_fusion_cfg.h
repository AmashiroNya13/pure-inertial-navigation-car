/**
 * @file module_vehicle_pose_fusion_cfg.h
 * @brief 车体位姿融合模块配置类型。
 */

#ifndef MAD_CIRCUITS_APP_MODULE_MODULE_VEHICLE_POSE_FUSION_CFG_H
#define MAD_CIRCUITS_APP_MODULE_MODULE_VEHICLE_POSE_FUSION_CFG_H

#include "Ifx_Types.h"

typedef struct
{
    float32 init_x_mm;               /**< 默认初始 X 坐标，单位：毫米。 */
    float32 init_y_mm;               /**< 默认初始 Y 坐标，单位：毫米。 */
    float32 init_theta_rad;          /**< 默认初始航向角，单位：弧度。 */
    float32 left_distance_weight;    /**< 左轮里程融合权重，范围：0.0 到 1.0。 */
    float32 right_distance_weight;   /**< 右轮里程融合权重，范围：0.0 到 1.0。 */
} module_vehicle_pose_fusion_cfg_t;

/**
 * @brief 获取只读的车体位姿融合配置。
 * @param[in] void 无参数。
 * @return 车体位姿融合配置指针。
 */
const module_vehicle_pose_fusion_cfg_t* module_vehicle_pose_fusion_cfg_get(void);

#endif
