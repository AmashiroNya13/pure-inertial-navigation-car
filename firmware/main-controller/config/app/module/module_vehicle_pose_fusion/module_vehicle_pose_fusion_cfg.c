/**
 * @file module_vehicle_pose_fusion_cfg.c
 * @brief 车体位姿融合模块默认配置。
 */

#include "./module_vehicle_pose_fusion_cfg.h"

static const module_vehicle_pose_fusion_cfg_t module_vehicle_pose_fusion_cfg =
{
    .init_x_mm = 0.0f,
    .init_y_mm = 0.0f,
    .init_theta_rad = 0.0f,
    .left_distance_weight = 0.5f,
    .right_distance_weight = 0.5f,
};

/**
 * @brief 获取只读的车体位姿融合配置。
 * @param[in] void 无参数。
 * @return 车体位姿融合配置指针。
 */
const module_vehicle_pose_fusion_cfg_t* module_vehicle_pose_fusion_cfg_get(void)
{
    return &module_vehicle_pose_fusion_cfg;
}
