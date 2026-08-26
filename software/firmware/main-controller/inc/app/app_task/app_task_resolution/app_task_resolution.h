/**
 * @file app_task_resolution.h
 * @brief 设备数据完成事件到应用任务的分辨调度接口。
 */

#ifndef MAD_CIRCUITS_APP_TASK_RESOLUTION_H
#define MAD_CIRCUITS_APP_TASK_RESOLUTION_H

#include "Ifx_Types.h"

typedef enum
{
    APP_TASK_RESOLUTION_DEVICE_IMU_1 = 0,
    APP_TASK_RESOLUTION_DEVICE_IMU_2 = 1,
    APP_TASK_RESOLUTION_DEVICE_MAGNETIC_ENCODER_1 = 2,
    APP_TASK_RESOLUTION_DEVICE_MAGNETIC_ENCODER_2 = 3,
    APP_TASK_RESOLUTION_DEVICE_ESC_1 = 4,
    APP_TASK_RESOLUTION_DEVICE_ESC_2 = 5,
    APP_TASK_RESOLUTION_DEVICE_ESC_3 = 6,
    APP_TASK_RESOLUTION_DEVICE_PHOTOTUBE_GROUP_0 = 7,
    APP_TASK_RESOLUTION_DEVICE_PHOTOTUBE_GROUP_1 = 8,
    APP_TASK_RESOLUTION_DEVICE_PHOTOTUBE_GROUP_2 = 9,
    APP_TASK_RESOLUTION_DEVICE_CARRIER_1 = 10,
    APP_TASK_RESOLUTION_DEVICE_CARRIER_2 = 11,
    APP_TASK_RESOLUTION_DEVICE_CARRIER_3 = 12,
    APP_TASK_RESOLUTION_DEVICE_CARRIER_4 = 13,
    APP_TASK_RESOLUTION_DEVICE_CARRIER_5 = 14,
    APP_TASK_RESOLUTION_DEVICE_CARRIER_6 = 15,
    APP_TASK_RESOLUTION_DEVICE_CARRIER_7 = 16,
    APP_TASK_RESOLUTION_DEVICE_CARRIER_8 = 17,
    APP_TASK_RESOLUTION_COUNT = 18,
} app_task_resolution_id_t;

typedef enum
{
    APP_TASK_RESOLUTION_GROUP_TASK1 = 0,
    APP_TASK_RESOLUTION_GROUP_TASK2 = 1,
    APP_TASK_RESOLUTION_GROUP_COUNT = 2,
} app_task_resolution_group_id_t;

void app_task_resolution_enable(boolean enable);

/**
 * @brief IMU1 数据完成后转发到 TASK1 分辨逻辑。
 * @param[in] void 无参数。
 * @return void
 */
void app_task_resolution_imu_1_decide(void);

/**
 * @brief IMU2 数据完成后转发到 TASK1 分辨逻辑。
 * @param[in] void 无参数。
 * @return void
 */
void app_task_resolution_imu_2_decide(void);

/**
 * @brief 磁编码器1数据完成后转发到 TASK1 分辨逻辑。
 * @param[in] void 无参数。
 * @return void
 */
void app_task_resolution_magnetic_encoder_1_decide(void);

/**
 * @brief 磁编码器2数据完成后转发到 TASK1 分辨逻辑。
 * @param[in] void 无参数。
 * @return void
 */
void app_task_resolution_magnetic_encoder_2_decide(void);

/**
 * @brief 光电管组0数据完成后转发到 TASK2 分辨逻辑。
 * @param[in] void 无参数。
 * @return void
 */
void app_task_resolution_phototube_group_0_decide(void);

/**
 * @brief 光电管组1数据完成后转发到 TASK2 分辨逻辑。
 * @param[in] void 无参数。
 * @return void
 */
void app_task_resolution_phototube_group_1_decide(void);

/**
 * @brief 光电管组2数据完成后转发到 TASK2 分辨逻辑。
 * @param[in] void 无参数。
 * @return void
 */
void app_task_resolution_phototube_group_2_decide(void);

#endif
