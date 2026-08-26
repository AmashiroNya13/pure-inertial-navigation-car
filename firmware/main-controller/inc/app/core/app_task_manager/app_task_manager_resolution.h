#ifndef MAD_CIRCUITS_APP_CORE_APP_TASK_MANAGER_RESOLUTION_H
#define MAD_CIRCUITS_APP_CORE_APP_TASK_MANAGER_RESOLUTION_H

#include "Ifx_Types.h"

#ifndef MAD_CIRCUITS_APP_CORE_APP_TASK_MANAGER_CALLBACK_T_DEFINED
#define MAD_CIRCUITS_APP_CORE_APP_TASK_MANAGER_CALLBACK_T_DEFINED
typedef void (*app_task_manager_callback_t)(void);
#endif

#include "../../../../inc/device/device_carrier/device_carrier.h"
#include "../../../../inc/device/device_esc/device_esc.h"
#include "../../../../inc/device/device_imu/device_imu.h"
#include "../../../../inc/device/device_magnetic_encoder/device_magnetic_encoder.h"
#include "../../../../inc/device/device_phototube/device_phototube.h"

void app_task_manager_app_task_resolution_imu_register(device_imu_id_t imu_id,
                                                       app_task_manager_callback_t app_task_resolution_callback);
void app_task_manager_app_task_resolution_magnetic_encoder_register(device_magnetic_encoder_id_t magnetic_encoder_id,
                                                                    app_task_manager_callback_t app_task_resolution_callback);
void app_task_manager_app_task_resolution_esc_register(device_esc_id_t esc_id,
                                                       app_task_manager_callback_t app_task_resolution_callback);
void app_task_manager_app_task_resolution_phototube_group_register(device_phototube_group_id_t group_id,
                                                                   app_task_manager_callback_t app_task_resolution_callback);
void app_task_manager_app_task_resolution_carrier_register(device_carrier_id_t carrier_id,
                                                           app_task_manager_callback_t app_task_resolution_callback);

#endif
