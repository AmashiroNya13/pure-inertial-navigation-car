#include "../../../../inc/app/core/app_task_manager/app_task_manager_resolution.h"

void app_task_manager_app_task_resolution_imu_register(device_imu_id_t imu_id,
                                                       app_task_manager_callback_t app_task_resolution_callback)
{
    if ((imu_id >= DEVICE_IMU_COUNT) || (app_task_resolution_callback == NULL_PTR))
    {
        return;
    }

    device_imu_register_callback(imu_id, app_task_resolution_callback);
}

void app_task_manager_app_task_resolution_magnetic_encoder_register(device_magnetic_encoder_id_t magnetic_encoder_id,
                                                                    app_task_manager_callback_t app_task_resolution_callback)
{
    if ((magnetic_encoder_id >= DEVICE_MAGNETIC_ENCODER_COUNT) || (app_task_resolution_callback == NULL_PTR))
    {
        return;
    }

    device_magnetic_encoder_register_callback(magnetic_encoder_id, app_task_resolution_callback);
}

void app_task_manager_app_task_resolution_esc_register(device_esc_id_t esc_id,
                                                       app_task_manager_callback_t app_task_resolution_callback)
{
    if ((esc_id >= DEVICE_ESC_COUNT) || (app_task_resolution_callback == NULL_PTR))
    {
        return;
    }

    device_esc_register_callback(esc_id, app_task_resolution_callback);
}

void app_task_manager_app_task_resolution_phototube_group_register(device_phototube_group_id_t group_id,
                                                                   app_task_manager_callback_t app_task_resolution_callback)
{
    if ((group_id >= DEVICE_PHOTOTUBE_GROUP_COUNT) || (app_task_resolution_callback == NULL_PTR))
    {
        return;
    }

    device_phototube_group_register_callback(group_id, app_task_resolution_callback);
}

void app_task_manager_app_task_resolution_carrier_register(device_carrier_id_t carrier_id,
                                                           app_task_manager_callback_t app_task_resolution_callback)
{
    if ((carrier_id >= DEVICE_CARRIER_COUNT) || (app_task_resolution_callback == NULL_PTR))
    {
        return;
    }

    device_carrier_register_callback(carrier_id, app_task_resolution_callback);
}
