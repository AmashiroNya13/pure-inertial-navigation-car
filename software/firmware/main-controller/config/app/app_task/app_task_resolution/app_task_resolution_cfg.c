#include "./app_task_resolution_cfg.h"

#define APP_TASK_RESOLUTION_FLAG(resolution_id) ((uint64)1u << (resolution_id))

static app_task_resolution_group_cfg_t app_task_resolution_group_cfg_table[APP_TASK_RESOLUTION_GROUP_COUNT] =
{
    {
        .group_id = APP_TASK_RESOLUTION_GROUP_TASK1,
        .required_flags_mask = APP_TASK_RESOLUTION_FLAG(APP_TASK_RESOLUTION_DEVICE_IMU_1)
                             | APP_TASK_RESOLUTION_FLAG(APP_TASK_RESOLUTION_DEVICE_IMU_2)
                             | APP_TASK_RESOLUTION_FLAG(APP_TASK_RESOLUTION_DEVICE_MAGNETIC_ENCODER_1)
                             | APP_TASK_RESOLUTION_FLAG(APP_TASK_RESOLUTION_DEVICE_MAGNETIC_ENCODER_2),
        .async_task_enabled = TRUE,
        .async_task_id = TASK1,
    },
    {
        .group_id = APP_TASK_RESOLUTION_GROUP_TASK2,
        .required_flags_mask = APP_TASK_RESOLUTION_FLAG(APP_TASK_RESOLUTION_DEVICE_PHOTOTUBE_GROUP_0)
                             | APP_TASK_RESOLUTION_FLAG(APP_TASK_RESOLUTION_DEVICE_PHOTOTUBE_GROUP_1)
                             | APP_TASK_RESOLUTION_FLAG(APP_TASK_RESOLUTION_DEVICE_PHOTOTUBE_GROUP_2),
        .async_task_enabled = TRUE,
        .async_task_id = TASK2,
    },
};

app_task_resolution_group_cfg_t* app_task_resolution_group_cfg_table_get(void)
{
    return app_task_resolution_group_cfg_table;
}
