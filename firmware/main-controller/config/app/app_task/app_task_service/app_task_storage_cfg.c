#include "./app_task_storage_cfg.h"

static app_task_service_storage_cfg_t app_task_service_storage_cfg =
{
    .enabled = TRUE,
};

app_task_service_storage_cfg_t* app_task_service_storage_cfg_get(void)
{
    return &app_task_service_storage_cfg;
}
