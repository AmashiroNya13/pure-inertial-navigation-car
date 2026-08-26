#include "../../../../inc/app/app_task/app_task_service/app_task_storage.h"

#include "../../../../config/app/app_task/app_task_service/app_task_storage_cfg.h"
#include "../../../../inc/app/service/service_storage/service_storage.h"

void app_task_service_storage_run(void)
{
    app_task_service_storage_cfg_t* app_task_service_storage_cfg = app_task_service_storage_cfg_get();

    if (app_task_service_storage_cfg->enabled == FALSE)
    {
        return;
    }

    service_storage_runAsync(SERVICE_STORAGE_1);
}
