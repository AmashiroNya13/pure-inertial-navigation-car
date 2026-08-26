#include "../../../../inc/app/core/app_task_manager/app_task_manager_service.h"

typedef struct
{
    app_task_manager_callback_t callback_table[16u];
    uint32 callback_count;
} app_task_manager_app_task_service_runtime_t;

static app_task_manager_app_task_service_runtime_t app_task_manager_app_task_service_runtime;

void app_task_manager_app_task_service_register(app_task_manager_callback_t app_task_service_callback)
{
    if ((app_task_service_callback == NULL_PTR)
        || (app_task_manager_app_task_service_runtime.callback_count
            >= (sizeof(app_task_manager_app_task_service_runtime.callback_table)
                / sizeof(app_task_manager_app_task_service_runtime.callback_table[0]))))
    {
        return;
    }

    app_task_manager_app_task_service_runtime
        .callback_table[app_task_manager_app_task_service_runtime.callback_count] = app_task_service_callback;
    app_task_manager_app_task_service_runtime.callback_count++;
}

void app_task_manager_app_task_service_run(void)
{
    for (uint32 callback_id = 0u;
         callback_id < app_task_manager_app_task_service_runtime.callback_count;
         callback_id++)
    {
        app_task_manager_app_task_service_runtime.callback_table[callback_id]();
    }
}
