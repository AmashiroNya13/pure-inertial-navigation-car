#include "../../../../inc/app/core/app_task_manager/app_task_manager_asynchronous.h"

void app_task_manager_app_task_asynchronous_register(task_id_t task_id,
                                                     app_task_manager_callback_t app_task_asynchronous_callback)
{
    if ((task_id >= TASK_COUNT) || (app_task_asynchronous_callback == NULL_PTR))
    {
        return;
    }

    task_register_callback(task_id, app_task_asynchronous_callback);
}
