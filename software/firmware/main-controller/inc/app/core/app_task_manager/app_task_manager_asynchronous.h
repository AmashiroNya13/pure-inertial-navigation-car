#ifndef MAD_CIRCUITS_APP_CORE_APP_TASK_MANAGER_ASYNCHRONOUS_H
#define MAD_CIRCUITS_APP_CORE_APP_TASK_MANAGER_ASYNCHRONOUS_H

#include "Ifx_Types.h"

#ifndef MAD_CIRCUITS_APP_CORE_APP_TASK_MANAGER_CALLBACK_T_DEFINED
#define MAD_CIRCUITS_APP_CORE_APP_TASK_MANAGER_CALLBACK_T_DEFINED
typedef void (*app_task_manager_callback_t)(void);
#endif

#include "../../../../inc/middleware/task/task.h"

void app_task_manager_app_task_asynchronous_register(task_id_t task_id,
                                                     app_task_manager_callback_t app_task_asynchronous_callback);

#endif
