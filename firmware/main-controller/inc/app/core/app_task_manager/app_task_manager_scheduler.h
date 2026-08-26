#ifndef MAD_CIRCUITS_APP_CORE_APP_TASK_MANAGER_SCHEDULER_H
#define MAD_CIRCUITS_APP_CORE_APP_TASK_MANAGER_SCHEDULER_H

#include "Ifx_Types.h"

#ifndef MAD_CIRCUITS_APP_CORE_APP_TASK_MANAGER_CALLBACK_T_DEFINED
#define MAD_CIRCUITS_APP_CORE_APP_TASK_MANAGER_CALLBACK_T_DEFINED
typedef void (*app_task_manager_callback_t)(void);
#endif

#include "../../../../inc/middleware/sysTick/sysTick.h"

void app_task_manager_app_task_scheduler_register(sysTick_id_t sysTick_id,
                                                  app_task_manager_callback_t app_task_scheduler_callback);

#endif
