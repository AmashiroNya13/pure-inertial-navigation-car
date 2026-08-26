#ifndef MAD_CIRCUITS_APP_APP_TASK_SCHEDULER_CFG_H
#define MAD_CIRCUITS_APP_APP_TASK_SCHEDULER_CFG_H

#include "Ifx_Types.h"

#include "../../../inc/middleware/sysTick/sysTick.h"

typedef struct
{
    sysTick_id_t sysTick_id;
} app_task_scheduler_cfg_t;

app_task_scheduler_cfg_t* app_task_scheduler_cfg_get(void);

#endif
