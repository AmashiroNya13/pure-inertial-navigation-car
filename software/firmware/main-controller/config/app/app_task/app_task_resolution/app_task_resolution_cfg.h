#ifndef MAD_CIRCUITS_APP_TASK_RESOLUTION_CFG_H
#define MAD_CIRCUITS_APP_TASK_RESOLUTION_CFG_H

#include "Ifx_Types.h"

#include "../../../inc/app/app_task/app_task_resolution/app_task_resolution.h"
#include "../../../inc/middleware/task/task.h"

typedef struct
{
    app_task_resolution_group_id_t group_id;
    uint64 required_flags_mask;
    boolean async_task_enabled;
    task_id_t async_task_id;
} app_task_resolution_group_cfg_t;

app_task_resolution_group_cfg_t* app_task_resolution_group_cfg_table_get(void);

#endif
