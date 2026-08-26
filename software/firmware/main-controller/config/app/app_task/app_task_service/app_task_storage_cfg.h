#ifndef MAD_CIRCUITS_APP_APP_TASK_SERVICE_STORAGE_CFG_H
#define MAD_CIRCUITS_APP_APP_TASK_SERVICE_STORAGE_CFG_H

#include "Ifx_Types.h"

typedef struct
{
    boolean enabled;
} app_task_service_storage_cfg_t;

app_task_service_storage_cfg_t* app_task_service_storage_cfg_get(void);

#endif
