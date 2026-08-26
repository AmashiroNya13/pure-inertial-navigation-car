#ifndef MAD_CIRCUITS_TOOLS_LOG_CFG_H
#define MAD_CIRCUITS_TOOLS_LOG_CFG_H

#include "IfxFlash_cfg.h"

#include "../../../../inc/driver/driver_flash/driver_flash.h"

typedef enum
{
    TOOLS_LOG_1 = 0,
    TOOLS_LOG_COUNT = 1,
} tools_log_id_t;

typedef struct
{
    tools_log_id_t log_id;
    driver_flash_cfg_t flash_cfg;
} tools_log_cfg_t;

typedef struct
{
    tools_log_id_t log_id;
    driver_flash_runtime_t flash_runtime;
} tools_log_runtime_t;

tools_log_cfg_t* tools_log_cfg_table_get(void);
tools_log_runtime_t* tools_log_runtime_table_get(void);

#endif
