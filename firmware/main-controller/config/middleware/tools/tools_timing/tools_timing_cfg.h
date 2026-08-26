#ifndef MAD_CIRCUITS_TOOLS_TIMING_CFG_H
#define MAD_CIRCUITS_TOOLS_TIMING_CFG_H

#include "../../../../inc/driver/driver_stm/driver_stm.h"

typedef enum
{
    TOOLS_TIMING_1 = 0,
    TOOLS_TIMING_COUNT = 1,
} tools_timing_id_t;

typedef struct
{
    tools_timing_id_t tools_timing_id;
    driver_stm_cfg_t stm_cfg;
} tools_timing_cfg_t;

typedef struct
{
    tools_timing_id_t tools_timing_id;
    driver_stm_runtime_t stm_runtime;
} tools_timing_runtime_t;

tools_timing_cfg_t* tools_timing_cfg_table_get(void);
tools_timing_runtime_t* tools_timing_runtime_table_get(void);

#endif
