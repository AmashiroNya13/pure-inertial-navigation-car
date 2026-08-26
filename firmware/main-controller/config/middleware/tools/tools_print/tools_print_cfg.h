#ifndef MAD_CIRCUITS_TOOLS_PRINT_CFG_H
#define MAD_CIRCUITS_TOOLS_PRINT_CFG_H

#include "../../../device/device_debug/device_debug_cfg.h"

#define TOOLS_PRINTF_BUFFER_SIZE 256u

typedef enum
{
    TOOLS_PRINT_1 = 0,
    TOOLS_PRINT_COUNT = 1,
} tools_print_id_t;

typedef struct
{
    tools_print_id_t tools_print_id;
    device_debug_id_t debug_id;
} tools_print_cfg_t;

tools_print_cfg_t* tools_print_cfg_table_get(void);

#endif
