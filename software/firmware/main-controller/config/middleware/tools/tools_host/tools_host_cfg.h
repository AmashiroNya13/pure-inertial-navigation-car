#ifndef MAD_CIRCUITS_TOOLS_HOST_CFG_H
#define MAD_CIRCUITS_TOOLS_HOST_CFG_H

#include "../../../device/device_debug/device_debug_cfg.h"

#define TOOLS_HOST_COMMAND_COUNT 48u
#define TOOLS_HOST_COMMAND_LENGTH 24u
#define TOOLS_HOST_LINE_LENGTH 96u
#define TOOLS_HOST_ARG_COUNT 24u
#define TOOLS_HOST_PENDING_LINE_COUNT 8u
#define TOOLS_HOST_BINARY_FRAME_MAX_LENGTH 544u
#define TOOLS_HOST_PENDING_BINARY_FRAME_COUNT 3u

typedef enum
{
    TOOLS_HOST_1 = 0,
    TOOLS_HOST_COUNT = 1,
} tools_host_id_t;

typedef struct
{
    tools_host_id_t tools_host_id;
    device_debug_id_t debug_id;
} tools_host_cfg_t;

tools_host_cfg_t* tools_host_cfg_table_get(void);

#endif
