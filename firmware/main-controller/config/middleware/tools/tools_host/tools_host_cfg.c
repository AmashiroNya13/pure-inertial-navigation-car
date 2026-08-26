#include "./tools_host_cfg.h"

tools_host_cfg_t tools_host_cfg_table[TOOLS_HOST_COUNT] =
{
    {
        .tools_host_id = TOOLS_HOST_1,
        .debug_id = DEVICE_DEBUG_1,
    },
};

tools_host_cfg_t* tools_host_cfg_table_get(void)
{
    return tools_host_cfg_table;
}
