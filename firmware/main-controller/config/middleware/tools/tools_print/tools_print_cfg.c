#include "./tools_print_cfg.h"

tools_print_cfg_t tools_print_cfg_table[TOOLS_PRINT_COUNT] =
{
    {
        .tools_print_id = TOOLS_PRINT_1,
        .debug_id = DEVICE_DEBUG_1,
    },
};

tools_print_cfg_t* tools_print_cfg_table_get(void)
{
    return tools_print_cfg_table;
}
