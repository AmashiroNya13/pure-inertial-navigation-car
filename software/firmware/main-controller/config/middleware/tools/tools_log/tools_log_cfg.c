#include "./tools_log_cfg.h"

tools_log_cfg_t tools_log_cfg_table[TOOLS_LOG_COUNT] =
{
    {
        .log_id = TOOLS_LOG_1,
        .flash_cfg.sector_group = DRIVER_FLASH_SECTOR_MASK_D0_0_0
                                | DRIVER_FLASH_SECTOR_MASK_D0_0_1
                                | DRIVER_FLASH_SECTOR_MASK_D0_0_2
                                | DRIVER_FLASH_SECTOR_MASK_D0_0_3
                                | DRIVER_FLASH_SECTOR_MASK_D0_0_4
                                | DRIVER_FLASH_SECTOR_MASK_D0_0_5
                                | DRIVER_FLASH_SECTOR_MASK_D0_0_6
                                | DRIVER_FLASH_SECTOR_MASK_D0_0_7
                                | DRIVER_FLASH_SECTOR_MASK_D0_0_8
                                | DRIVER_FLASH_SECTOR_MASK_D0_0_9
                                | DRIVER_FLASH_SECTOR_MASK_D0_0_10
                                | DRIVER_FLASH_SECTOR_MASK_D0_0_11,
    },
};

tools_log_runtime_t tools_log_runtime_table[TOOLS_LOG_COUNT] =
{
    {
        .log_id = TOOLS_LOG_1,
    },
};

tools_log_cfg_t* tools_log_cfg_table_get(void)
{
    return tools_log_cfg_table;
}

tools_log_runtime_t* tools_log_runtime_table_get(void)
{
    return tools_log_runtime_table;
}
