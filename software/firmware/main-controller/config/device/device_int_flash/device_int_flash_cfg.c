#include "./device_int_flash_cfg.h"

device_int_flash_cfg_t device_int_flash_cfg_table[DEVICE_INT_FLASH_COUNT] =
{
    {
        .int_flash_id = DEVICE_INT_FLASH_1,
        .flash_cfg.sector_group = DRIVER_FLASH_SECTOR_MASK_P1_0_0
                                | DRIVER_FLASH_SECTOR_MASK_P1_0_1
                                | DRIVER_FLASH_SECTOR_MASK_P1_0_2
                                | DRIVER_FLASH_SECTOR_MASK_P1_0_3
                                | DRIVER_FLASH_SECTOR_MASK_P1_0_4
                                | DRIVER_FLASH_SECTOR_MASK_P1_0_5
                                | DRIVER_FLASH_SECTOR_MASK_P1_0_6
                                | DRIVER_FLASH_SECTOR_MASK_P1_0_7
                                | DRIVER_FLASH_SECTOR_MASK_P1_0_8
                                | DRIVER_FLASH_SECTOR_MASK_P1_0_9
                                | DRIVER_FLASH_SECTOR_MASK_P1_0_10
                                | DRIVER_FLASH_SECTOR_MASK_P1_0_11
                                | DRIVER_FLASH_SECTOR_MASK_P1_0_12
                                | DRIVER_FLASH_SECTOR_MASK_P1_0_13
                                | DRIVER_FLASH_SECTOR_MASK_P1_0_14
                                | DRIVER_FLASH_SECTOR_MASK_P1_0_15
                                | DRIVER_FLASH_SECTOR_MASK_P1_0_16
                                | DRIVER_FLASH_SECTOR_MASK_P1_0_17
                                | DRIVER_FLASH_SECTOR_MASK_P1_1_18
                                | DRIVER_FLASH_SECTOR_MASK_P1_1_19
                                | DRIVER_FLASH_SECTOR_MASK_P1_1_20
                                | DRIVER_FLASH_SECTOR_MASK_P1_1_21
                                | DRIVER_FLASH_SECTOR_MASK_P1_1_22
                                | DRIVER_FLASH_SECTOR_MASK_P1_2_23
                                | DRIVER_FLASH_SECTOR_MASK_P1_2_24
    },
};

device_int_flash_runtime_t device_int_flash_runtime_table[DEVICE_INT_FLASH_COUNT] =
{
    {
        .int_flash_id = DEVICE_INT_FLASH_1,
    },
};

device_int_flash_cfg_t* device_int_flash_cfg_table_get(void)
{
    return device_int_flash_cfg_table;
}

device_int_flash_runtime_t* device_int_flash_runtime_table_get(void)
{
    return device_int_flash_runtime_table;
}
