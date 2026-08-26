#ifndef MAD_CIRCUITS_DEVICE_INT_FLASH_CFG_H
#define MAD_CIRCUITS_DEVICE_INT_FLASH_CFG_H

#include "IfxFlash_cfg.h"

#include "../../../inc/driver/driver_flash/driver_flash.h"

typedef enum
{
    DEVICE_INT_FLASH_1 = 0,
    DEVICE_INT_FLASH_COUNT = 1,
} device_int_flash_id_t;

typedef struct
{
    device_int_flash_id_t int_flash_id;
    driver_flash_cfg_t flash_cfg;
} device_int_flash_cfg_t;

typedef struct
{
    device_int_flash_id_t int_flash_id;
    driver_flash_runtime_t flash_runtime;
} device_int_flash_runtime_t;

device_int_flash_cfg_t* device_int_flash_cfg_table_get(void);
device_int_flash_runtime_t* device_int_flash_runtime_table_get(void);

#endif
