#include "./service_storage_cfg.h"

service_storage_cfg_t service_storage_cfg_table[SERVICE_STORAGE_COUNT] =
{
    {
        .storage_id = SERVICE_STORAGE_1,
        .int_flash_id = DEVICE_INT_FLASH_1,
    },
};

service_storage_runtime_t service_storage_runtime_table[SERVICE_STORAGE_COUNT] =
{
    {
        .storage_id = SERVICE_STORAGE_1,
        .int_flash_runtime = NULL_PTR,
    },
};

service_storage_cfg_t* service_storage_cfg_table_get(void)
{
    return service_storage_cfg_table;
}

service_storage_runtime_t* service_storage_runtime_table_get(void)
{
    return service_storage_runtime_table;
}
