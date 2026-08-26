#ifndef MAD_CIRCUITS_DEVICE_EXT_FLASH_CFG_H
#define MAD_CIRCUITS_DEVICE_EXT_FLASH_CFG_H

#include "../../../inc/driver/driver_dma/driver_dma.h"
#include "../../../inc/driver/driver_qspi/driver_qspi.h"

typedef enum
{
    DEVICE_EXT_FLASH_1 = 0,
    DEVICE_EXT_FLASH_COUNT = 1,
}device_ext_flash_id_t;

typedef struct
{
    device_ext_flash_id_t ext_flash_id;
    driver_dma_cfg_t tx_dma_cfg;
    driver_qspi_cfg_t qspi_cfg;
    driver_dma_cfg_t rx_dma_cfg;
}device_ext_flash_cfg_t;

typedef struct
{
    device_ext_flash_id_t ext_flash_id;
    driver_dma_runtime_t tx_dma_runtime;
    driver_qspi_runtime_t qspi_runtime;
    driver_dma_runtime_t rx_dma_runtime;
}device_ext_flash_runtime_t;

device_ext_flash_cfg_t* device_ext_flash_cfg_table_get(void);
device_ext_flash_runtime_t* device_ext_flash_runtime_table_get(void);

#endif
