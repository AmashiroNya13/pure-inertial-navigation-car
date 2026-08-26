#ifndef MAD_CIRCUITS_DEVICE_CARRIER_CFG_H
#define MAD_CIRCUITS_DEVICE_CARRIER_CFG_H

#include "../../../inc/driver/driver_dma/driver_dma.h"

typedef enum
{
    DEVICE_CARRIER_1 = 0,
    DEVICE_CARRIER_2 = 1,
    DEVICE_CARRIER_3 = 2,
    DEVICE_CARRIER_4 = 3,
    DEVICE_CARRIER_5 = 4,
    DEVICE_CARRIER_6 = 5,
    DEVICE_CARRIER_7 = 6,
    DEVICE_CARRIER_8 = 7,
    DEVICE_CARRIER_COUNT = 8,
}device_carrier_id_t;

typedef struct
{
    device_carrier_id_t carrier_id;
    driver_dma_cfg_t dma_cfg;
}device_carrier_cfg_t;

typedef struct
{
    device_carrier_id_t carrier_id;
    driver_dma_runtime_t dma_runtime;
    void (*device_carrier_callback) (void);
}device_carrier_runtime_t;

device_carrier_cfg_t* device_carrier_cfg_table_get(void);
device_carrier_runtime_t* device_carrier_runtime_table_get(void);

#endif
