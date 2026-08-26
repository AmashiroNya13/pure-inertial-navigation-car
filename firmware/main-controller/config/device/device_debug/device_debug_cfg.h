#ifndef MAD_CIRCUITS_DEVICE_DEBUG_CFG_H
#define MAD_CIRCUITS_DEVICE_DEBUG_CFG_H

#include "../../../inc/driver/driver_asclin/driver_asclin_asc.h"

#define DEVICE_DEBUG_TXFIFO_SIZE 10240
#define DEVICE_DEBUG_RXFIFO_SIZE 10240
#define IFX_FIFO_BUFFER_OVERHEAD (sizeof(Ifx_Fifo) + 8)

#define DEVICE_DEBUG_TXFIFO_BUFFER_BYTE (DEVICE_DEBUG_TXFIFO_SIZE + IFX_FIFO_BUFFER_OVERHEAD)
#define DEVICE_DEBUG_RXFIFO_BUFFER_BYTE (DEVICE_DEBUG_RXFIFO_SIZE + IFX_FIFO_BUFFER_OVERHEAD)

typedef enum
{
    DEVICE_DEBUG_1 = 0,
    DEVICE_DEBUG_COUNT = 1,
} device_debug_id_t;

typedef struct
{
    device_debug_id_t debug_id;
    driver_asclin_asc_cfg_t asclin_asc_cfg;
} device_debug_cfg_t;

typedef struct
{
    device_debug_id_t debug_id;
    driver_asclin_asc_runtime_t asclin_asc_runtime;
    uint8 txFifo_Buffer[DEVICE_DEBUG_TXFIFO_BUFFER_BYTE];
    uint8 rxFifo_Buffer[DEVICE_DEBUG_RXFIFO_BUFFER_BYTE];
    void (*device_debug_callback)(void);
} device_debug_runtime_t;

device_debug_cfg_t* device_debug_cfg_table_get(void);
device_debug_runtime_t* device_debug_runtime_table_get(void);

#endif
