#ifndef MAD_CIRCUITS_DEVICE_MAGNETIC_ENCODER_CFG_H
#define MAD_CIRCUITS_DEVICE_MAGNETIC_ENCODER_CFG_H

#include "../../../inc/driver/driver_dma/driver_dma.h"
#include "../../../inc/driver/driver_gtm/driver_gtm_atom_timer.h"
#include "../../../inc/driver/driver_qspi/driver_qspi.h"

typedef enum
{
    DEVICE_MAGNETIC_ENCODER_1 = 0,
    DEVICE_MAGNETIC_ENCODER_2 = 1,
    DEVICE_MAGNETIC_ENCODER_COUNT = 2,
} device_magnetic_encoder_id_t;

typedef struct
{
    device_magnetic_encoder_id_t magnetic_encoder_id;
    driver_gtm_atom_timer_cfg_t gtm_atom_timer_cfg;
    driver_dma_cfg_t tx_dma_cfg;
    driver_qspi_cfg_t qspi_cfg;
    driver_dma_cfg_t rx_dma_cfg;
} device_magnetic_encoder_cfg_t;

typedef struct
{
    device_magnetic_encoder_id_t magnetic_encoder_id;
    driver_gtm_atom_timer_runtime_t gtm_atom_timer_runtime;
    driver_dma_runtime_t tx_dma_runtime;
    driver_qspi_runtime_t qspi_runtime;
    driver_dma_runtime_t rx_dma_runtime;
    uint16 dma_command_buffer;
    uint16 dma_receive_buffer;
    uint16 last_frame;
    void (*device_magnetic_encoder_callback) (void);
} device_magnetic_encoder_runtime_t;

device_magnetic_encoder_cfg_t* device_magnetic_encoder_cfg_table_get(void);
device_magnetic_encoder_runtime_t* device_magnetic_encoder_runtime_table_get(void);

#endif
