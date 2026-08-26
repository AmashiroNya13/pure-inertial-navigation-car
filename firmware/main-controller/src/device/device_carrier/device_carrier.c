#include "../../../inc/device/device_carrier/device_carrier.h"

void device_carrier_init(device_carrier_id_t carrier_id)
{
    device_carrier_cfg_t* carrier_cfg = device_carrier_cfg_table_get();
    device_carrier_runtime_t* carrier_runtime = device_carrier_runtime_table_get();

    driver_dma_init(&carrier_cfg[carrier_id].dma_cfg, &carrier_runtime[carrier_id].dma_runtime);
    driver_dma_start(&carrier_runtime[carrier_id].dma_runtime);
}

void device_carrier_init_all(void)
{
    device_carrier_init(DEVICE_CARRIER_1);
    device_carrier_init(DEVICE_CARRIER_2);
    device_carrier_init(DEVICE_CARRIER_3);
    device_carrier_init(DEVICE_CARRIER_4);
    device_carrier_init(DEVICE_CARRIER_5);
    device_carrier_init(DEVICE_CARRIER_6);
    device_carrier_init(DEVICE_CARRIER_7);
    device_carrier_init(DEVICE_CARRIER_8);
}

void device_carrier_register_callback(device_carrier_id_t carrier_id, void (*device_carrier_callback) (void))
{
    device_carrier_runtime_t* carrier_runtime = device_carrier_runtime_table_get();

    carrier_runtime[carrier_id].device_carrier_callback = device_carrier_callback;
}

void device_carrier_setTask(device_carrier_id_t carrier_id, uint32 sourceAddress, uint32 destinationAddress)
{
    device_carrier_runtime_t* carrier_runtime = device_carrier_runtime_table_get();

    driver_dma_setSourceDestinationAddress(&carrier_runtime[carrier_id].dma_runtime, sourceAddress, destinationAddress);
    driver_dma_start(&carrier_runtime[carrier_id].dma_runtime);
}
