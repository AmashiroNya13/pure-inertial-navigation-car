#include "../../../inc/driver/driver_dma/driver_dma.h"

void driver_dma_init(driver_dma_cfg_t* dma_cfg, driver_dma_runtime_t* dma_runtime)
{
    dma_cfg->dma_modulecfg.dma = dma_cfg->dma_module;
    IfxDma_Dma_initModule(&dma_runtime->dma_modulehn, &dma_cfg->dma_modulecfg);
    dma_cfg->dma_channelcfg.module = &dma_runtime->dma_modulehn;
    IfxDma_Dma_initChannel(&dma_runtime->dma_channelhn, &dma_cfg->dma_channelcfg);
}

IFX_INLINE void driver_dma_setSourceAddress(driver_dma_runtime_t* dma_runtime, uint32 source_address)
{
    IfxDma_Dma_setChannelSourceAddress(&dma_runtime->dma_channelhn, source_address);
}

IFX_INLINE void driver_dma_setDestinationAddress(driver_dma_runtime_t* dma_runtime, uint32 destination_address)
{
    IfxDma_Dma_setChannelDestinationAddress(&dma_runtime->dma_channelhn, destination_address);
}

void driver_dma_setSourceDestinationAddress(driver_dma_runtime_t* dma_runtime, uint32 source_address, uint32 destination_address)
{
    driver_dma_setSourceAddress(dma_runtime, source_address);
    driver_dma_setDestinationAddress(dma_runtime, destination_address);
}

void driver_dma_start(driver_dma_runtime_t* dma_runtime)
{
    IfxDma_Dma_startChannelTransaction(&dma_runtime->dma_channelhn);
}
