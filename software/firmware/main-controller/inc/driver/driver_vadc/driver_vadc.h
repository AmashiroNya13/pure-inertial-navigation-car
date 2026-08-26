#ifndef MAD_CIRCUITS_DRIVER_VADC_H
#define MAD_CIRCUITS_DRIVER_VADC_H

#include "IfxVadc_Adc.h"

typedef struct
{
    Ifx_VADC* vadc_module;
    IfxVadc_Adc_Config *vadc_modulecfg;
    IfxVadc_Adc_GroupConfig *vadc_groupcfg;
    IfxVadc_Adc_ChannelConfig *vadc_channelcfg;
} driver_vadc_cfg_t;

typedef struct
{
    IfxVadc_Adc *vadc_modulehn;
    IfxVadc_Adc_Group *vadc_grouphn;
    IfxVadc_Adc_Channel *vadc_channelhn;
} driver_vadc_runtime_t;

void driver_vadc_init(driver_vadc_cfg_t* vadc_cfg, driver_vadc_runtime_t* vadc_runtime);
void driver_vadc_scanSetChannels(driver_vadc_runtime_t* vadc_runtime, uint32 channels, uint32 mask);
void driver_vadc_scanStart(driver_vadc_runtime_t* vadc_runtime);

#endif
