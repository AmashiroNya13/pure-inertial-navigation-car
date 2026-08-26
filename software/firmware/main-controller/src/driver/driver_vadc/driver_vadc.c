#include "../../../inc/driver/driver_vadc/driver_vadc.h"

void driver_vadc_init(driver_vadc_cfg_t* vadc_cfg, driver_vadc_runtime_t* vadc_runtime)
{
    IfxVadc_Adc_initModule(vadc_runtime->vadc_modulehn, vadc_cfg->vadc_modulecfg);
    vadc_cfg->vadc_groupcfg->module = vadc_runtime->vadc_modulehn;
    IfxVadc_Adc_initGroup(vadc_runtime->vadc_grouphn, vadc_cfg->vadc_groupcfg);
    vadc_cfg->vadc_channelcfg->group = vadc_runtime->vadc_grouphn;
    IfxVadc_Adc_initChannel(vadc_runtime->vadc_channelhn, vadc_cfg->vadc_channelcfg);
}

void driver_vadc_scanSetChannels(driver_vadc_runtime_t* vadc_runtime, uint32 channels, uint32 mask)
{
    IfxVadc_Adc_setScan(vadc_runtime->vadc_grouphn, channels, mask);
}

void driver_vadc_scanStart(driver_vadc_runtime_t* vadc_runtime)
{
    IfxVadc_Adc_startScan(vadc_runtime->vadc_grouphn);
}
