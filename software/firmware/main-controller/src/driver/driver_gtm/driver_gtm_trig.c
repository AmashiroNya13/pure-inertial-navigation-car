#include "../../../inc/driver/driver_gtm/driver_gtm_trig.h"

void driver_gtm_trig_tovadc_init(driver_gtm_trig_tovadc_cfg_t* gtm_trig_tovadc_cfg, driver_gtm_trig_tovadc_runtime_t* gtm_trig_tovadc_runtime)
{
    IfxGtm_Trig_toVadc(gtm_trig_tovadc_cfg->gtm_module,
                       gtm_trig_tovadc_cfg->gtm_trig_tovadc_groupcfg,
                       gtm_trig_tovadc_cfg->gtm_trig_tovadc_trigcfg,
                       gtm_trig_tovadc_cfg->gtm_trig_tovadc_trigsourcecfg,
                       gtm_trig_tovadc_cfg->gtm_trig_tovadc_trigchannelcfg);
    gtm_trig_tovadc_runtime->gtm_trig_tovadchn = gtm_trig_tovadc_cfg;
}
