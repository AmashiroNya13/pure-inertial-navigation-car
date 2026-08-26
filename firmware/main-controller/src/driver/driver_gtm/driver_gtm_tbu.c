#include "../../../inc/driver/driver_gtm/driver_gtm_tbu.h"

void driver_gtm_tbu_init(driver_gtm_tbu_cfg_t* gtm_tbu_cfg, driver_gtm_tbu_runtime_t* gtm_tbu_runtime)
{
    gtm_tbu_runtime->gtm_tbu_modulehn = gtm_tbu_cfg;
}

void driver_gtm_tbu_enable(driver_gtm_tbu_runtime_t* gtm_tbu_runtime)
{
    IfxGtm_Tbu_enableChannel(gtm_tbu_runtime->gtm_tbu_modulehn->gtm_module, gtm_tbu_runtime->gtm_tbu_modulehn->tbu_channel);
}

boolean driver_gtm_tbu_isEnabled(driver_gtm_tbu_runtime_t* gtm_tbu_runtime)
{
    return IfxGtm_Tbu_isChannelEnabled(gtm_tbu_runtime->gtm_tbu_modulehn->gtm_module, gtm_tbu_runtime->gtm_tbu_modulehn->tbu_channel);
}

float32 driver_gtm_tbu_getClockFrequency(driver_gtm_tbu_runtime_t* gtm_tbu_runtime)
{
    return IfxGtm_Tbu_getClockFrequency(gtm_tbu_runtime->gtm_tbu_modulehn->gtm_module, gtm_tbu_runtime->gtm_tbu_modulehn->tbu_channel);
}
