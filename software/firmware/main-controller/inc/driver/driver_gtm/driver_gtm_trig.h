#ifndef MAD_CIRCUITS_DRIVER_GTM_TRIG_H
#define MAD_CIRCUITS_DRIVER_GTM_TRIG_H

#include "IfxGtm_Trig.h"

typedef struct
{
    Ifx_GTM* gtm_module;
    IfxGtm_Trig_AdcGroup gtm_trig_tovadc_groupcfg;
    IfxGtm_Trig_AdcTrig gtm_trig_tovadc_trigcfg;
    IfxGtm_Trig_AdcTrigSource gtm_trig_tovadc_trigsourcecfg;
    IfxGtm_Trig_AdcTrigChannel gtm_trig_tovadc_trigchannelcfg;
} driver_gtm_trig_tovadc_cfg_t;

typedef struct
{
    driver_gtm_trig_tovadc_cfg_t* gtm_trig_tovadchn;
} driver_gtm_trig_tovadc_runtime_t;

void driver_gtm_trig_tovadc_init(driver_gtm_trig_tovadc_cfg_t* gtm_trig_tovadc_cfg, driver_gtm_trig_tovadc_runtime_t* gtm_trig_tovadc_runtime);

#endif
