#ifndef MAD_CIRCUITS_DRIVER_GTM_TBU_H
#define MAD_CIRCUITS_DRIVER_GTM_TBU_H

#include "IfxGtm_Tbu.h"

typedef struct
{
    Ifx_GTM* gtm_module;
    IfxGtm_Tbu_Ts tbu_channel;
} driver_gtm_tbu_cfg_t;

typedef struct
{
    driver_gtm_tbu_cfg_t* gtm_tbu_modulehn;
} driver_gtm_tbu_runtime_t;

void driver_gtm_tbu_init(driver_gtm_tbu_cfg_t* gtm_tbu_cfg, driver_gtm_tbu_runtime_t* gtm_tbu_runtime);
void driver_gtm_tbu_enable(driver_gtm_tbu_runtime_t* gtm_tbu_runtime);
boolean driver_gtm_tbu_isEnabled(driver_gtm_tbu_runtime_t* gtm_tbu_runtime);
float32 driver_gtm_tbu_getClockFrequency(driver_gtm_tbu_runtime_t* gtm_tbu_runtime);

#endif
