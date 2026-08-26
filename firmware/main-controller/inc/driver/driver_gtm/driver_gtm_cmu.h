#ifndef MAD_CIRCUITS_DRIVER_GTM_CMU_H
#define MAD_CIRCUITS_DRIVER_GTM_CMU_H

#include "IfxGtm_Cmu.h"

typedef struct
{
    Ifx_GTM* gtm_module;
    float32 frequency;
} driver_gtm_cmu_gclk_cfg_t;

typedef struct
{
    Ifx_GTM* gtm_module;
    IfxGtm_Cmu_Clk clk_id;
    uint32 clk_mask;
    float32 frequency;
} driver_gtm_cmu_clk_cfg_t;

void driver_gtm_cmu_gclk_init(driver_gtm_cmu_gclk_cfg_t* gtm_cmu_gclk_cfg);
void driver_gtm_cmu_clk_init(driver_gtm_cmu_clk_cfg_t* gtm_cmu_clk_cfg);

#endif
