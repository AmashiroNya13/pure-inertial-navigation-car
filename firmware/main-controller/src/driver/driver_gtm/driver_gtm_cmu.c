#include "../../../inc/driver/driver_gtm/driver_gtm_cmu.h"

void driver_gtm_cmu_gclk_init(driver_gtm_cmu_gclk_cfg_t* gtm_cmu_gclk_cfg)
{
    if (!IfxGtm_Cmu_getGclkFrequency(gtm_cmu_gclk_cfg->gtm_module))
    {
        IfxGtm_Cmu_setGclkFrequency(gtm_cmu_gclk_cfg->gtm_module, gtm_cmu_gclk_cfg->frequency);
    }
}

void driver_gtm_cmu_clk_init(driver_gtm_cmu_clk_cfg_t* gtm_cmu_clk_cfg)
{
    if (!IfxGtm_Cmu_getClkFrequency(gtm_cmu_clk_cfg->gtm_module, gtm_cmu_clk_cfg->clk_id, TRUE))
    {
        IfxGtm_Cmu_setClkFrequency(gtm_cmu_clk_cfg->gtm_module, gtm_cmu_clk_cfg->clk_id, gtm_cmu_clk_cfg->frequency);
    }

    if (!IfxGtm_Cmu_isClkClockEnabled(gtm_cmu_clk_cfg->gtm_module, gtm_cmu_clk_cfg->clk_id))
    {
        IfxGtm_Cmu_enableClocks(gtm_cmu_clk_cfg->gtm_module, gtm_cmu_clk_cfg->clk_mask);
    }
}
