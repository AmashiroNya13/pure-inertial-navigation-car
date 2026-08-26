#include "../../../../inc/middleware/gtm_preinit/gtm_cmu_gclk_preinit/gtm_cmu_gclk_preinit.h"

void gtm_cmu_gclk_preinit(void)
{
    driver_gtm_cmu_gclk_cfg_t gtm_cmu_gclk_cfg;
    gtm_cmu_gclk_cfg.gtm_module = &MODULE_GTM;
    gtm_cmu_gclk_cfg.frequency = 100000000;
    driver_gtm_cmu_gclk_init(&gtm_cmu_gclk_cfg);
}
