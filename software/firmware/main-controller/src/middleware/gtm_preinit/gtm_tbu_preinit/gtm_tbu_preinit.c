#include "../../../../inc/middleware/gtm_preinit/gtm_tbu_preinit/gtm_tbu_preinit.h"

void gtm_tbu_preinit(void)
{
    driver_gtm_tbu_cfg_t gtm_tbu_cfg;
    driver_gtm_tbu_runtime_t gtm_tbu_runtime;

    gtm_tbu_cfg.gtm_module = &MODULE_GTM;
    gtm_tbu_cfg.tbu_channel = IfxGtm_Tbu_Ts_0;

    driver_gtm_tbu_init(&gtm_tbu_cfg, &gtm_tbu_runtime);
    driver_gtm_tbu_enable(&gtm_tbu_runtime);
}
