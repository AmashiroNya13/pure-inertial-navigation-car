#include "../../../../inc/middleware/gtm_preinit/gtm_cmu_clk_preinit/gtm_cmu_clk_preinit.h"

void gtm_cmu_clk_preinit(void)
{
    driver_gtm_cmu_clk_cfg_t gtm_cmu_clk_cfg;
    gtm_cmu_clk_cfg.gtm_module = &MODULE_GTM;
    gtm_cmu_clk_cfg.frequency = 100000000;
    gtm_cmu_clk_cfg.clk_id = IfxGtm_Cmu_Clk_0;
    gtm_cmu_clk_cfg.clk_mask = IFXGTM_CMU_CLKEN_CLK0;
    driver_gtm_cmu_clk_init(&gtm_cmu_clk_cfg);
    /*
    gtm_cmu_clk_cfg.gtm_module = &MODULE_GTM;
    gtm_cmu_clk_cfg.frequency = 3000000;
    gtm_cmu_clk_cfg.clk_id = IfxGtm_Cmu_Clk_1;
    gtm_cmu_clk_cfg.clk_mask = IFXGTM_CMU_CLKEN_CLK1;
    driver_gtm_cmu_clk_init(&gtm_cmu_clk_cfg);
    gtm_cmu_clk_cfg.gtm_module = &MODULE_GTM;
    gtm_cmu_clk_cfg.frequency = 3000000;
    gtm_cmu_clk_cfg.clk_id = IfxGtm_Cmu_Clk_2;
    gtm_cmu_clk_cfg.clk_mask = IFXGTM_CMU_CLKEN_CLK2;
    driver_gtm_cmu_clk_init(&gtm_cmu_clk_cfg);
    gtm_cmu_clk_cfg.gtm_module = &MODULE_GTM;
    gtm_cmu_clk_cfg.frequency = 3000000;
    gtm_cmu_clk_cfg.clk_id = IfxGtm_Cmu_Clk_3;
    gtm_cmu_clk_cfg.clk_mask = IFXGTM_CMU_CLKEN_CLK3;
    driver_gtm_cmu_clk_init(&gtm_cmu_clk_cfg);
    gtm_cmu_clk_cfg.gtm_module = &MODULE_GTM;
    gtm_cmu_clk_cfg.frequency = 3000000;
    gtm_cmu_clk_cfg.clk_id = IfxGtm_Cmu_Clk_4;
    gtm_cmu_clk_cfg.clk_mask = IFXGTM_CMU_CLKEN_CLK4;
    driver_gtm_cmu_clk_init(&gtm_cmu_clk_cfg);
    gtm_cmu_clk_cfg.gtm_module = &MODULE_GTM;
    gtm_cmu_clk_cfg.frequency = 3000000;
    gtm_cmu_clk_cfg.clk_id = IfxGtm_Cmu_Clk_5;
    gtm_cmu_clk_cfg.clk_mask = IFXGTM_CMU_CLKEN_CLK5;
    driver_gtm_cmu_clk_init(&gtm_cmu_clk_cfg);
    gtm_cmu_clk_cfg.gtm_module = &MODULE_GTM;
    gtm_cmu_clk_cfg.frequency = 3000000;
    gtm_cmu_clk_cfg.clk_id = IfxGtm_Cmu_Clk_6;
    gtm_cmu_clk_cfg.clk_mask = IFXGTM_CMU_CLKEN_CLK6;
    driver_gtm_cmu_clk_init(&gtm_cmu_clk_cfg);
    gtm_cmu_clk_cfg.gtm_module = &MODULE_GTM;
    gtm_cmu_clk_cfg.frequency = 3000000;
    gtm_cmu_clk_cfg.clk_id = IfxGtm_Cmu_Clk_7;
    gtm_cmu_clk_cfg.clk_mask = IFXGTM_CMU_CLKEN_CLK7;
    driver_gtm_cmu_clk_init(&gtm_cmu_clk_cfg);
    */
}
