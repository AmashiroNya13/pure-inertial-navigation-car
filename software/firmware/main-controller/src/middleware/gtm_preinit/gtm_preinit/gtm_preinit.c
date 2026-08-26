#include "../../../../inc/middleware/gtm_preinit/gtm_preinit/gtm_preinit.h"

void gtm_preinit(void)
{
    Ifx_GTM* gtm_module = &MODULE_GTM;
    driver_gtm_init(gtm_module);
}
