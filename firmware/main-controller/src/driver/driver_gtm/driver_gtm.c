#include "../../../inc/driver/driver_gtm/driver_gtm.h"

void driver_gtm_init(Ifx_GTM* gtm_module)
{
    if (!IfxGtm_isEnabled(gtm_module))
    {
        IfxGtm_enable(gtm_module);
    }
}
