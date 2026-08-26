#include "../../../inc/driver/driver_gtm/driver_gtm_atom_pwmhl.h"

void driver_gtm_atom_pwmhl_init(driver_gtm_atom_pwmhl_cfg_t *gtm_atom_pwmhl_cfg, driver_gtm_atom_pwmhl_runtime_t *gtm_atom_pwmhl_runtime)
{
    IfxGtm_Atom_PwmHl_init(&gtm_atom_pwmhl_runtime->atom_pwmhl_modulehn, &gtm_atom_pwmhl_cfg->atom_pwmhl_modulecfg);
}

void driver_gtm_atom_pwmhl_start(driver_gtm_atom_pwmhl_runtime_t *gtm_atom_pwmhl_runtime)
{
    IfxGtm_Atom_Timer_run(gtm_atom_pwmhl_runtime->atom_pwmhl_modulehn.timer);
}
