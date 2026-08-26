#include "../../../inc/driver/driver_gtm/driver_gtm_atom_timer.h"

void driver_gtm_atom_timer_init(driver_gtm_atom_timer_cfg_t* gtm_atom_timer_cfg, driver_gtm_atom_timer_runtime_t* gtm_atom_timer_runtime)
{
    IfxGtm_Atom_Timer_init(&gtm_atom_timer_runtime->atom_timer_modulehn, &gtm_atom_timer_cfg->atom_timer_modulecfg);
}

void driver_gtm_atom_timer_start(driver_gtm_atom_timer_runtime_t* gtm_atom_timer_runtime)
{
    IfxGtm_Atom_Timer_run(&gtm_atom_timer_runtime->atom_timer_modulehn);
}

void driver_gtm_atom_agc_setTimeTrigger(driver_gtm_atom_timer_runtime_t* gtm_atom_timer_runtime, IfxGtm_Tbu_Ts tbu_channel, uint32 value)
{
    IfxGtm_Atom_Agc_setTimeTrigger(gtm_atom_timer_runtime->atom_timer_modulehn.agc, tbu_channel, value);
}
