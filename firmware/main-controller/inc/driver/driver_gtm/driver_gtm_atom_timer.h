#ifndef MAD_CIRCUITS_DRIVER_GTM_ATOM_TIMER_H
#define MAD_CIRCUITS_DRIVER_GTM_ATOM_TIMER_H

#include "IfxGtm_Atom_Timer.h"

typedef struct
{
    Ifx_GTM* gtm_module;
    IfxGtm_Atom_Timer_Config atom_timer_modulecfg;
} driver_gtm_atom_timer_cfg_t;

typedef struct
{
    IfxGtm_Atom_Timer atom_timer_modulehn;
} driver_gtm_atom_timer_runtime_t;

void driver_gtm_atom_timer_init(driver_gtm_atom_timer_cfg_t* gtm_atom_timer_cfg, driver_gtm_atom_timer_runtime_t* gtm_atom_timer_runtime);
void driver_gtm_atom_timer_start(driver_gtm_atom_timer_runtime_t* gtm_atom_timer_runtime);
void driver_gtm_atom_agc_setTimeTrigger(driver_gtm_atom_timer_runtime_t* gtm_atom_timer_runtime, IfxGtm_Tbu_Ts tbu_channel, uint32 value);

#endif
