#ifndef MAD_CIRCUITS_DRIVER_GTM_ATOM_PWMHL_H
#define MAD_CIRCUITS_DRIVER_GTM_ATOM_PWMHL_H

#include "IfxGtm_Atom_PwmHl.h"

typedef struct
{
    Ifx_GTM *gtm_module;
    IfxGtm_Atom_PwmHl_Config atom_pwmhl_modulecfg;
} driver_gtm_atom_pwmhl_cfg_t;

typedef struct
{
    IfxGtm_Atom_PwmHl atom_pwmhl_modulehn;
} driver_gtm_atom_pwmhl_runtime_t;

void driver_gtm_atom_pwmhl_init(driver_gtm_atom_pwmhl_cfg_t *gtm_atom_pwmhl_cfg, driver_gtm_atom_pwmhl_runtime_t *gtm_atom_pwmhl_runtime);
void driver_gtm_atom_pwmhl_start(driver_gtm_atom_pwmhl_runtime_t *gtm_atom_pwmhl_runtime);

#endif
