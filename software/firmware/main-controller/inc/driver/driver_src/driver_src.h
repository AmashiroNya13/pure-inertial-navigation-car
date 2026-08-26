#ifndef MAD_CIRCUITS_DRIVER_SRC_H
#define MAD_CIRCUITS_DRIVER_SRC_H

#include "IfxSrc.h"

typedef struct
{
    volatile Ifx_SRC_SRCR* src;
    IfxSrc_Tos typOfService;
    Ifx_Priority priority;
} driver_src_cfg_t;

typedef struct
{
    volatile Ifx_SRC_SRCR* src;
} driver_src_runtime_t;

void driver_src_init(driver_src_cfg_t* src_cfg, driver_src_runtime_t* src_runtime);
void driver_src_SWTrap(driver_src_runtime_t* src_runtime);

#endif
