#ifndef MAD_CIRCUITS_DRIVER_STM_H
#define MAD_CIRCUITS_DRIVER_STM_H

#include "IfxStm_Timer.h"

typedef struct
{
    Ifx_STM* stm;
    IfxStm_Timer_Config stm_timer_modulecfg;
    IfxStm_CompareConfig stm_compare_cfg;
} driver_stm_cfg_t;

typedef struct
{
    IfxStm_Timer stm_timer_modulehn;
} driver_stm_runtime_t;

void driver_stm_init(driver_stm_cfg_t* stm_cfg, driver_stm_runtime_t* stm_runtime);
void driver_stm_start(driver_stm_runtime_t* stm_runtime);
uint64 driver_stm_getTick(driver_stm_runtime_t* stm_runtime);
uint64 driver_stm_getFrequencyHz(driver_stm_runtime_t* stm_runtime);
uint64 driver_stm_ticksToMicroseconds(driver_stm_runtime_t* stm_runtime, uint64 ticks);
uint64 driver_stm_ticksToMilliseconds(driver_stm_runtime_t* stm_runtime, uint64 ticks);
uint32 driver_stm_getTicksFromMicroseconds(driver_stm_runtime_t* stm_runtime, uint32 microseconds);
uint32 driver_stm_getTicksFromMilliseconds(driver_stm_runtime_t* stm_runtime, uint32 milliseconds);

#endif
