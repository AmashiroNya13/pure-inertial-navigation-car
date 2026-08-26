#include "../../../inc/driver/driver_stm/driver_stm.h"

static uint64 driver_stm_ticksToUnit(driver_stm_runtime_t* stm_runtime, uint64 ticks, uint64 unit_scale)
{
    uint64 freq_hz = driver_stm_getFrequencyHz(stm_runtime);
    uint64 whole_units;
    uint64 remain_ticks;

    if (freq_hz == 0u)
    {
        return 0u;
    }

    whole_units = (ticks / freq_hz) * unit_scale;
    remain_ticks = ticks % freq_hz;

    return whole_units + ((remain_ticks * unit_scale) / freq_hz);
}

void driver_stm_init(driver_stm_cfg_t* stm_cfg, driver_stm_runtime_t* stm_runtime)
{
    IfxStm_Timer_init(&stm_runtime->stm_timer_modulehn, &stm_cfg->stm_timer_modulecfg);
    IfxStm_initCompare(stm_cfg->stm, &stm_cfg->stm_compare_cfg);
}

void driver_stm_start(driver_stm_runtime_t* stm_runtime)
{
    //IfxStm_Timer_run(&stm_runtime->stm_timer_modulehn);
}

uint64 driver_stm_getTick(driver_stm_runtime_t* stm_runtime)
{
    return IfxStm_get(stm_runtime->stm_timer_modulehn.stm);
}

uint64 driver_stm_getFrequencyHz(driver_stm_runtime_t* stm_runtime)
{
    return (uint64)IfxStm_getFrequency(stm_runtime->stm_timer_modulehn.stm);
}

uint64 driver_stm_ticksToMicroseconds(driver_stm_runtime_t* stm_runtime, uint64 ticks)
{
    return driver_stm_ticksToUnit(stm_runtime, ticks, 1000000u);
}

uint64 driver_stm_ticksToMilliseconds(driver_stm_runtime_t* stm_runtime, uint64 ticks)
{
    return driver_stm_ticksToUnit(stm_runtime, ticks, 1000u);
}

uint32 driver_stm_getTicksFromMicroseconds(driver_stm_runtime_t* stm_runtime, uint32 microseconds)
{
    return (uint32)IfxStm_getTicksFromMicroseconds(stm_runtime->stm_timer_modulehn.stm, microseconds);
}

uint32 driver_stm_getTicksFromMilliseconds(driver_stm_runtime_t* stm_runtime, uint32 milliseconds)
{
    return (uint32)IfxStm_getTicksFromMilliseconds(stm_runtime->stm_timer_modulehn.stm, milliseconds);
}
