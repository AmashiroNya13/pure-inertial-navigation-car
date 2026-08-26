#include "../../../inc/middleware/sysTick/sysTick.h"

void sysTick_delay_blockMicroseconds(sysTick_id_t sysTick_id, uint32 microseconds)
{
    uint64 start_tick = sysTick_getTick(sysTick_id);
    uint32 duration_tick = sysTick_getTicksFromMicroseconds(sysTick_id, microseconds);

    while ((sysTick_getTick(sysTick_id) - start_tick) < duration_tick)
    {
    }
}

void sysTick_delay_blockMilliseconds(sysTick_id_t sysTick_id, uint32 milliseconds)
{
    uint64 start_tick = sysTick_getTick(sysTick_id);
    uint32 duration_tick = sysTick_getTicksFromMilliseconds(sysTick_id, milliseconds);

    while ((sysTick_getTick(sysTick_id) - start_tick) < duration_tick)
    {
    }
}

void sysTick_delay_nonBlockingStartMicroseconds(sysTick_id_t sysTick_id,
                                                sysTick_delay_t* delay,
                                                uint32 microseconds)
{
    if (delay == NULL_PTR)
    {
        return;
    }

    delay->active = TRUE;
    delay->start_tick = sysTick_getTick(sysTick_id);
    delay->duration_tick = sysTick_getTicksFromMicroseconds(sysTick_id, microseconds);
}

void sysTick_delay_nonBlockingStartMilliseconds(sysTick_id_t sysTick_id,
                                                sysTick_delay_t* delay,
                                                uint32 milliseconds)
{
    if (delay == NULL_PTR)
    {
        return;
    }

    delay->active = TRUE;
    delay->start_tick = sysTick_getTick(sysTick_id);
    delay->duration_tick = sysTick_getTicksFromMilliseconds(sysTick_id, milliseconds);
}

boolean sysTick_delay_isElapsed(sysTick_id_t sysTick_id, sysTick_delay_t* delay)
{
    if (delay == NULL_PTR)
    {
        return FALSE;
    }

    if (delay->active == FALSE)
    {
        return TRUE;
    }

    if ((sysTick_getTick(sysTick_id) - delay->start_tick) < delay->duration_tick)
    {
        return FALSE;
    }

    delay->active = FALSE;
    return TRUE;
}

void sysTick_delay_reset(sysTick_delay_t* delay)
{
    if (delay == NULL_PTR)
    {
        return;
    }

    delay->active = FALSE;
    delay->start_tick = 0u;
    delay->duration_tick = 0u;
}

void sysTick_init(sysTick_id_t sysTick_id)
{
    sysTick_cfg_t* sysTick_cfg = sysTick_cfg_table_get();
    sysTick_runtime_t* sysTick_runtime = sysTick_runtime_table_get();

    driver_stm_init(&sysTick_cfg[sysTick_id].stm_cfg, &sysTick_runtime[sysTick_id].stm_runtime);
}

void sysTick_init_all(void)
{
    sysTick_init(SYSTICK1);
}

void sysTick_register_callback(sysTick_id_t sysTick_id, void (*sysTick_callback)(void))
{
    sysTick_runtime_t* sysTick_runtime = sysTick_runtime_table_get();

    sysTick_runtime[sysTick_id].sysTick_callback = sysTick_callback;
}

uint64 sysTick_getTick(sysTick_id_t sysTick_id)
{
    sysTick_runtime_t* sysTick_runtime = sysTick_runtime_table_get();

    return driver_stm_getTick(&sysTick_runtime[sysTick_id].stm_runtime);
}

uint64 sysTick_getFrequencyHz(sysTick_id_t sysTick_id)
{
    sysTick_runtime_t* sysTick_runtime = sysTick_runtime_table_get();

    return driver_stm_getFrequencyHz(&sysTick_runtime[sysTick_id].stm_runtime);
}

uint64 sysTick_ticksToMicroseconds(sysTick_id_t sysTick_id, uint64 ticks)
{
    sysTick_runtime_t* sysTick_runtime = sysTick_runtime_table_get();

    return driver_stm_ticksToMicroseconds(&sysTick_runtime[sysTick_id].stm_runtime, ticks);
}

uint64 sysTick_ticksToMilliseconds(sysTick_id_t sysTick_id, uint64 ticks)
{
    sysTick_runtime_t* sysTick_runtime = sysTick_runtime_table_get();

    return driver_stm_ticksToMilliseconds(&sysTick_runtime[sysTick_id].stm_runtime, ticks);
}

uint32 sysTick_getTicksFromMicroseconds(sysTick_id_t sysTick_id, uint32 microseconds)
{
    sysTick_runtime_t* sysTick_runtime = sysTick_runtime_table_get();

    return driver_stm_getTicksFromMicroseconds(&sysTick_runtime[sysTick_id].stm_runtime, microseconds);
}

uint32 sysTick_getTicksFromMilliseconds(sysTick_id_t sysTick_id, uint32 milliseconds)
{
    sysTick_runtime_t* sysTick_runtime = sysTick_runtime_table_get();

    return driver_stm_getTicksFromMilliseconds(&sysTick_runtime[sysTick_id].stm_runtime, milliseconds);
}
