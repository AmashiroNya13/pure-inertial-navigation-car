#include "../../../../inc/middleware/tools/tools_timing/tools_timing.h"

#include "../../../../inc/middleware/tools/tools_print/tools_print.h"

static boolean tools_timing_pair_started[TOOLS_TIMING_COUNT];
static uint64 tools_timing_pair_start_tick[TOOLS_TIMING_COUNT];

static void tools_timing_empty_callback(void)
{
}

static uint64 tools_timing_measure_ticks(tools_timing_id_t tools_timing_id,
                                         tools_timing_callback_t tools_timing_callback,
                                         uint32 loop_count,
                                         boolean disable_interrupts)
{
    boolean interrupt_state = FALSE;
    uint64 start_tick;
    uint64 end_tick;

    if (disable_interrupts != FALSE)
    {
        interrupt_state = IfxCpu_disableInterrupts();
    }

    start_tick = tools_timing_getTick(tools_timing_id);

    for (uint32 i = 0u; i < loop_count; i++)
    {
        tools_timing_callback();
    }

    end_tick = tools_timing_getTick(tools_timing_id);

    if (disable_interrupts != FALSE)
    {
        IfxCpu_restoreInterrupts(interrupt_state);
    }

    return end_tick - start_tick;
}

void tools_timing_init(tools_timing_id_t tools_timing_id)
{
    tools_timing_cfg_t* tools_timing_cfg = tools_timing_cfg_table_get();
    tools_timing_runtime_t* tools_timing_runtime = tools_timing_runtime_table_get();

    driver_stm_init(&tools_timing_cfg[tools_timing_id].stm_cfg, &tools_timing_runtime[tools_timing_id].stm_runtime);
}

void tools_timing_init_all(void)
{
    tools_timing_init(TOOLS_TIMING_1);
}

uint64 tools_timing_getTick(tools_timing_id_t tools_timing_id)
{
    tools_timing_runtime_t* tools_timing_runtime = tools_timing_runtime_table_get();

    return driver_stm_getTick(&tools_timing_runtime[tools_timing_id].stm_runtime);
}

uint64 tools_timing_getFrequencyHz(tools_timing_id_t tools_timing_id)
{
    tools_timing_runtime_t* tools_timing_runtime = tools_timing_runtime_table_get();

    return driver_stm_getFrequencyHz(&tools_timing_runtime[tools_timing_id].stm_runtime);
}

uint64 tools_timing_ticksToMicroseconds(tools_timing_id_t tools_timing_id, uint64 ticks)
{
    tools_timing_runtime_t* tools_timing_runtime = tools_timing_runtime_table_get();

    return driver_stm_ticksToMicroseconds(&tools_timing_runtime[tools_timing_id].stm_runtime, ticks);
}

uint64 tools_timing_ticksToMilliseconds(tools_timing_id_t tools_timing_id, uint64 ticks)
{
    tools_timing_runtime_t* tools_timing_runtime = tools_timing_runtime_table_get();

    return driver_stm_ticksToMilliseconds(&tools_timing_runtime[tools_timing_id].stm_runtime, ticks);
}

uint32 tools_timing_getTicksFromMicroseconds(tools_timing_id_t tools_timing_id, uint32 microseconds)
{
    tools_timing_runtime_t* tools_timing_runtime = tools_timing_runtime_table_get();

    return driver_stm_getTicksFromMicroseconds(&tools_timing_runtime[tools_timing_id].stm_runtime, microseconds);
}

uint32 tools_timing_getTicksFromMilliseconds(tools_timing_id_t tools_timing_id, uint32 milliseconds)
{
    tools_timing_runtime_t* tools_timing_runtime = tools_timing_runtime_table_get();

    return driver_stm_getTicksFromMilliseconds(&tools_timing_runtime[tools_timing_id].stm_runtime, milliseconds);
}

void tools_timing_pair_and_print(tools_timing_id_t tools_timing_id,
                                 const uint8* tag)
{
    uint64 current_tick = tools_timing_getTick(tools_timing_id);
    uint64 delta_tick;
    uint64 delta_us;
    uint64 delta_ms;

    if (tools_timing_pair_started[tools_timing_id] == FALSE)
    {
        tools_timing_pair_start_tick[tools_timing_id] = current_tick;
        tools_timing_pair_started[tools_timing_id] = TRUE;
        return;
    }

    delta_tick = current_tick - tools_timing_pair_start_tick[tools_timing_id];
    delta_us = tools_timing_ticksToMicroseconds(tools_timing_id, delta_tick);
    delta_ms = tools_timing_ticksToMilliseconds(tools_timing_id, delta_tick);

    tools_timing_pair_started[tools_timing_id] = FALSE;

    if ((tag != NULL_PTR) && (tag[0] != '\0'))
    {
        tools_printf("%s PAIR_TK=%llu PAIR_US=%llu PAIR_MS=%llu\r\n",
                     tag,
                     (unsigned long long)delta_tick,
                     (unsigned long long)delta_us,
                     (unsigned long long)delta_ms);
    }
    else
    {
        tools_printf("PAIR_TK=%llu PAIR_US=%llu PAIR_MS=%llu\r\n",
                     (unsigned long long)delta_tick,
                     (unsigned long long)delta_us,
                     (unsigned long long)delta_ms);
    }
}

void tools_timing_benchmark_and_print(tools_timing_id_t tools_timing_id,
                                      tools_timing_callback_t tools_timing_callback,
                                      uint32 loop_count,
                                      boolean disable_interrupts,
                                      const uint8* tag)
{
    uint64 raw_total_tick;
    uint64 raw_average_tick;
    uint64 base_total_tick;
    uint64 base_average_tick;
    uint64 net_total_tick;
    uint64 net_average_tick;
    uint64 raw_total_us;
    uint64 raw_average_us;
    uint64 net_total_us;
    uint64 net_average_us;
    uint64 net_average_ms;

    if ((tools_timing_callback == NULL_PTR) || (loop_count == 0u))
    {
        return;
    }

    base_total_tick = tools_timing_measure_ticks(tools_timing_id,
                                                 tools_timing_empty_callback,
                                                 loop_count,
                                                 disable_interrupts);

    raw_total_tick = tools_timing_measure_ticks(tools_timing_id,
                                                tools_timing_callback,
                                                loop_count,
                                                disable_interrupts);

    raw_average_tick = raw_total_tick / loop_count;
    base_average_tick = base_total_tick / loop_count;

    if (raw_total_tick >= base_total_tick)
    {
        net_total_tick = raw_total_tick - base_total_tick;
    }
    else
    {
        net_total_tick = 0u;
    }

    if (raw_average_tick >= base_average_tick)
    {
        net_average_tick = raw_average_tick - base_average_tick;
    }
    else
    {
        net_average_tick = 0u;
    }

    raw_total_us = tools_timing_ticksToMicroseconds(tools_timing_id, raw_total_tick);
    raw_average_us = tools_timing_ticksToMicroseconds(tools_timing_id, raw_average_tick);
    net_total_us = tools_timing_ticksToMicroseconds(tools_timing_id, net_total_tick);
    net_average_us = tools_timing_ticksToMicroseconds(tools_timing_id, net_average_tick);
    net_average_ms = tools_timing_ticksToMilliseconds(tools_timing_id, net_average_tick);

    if ((tag != NULL_PTR) && (tag[0] != '\0'))
    {
        tools_printf("%s LP=%lu RAW_TK=%llu BASE_TK=%llu NET_TK=%llu AVG_TK=%llu RAW_US=%llu NET_US=%llu AVG_US=%llu AVG_MS=%llu INT=%u\r\n",
                     tag,
                     (unsigned long)loop_count,
                     (unsigned long long)raw_total_tick,
                     (unsigned long long)base_total_tick,
                     (unsigned long long)net_total_tick,
                     (unsigned long long)net_average_tick,
                     (unsigned long long)raw_total_us,
                     (unsigned long long)net_total_us,
                     (unsigned long long)net_average_us,
                     (unsigned long long)net_average_ms,
                     (unsigned int)((disable_interrupts != FALSE) ? 1u : 0u));
    }
    else
    {
        tools_printf("LP=%lu RAW_TK=%llu BASE_TK=%llu NET_TK=%llu AVG_TK=%llu RAW_US=%llu NET_US=%llu AVG_US=%llu AVG_MS=%llu INT=%u\r\n",
                     (unsigned long)loop_count,
                     (unsigned long long)raw_total_tick,
                     (unsigned long long)base_total_tick,
                     (unsigned long long)net_total_tick,
                     (unsigned long long)net_average_tick,
                     (unsigned long long)raw_total_us,
                     (unsigned long long)net_total_us,
                     (unsigned long long)net_average_us,
                     (unsigned long long)net_average_ms,
                     (unsigned int)((disable_interrupts != FALSE) ? 1u : 0u));
    }
}
