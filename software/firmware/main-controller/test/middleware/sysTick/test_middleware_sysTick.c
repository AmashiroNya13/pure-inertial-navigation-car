#include "test_middleware_sysTick.h"
#include "tools_print.h"
#include "tools_timing.h"

#define TEST_EMPTY_FUNC_LOOP_COUNT 10000u

#define TEST_WORK_FUNC_LOOP_COUNT 10000u

static void empty_func(void) { }

static void work_func(void)
{
    volatile uint32 i;

    for (i = 0u; i < 100u; i++)
    {
    }
}

void test_middleware_sysTick_empty_func_overhead(void)
{
    tools_timing_benchmark_and_print(TOOLS_TIMING_1,
                                     empty_func,
                                     TEST_EMPTY_FUNC_LOOP_COUNT,
                                     TRUE,
                                     (const uint8*)"TEST_EMPTY_FUNC");
}

void test_middleware_sysTick_work_func_overhead(void)
{
    tools_timing_benchmark_and_print(TOOLS_TIMING_1,
                                     work_func,
                                     TEST_WORK_FUNC_LOOP_COUNT,
                                     TRUE,
                                     (const uint8*)"TEST_WORK_FUNC");
}

void test_middleware_sysTick_isr_callback(void)
{
    static uint64 prev_tick = 0u;
    static boolean first = (boolean)1u;
    uint64 tick = tools_timing_getTick(TOOLS_TIMING_1);

    if (first)
    {
        tools_printf("SYSTICK_TICK=%llu FIRST\r\n", (unsigned long long)tick);
        first = (boolean)0u;
    }
    else
    {
        uint64 delta = tick - prev_tick;
        tools_printf("SYSTICK_TICK=%llu DELTA=%llu\r\n",
                     (unsigned long long)tick,
                     (unsigned long long)delta);
    }

    prev_tick = tick;
}
