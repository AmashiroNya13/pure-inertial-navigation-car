#include "test_middleware_tools_timing.h"
#include "tools_print.h"
#include "tools_timing.h"

#define TEST_TIMING_DELAY_COUNT 50000u

static void test_delay_func(void)
{
    for (volatile uint32 i = 0; i < TEST_TIMING_DELAY_COUNT; i++) { }
}

void test_middleware_tools_timing_pair(void)
{
    static uint32 step = 0u;

    if (step == 0u)
    {
        tools_println((const uint8*)"pair start");
        tools_timing_pair_and_print(TOOLS_TIMING_1, NULL_PTR);
        step = 1u;
    }
    else
    {
        test_delay_func();
        tools_print((const uint8*)"pair end: ");
        tools_timing_pair_and_print(TOOLS_TIMING_1, (const uint8*)"pair_test");
        step = 0u;
    }
}

void test_middleware_tools_timing_benchmark(void)
{
    static uint32 step = 0u;

    switch (step)
    {
    case 0u:
        tools_println((const uint8*)"--- benchmark test ---");
        tools_timing_benchmark_and_print(TOOLS_TIMING_1,
                                         test_delay_func,
                                         100u,
                                         FALSE,
                                         (const uint8*)"bm_int_on");
        break;
    case 1u:
        tools_timing_benchmark_and_print(TOOLS_TIMING_1,
                                         test_delay_func,
                                         100u,
                                         TRUE,
                                         (const uint8*)"bm_int_off");
        break;
    case 2u:
        tools_timing_benchmark_and_print(TOOLS_TIMING_1,
                                         test_delay_func,
                                         1000u,
                                         FALSE,
                                         (const uint8*)"bm_1000");
        break;
    case 3u:
        tools_timing_benchmark_and_print(TOOLS_TIMING_1,
                                         test_delay_func,
                                         10000u,
                                         FALSE,
                                         (const uint8*)"bm_10000");
        break;
    default:
        step = 0u;
        return;
    }

    step++;
}
