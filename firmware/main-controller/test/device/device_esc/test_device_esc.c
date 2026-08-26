#include "test_device_esc.h"
#include "../../../inc/device/device_esc/device_esc.h"
#include "../../../inc/middleware/tools/tools_print/tools_print.h"
#include "../../../inc/middleware/tools/tools_timing/tools_timing.h"

#define TEST_ESC_ARM_TIME_MS  3000u
#define TEST_ESC_STEP_TIME_MS 5000u
#define TEST_ESC_PERIOD_TICKS 50000u

#define TEST_ESC_DUTY_LOW_PERCENT  15u
#define TEST_ESC_DUTY_HIGH_PERCENT 50u

#define TEST_ESC_DUTY_FROM_PERCENT(percent) \
    ((uint32)((uint32)(percent) * TEST_ESC_PERIOD_TICKS / 100u))

static void test_device_esc_apply_all(uint32 duty)
{
    device_esc_set_duty_cycle(DEVICE_ESC_1, duty);
    device_esc_set_duty_cycle(DEVICE_ESC_2, duty);
    device_esc_set_duty_cycle(DEVICE_ESC_3, duty);
}

void test_device_esc_stop(void)
{
    test_device_esc_apply_all(0u);
}

void test_device_esc_run(void)
{
    static uint8 initialized = 0u;
    static uint64 arm_start_tick = 0u;
    static uint64 last_change_tick = 0u;
    static uint8 step = 0u;
    uint64 now = tools_timing_getTick(TOOLS_TIMING_1);
    uint64 arm_ticks = tools_timing_getTicksFromMilliseconds(TOOLS_TIMING_1, TEST_ESC_ARM_TIME_MS);
    uint64 step_ticks = tools_timing_getTicksFromMilliseconds(TOOLS_TIMING_1, TEST_ESC_STEP_TIME_MS);

    if (initialized == 0u)
    {
        initialized = 1u;
        arm_start_tick = now;
        last_change_tick = now;
        test_device_esc_apply_all(0u);
        tools_printf("ESC test start: 1000Hz period=%lu ticks, init duty=0 for %lu ms\r\n",
                     (unsigned long)TEST_ESC_PERIOD_TICKS,
                     (unsigned long)TEST_ESC_ARM_TIME_MS);
        return;
    }

    if ((now - arm_start_tick) < arm_ticks)
    {
        return;
    }

    if ((now - last_change_tick) < step_ticks)
    {
        return;
    }

    last_change_tick = now;

    uint32 duty;
    uint32 duty_percent;

    switch (step)
    {
        case 0u:
            duty_percent = TEST_ESC_DUTY_LOW_PERCENT;
            duty = TEST_ESC_DUTY_FROM_PERCENT(TEST_ESC_DUTY_LOW_PERCENT);
            break;
        default:
            duty_percent = TEST_ESC_DUTY_HIGH_PERCENT;
            duty = TEST_ESC_DUTY_FROM_PERCENT(TEST_ESC_DUTY_HIGH_PERCENT);
            break;
    }

    test_device_esc_apply_all(duty);
    tools_printf("ESC throttle=%lu%% duty=%lu/%lu on ESC1-ESC3\r\n",
                 (unsigned long)duty_percent,
                 (unsigned long)duty,
                 (unsigned long)TEST_ESC_PERIOD_TICKS);

    step = (uint8)((step + 1u) % 2u);
}
