#include "test_app_task_scheduler.h"

#include "../../../../inc/middleware/tools/tools_print/tools_print.h"
#include "../../../../inc/middleware/sysTick/sysTick.h"

void app_test_app_task_scheduler_tick_1_callback(void)
{
    uint64 tick = sysTick_getTick(SYSTICK1);
    tools_printf("SCH1_TICK=%llu ", (unsigned long long)tick);
}

void app_test_app_task_scheduler_tick_2_callback(void)
{
    uint64 tick = sysTick_getTick(SYSTICK1);
    tools_printf("SCH2_TICK=%llu ", (unsigned long long)tick);
}
