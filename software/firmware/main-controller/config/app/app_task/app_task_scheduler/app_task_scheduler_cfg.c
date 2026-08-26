#include "./app_task_scheduler_cfg.h"

static app_task_scheduler_cfg_t app_task_scheduler_cfg =
{
    .sysTick_id = SYSTICK1,
};

app_task_scheduler_cfg_t* app_task_scheduler_cfg_get(void)
{
    return &app_task_scheduler_cfg;
}
