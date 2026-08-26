#include "../../../../inc/app/app_task/app_task_scheduler/app_task_gui.h"

#include "../../../../config/app/app_task/app_task_scheduler/app_task_scheduler_cfg.h"
#include "../../../../config/app/gui/gui_cfg.h"
#include "../../../../inc/app/gui/gui.h"

typedef struct
{
    uint64 last_run_tick;
    boolean initialized;
} app_task_scheduler_gui_runtime_t;

static app_task_scheduler_gui_runtime_t app_task_scheduler_gui_runtime;

static void app_task_scheduler_gui_runtime_ensure_initialized(void)
{
    app_task_scheduler_cfg_t* app_task_scheduler_cfg = app_task_scheduler_cfg_get();

    if (app_task_scheduler_gui_runtime.initialized == TRUE)
    {
        return;
    }

    app_task_scheduler_gui_runtime.last_run_tick = sysTick_getTick(app_task_scheduler_cfg->sysTick_id);
    app_task_scheduler_gui_runtime.initialized = TRUE;
}

void app_task_scheduler_gui_run(void)
{
    app_task_scheduler_cfg_t* app_task_scheduler_cfg = app_task_scheduler_cfg_get();
    gui_cfg_t* gui_cfg = gui_cfg_get();
    uint64 current_tick;
    uint32 gui_period_tick;

    app_task_scheduler_gui_runtime_ensure_initialized();
    current_tick = sysTick_getTick(app_task_scheduler_cfg->sysTick_id);
    gui_period_tick = sysTick_getTicksFromMilliseconds(app_task_scheduler_cfg->sysTick_id,
                                                       gui_cfg->scheduler_period_ms);

    if ((current_tick - app_task_scheduler_gui_runtime.last_run_tick) < gui_period_tick)
    {
        return;
    }

    app_task_scheduler_gui_runtime.last_run_tick = current_tick;
    gui_scheduler_callback();
}
