#include "../../../../inc/app/core/app_task_manager/app_task_manager_scheduler.h"

typedef struct
{
    app_task_manager_callback_t callback_table[16u];
    uint32 callback_count;
} app_task_manager_app_task_scheduler_runtime_t;

static app_task_manager_app_task_scheduler_runtime_t app_task_manager_app_task_scheduler_runtime[SYSTICK_COUNT];
static boolean app_task_manager_app_task_scheduler_registered[SYSTICK_COUNT];

static void app_task_manager_app_task_scheduler_run_all(sysTick_id_t sysTick_id)
{
    for (uint32 callback_id = 0u;
         callback_id < app_task_manager_app_task_scheduler_runtime[sysTick_id].callback_count;
         callback_id++)
    {
        app_task_manager_app_task_scheduler_runtime[sysTick_id].callback_table[callback_id]();
    }
}

static void app_task_manager_app_task_scheduler_sysTick1_dispatch(void)
{
    app_task_manager_app_task_scheduler_run_all(SYSTICK1);
}

static void (*const app_task_manager_app_task_scheduler_dispatch_table[SYSTICK_COUNT])(void) =
{
    app_task_manager_app_task_scheduler_sysTick1_dispatch,
};

void app_task_manager_app_task_scheduler_register(sysTick_id_t sysTick_id,
                                                  app_task_manager_callback_t app_task_scheduler_callback)
{
    if ((sysTick_id >= SYSTICK_COUNT) || (app_task_scheduler_callback == NULL_PTR))
    {
        return;
    }

    if (app_task_manager_app_task_scheduler_runtime[sysTick_id].callback_count
        >= (sizeof(app_task_manager_app_task_scheduler_runtime[sysTick_id].callback_table)
            / sizeof(app_task_manager_app_task_scheduler_runtime[sysTick_id].callback_table[0])))
    {
        return;
    }

    app_task_manager_app_task_scheduler_runtime[sysTick_id]
        .callback_table[app_task_manager_app_task_scheduler_runtime[sysTick_id].callback_count] =
        app_task_scheduler_callback;
    app_task_manager_app_task_scheduler_runtime[sysTick_id].callback_count++;

    if (app_task_manager_app_task_scheduler_registered[sysTick_id] == FALSE)
    {
        sysTick_register_callback(sysTick_id, app_task_manager_app_task_scheduler_dispatch_table[sysTick_id]);
        app_task_manager_app_task_scheduler_registered[sysTick_id] = TRUE;
    }
}
