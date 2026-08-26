#include "../../../inc/middleware/task/task.h"

void task_init(task_id_t task_id)
{
    task_cfg_t* task_cfg = task_cfg_table_get();
    task_runtime_t* task_runtime = task_runtime_table_get();

    driver_src_init(&task_cfg[task_id].src_cfg, &task_runtime[task_id].src_runtime);
}

void task_init_all(void)
{
    task_init(TASK1);
    task_init(TASK2);
    task_init(TASK3);
    task_init(TASK4);
    task_init(TASK5);
    task_init(TASK6);
    task_init(TASK7);
    task_init(TASK8);
}

void task_register_callback(task_id_t task_id, void (*task_callback)(void))
{
    task_runtime_t* task_runtime = task_runtime_table_get();

    task_runtime[task_id].task_callback = task_callback;
}

void task_trap(task_id_t task_id)
{
    task_runtime_t* task_runtime = task_runtime_table_get();

    driver_src_SWTrap(&task_runtime[task_id].src_runtime);
}
