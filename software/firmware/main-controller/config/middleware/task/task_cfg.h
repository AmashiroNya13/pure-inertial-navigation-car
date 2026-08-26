#ifndef MAD_CIRCUITS_TASK_CFG_H
#define MAD_CIRCUITS_TASK_CFG_H

#include "../../../inc/driver/driver_src/driver_src.h"

typedef enum
{
    TASK1 = 0,
    TASK2 = 1,
    TASK3 = 2,
    TASK4 = 3,
    TASK5 = 4,
    TASK6 = 5,
    TASK7 = 6,
    TASK8 = 7,
    TASK_COUNT = 8,
} task_id_t;

typedef struct
{
    task_id_t task_id;
    driver_src_cfg_t src_cfg;
} task_cfg_t;

typedef struct
{
    task_id_t task_id;
    driver_src_runtime_t src_runtime;
    void (*task_callback)(void);
} task_runtime_t;

task_cfg_t* task_cfg_table_get(void);
task_runtime_t* task_runtime_table_get(void);

#endif
