#ifndef MAD_CIRCUITS_TASK_H
#define MAD_CIRCUITS_TASK_H

#include "../../../config/middleware/task/task_cfg.h"

void task_init(task_id_t task_id);
void task_init_all(void);
void task_register_callback(task_id_t task_id, void (*task_callback)(void));
void task_trap(task_id_t task_id);

#endif
