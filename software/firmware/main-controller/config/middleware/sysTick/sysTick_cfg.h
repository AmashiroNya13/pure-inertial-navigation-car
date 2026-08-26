#ifndef MAD_CIRCUITS_SYSTICK_CFG_H
#define MAD_CIRCUITS_SYSTICK_CFG_H

#include "../../../inc/driver/driver_stm/driver_stm.h"

#define SYSTICK_TICK 350000u

typedef enum
{
    SYSTICK1 = 0,
    SYSTICK_COUNT = 1,
} sysTick_id_t;

typedef struct
{
    sysTick_id_t sysTick_id;
    driver_stm_cfg_t stm_cfg;
} sysTick_cfg_t;

typedef struct
{
    sysTick_id_t sysTick_id;
    driver_stm_runtime_t stm_runtime;
    void (*sysTick_callback)(void);
} sysTick_runtime_t;

sysTick_cfg_t* sysTick_cfg_table_get(void);
sysTick_runtime_t* sysTick_runtime_table_get(void);

#endif
