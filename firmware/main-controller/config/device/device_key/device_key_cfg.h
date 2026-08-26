#ifndef MAD_CIRCUITS_DEVICE_KEY_CFG_H
#define MAD_CIRCUITS_DEVICE_KEY_CFG_H

#include "../../../inc/driver/driver_gpio/driver_gpio.h"

#define DEVICE_KEY_LONG_PRESS_PERIOD 1000u
#define DEVICE_KEY_MAX_SHOCK_PERIOD 20u

typedef enum
{
    DEVICE_KEY_1 = 0,
    DEVICE_KEY_2 = 1,
    DEVICE_KEY_COUNT = 2,
} device_key_id_t;

typedef enum
{
    DEVICE_KEY_ONPRESS = FALSE,
    DEVICE_KEY_NOTPRESS = TRUE,
} device_key_state_t;

typedef enum
{
    DEVICE_KEY_RELEASE_FLAG,
    DEVICE_KEY_SHORT_PRESS_FLAG,
    DEVICE_KEY_LONG_PRESS_FLAG,
} device_key_flag_t;

typedef struct
{
    device_key_id_t key_id;
    driver_gpio_cfg_t gpio_cfg;
} device_key_cfg_t;

typedef struct
{
    device_key_id_t key_id;
    driver_gpio_runtime_t gpio_runtime;
    uint32 key_press_time;
    device_key_flag_t key_flag;
} device_key_runtime_t;

device_key_cfg_t* device_key_cfg_table_get(void);
device_key_runtime_t* device_key_runtime_table_get(void);

#endif
