#include "test_device_key.h"
#include "device_key.h"
#include "tools_print.h"

static device_key_state_t prev_state[DEVICE_KEY_COUNT];
static device_key_flag_t prev_flag[DEVICE_KEY_COUNT];

void test_device_key_scanner(void)
{
    static uint16 warmup = 50000u;

    device_key_scanner();

    if (warmup > 0u)
    {
        for (uint8 i = 0; i < DEVICE_KEY_COUNT; i++)
        {
            prev_state[i] = device_key_getState(i);
            prev_flag[i] = device_key_getFlag(i);
        }
        warmup--;
        return;
    }

    for (uint8 i = 0; i < DEVICE_KEY_COUNT; i++)
    {
        device_key_state_t state = device_key_getState(i);
        if (state != prev_state[i])
        {
            prev_state[i] = state;
            if (state == DEVICE_KEY_ONPRESS)
            {
                tools_printf("KEY%u PRESSED\r\n", (unsigned int)i);
            }
            else
            {
                tools_printf("KEY%u RELEASED\r\n", (unsigned int)i);
            }
        }

        device_key_flag_t flag = device_key_getFlag(i);
        if (flag != prev_flag[i])
        {
            prev_flag[i] = flag;
            if (flag == DEVICE_KEY_SHORT_PRESS_FLAG)
            {
                tools_printf("KEY%u SHORT_PRESS\r\n", (unsigned int)i);
            }
            else if (flag == DEVICE_KEY_LONG_PRESS_FLAG)
            {
                tools_printf("KEY%u LONG_PRESS\r\n", (unsigned int)i);
            }
            device_key_clearFlag(i);
        }
    }
}
