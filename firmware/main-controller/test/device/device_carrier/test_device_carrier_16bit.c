#include "test_device_carrier_16bit.h"
#include "device_carrier.h"
#include "driver_dma.h"
#include "tools_print.h"

#define TEST_BUF_SIZE_16 16u
#define BUF_CENTER_16     8u
#define TRIGGER_DELAY    50u

static uint16 src_buf_16[TEST_BUF_SIZE_16] = {1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 9u, 10u, 11u, 12u, 13u, 14u, 15u, 16u};
static uint16 dst_buf_16[TEST_BUF_SIZE_16];
static boolean task_set_16 = (boolean)0u;
static uint16 tick_16 = 0u;

static void print_array_16(const uint16* buf, uint16 len)
{
    uint16 i;
    for (i = 0u; i < len; i++)
    {
        tools_printf("%3u ", (unsigned int)buf[i]);
    }
    tools_printf("\r\n");
}

void test_device_carrier_16bit_callback(void)
{
    print_array_16(src_buf_16, TEST_BUF_SIZE_16);
    print_array_16(dst_buf_16, TEST_BUF_SIZE_16);
}

void test_device_carrier_16bit_run(void)
{
    device_carrier_runtime_t* runtime;

    if (task_set_16 == (boolean)0u)
    {
        device_carrier_setTask(DEVICE_CARRIER_1,
                               (uint32)(&src_buf_16[BUF_CENTER_16]),
                               (uint32)(&dst_buf_16[BUF_CENTER_16]));
        task_set_16 = (boolean)1u;
        tick_16 = 0u;
        return;
    }

    tick_16++;
    if (tick_16 >= TRIGGER_DELAY)
    {
        tick_16 = 0u;
        runtime = device_carrier_runtime_table_get();
        driver_dma_start(&runtime[DEVICE_CARRIER_1].dma_runtime);
    }
}
