#include "test_device_carrier.h"
#include "device_carrier.h"
#include "tools_print.h"

#define TEST_BUF_SIZE 16u
#define BUF_CENTER    8u

static uint8 src_buf[TEST_BUF_SIZE] = {1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 9u, 10u, 11u, 12u, 13u, 14u, 15u, 16u};
static uint8 dst_buf[TEST_BUF_SIZE];
static boolean transfer_pending = (boolean)0u;

static void print_array(const uint8* buf, uint16 len)
{
    uint16 i;
    for (i = 0u; i < len; i++)
    {
        tools_printf("%3u ", (unsigned int)buf[i]);
    }
    tools_printf("\r\n");
}

void test_device_carrier_callback(void)
{
    print_array(src_buf, TEST_BUF_SIZE);
    print_array(dst_buf, TEST_BUF_SIZE);
    transfer_pending = (boolean)0u;
}

void test_device_carrier_run(void)
{
    if (transfer_pending)
    {
        return;
    }

    device_carrier_setTask(DEVICE_CARRIER_1,
                           (uint32)(&src_buf[BUF_CENTER]),
                           (uint32)(&dst_buf[BUF_CENTER]));

    transfer_pending = (boolean)1u;
}

