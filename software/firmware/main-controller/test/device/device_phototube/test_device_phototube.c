#include "test_device_phototube.h"
#include "device_phototube.h"
#include "tools_print.h"

static uint8 test_device_phototube_print_divider = 0;
static boolean test_device_phototube_print_enabled = FALSE;

void test_device_phototube_group0_rx_callback(void)
{
    uint16 *buffer = device_phototube_dma_receive_buffer;
    test_device_phototube_print_divider++;
    test_device_phototube_print_enabled = ((test_device_phototube_print_divider % 4U) == 0U);

    if (test_device_phototube_print_enabled == FALSE)
    {
        return;
    }

    tools_printf("%u,%u,%u,%u,%u,%u,%u,%u,",
                 (unsigned int)buffer[0],
                 (unsigned int)buffer[1],
                 (unsigned int)buffer[2],
                 (unsigned int)buffer[3],
                 (unsigned int)buffer[4],
                 (unsigned int)buffer[5],
                 (unsigned int)buffer[6],
                 (unsigned int)buffer[7]);
}

void test_device_phototube_group1_rx_callback(void)
{
    uint16 *buffer = device_phototube_dma_receive_buffer;
    if (test_device_phototube_print_enabled == FALSE)
    {
        return;
    }

    tools_printf("%u,%u,%u,%u,%u,%u,",
                 (unsigned int)buffer[8],
                 (unsigned int)buffer[9],
                 (unsigned int)buffer[10],
                 (unsigned int)buffer[11],
                 (unsigned int)buffer[12],
                 (unsigned int)buffer[13]);
}

void test_device_phototube_group2_rx_callback(void)
{
    uint16 *buffer = device_phototube_dma_receive_buffer;
    if (test_device_phototube_print_enabled == FALSE)
    {
        return;
    }

    tools_printf("%u,%u\r\n",
                 (unsigned int)buffer[14],
                 (unsigned int)buffer[15]);
}
