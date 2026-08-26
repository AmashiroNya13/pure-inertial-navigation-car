#include "test_app_task_asynchronous_phototube.h"

#include "../../../../inc/middleware/tools/tools_print/tools_print.h"
#include "../../../../inc/device/device_phototube/device_phototube.h"

void app_test_app_task_asynchronous_phototube_print_all(void)
{
    static uint8 call_count = 0;
    device_phototube_runtime_t *runtime = device_phototube_runtime_table_get();

    call_count++;
    if ((call_count % 10U) != 0U)
    {
        return;
    }

    tools_printf("{pt}:%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                 (unsigned int)(*runtime[DEVICE_PHOTOTUBE_1].dma_receive_buffer),
                 (unsigned int)(*runtime[DEVICE_PHOTOTUBE_2].dma_receive_buffer),
                 (unsigned int)(*runtime[DEVICE_PHOTOTUBE_3].dma_receive_buffer),
                 (unsigned int)(*runtime[DEVICE_PHOTOTUBE_4].dma_receive_buffer),
                 (unsigned int)(*runtime[DEVICE_PHOTOTUBE_5].dma_receive_buffer),
                 (unsigned int)(*runtime[DEVICE_PHOTOTUBE_6].dma_receive_buffer),
                 (unsigned int)(*runtime[DEVICE_PHOTOTUBE_7].dma_receive_buffer),
                 (unsigned int)(*runtime[DEVICE_PHOTOTUBE_8].dma_receive_buffer),
                 (unsigned int)(*runtime[DEVICE_PHOTOTUBE_9].dma_receive_buffer),
                 (unsigned int)(*runtime[DEVICE_PHOTOTUBE_10].dma_receive_buffer),
                 (unsigned int)(*runtime[DEVICE_PHOTOTUBE_11].dma_receive_buffer),
                 (unsigned int)(*runtime[DEVICE_PHOTOTUBE_12].dma_receive_buffer),
                 (unsigned int)(*runtime[DEVICE_PHOTOTUBE_13].dma_receive_buffer),
                 (unsigned int)(*runtime[DEVICE_PHOTOTUBE_14].dma_receive_buffer),
                 (unsigned int)(*runtime[DEVICE_PHOTOTUBE_15].dma_receive_buffer),
                 (unsigned int)(*runtime[DEVICE_PHOTOTUBE_16].dma_receive_buffer));
}
