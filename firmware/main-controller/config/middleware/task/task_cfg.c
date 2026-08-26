#include "./task_cfg.h"
#include "../../../isr/isr_config.h"
#include "../../../inc/driver/driver_dma/driver_dma.h"

task_cfg_t task_cfg_table[TASK_COUNT] =
{
    {
        .task_id = TASK1,
        .src_cfg.src = &MODULE_SRC.DMA.DMA[0].CH[DMA_CHANNEL_TASK1_INTERRUPT],
        .src_cfg.typOfService = ISR_CONFIG_TOS_TASK1_BUSINESS_INTERRUPT,
        .src_cfg.priority = ISR_CONFIG_PRIORITY_TASK1_BUSINESS_INTERRUPT,
    },
    {
        .task_id = TASK2,
        .src_cfg.src = &MODULE_SRC.DMA.DMA[0].CH[DMA_CHANNEL_TASK2_INTERRUPT],
        .src_cfg.typOfService = ISR_CONFIG_TOS_TASK2_BUSINESS_INTERRUPT,
        .src_cfg.priority = ISR_CONFIG_PRIORITY_TASK2_BUSINESS_INTERRUPT,
    },
    {
        .task_id = TASK3,
        .src_cfg.src = &MODULE_SRC.DMA.DMA[0].CH[DMA_CHANNEL_TASK3_INTERRUPT],
        .src_cfg.typOfService = ISR_CONFIG_TOS_TASK3_BUSINESS_INTERRUPT,
        .src_cfg.priority = ISR_CONFIG_PRIORITY_TASK3_BUSINESS_INTERRUPT,
    },
    {
        .task_id = TASK4,
        .src_cfg.src = &MODULE_SRC.DMA.DMA[0].CH[DMA_CHANNEL_TASK4_INTERRUPT],
        .src_cfg.typOfService = ISR_CONFIG_TOS_TASK4_BUSINESS_INTERRUPT,
        .src_cfg.priority = ISR_CONFIG_PRIORITY_TASK4_BUSINESS_INTERRUPT,
    },
    {
        .task_id = TASK5,
        .src_cfg.src = &MODULE_SRC.DMA.DMA[0].CH[DMA_CHANNEL_TASK5_INTERRUPT],
        .src_cfg.typOfService = ISR_CONFIG_TOS_TASK5_BUSINESS_INTERRUPT,
        .src_cfg.priority = ISR_CONFIG_PRIORITY_TASK5_BUSINESS_INTERRUPT,
    },
    {
        .task_id = TASK6,
        .src_cfg.src = &MODULE_SRC.DMA.DMA[0].CH[DMA_CHANNEL_TASK6_INTERRUPT],
        .src_cfg.typOfService = ISR_CONFIG_TOS_TASK6_BUSINESS_INTERRUPT,
        .src_cfg.priority = ISR_CONFIG_PRIORITY_TASK6_BUSINESS_INTERRUPT,
    },
    {
        .task_id = TASK7,
        .src_cfg.src = &MODULE_SRC.DMA.DMA[0].CH[DMA_CHANNEL_TASK7_INTERRUPT],
        .src_cfg.typOfService = ISR_CONFIG_TOS_TASK7_BUSINESS_INTERRUPT,
        .src_cfg.priority = ISR_CONFIG_PRIORITY_TASK7_BUSINESS_INTERRUPT,
    },
    {
        .task_id = TASK8,
        .src_cfg.src = &MODULE_SRC.DMA.DMA[0].CH[DMA_CHANNEL_TASK8_INTERRUPT],
        .src_cfg.typOfService = ISR_CONFIG_TOS_TASK8_BUSINESS_INTERRUPT,
        .src_cfg.priority = ISR_CONFIG_PRIORITY_TASK8_BUSINESS_INTERRUPT,
    },
};

task_runtime_t task_runtime_table[TASK_COUNT] =
{
    { .task_id = TASK1, .task_callback = NULL_PTR },
    { .task_id = TASK2, .task_callback = NULL_PTR },
    { .task_id = TASK3, .task_callback = NULL_PTR },
    { .task_id = TASK4, .task_callback = NULL_PTR },
    { .task_id = TASK5, .task_callback = NULL_PTR },
    { .task_id = TASK6, .task_callback = NULL_PTR },
    { .task_id = TASK7, .task_callback = NULL_PTR },
    { .task_id = TASK8, .task_callback = NULL_PTR },
};

task_cfg_t* task_cfg_table_get(void)
{
    return task_cfg_table;
}

task_runtime_t* task_runtime_table_get(void)
{
    return task_runtime_table;
}
