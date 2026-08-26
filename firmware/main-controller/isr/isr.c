/**
 * @file isr.c
 * @brief Interrupt service routines.
 */

#include "./isr.h"
#include "./isr_config.h"

#include "../inc/device/device_carrier/device_carrier.h"
#include "../inc/device/device_debug/device_debug.h"
#include "../inc/device/device_esc/device_esc.h"
#include "../inc/device/device_imu/device_imu.h"
#include "../inc/device/device_magnetic_encoder/device_magnetic_encoder.h"
#include "../inc/device/device_phototube/device_phototube.h"

#include "../inc/middleware/sysTick/sysTick.h"
#include "../inc/middleware/task/task.h"

IFX_INTERRUPT(isr_device_debug_tx_interrupt, ISR_CONFIG_VECTABNUM_DEBUG_TX_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_DEBUG_TX_BUSINESS_INTERRUPT)
{
    IfxCpu_enableInterrupts();
    device_debug_runtime_t *debug_runtime = device_debug_runtime_table_get();
    IfxAsclin_Asc_isrTransmit(&debug_runtime[DEVICE_DEBUG_1].asclin_asc_runtime.asclin_asc_modulehn);
}

IFX_INTERRUPT(isr_device_debug_rx_interrupt, ISR_CONFIG_VECTABNUM_DEBUG_RX_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_DEBUG_RX_BUSINESS_INTERRUPT)
{
    IfxCpu_enableInterrupts();
    device_debug_runtime_t *debug_runtime = device_debug_runtime_table_get();
    IfxAsclin_Asc_isrReceive(&debug_runtime[DEVICE_DEBUG_1].asclin_asc_runtime.asclin_asc_modulehn);
    debug_runtime[DEVICE_DEBUG_1].device_debug_callback();
}

IFX_INTERRUPT(isr_device_esc_uart_tx_interrupt, ISR_CONFIG_VECTABNUM_ESC_UART_TX_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_ESC_UART_TX_BUSINESS_INTERRUPT)
{
    IfxCpu_enableInterrupts();
    device_esc_uart_runtime_t *esc_uart_runtime = device_esc_uart_runtime_get();
    IfxAsclin_Asc_isrTransmit(&esc_uart_runtime->asclin_asc_runtime.asclin_asc_modulehn);
}

IFX_INTERRUPT(isr_device_imu1_interrupt, ISR_CONFIG_VECTABNUM_IMU1_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_IMU1_BUSINESS_INTERRUPT)
{
    device_imu_runtime_t *imu_runtime = device_imu_runtime_table_get();
    boolean imu_updated = device_imu_drain_fifo(DEVICE_IMU_1);

    if ((imu_updated == TRUE) && (imu_runtime[DEVICE_IMU_1].device_imu_callback != NULL_PTR))
    {
        imu_runtime[DEVICE_IMU_1].device_imu_callback();
    }
}

IFX_INTERRUPT(isr_device_imu2_interrupt, ISR_CONFIG_VECTABNUM_IMU2_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_IMU2_BUSINESS_INTERRUPT)
{
    device_imu_runtime_t *imu_runtime = device_imu_runtime_table_get();
    boolean imu_updated = device_imu_drain_fifo(DEVICE_IMU_2);

    if ((imu_updated == TRUE) && (imu_runtime[DEVICE_IMU_2].device_imu_callback != NULL_PTR))
    {
        imu_runtime[DEVICE_IMU_2].device_imu_callback();
    }
}

/**
 * @brief 磁编码器1接收完成中断入口，转发编码器1业务回调。
 * @param[in] void 无参数。
 * @return void
 */
IFX_INTERRUPT(isr_device_magnetic_encoder1_interrupt, ISR_CONFIG_VECTABNUM_MAGNETIC_ENCODER1_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_MAGNETIC_ENCODER1_BUSINESS_INTERRUPT)
{
    device_magnetic_encoder_runtime_t *magnetic_encoder_runtime = device_magnetic_encoder_runtime_table_get();

    if (magnetic_encoder_runtime[DEVICE_MAGNETIC_ENCODER_1].device_magnetic_encoder_callback != NULL_PTR)
    {
        magnetic_encoder_runtime[DEVICE_MAGNETIC_ENCODER_1].device_magnetic_encoder_callback();
    }
}

/**
 * @brief 磁编码器2接收完成中断入口，转发编码器2业务回调。
 * @param[in] void 无参数。
 * @return void
 */
IFX_INTERRUPT(isr_device_magnetic_encoder2_interrupt, ISR_CONFIG_VECTABNUM_MAGNETIC_ENCODER2_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_MAGNETIC_ENCODER2_BUSINESS_INTERRUPT)
{
    device_magnetic_encoder_runtime_t *magnetic_encoder_runtime = device_magnetic_encoder_runtime_table_get();

    if (magnetic_encoder_runtime[DEVICE_MAGNETIC_ENCODER_2].device_magnetic_encoder_callback != NULL_PTR)
    {
        magnetic_encoder_runtime[DEVICE_MAGNETIC_ENCODER_2].device_magnetic_encoder_callback();
    }
}

IFX_INTERRUPT(isr_device_phototube_switch_interrupt, ISR_CONFIG_VECTABNUM_PHOTOTUBE_SWITCH_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_PHOTOTUBE_SWITCH_BUSINESS_INTERRUPT)
{
    device_phototube_runtime_t *phototube_runtime = device_phototube_runtime_table_get();

    if (device_phototube_is_enabled() == FALSE)
    {
        return;
    }

    driver_vadc_scanStart(&phototube_runtime[DEVICE_PHOTOTUBE_1].vadc_runtime);
    driver_vadc_scanStart(&phototube_runtime[DEVICE_PHOTOTUBE_9].vadc_runtime);
    driver_vadc_scanStart(&phototube_runtime[DEVICE_PHOTOTUBE_15].vadc_runtime);
}

IFX_INTERRUPT(isr_device_phototube_group0_interrupt, ISR_CONFIG_VECTABNUM_PHOTOTUBE_GROUP0_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_PHOTOTUBE_GROUP0_BUSINESS_INTERRUPT)
{
    device_phototube_runtime_t *phototube_runtime = device_phototube_runtime_table_get();
    device_phototube_group_runtime_t *group_runtime = phototube_runtime[DEVICE_PHOTOTUBE_1].phototube_group_runtime;

    device_phototube_group_result_buffer_update(DEVICE_PHOTOTUBE_GROUP_0);

    if (group_runtime->device_phototube_group_callback != NULL_PTR)
    {
        group_runtime->device_phototube_group_callback();
    }
}

IFX_INTERRUPT(isr_device_phototube_group1_interrupt, ISR_CONFIG_VECTABNUM_PHOTOTUBE_GROUP1_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_PHOTOTUBE_GROUP1_BUSINESS_INTERRUPT)
{
    device_phototube_runtime_t *phototube_runtime = device_phototube_runtime_table_get();
    device_phototube_group_runtime_t *group_runtime = phototube_runtime[DEVICE_PHOTOTUBE_9].phototube_group_runtime;

    device_phototube_group_result_buffer_update(DEVICE_PHOTOTUBE_GROUP_1);

    if (group_runtime->device_phototube_group_callback != NULL_PTR)
    {
        group_runtime->device_phototube_group_callback();
    }
}

IFX_INTERRUPT(isr_device_phototube_group2_interrupt, ISR_CONFIG_VECTABNUM_PHOTOTUBE_GROUP2_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_PHOTOTUBE_GROUP2_BUSINESS_INTERRUPT)
{
    device_phototube_runtime_t *phototube_runtime = device_phototube_runtime_table_get();
    device_phototube_group_runtime_t *group_runtime = phototube_runtime[DEVICE_PHOTOTUBE_15].phototube_group_runtime;

    device_phototube_group_result_buffer_update(DEVICE_PHOTOTUBE_GROUP_2);

    if (group_runtime->device_phototube_group_callback != NULL_PTR)
    {
        group_runtime->device_phototube_group_callback();
    }
}

IFX_INTERRUPT(isr_task1_interrupt, ISR_CONFIG_VECTABNUM_TASK1_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_TASK1_BUSINESS_INTERRUPT)
{
    task_runtime_t *task_runtime = task_runtime_table_get();

    if (task_runtime[TASK1].task_callback != NULL_PTR)
    {
        task_runtime[TASK1].task_callback();
    }
}

IFX_INTERRUPT(isr_task2_interrupt, ISR_CONFIG_VECTABNUM_TASK2_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_TASK2_BUSINESS_INTERRUPT)
{
    task_runtime_t *task_runtime = task_runtime_table_get();

    if (task_runtime[TASK2].task_callback != NULL_PTR)
    {
        task_runtime[TASK2].task_callback();
    }
}

IFX_INTERRUPT(isr_task3_interrupt, ISR_CONFIG_VECTABNUM_TASK3_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_TASK3_BUSINESS_INTERRUPT)
{
    task_runtime_t *task_runtime = task_runtime_table_get();

    if (task_runtime[TASK3].task_callback != NULL_PTR)
    {
        task_runtime[TASK3].task_callback();
    }
}

IFX_INTERRUPT(isr_task4_interrupt, ISR_CONFIG_VECTABNUM_TASK4_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_TASK4_BUSINESS_INTERRUPT)
{
    task_runtime_t *task_runtime = task_runtime_table_get();

    if (task_runtime[TASK4].task_callback != NULL_PTR)
    {
        task_runtime[TASK4].task_callback();
    }
}

IFX_INTERRUPT(isr_task5_interrupt, ISR_CONFIG_VECTABNUM_TASK5_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_TASK5_BUSINESS_INTERRUPT)
{
    task_runtime_t *task_runtime = task_runtime_table_get();

    if (task_runtime[TASK5].task_callback != NULL_PTR)
    {
        task_runtime[TASK5].task_callback();
    }
}

IFX_INTERRUPT(isr_task6_interrupt, ISR_CONFIG_VECTABNUM_TASK6_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_TASK6_BUSINESS_INTERRUPT)
{
    task_runtime_t *task_runtime = task_runtime_table_get();

    if (task_runtime[TASK6].task_callback != NULL_PTR)
    {
        task_runtime[TASK6].task_callback();
    }
}

IFX_INTERRUPT(isr_task7_interrupt, ISR_CONFIG_VECTABNUM_TASK7_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_TASK7_BUSINESS_INTERRUPT)
{
    task_runtime_t *task_runtime = task_runtime_table_get();

    if (task_runtime[TASK7].task_callback != NULL_PTR)
    {
        task_runtime[TASK7].task_callback();
    }
}

IFX_INTERRUPT(isr_task8_interrupt, ISR_CONFIG_VECTABNUM_TASK8_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_TASK8_BUSINESS_INTERRUPT)
{
    task_runtime_t *task_runtime = task_runtime_table_get();

    if (task_runtime[TASK8].task_callback != NULL_PTR)
    {
        task_runtime[TASK8].task_callback();
    }
}

IFX_INTERRUPT(isr_sysTick1_interrupt, ISR_CONFIG_VECTABNUM_SYSTICK1_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_SYSTICK1_BUSINESS_INTERRUPT)
{
    IfxCpu_enableInterrupts();
    sysTick_runtime_t *sysTick_runtime = sysTick_runtime_table_get();
    uint32 match_tick = (uint32)IfxStm_get(sysTick_runtime[SYSTICK1].stm_runtime.stm_timer_modulehn.stm) + SYSTICK_TICK;
    sysTick_runtime[SYSTICK1].sysTick_callback();
    IfxStm_clearCompareFlag(sysTick_runtime[SYSTICK1].stm_runtime.stm_timer_modulehn.stm, IfxStm_Comparator_0);
    IfxStm_updateCompare(sysTick_runtime[SYSTICK1].stm_runtime.stm_timer_modulehn.stm, IfxStm_Comparator_0, match_tick);
}

IFX_INTERRUPT(isr_device_carrier1_interrupt, ISR_CONFIG_VECTABNUM_CARRIER1_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_CARRIER1_BUSINESS_INTERRUPT)
{
    device_carrier_runtime_t *carrier_runtime = device_carrier_runtime_table_get();

    if (carrier_runtime[DEVICE_CARRIER_1].device_carrier_callback != NULL_PTR)
    {
        carrier_runtime[DEVICE_CARRIER_1].device_carrier_callback();
    }
}

IFX_INTERRUPT(isr_device_carrier2_interrupt, ISR_CONFIG_VECTABNUM_CARRIER2_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_CARRIER2_BUSINESS_INTERRUPT)
{
    device_carrier_runtime_t *carrier_runtime = device_carrier_runtime_table_get();

    if (carrier_runtime[DEVICE_CARRIER_2].device_carrier_callback != NULL_PTR)
    {
        carrier_runtime[DEVICE_CARRIER_2].device_carrier_callback();
    }
}

IFX_INTERRUPT(isr_device_carrier3_interrupt, ISR_CONFIG_VECTABNUM_CARRIER3_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_CARRIER3_BUSINESS_INTERRUPT)
{
    device_carrier_runtime_t *carrier_runtime = device_carrier_runtime_table_get();

    if (carrier_runtime[DEVICE_CARRIER_3].device_carrier_callback != NULL_PTR)
    {
        carrier_runtime[DEVICE_CARRIER_3].device_carrier_callback();
    }
}

IFX_INTERRUPT(isr_device_carrier4_interrupt, ISR_CONFIG_VECTABNUM_CARRIER4_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_CARRIER4_BUSINESS_INTERRUPT)
{
    device_carrier_runtime_t *carrier_runtime = device_carrier_runtime_table_get();

    if (carrier_runtime[DEVICE_CARRIER_4].device_carrier_callback != NULL_PTR)
    {
        carrier_runtime[DEVICE_CARRIER_4].device_carrier_callback();
    }
}

IFX_INTERRUPT(isr_device_carrier5_interrupt, ISR_CONFIG_VECTABNUM_CARRIER5_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_CARRIER5_BUSINESS_INTERRUPT)
{
    device_carrier_runtime_t *carrier_runtime = device_carrier_runtime_table_get();

    if (carrier_runtime[DEVICE_CARRIER_5].device_carrier_callback != NULL_PTR)
    {
        carrier_runtime[DEVICE_CARRIER_5].device_carrier_callback();
    }
}

IFX_INTERRUPT(isr_device_carrier6_interrupt, ISR_CONFIG_VECTABNUM_CARRIER6_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_CARRIER6_BUSINESS_INTERRUPT)
{
    device_carrier_runtime_t *carrier_runtime = device_carrier_runtime_table_get();

    if (carrier_runtime[DEVICE_CARRIER_6].device_carrier_callback != NULL_PTR)
    {
        carrier_runtime[DEVICE_CARRIER_6].device_carrier_callback();
    }
}

IFX_INTERRUPT(isr_device_carrier7_interrupt, ISR_CONFIG_VECTABNUM_CARRIER7_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_CARRIER7_BUSINESS_INTERRUPT)
{
    device_carrier_runtime_t *carrier_runtime = device_carrier_runtime_table_get();

    if (carrier_runtime[DEVICE_CARRIER_7].device_carrier_callback != NULL_PTR)
    {
        carrier_runtime[DEVICE_CARRIER_7].device_carrier_callback();
    }
}

IFX_INTERRUPT(isr_device_carrier8_interrupt, ISR_CONFIG_VECTABNUM_CARRIER8_BUSINESS_INTERRUPT, ISR_CONFIG_PRIORITY_CARRIER8_BUSINESS_INTERRUPT)
{
    device_carrier_runtime_t *carrier_runtime = device_carrier_runtime_table_get();

    if (carrier_runtime[DEVICE_CARRIER_8].device_carrier_callback != NULL_PTR)
    {
        carrier_runtime[DEVICE_CARRIER_8].device_carrier_callback();
    }
}
