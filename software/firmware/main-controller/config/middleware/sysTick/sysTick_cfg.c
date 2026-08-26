#include "./sysTick_cfg.h"
#include "../../../isr/isr_config.h"

sysTick_cfg_t sysTick_cfg_table[SYSTICK_COUNT] =
{
    {
        .sysTick_id = SYSTICK1,

        .stm_cfg.stm = &MODULE_STM0,

        .stm_cfg.stm_timer_modulecfg.stm = &MODULE_STM0,
        .stm_cfg.stm_timer_modulecfg.comparator = IfxStm_Comparator_0,
        .stm_cfg.stm_timer_modulecfg.base.frequency = 10.0f,
        .stm_cfg.stm_timer_modulecfg.base.isrPriority = 0,
        .stm_cfg.stm_timer_modulecfg.base.isrProvider = IfxSrc_Tos_cpu0,
        .stm_cfg.stm_timer_modulecfg.base.minResolution = 0.0f,
        .stm_cfg.stm_timer_modulecfg.base.trigger.enabled = FALSE,
        .stm_cfg.stm_timer_modulecfg.base.trigger.triggerPoint = 0,
        .stm_cfg.stm_timer_modulecfg.base.trigger.isrPriority = 0,
        .stm_cfg.stm_timer_modulecfg.base.trigger.isrProvider = IfxSrc_Tos_cpu0,
        .stm_cfg.stm_timer_modulecfg.base.trigger.outputMode = IfxPort_OutputMode_none,
        .stm_cfg.stm_timer_modulecfg.base.trigger.outputDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
        .stm_cfg.stm_timer_modulecfg.base.trigger.risingEdgeAtPeriod = FALSE,
        .stm_cfg.stm_timer_modulecfg.base.trigger.outputEnabled = FALSE,
        .stm_cfg.stm_timer_modulecfg.base.countDir = IfxStdIf_Timer_CountDir_up,
        .stm_cfg.stm_timer_modulecfg.base.startOffset = 0.0f,

        .stm_cfg.stm_compare_cfg.comparator = IfxStm_Comparator_0,
        .stm_cfg.stm_compare_cfg.comparatorInterrupt = IfxStm_ComparatorInterrupt_ir0,
        .stm_cfg.stm_compare_cfg.compareOffset = IfxStm_ComparatorOffset_0,
        .stm_cfg.stm_compare_cfg.compareSize = IfxStm_ComparatorSize_32Bits,
        .stm_cfg.stm_compare_cfg.ticks = SYSTICK_TICK,
        .stm_cfg.stm_compare_cfg.triggerPriority = ISR_CONFIG_PRIORITY_SYSTICK1_BUSINESS_INTERRUPT,
        .stm_cfg.stm_compare_cfg.typeOfService = ISR_CONFIG_TOS_SYSTICK1_BUSINESS_INTERRUPT,
    },
};

sysTick_runtime_t sysTick_runtime_table[SYSTICK_COUNT] =
{
    {
        .sysTick_id = SYSTICK1,
        .sysTick_callback = NULL_PTR,
    },
};

sysTick_cfg_t* sysTick_cfg_table_get(void)
{
    return sysTick_cfg_table;
}

sysTick_runtime_t* sysTick_runtime_table_get(void)
{
    return sysTick_runtime_table;
}
