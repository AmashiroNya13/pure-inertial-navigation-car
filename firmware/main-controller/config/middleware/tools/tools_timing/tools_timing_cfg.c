#include "./tools_timing_cfg.h"

tools_timing_cfg_t tools_timing_cfg_table[TOOLS_TIMING_COUNT] =
{
    {
        .tools_timing_id = TOOLS_TIMING_1,

        .stm_cfg.stm = &MODULE_STM1,

        .stm_cfg.stm_timer_modulecfg.stm = &MODULE_STM1,
        .stm_cfg.stm_timer_modulecfg.comparator = IfxStm_Comparator_0,
        .stm_cfg.stm_timer_modulecfg.base.frequency = 10.0f,
        .stm_cfg.stm_timer_modulecfg.base.isrPriority = 0u,
        .stm_cfg.stm_timer_modulecfg.base.isrProvider = IfxSrc_Tos_cpu0,
        .stm_cfg.stm_timer_modulecfg.base.minResolution = 0.0f,
        .stm_cfg.stm_timer_modulecfg.base.trigger.enabled = FALSE,
        .stm_cfg.stm_timer_modulecfg.base.trigger.triggerPoint = 0u,
        .stm_cfg.stm_timer_modulecfg.base.trigger.isrPriority = 0u,
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
        .stm_cfg.stm_compare_cfg.ticks = 35000u,
        .stm_cfg.stm_compare_cfg.triggerPriority = 0u,
        .stm_cfg.stm_compare_cfg.typeOfService = IfxSrc_Tos_cpu0,
    },
};

tools_timing_runtime_t tools_timing_runtime_table[TOOLS_TIMING_COUNT] =
{
    {
        .tools_timing_id = TOOLS_TIMING_1,
    },
};

tools_timing_cfg_t* tools_timing_cfg_table_get(void)
{
    return tools_timing_cfg_table;
}

tools_timing_runtime_t* tools_timing_runtime_table_get(void)
{
    return tools_timing_runtime_table;
}
