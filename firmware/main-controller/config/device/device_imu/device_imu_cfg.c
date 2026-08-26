#include "./device_imu_cfg.h"
#include "../../../isr/isr_config.h"

#define DEVICE_IMU_SAMPLE_FREQUENCY 480u

device_imu_cfg_t device_imu_cfg_table[DEVICE_IMU_COUNT] =
{
    {
        .imu_id = DEVICE_IMU_1,
        .gtm_atom_timer_cfg =
        {
            .gtm_module = &MODULE_GTM,

            .atom_timer_modulecfg.gtm = &MODULE_GTM,
            .atom_timer_modulecfg.atom = IfxGtm_Atom_0,
            .atom_timer_modulecfg.timerChannel = IfxGtm_Atom_Ch_1,
            .atom_timer_modulecfg.triggerOut = NULL_PTR,
            .atom_timer_modulecfg.clock = IfxGtm_Cmu_Clk_0,
            .atom_timer_modulecfg.base.frequency = DEVICE_IMU_SAMPLE_FREQUENCY,
            .atom_timer_modulecfg.base.isrPriority = ISR_CONFIG_PRIORITY_IMU1_BUSINESS_INTERRUPT,
            .atom_timer_modulecfg.base.isrProvider = ISR_CONFIG_TOS_IMU1_BUSINESS_INTERRUPT,
            .atom_timer_modulecfg.base.minResolution = 0,
            .atom_timer_modulecfg.base.trigger.outputMode = IfxPort_OutputMode_pushPull,
            .atom_timer_modulecfg.base.trigger.outputDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
            .atom_timer_modulecfg.base.trigger.risingEdgeAtPeriod = FALSE,
            .atom_timer_modulecfg.base.trigger.outputEnabled = FALSE,
            .atom_timer_modulecfg.base.trigger.enabled = FALSE,
            .atom_timer_modulecfg.base.trigger.triggerPoint = 0,
            .atom_timer_modulecfg.base.trigger.isrPriority = 0,
            .atom_timer_modulecfg.base.trigger.isrProvider = IfxSrc_Tos_cpu0,
            .atom_timer_modulecfg.base.countDir = IfxStdIf_Timer_CountDir_up,
            .atom_timer_modulecfg.base.startOffset = 0.0,
            .atom_timer_modulecfg.irqModeTimer = IfxGtm_IrqMode_pulse,
            .atom_timer_modulecfg.irqModeTrigger = IfxGtm_IrqMode_pulse,
            .atom_timer_modulecfg.initPins = FALSE,
        },
        .qspi_cfg =
        {
            .qspi_index = IfxQspi_Index_0,

            .qspi_module = NULL_PTR,

            .qspi_modulecfg.mode = IfxQspi_Mode_master,

            .qspi_modulecfg.rxPriority = 0,
            .qspi_modulecfg.txPriority = 0,
            .qspi_modulecfg.erPriority = 0,
            .qspi_modulecfg.isrProvider = IfxSrc_Tos_cpu0,

            .qspi_modulecfg.bufferSize = 0,
            .qspi_modulecfg.buffer = NULL_PTR,

            .qspi_modulecfg.qspi = NULL_PTR,

            .qspi_modulecfg.allowSleepMode = FALSE,

            .qspi_modulecfg.pauseOnBaudrateSpikeErrors = FALSE,

            .qspi_modulecfg.pauseRunTransition = IfxQspi_PauseRunTransition_run,

            .qspi_modulecfg.txFifoThreshold = IfxQspi_TxFifoInt_1,
            .qspi_modulecfg.rxFifoThreshold = IfxQspi_RxFifoInt_0,
            .qspi_modulecfg.txFifoMode = IfxQspi_FifoMode_singleMove,
            .qspi_modulecfg.rxFifoMode = IfxQspi_FifoMode_singleMove,

            .qspi_modulecfg.pins = NULL_PTR,

            .qspi_modulecfg.dma.rxDmaChannelId = IfxDma_ChannelId_none,
            .qspi_modulecfg.dma.txDmaChannelId = IfxDma_ChannelId_none,
            .qspi_modulecfg.dma.useDma = FALSE,

            .qspi_modulecfg.maximumBaudrate = 50000000,

            .qspi_pins.sclk = &IfxQspi0_SCLK_P20_13_OUT,
            .qspi_pins.sclkMode = IfxPort_OutputMode_pushPull,
            .qspi_pins.mtsr = &IfxQspi0_MTSR_P20_14_OUT,
            .qspi_pins.mtsrMode = IfxPort_OutputMode_pushPull,
            .qspi_pins.mrst = &IfxQspi0_MRSTA_P20_12_IN,
            .qspi_pins.mrstMode = IfxPort_InputMode_pullDown,
            .qspi_pins.pinDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1,

            .qspi_channelcfg.ch.baudrate = 10000000,
            .qspi_channelcfg.ch.mode.enabled = TRUE,
            .qspi_channelcfg.ch.mode.autoCS = TRUE,
            .qspi_channelcfg.ch.mode.loopback = FALSE,
            .qspi_channelcfg.ch.mode.clockPolarity = IfxQspi_ClockPolarity_idleLow,
            .qspi_channelcfg.ch.mode.shiftClock = IfxQspi_ShiftClock_shiftTransmitDataOnTrailingEdge,
            .qspi_channelcfg.ch.mode.dataHeading = IfxQspi_DataHeading_msbFirst,
            .qspi_channelcfg.ch.mode.dataWidth = 8,
            .qspi_channelcfg.ch.mode.csActiveLevel = Ifx_ActiveState_low,
            .qspi_channelcfg.ch.mode.csLeadDelay = IfxQspi_SlsoTiming_1,
            .qspi_channelcfg.ch.mode.csTrailDelay = IfxQspi_SlsoTiming_1,
            .qspi_channelcfg.ch.mode.csInactiveDelay = IfxQspi_SlsoTiming_1,
            .qspi_channelcfg.ch.mode.parityCheck = FALSE,
            .qspi_channelcfg.ch.mode.parityMode = IfxQspi_ParityMode_even,
            .qspi_channelcfg.ch.errorChecks.baudrate = FALSE,
            .qspi_channelcfg.ch.errorChecks.phase = FALSE,
            .qspi_channelcfg.ch.errorChecks.receive = FALSE,
            .qspi_channelcfg.ch.errorChecks.transmit = FALSE,
            .qspi_channelcfg.ch.channelId = IfxQspi_ChannelId_6,
            .qspi_channelcfg.channelBasedCs = IfxQspi_SpiMaster_ChannelBasedCs_disabled,
            .qspi_channelcfg.mode = IfxQspi_SpiMaster_Mode_short,
            .qspi_channelcfg.dummyTxValue = 0xFFFFFFFFu,
            .qspi_channelcfg.dummyRxValue = 0u,
            .qspi_channelcfg.spiMaster = NULL_PTR,
            .qspi_channelcfg.qspi = NULL_PTR,
            .qspi_channelcfg.dma = NULL_PTR,

            .qspi_sls.output.pin = &IfxQspi0_SLSO6_P20_10_OUT,
            .qspi_sls.output.mode = IfxPort_OutputMode_pushPull,
            .qspi_sls.output.driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
        },
    },
    {
        .imu_id = DEVICE_IMU_2,
        .gtm_atom_timer_cfg =
        {
            .gtm_module = &MODULE_GTM,

            .atom_timer_modulecfg.gtm = &MODULE_GTM,
            .atom_timer_modulecfg.atom = IfxGtm_Atom_0,
            .atom_timer_modulecfg.timerChannel = IfxGtm_Atom_Ch_2,
            .atom_timer_modulecfg.triggerOut = NULL_PTR,
            .atom_timer_modulecfg.clock = IfxGtm_Cmu_Clk_0,
            .atom_timer_modulecfg.base.frequency = DEVICE_IMU_SAMPLE_FREQUENCY,
            .atom_timer_modulecfg.base.isrPriority = ISR_CONFIG_PRIORITY_IMU2_BUSINESS_INTERRUPT,
            .atom_timer_modulecfg.base.isrProvider = ISR_CONFIG_TOS_IMU2_BUSINESS_INTERRUPT,
            .atom_timer_modulecfg.base.minResolution = 0,
            .atom_timer_modulecfg.base.trigger.outputMode = IfxPort_OutputMode_pushPull,
            .atom_timer_modulecfg.base.trigger.outputDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
            .atom_timer_modulecfg.base.trigger.risingEdgeAtPeriod = FALSE,
            .atom_timer_modulecfg.base.trigger.outputEnabled = FALSE,
            .atom_timer_modulecfg.base.trigger.enabled = FALSE,
            .atom_timer_modulecfg.base.trigger.triggerPoint = 0,
            .atom_timer_modulecfg.base.trigger.isrPriority = 0,
            .atom_timer_modulecfg.base.trigger.isrProvider = IfxSrc_Tos_cpu0,
            .atom_timer_modulecfg.base.countDir = IfxStdIf_Timer_CountDir_up,
            .atom_timer_modulecfg.base.startOffset = 0.0,
            .atom_timer_modulecfg.irqModeTimer = IfxGtm_IrqMode_pulse,
            .atom_timer_modulecfg.irqModeTrigger = IfxGtm_IrqMode_pulse,
            .atom_timer_modulecfg.initPins = FALSE,
        },
        .qspi_cfg =
        {
            .qspi_index = IfxQspi_Index_1,

            .qspi_module = NULL_PTR,

            .qspi_modulecfg.mode = IfxQspi_Mode_master,

            .qspi_modulecfg.rxPriority = 0,
            .qspi_modulecfg.txPriority = 0,
            .qspi_modulecfg.erPriority = 0,
            .qspi_modulecfg.isrProvider = IfxSrc_Tos_cpu0,

            .qspi_modulecfg.bufferSize = 0,
            .qspi_modulecfg.buffer = NULL_PTR,

            .qspi_modulecfg.qspi = NULL_PTR,

            .qspi_modulecfg.allowSleepMode = FALSE,

            .qspi_modulecfg.pauseOnBaudrateSpikeErrors = FALSE,

            .qspi_modulecfg.pauseRunTransition = IfxQspi_PauseRunTransition_run,

            .qspi_modulecfg.txFifoThreshold = IfxQspi_TxFifoInt_1,
            .qspi_modulecfg.rxFifoThreshold = IfxQspi_RxFifoInt_0,
            .qspi_modulecfg.txFifoMode = IfxQspi_FifoMode_singleMove,
            .qspi_modulecfg.rxFifoMode = IfxQspi_FifoMode_singleMove,

            .qspi_modulecfg.pins = NULL_PTR,

            .qspi_modulecfg.dma.rxDmaChannelId = IfxDma_ChannelId_none,
            .qspi_modulecfg.dma.txDmaChannelId = IfxDma_ChannelId_none,
            .qspi_modulecfg.dma.useDma = FALSE,

            .qspi_modulecfg.maximumBaudrate = 50000000,

            .qspi_pins.sclk = &IfxQspi1_SCLK_P10_2_OUT,
            .qspi_pins.sclkMode = IfxPort_OutputMode_pushPull,
            .qspi_pins.mtsr = &IfxQspi1_MTSR_P10_3_OUT,
            .qspi_pins.mtsrMode = IfxPort_OutputMode_pushPull,
            .qspi_pins.mrst = &IfxQspi1_MRSTA_P10_1_IN,
            .qspi_pins.mrstMode = IfxPort_InputMode_pullDown,
            .qspi_pins.pinDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1,

            .qspi_channelcfg.ch.baudrate = 10000000,
            .qspi_channelcfg.ch.mode.enabled = TRUE,
            .qspi_channelcfg.ch.mode.autoCS = TRUE,
            .qspi_channelcfg.ch.mode.loopback = FALSE,
            .qspi_channelcfg.ch.mode.clockPolarity = IfxQspi_ClockPolarity_idleLow,
            .qspi_channelcfg.ch.mode.shiftClock = IfxQspi_ShiftClock_shiftTransmitDataOnTrailingEdge,
            .qspi_channelcfg.ch.mode.dataHeading = IfxQspi_DataHeading_msbFirst,
            .qspi_channelcfg.ch.mode.dataWidth = 8,
            .qspi_channelcfg.ch.mode.csActiveLevel = Ifx_ActiveState_low,
            .qspi_channelcfg.ch.mode.csLeadDelay = IfxQspi_SlsoTiming_1,
            .qspi_channelcfg.ch.mode.csTrailDelay = IfxQspi_SlsoTiming_1,
            .qspi_channelcfg.ch.mode.csInactiveDelay = IfxQspi_SlsoTiming_1,
            .qspi_channelcfg.ch.mode.parityCheck = FALSE,
            .qspi_channelcfg.ch.mode.parityMode = IfxQspi_ParityMode_even,
            .qspi_channelcfg.ch.errorChecks.baudrate = FALSE,
            .qspi_channelcfg.ch.errorChecks.phase = FALSE,
            .qspi_channelcfg.ch.errorChecks.receive = FALSE,
            .qspi_channelcfg.ch.errorChecks.transmit = FALSE,
            .qspi_channelcfg.ch.channelId = IfxQspi_ChannelId_4,
            .qspi_channelcfg.channelBasedCs = IfxQspi_SpiMaster_ChannelBasedCs_disabled,
            .qspi_channelcfg.mode = IfxQspi_SpiMaster_Mode_short,
            .qspi_channelcfg.dummyTxValue = 0xFFFFFFFFu,
            .qspi_channelcfg.dummyRxValue = 0u,
            .qspi_channelcfg.spiMaster = NULL_PTR,
            .qspi_channelcfg.qspi = NULL_PTR,
            .qspi_channelcfg.dma = NULL_PTR,

            .qspi_sls.output.pin = &IfxQspi1_SLSO4_P11_11_OUT,
            .qspi_sls.output.mode = IfxPort_OutputMode_pushPull,
            .qspi_sls.output.driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
        },
    },
};

device_imu_runtime_t device_imu_runtime_table[DEVICE_IMU_COUNT] =
{
    {
        .imu_id = DEVICE_IMU_1,
        .imu_temperature_buffer =
        {
            0x00u,
            0x00u,
        },
        .imu_accelerometer_buffer =
        {
            0x00u,
            0x00u,
            0x00u,
            0x00u,
            0x00u,
            0x00u,
        },
        .imu_gyroscope_buffer =
        {
            0x00u,
            0x00u,
            0x00u,
            0x00u,
            0x00u,
            0x00u,
        },
        .imu_timestamp_buffer =
        {
            0x00u,
            0x00u,
            0x00u,
            0x00u,
        },
        .imu_sflp_game_buffer =
        {
            0x00u,
            0x00u,
            0x00u,
            0x00u,
            0x00u,
            0x00u,
        },
        .imu_sflp_gbias_buffer =
        {
            0x00u,
            0x00u,
            0x00u,
            0x00u,
            0x00u,
            0x00u,
        },
        .imu_sflp_game_quaternion =
        {
            1.0f,
            0.0f,
            0.0f,
            0.0f,
        },
        .imu_sflp_game_yaw_rad = 0.0f,
        .imu_sflp_game_valid = FALSE,
        .imu_sflp_gbias_dps =
        {
            0.0f,
            0.0f,
            0.0f,
        },
        .imu_sflp_gbias_valid = FALSE,
        .imu_fifo_word_buffer =
        {
            0x00u,
            0x00u,
            0x00u,
            0x00u,
            0x00u,
            0x00u,
            0x00u,
        },
        .imu_fifo_level = 0u,
        .imu_fifo_level_before_drain = 0u,
        .imu_fifo_level_after_drain = 0u,
        .imu_fifo_last_drain_words = 0u,
        .imu_fifo_drain_count = 0u,
        .imu_fifo_overrun_count = 0u,
        .imu_fifo_gyro_sample_count = 0u,
        .imu_fifo_timestamp_sample_count = 0u,
        .imu_fifo_sflp_game_sample_count = 0u,
        .imu_fifo_sflp_gbias_sample_count = 0u,
        .imu_raw_update_count = 0u,
        .imu_sflp_game_update_count = 0u,
        .imu_fifo_last_tag = 0u,
        .imu_fifo_last_tag_raw = 0u,
        .device_imu_callback = NULL_PTR,
    },
    {
        .imu_id = DEVICE_IMU_2,
        .imu_temperature_buffer =
        {
            0x00u,
            0x00u,
        },
        .imu_accelerometer_buffer =
        {
            0x00u,
            0x00u,
            0x00u,
            0x00u,
            0x00u,
            0x00u,
        },
        .imu_gyroscope_buffer =
        {
            0x00u,
            0x00u,
            0x00u,
            0x00u,
            0x00u,
            0x00u,
        },
        .imu_timestamp_buffer =
        {
            0x00u,
            0x00u,
            0x00u,
            0x00u,
        },
        .imu_sflp_game_buffer =
        {
            0x00u,
            0x00u,
            0x00u,
            0x00u,
            0x00u,
            0x00u,
        },
        .imu_sflp_gbias_buffer =
        {
            0x00u,
            0x00u,
            0x00u,
            0x00u,
            0x00u,
            0x00u,
        },
        .imu_sflp_game_quaternion =
        {
            1.0f,
            0.0f,
            0.0f,
            0.0f,
        },
        .imu_sflp_game_yaw_rad = 0.0f,
        .imu_sflp_game_valid = FALSE,
        .imu_sflp_gbias_dps =
        {
            0.0f,
            0.0f,
            0.0f,
        },
        .imu_sflp_gbias_valid = FALSE,
        .imu_fifo_word_buffer =
        {
            0x00u,
            0x00u,
            0x00u,
            0x00u,
            0x00u,
            0x00u,
            0x00u,
        },
        .imu_fifo_level = 0u,
        .imu_fifo_level_before_drain = 0u,
        .imu_fifo_level_after_drain = 0u,
        .imu_fifo_last_drain_words = 0u,
        .imu_fifo_drain_count = 0u,
        .imu_fifo_overrun_count = 0u,
        .imu_fifo_gyro_sample_count = 0u,
        .imu_fifo_timestamp_sample_count = 0u,
        .imu_fifo_sflp_game_sample_count = 0u,
        .imu_fifo_sflp_gbias_sample_count = 0u,
        .imu_raw_update_count = 0u,
        .imu_sflp_game_update_count = 0u,
        .imu_fifo_last_tag = 0u,
        .imu_fifo_last_tag_raw = 0u,
        .device_imu_callback = NULL_PTR,
    },
};

device_imu_cfg_t* device_imu_cfg_table_get(void)
{
    return device_imu_cfg_table;
}

device_imu_runtime_t* device_imu_runtime_table_get(void)
{
    return device_imu_runtime_table;
}
