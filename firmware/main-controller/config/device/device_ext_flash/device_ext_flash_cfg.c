#include "./device_ext_flash_cfg.h"

device_ext_flash_cfg_t device_ext_flash_cfg_table[DEVICE_EXT_FLASH_COUNT] =
{
    {
        .ext_flash_id = DEVICE_EXT_FLASH_1,
        .tx_dma_cfg =
        {
            .dma_module = &MODULE_DMA,
            .dma_modulecfg.dma = &MODULE_DMA,

            .dma_channelcfg.module = NULL_PTR,

            .dma_channelcfg.channelId = IfxDma_ChannelId_28,
            .dma_channelcfg.sourceAddress = 0,
            .dma_channelcfg.destinationAddress = 0,
            .dma_channelcfg.shadowAddress = 0,
            .dma_channelcfg.readDataCrc = 0,
            .dma_channelcfg.sourceDestinationAddressCrc = 0,
            .dma_channelcfg.transferCount = 0,
            .dma_channelcfg.blockMode = IfxDma_ChannelMove_1,
            .dma_channelcfg.requestMode = IfxDma_ChannelRequestMode_oneTransferPerRequest,
            .dma_channelcfg.operationMode = IfxDma_ChannelOperationMode_continuous,
            .dma_channelcfg.moveSize = IfxDma_ChannelMoveSize_32bit,
            .dma_channelcfg.pattern = IfxDma_ChannelPattern_0_disable,
            .dma_channelcfg.requestSource = IfxDma_ChannelRequestSource_peripheral,
            .dma_channelcfg.busPriority = IfxDma_ChannelBusPriority_medium,
            .dma_channelcfg.hardwareRequestEnabled = FALSE,
            .dma_channelcfg.sourceAddressIncrementStep = IfxDma_ChannelIncrementStep_1,
            .dma_channelcfg.sourceAddressIncrementDirection = IfxDma_ChannelIncrementDirection_positive,
            .dma_channelcfg.sourceAddressCircularRange = IfxDma_ChannelIncrementCircular_8,
            .dma_channelcfg.destinationAddressIncrementStep = IfxDma_ChannelIncrementStep_1,
            .dma_channelcfg.destinationAddressIncrementDirection = IfxDma_ChannelIncrementDirection_positive,
            .dma_channelcfg.destinationAddressCircularRange = IfxDma_ChannelIncrementCircular_4,
            .dma_channelcfg.shadowControl = IfxDma_ChannelShadow_none,
            .dma_channelcfg.sourceCircularBufferEnabled = TRUE,
            .dma_channelcfg.destinationCircularBufferEnabled = TRUE,
            .dma_channelcfg.timestampEnabled = FALSE,
            .dma_channelcfg.wrapSourceInterruptEnabled = FALSE,
            .dma_channelcfg.wrapDestinationInterruptEnabled = FALSE,
            .dma_channelcfg.channelInterruptEnabled = FALSE,
            .dma_channelcfg.channelInterruptControl = IfxDma_ChannelInterruptControl_thresholdLimitMatch,
            .dma_channelcfg.interruptRaiseThreshold = 0,
            .dma_channelcfg.transactionRequestLostInterruptEnabled = FALSE,
            .dma_channelcfg.channelInterruptPriority = 0,
            .dma_channelcfg.channelInterruptTypeOfService = IfxSrc_Tos_cpu0,
        },
        .qspi_cfg =
        {
            .qspi_index = IfxQspi_Index_2,

            .qspi_module = NULL_PTR,

            .qspi_modulecfg.mode = IfxQspi_Mode_master,

            .qspi_modulecfg.rxPriority = 29,
            .qspi_modulecfg.txPriority = 0,
            .qspi_modulecfg.erPriority = 0,
            .qspi_modulecfg.isrProvider = IfxSrc_Tos_dma,

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

            .qspi_modulecfg.maximumBaudrate = 50000001,

            .qspi_pins.sclk = &IfxQspi2_SCLK_P15_3_OUT,
            .qspi_pins.sclkMode = IfxPort_OutputMode_pushPull,
            .qspi_pins.mtsr = &IfxQspi2_MTSR_P15_5_OUT,
            .qspi_pins.mtsrMode = IfxPort_OutputMode_pushPull,
            .qspi_pins.mrst = &IfxQspi2_MRSTA_P15_4_IN,
            .qspi_pins.mrstMode = IfxPort_InputMode_pullDown,
            .qspi_pins.pinDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1,

            .qspi_channelcfg.ch.baudrate = 50000000,
            .qspi_channelcfg.ch.mode.enabled = TRUE,
            .qspi_channelcfg.ch.mode.autoCS = FALSE,
            .qspi_channelcfg.ch.mode.loopback = FALSE,
            .qspi_channelcfg.ch.mode.clockPolarity = IfxQspi_ClockPolarity_idleLow,
            .qspi_channelcfg.ch.mode.shiftClock = IfxQspi_ShiftClock_shiftTransmitDataOnLeadingEdge,
            .qspi_channelcfg.ch.mode.dataHeading = IfxQspi_DataHeading_msbFirst,
            .qspi_channelcfg.ch.mode.dataWidth = 16,
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
            .qspi_channelcfg.ch.channelId = IfxQspi_ChannelId_0,
            .qspi_channelcfg.channelBasedCs = IfxQspi_SpiMaster_ChannelBasedCs_enabled,
            .qspi_channelcfg.mode = IfxQspi_SpiMaster_Mode_shortContinuous,
            .qspi_channelcfg.dummyTxValue = 0xFFFFFFFF,
            .qspi_channelcfg.dummyRxValue = 0,
            .qspi_channelcfg.spiMaster = NULL_PTR,
            .qspi_channelcfg.qspi = NULL_PTR,
            .qspi_channelcfg.dma = NULL_PTR,

            .qspi_sls.output.pin = &IfxQspi2_SLSO0_P15_2_OUT,
            .qspi_sls.output.mode = IfxPort_OutputMode_pushPull,
            .qspi_sls.output.driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
        },
        .rx_dma_cfg =
        {
            .dma_module = &MODULE_DMA,
            .dma_modulecfg.dma = &MODULE_DMA,

            .dma_channelcfg.module = NULL_PTR,

            .dma_channelcfg.channelId = IfxDma_ChannelId_29,
            .dma_channelcfg.sourceAddress = 0,
            .dma_channelcfg.destinationAddress = 0,
            .dma_channelcfg.shadowAddress = 0,
            .dma_channelcfg.readDataCrc = 0,
            .dma_channelcfg.sourceDestinationAddressCrc = 0,
            .dma_channelcfg.transferCount = 2,
            .dma_channelcfg.blockMode = IfxDma_ChannelMove_1,
            .dma_channelcfg.requestMode = IfxDma_ChannelRequestMode_oneTransferPerRequest,
            .dma_channelcfg.operationMode = IfxDma_ChannelOperationMode_continuous,
            .dma_channelcfg.moveSize = IfxDma_ChannelMoveSize_8bit,
            .dma_channelcfg.pattern = IfxDma_ChannelPattern_0_disable,
            .dma_channelcfg.requestSource = IfxDma_ChannelRequestSource_peripheral,
            .dma_channelcfg.busPriority = IfxDma_ChannelBusPriority_medium,
            .dma_channelcfg.hardwareRequestEnabled = TRUE,
            .dma_channelcfg.sourceAddressIncrementStep = IfxDma_ChannelIncrementStep_1,
            .dma_channelcfg.sourceAddressIncrementDirection = IfxDma_ChannelIncrementDirection_positive,
            .dma_channelcfg.sourceAddressCircularRange = IfxDma_ChannelIncrementCircular_2,
            .dma_channelcfg.destinationAddressIncrementStep = IfxDma_ChannelIncrementStep_1,
            .dma_channelcfg.destinationAddressIncrementDirection = IfxDma_ChannelIncrementDirection_positive,
            .dma_channelcfg.destinationAddressCircularRange = IfxDma_ChannelIncrementCircular_2,
            .dma_channelcfg.shadowControl = IfxDma_ChannelShadow_none,
            .dma_channelcfg.sourceCircularBufferEnabled = FALSE,
            .dma_channelcfg.destinationCircularBufferEnabled = TRUE,
            .dma_channelcfg.timestampEnabled = FALSE,
            .dma_channelcfg.wrapSourceInterruptEnabled = FALSE,
            .dma_channelcfg.wrapDestinationInterruptEnabled = FALSE,
            .dma_channelcfg.channelInterruptEnabled = FALSE,
            .dma_channelcfg.channelInterruptControl = IfxDma_ChannelInterruptControl_transferCountDecremented,
            .dma_channelcfg.interruptRaiseThreshold = 0,
            .dma_channelcfg.transactionRequestLostInterruptEnabled = FALSE,
            .dma_channelcfg.channelInterruptPriority = 50,
            .dma_channelcfg.channelInterruptTypeOfService = IfxSrc_Tos_cpu0,
        },
    },
};

device_ext_flash_runtime_t device_ext_flash_runtime_table[DEVICE_EXT_FLASH_COUNT] =
{
    {
        .ext_flash_id = DEVICE_EXT_FLASH_COUNT,
    },
};

device_ext_flash_cfg_t* device_ext_flash_cfg_table_get(void)
{
    return device_ext_flash_cfg_table;
}

device_ext_flash_runtime_t* device_ext_flash_runtime_table_get(void)
{
    return device_ext_flash_runtime_table;
}
