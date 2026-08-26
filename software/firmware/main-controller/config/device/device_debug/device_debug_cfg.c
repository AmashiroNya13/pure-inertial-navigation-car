#include "./device_debug_cfg.h"
#include "../../../isr/isr_config.h"

device_debug_cfg_t device_debug_cfg_table[DEVICE_DEBUG_COUNT] =
{
    {
        .debug_id = DEVICE_DEBUG_1,
        .asclin_asc_cfg =
        {
            .asclin_index = IfxAsclin_Index_0,

            .asclin_asc_modulecfg.asclin = NULL_PTR,

            .asclin_asc_modulecfg.baudrate.baudrate = 2000000,
            .asclin_asc_modulecfg.baudrate.prescaler = 1,
            .asclin_asc_modulecfg.baudrate.oversampling = IfxAsclin_OversamplingFactor_4,

            .asclin_asc_modulecfg.bitTiming.medianFilter = IfxAsclin_SamplesPerBit_one,
            .asclin_asc_modulecfg.bitTiming.samplePointPosition = IfxAsclin_SamplePointPosition_3,

            .asclin_asc_modulecfg.frame.idleDelay = IfxAsclin_IdleDelay_0,
            .asclin_asc_modulecfg.frame.stopBit = IfxAsclin_StopBit_1,
            .asclin_asc_modulecfg.frame.frameMode = IfxAsclin_FrameMode_asc,
            .asclin_asc_modulecfg.frame.shiftDir = IfxAsclin_ShiftDirection_lsbFirst,
            .asclin_asc_modulecfg.frame.parityBit = FALSE,
            .asclin_asc_modulecfg.frame.parityType = IfxAsclin_ParityType_even,
            .asclin_asc_modulecfg.frame.dataLength = IfxAsclin_DataLength_8,

            .asclin_asc_modulecfg.fifo.inWidth = IfxAsclin_TxFifoInletWidth_1,
            .asclin_asc_modulecfg.fifo.outWidth = IfxAsclin_RxFifoOutletWidth_1,
            .asclin_asc_modulecfg.fifo.txFifoInterruptLevel = IfxAsclin_TxFifoInterruptLevel_0,
            .asclin_asc_modulecfg.fifo.rxFifoInterruptLevel = IfxAsclin_RxFifoInterruptLevel_1,
            .asclin_asc_modulecfg.fifo.buffMode = IfxAsclin_ReceiveBufferMode_rxFifo,

            .asclin_asc_modulecfg.interrupt.rxPriority = ISR_CONFIG_PRIORITY_DEBUG_RX_BUSINESS_INTERRUPT,
            .asclin_asc_modulecfg.interrupt.txPriority = ISR_CONFIG_PRIORITY_DEBUG_TX_BUSINESS_INTERRUPT,
            .asclin_asc_modulecfg.interrupt.erPriority = 0,
            .asclin_asc_modulecfg.interrupt.typeOfService = ISR_CONFIG_TOS_DEBUG_TXRXER_BUSINESS_INTERRUPT,

            .asclin_asc_modulecfg.pins = NULL_PTR,

            .asclin_asc_modulecfg.clockSource = IfxAsclin_ClockSource_kernelClock,

            .asclin_asc_modulecfg.errorFlags.ALL = 0xFF,

            .asclin_asc_modulecfg.txBufferSize = DEVICE_DEBUG_TXFIFO_SIZE,
            .asclin_asc_modulecfg.txBuffer = NULL_PTR,
            .asclin_asc_modulecfg.rxBufferSize = DEVICE_DEBUG_RXFIFO_SIZE,
            .asclin_asc_modulecfg.rxBuffer = NULL_PTR,

            .asclin_asc_modulecfg.loopBack = FALSE,

            .asclin_asc_modulecfg.dataBufferMode = IfxAsclin_DataBufferMode_normal,

            .asclin_asc_pins.cts = NULL_PTR,
            .asclin_asc_pins.ctsMode = IfxPort_InputMode_pullUp,
            .asclin_asc_pins.rx = &IfxAsclin0_RXA_P14_1_IN,
            .asclin_asc_pins.rxMode = IfxPort_InputMode_pullUp,
            .asclin_asc_pins.rts = NULL_PTR,
            .asclin_asc_pins.rtsMode = IfxPort_OutputMode_pushPull,
            .asclin_asc_pins.tx = &IfxAsclin0_TX_P14_0_OUT,
            .asclin_asc_pins.txMode = IfxPort_OutputMode_pushPull,
            .asclin_asc_pins.pinDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
        },
    },
};

BEGIN_DATA_SECTION(.bss_lmu)
device_debug_runtime_t device_debug_runtime_table[DEVICE_DEBUG_COUNT];
END_DATA_SECTION

device_debug_cfg_t* device_debug_cfg_table_get(void)
{
    return device_debug_cfg_table;
}

device_debug_runtime_t* device_debug_runtime_table_get(void)
{
    return device_debug_runtime_table;
}
