#include "./device_esc_cfg.h"
#include "../../../isr/isr_config.h"

#include "Ifx_Cfg.h"

#define DEVICE_ESC_PWM_CLOCK_HZ         100000000u
#define DEVICE_ESC_DRIVE_COMMAND_HZ          4000u
#define DEVICE_ESC_SUCTION_COMMAND_HZ        2000u
#define DEVICE_ESC_MAX_PULSE_US               150u
#define DEVICE_ESC_DRIVE_PERIOD_TICKS      (DEVICE_ESC_PWM_CLOCK_HZ / DEVICE_ESC_DRIVE_COMMAND_HZ)
#define DEVICE_ESC_SUCTION_PERIOD_TICKS    (DEVICE_ESC_PWM_CLOCK_HZ / DEVICE_ESC_SUCTION_COMMAND_HZ)
#define DEVICE_ESC_MIN_PULSE_TICKS             0u
#define DEVICE_ESC_MAX_PULSE_TICKS         ((DEVICE_ESC_PWM_CLOCK_HZ / 1000000u) * DEVICE_ESC_MAX_PULSE_US)
#define DEVICE_ESC_UART_BAUDRATE          460800u

device_esc_cfg_t device_esc_cfg_table[DEVICE_ESC_COUNT] =
{
    {
        .esc_id = DEVICE_ESC_1,
        .min_duty_cycle = DEVICE_ESC_MIN_PULSE_TICKS,
        .max_duty_cycle = DEVICE_ESC_MAX_PULSE_TICKS,
        .atom_pwm_cfg =
        {
            .gtm_module = &MODULE_GTM,
            .atom_pwm_modulecfg.gtm = &MODULE_GTM,
            .atom_pwm_modulecfg.atom = IfxGtm_Atom_1,
            .atom_pwm_modulecfg.atomChannel = IfxGtm_Atom_Ch_1,
            .atom_pwm_modulecfg.mode = IfxGtm_Atom_Mode_outputPwm,
            .atom_pwm_modulecfg.period = DEVICE_ESC_DRIVE_PERIOD_TICKS,
            .atom_pwm_modulecfg.dutyCycle = DEVICE_ESC_MIN_PULSE_TICKS,
            .atom_pwm_modulecfg.signalLevel = Ifx_ActiveState_high,
            .atom_pwm_modulecfg.oneShotModeEnabled = FALSE,
            .atom_pwm_modulecfg.synchronousUpdateEnabled = TRUE,
            .atom_pwm_modulecfg.immediateStartEnabled = FALSE,
            .atom_pwm_modulecfg.interrupt.ccu0Enabled = FALSE,
            .atom_pwm_modulecfg.interrupt.ccu1Enabled = FALSE,
            .atom_pwm_modulecfg.interrupt.mode = IfxGtm_IrqMode_pulse,
            .atom_pwm_modulecfg.interrupt.isrProvider = IfxSrc_Tos_cpu0,
            .atom_pwm_modulecfg.interrupt.isrPriority = 0u,
            .atom_pwm_modulecfg.pin.outputPin = &IfxGtm_ATOM1_1_TOUT10_P00_1_OUT,
            .atom_pwm_modulecfg.pin.outputMode = IfxPort_OutputMode_pushPull,
            .atom_pwm_modulecfg.pin.padDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
        },
    },
    {
        .esc_id = DEVICE_ESC_2,
        .min_duty_cycle = DEVICE_ESC_MIN_PULSE_TICKS,
        .max_duty_cycle = DEVICE_ESC_MAX_PULSE_TICKS,
        .atom_pwm_cfg =
        {
            .gtm_module = &MODULE_GTM,
            .atom_pwm_modulecfg.gtm = &MODULE_GTM,
            .atom_pwm_modulecfg.atom = IfxGtm_Atom_1,
            .atom_pwm_modulecfg.atomChannel = IfxGtm_Atom_Ch_2,
            .atom_pwm_modulecfg.mode = IfxGtm_Atom_Mode_outputPwm,
            .atom_pwm_modulecfg.period = DEVICE_ESC_DRIVE_PERIOD_TICKS,
            .atom_pwm_modulecfg.dutyCycle = DEVICE_ESC_MIN_PULSE_TICKS,
            .atom_pwm_modulecfg.signalLevel = Ifx_ActiveState_high,
            .atom_pwm_modulecfg.oneShotModeEnabled = FALSE,
            .atom_pwm_modulecfg.synchronousUpdateEnabled = TRUE,
            .atom_pwm_modulecfg.immediateStartEnabled = FALSE,
            .atom_pwm_modulecfg.interrupt.ccu0Enabled = FALSE,
            .atom_pwm_modulecfg.interrupt.ccu1Enabled = FALSE,
            .atom_pwm_modulecfg.interrupt.mode = IfxGtm_IrqMode_pulse,
            .atom_pwm_modulecfg.interrupt.isrProvider = IfxSrc_Tos_cpu0,
            .atom_pwm_modulecfg.interrupt.isrPriority = 0u,
            .atom_pwm_modulecfg.pin.outputPin = &IfxGtm_ATOM1_2_TOUT12_P00_3_OUT,
            .atom_pwm_modulecfg.pin.outputMode = IfxPort_OutputMode_pushPull,
            .atom_pwm_modulecfg.pin.padDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
        },
    },
    {
        .esc_id = DEVICE_ESC_3,
        .min_duty_cycle = DEVICE_ESC_MIN_PULSE_TICKS,
        .max_duty_cycle = DEVICE_ESC_MAX_PULSE_TICKS,
        .atom_pwm_cfg =
        {
            .gtm_module = &MODULE_GTM,
            .atom_pwm_modulecfg.gtm = &MODULE_GTM,
            .atom_pwm_modulecfg.atom = IfxGtm_Atom_1,
            .atom_pwm_modulecfg.atomChannel = IfxGtm_Atom_Ch_3,
            .atom_pwm_modulecfg.mode = IfxGtm_Atom_Mode_outputPwm,
            .atom_pwm_modulecfg.period = DEVICE_ESC_SUCTION_PERIOD_TICKS,
            .atom_pwm_modulecfg.dutyCycle = DEVICE_ESC_MIN_PULSE_TICKS,
            .atom_pwm_modulecfg.signalLevel = Ifx_ActiveState_high,
            .atom_pwm_modulecfg.oneShotModeEnabled = FALSE,
            .atom_pwm_modulecfg.synchronousUpdateEnabled = TRUE,
            .atom_pwm_modulecfg.immediateStartEnabled = FALSE,
            .atom_pwm_modulecfg.interrupt.ccu0Enabled = FALSE,
            .atom_pwm_modulecfg.interrupt.ccu1Enabled = FALSE,
            .atom_pwm_modulecfg.interrupt.mode = IfxGtm_IrqMode_pulse,
            .atom_pwm_modulecfg.interrupt.isrProvider = IfxSrc_Tos_cpu0,
            .atom_pwm_modulecfg.interrupt.isrPriority = 0u,
            .atom_pwm_modulecfg.pin.outputPin = &IfxGtm_ATOM1_3_TOUT13_P00_4_OUT,
            .atom_pwm_modulecfg.pin.outputMode = IfxPort_OutputMode_pushPull,
            .atom_pwm_modulecfg.pin.padDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
        },
    },
};

device_esc_runtime_t device_esc_runtime_table[DEVICE_ESC_COUNT] =
{
    {
        .esc_id = DEVICE_ESC_1,
        .device_esc_callback = NULL_PTR,
    },
    {
        .esc_id = DEVICE_ESC_2,
        .device_esc_callback = NULL_PTR,
    },
    {
        .esc_id = DEVICE_ESC_3,
        .device_esc_callback = NULL_PTR,
    },
};

device_esc_uart_cfg_t device_esc_uart_cfg =
{
    .asclin_asc_cfg =
    {
        .asclin_index = IfxAsclin_Index_1,

        .asclin_asc_modulecfg.asclin = NULL_PTR,

        .asclin_asc_modulecfg.baudrate.baudrate = DEVICE_ESC_UART_BAUDRATE,
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

        .asclin_asc_modulecfg.interrupt.rxPriority = 0,
        .asclin_asc_modulecfg.interrupt.txPriority = ISR_CONFIG_PRIORITY_ESC_UART_TX_BUSINESS_INTERRUPT,
        .asclin_asc_modulecfg.interrupt.erPriority = 0,
        .asclin_asc_modulecfg.interrupt.typeOfService = ISR_CONFIG_TOS_ESC_UART_TX_BUSINESS_INTERRUPT,

        .asclin_asc_modulecfg.pins = NULL_PTR,

        .asclin_asc_modulecfg.clockSource = IfxAsclin_ClockSource_kernelClock,

        .asclin_asc_modulecfg.errorFlags.ALL = 0xFF,

        .asclin_asc_modulecfg.txBufferSize = DEVICE_ESC_UART_TXFIFO_SIZE,
        .asclin_asc_modulecfg.txBuffer = NULL_PTR,
        .asclin_asc_modulecfg.rxBufferSize = DEVICE_ESC_UART_RXFIFO_SIZE,
        .asclin_asc_modulecfg.rxBuffer = NULL_PTR,

        .asclin_asc_modulecfg.loopBack = FALSE,

        .asclin_asc_modulecfg.dataBufferMode = IfxAsclin_DataBufferMode_normal,

        .asclin_asc_pins.cts = NULL_PTR,
        .asclin_asc_pins.ctsMode = IfxPort_InputMode_pullUp,
        .asclin_asc_pins.rx = &IfxAsclin1_RXE_P11_10_IN,
        .asclin_asc_pins.rxMode = IfxPort_InputMode_pullUp,
        .asclin_asc_pins.rts = NULL_PTR,
        .asclin_asc_pins.rtsMode = IfxPort_OutputMode_pushPull,
        .asclin_asc_pins.tx = &IfxAsclin1_TX_P11_12_OUT,
        .asclin_asc_pins.txMode = IfxPort_OutputMode_pushPull,
        .asclin_asc_pins.pinDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },
};

device_esc_uart_runtime_t device_esc_uart_runtime =
{
    .txFifo_Buffer = {0},
    .rxFifo_Buffer = {0},
};

device_esc_cfg_t* device_esc_cfg_table_get(void)
{
    return device_esc_cfg_table;
}

device_esc_runtime_t* device_esc_runtime_table_get(void)
{
    return device_esc_runtime_table;
}

device_esc_uart_cfg_t* device_esc_uart_cfg_get(void)
{
    return &device_esc_uart_cfg;
}

device_esc_uart_runtime_t* device_esc_uart_runtime_get(void)
{
    return &device_esc_uart_runtime;
}
