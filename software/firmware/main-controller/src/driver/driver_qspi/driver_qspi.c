#include "../../../inc/driver/driver_qspi/driver_qspi.h"

IFX_INLINE void driver_qspi_keep_sls(driver_qspi_runtime_t* qspi_runtime);
IFX_INLINE void driver_qspi_auto_sls(driver_qspi_runtime_t* qspi_runtime);
IFX_INLINE void driver_qspi_begin_stream(driver_qspi_runtime_t* qspi_runtime);
IFX_INLINE void driver_qspi_end_stream(driver_qspi_runtime_t* qspi_runtime);
IFX_INLINE void driver_qspi_clear_fifo(driver_qspi_runtime_t* qspi_runtime);

void driver_qspi_init(driver_qspi_cfg_t* qspi_cfg, driver_qspi_runtime_t* qspi_runtime)
{
    qspi_cfg->qspi_module = IfxQspi_getAddress(qspi_cfg->qspi_index);
    qspi_cfg->qspi_modulecfg.qspi = qspi_cfg->qspi_module;
    qspi_cfg->qspi_modulecfg.pins = &qspi_cfg->qspi_pins;
    IfxQspi_SpiMaster_initModule(&qspi_runtime->qspi_modulehn, &qspi_cfg->qspi_modulecfg);
    qspi_cfg->qspi_channelcfg.qspi = qspi_runtime->qspi_modulehn.qspi;
    qspi_cfg->qspi_channelcfg.dma = &qspi_runtime->qspi_modulehn.dma;
    qspi_cfg->qspi_channelcfg.spiMaster = &qspi_runtime->qspi_modulehn;
    qspi_cfg->qspi_channelcfg.sls = qspi_cfg->qspi_sls;
    IfxQspi_SpiMaster_initChannel(&qspi_runtime->qspi_channelhn, &qspi_cfg->qspi_channelcfg);

    //IfxQspi_configPT1Event(qspi_runtime->qspi_channelhn.spiMaster->qspi, IfxQspi_PhaseTransitionEvent_endOfFrame);
    //IfxQspi_enablePT1Event(qspi_runtime->qspi_channelhn.spiMaster->qspi, TRUE);

    driver_qspi_auto_sls(qspi_runtime);
}

void driver_qspi_write_8bit(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint8 data)
{
    Ifx_QSPI* qspi = qspi_runtime->qspi_channelhn.spiMaster->qspi;
    driver_qspi_end_stream(qspi_runtime);
    IfxQspi_writeTransmitFifo(qspi, (uint32)data);
    while (qspi->STATUS.B.TXFIFOLEVEL != 0) {}
    IfxQspi_clearAllEventFlags(qspi);
}

void driver_qspi_write_16bit(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint16 data)
{
    Ifx_QSPI* qspi = qspi_runtime->qspi_channelhn.spiMaster->qspi;
    driver_qspi_end_stream(qspi_runtime);
    IfxQspi_writeTransmitFifo(qspi, (uint32)data);
    while (qspi->STATUS.B.TXFIFOLEVEL != 0) {}
    IfxQspi_clearAllEventFlags(qspi);
}

void driver_qspi_write_32bit(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint32 data)
{
    Ifx_QSPI* qspi = qspi_runtime->qspi_channelhn.spiMaster->qspi;

    driver_qspi_end_stream(qspi_runtime);
    IfxQspi_writeTransmitFifo(qspi, (uint32)data);
    while (qspi->STATUS.B.TXFIFOLEVEL != 0) {}
    IfxQspi_clearAllEventFlags(qspi);
}

void driver_qspi_write_8bit_register(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint8 register_name, IFX_CONST uint8 data)
{
    Ifx_QSPI* qspi = qspi_runtime->qspi_channelhn.spiMaster->qspi;

    driver_qspi_begin_stream(qspi_runtime);
    IfxQspi_writeTransmitFifo(qspi, (uint32)register_name);
    while (qspi->STATUS.B.TXFIFOLEVEL != 0) {}
    driver_qspi_end_stream(qspi_runtime);
    IfxQspi_writeTransmitFifo(qspi, (uint32)data);
    while (qspi->STATUS.B.TXFIFOLEVEL != 0) {}
    IfxQspi_clearAllEventFlags(qspi);
}

void driver_qspi_write_16bit_register(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint16 register_name, IFX_CONST uint16 data)
{
    Ifx_QSPI* qspi = qspi_runtime->qspi_channelhn.spiMaster->qspi;

    driver_qspi_begin_stream(qspi_runtime);
    IfxQspi_writeTransmitFifo(qspi, (uint32)register_name);
    while (qspi->STATUS.B.TXFIFOLEVEL != 0) {}
    driver_qspi_end_stream(qspi_runtime);
    IfxQspi_writeTransmitFifo(qspi, (uint32)data);
    while (qspi->STATUS.B.TXFIFOLEVEL != 0) {}
    IfxQspi_clearAllEventFlags(qspi);
}

void driver_qspi_write_32bit_register(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint32 register_name, IFX_CONST uint32 data)
{
    Ifx_QSPI* qspi = qspi_runtime->qspi_channelhn.spiMaster->qspi;

    driver_qspi_begin_stream(qspi_runtime);
    IfxQspi_writeTransmitFifo(qspi, (uint32)register_name);
    while (qspi->STATUS.B.TXFIFOLEVEL != 0) {}
    driver_qspi_end_stream(qspi_runtime);
    IfxQspi_writeTransmitFifo(qspi, (uint32)data);
    while (qspi->STATUS.B.TXFIFOLEVEL != 0) {}
    IfxQspi_clearAllEventFlags(qspi);
}

void driver_qspi_write_8bit_array(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint8* data, uint32 len)
{
    Ifx_QSPI* qspi = qspi_runtime->qspi_channelhn.spiMaster->qspi;

    driver_qspi_begin_stream(qspi_runtime);
    do
    {
        if (len == 1)
        {
            driver_qspi_end_stream(qspi_runtime);
        }
        IfxQspi_writeTransmitFifo(qspi, (uint32)(*data++));
        while (qspi->STATUS.B.TXFIFOLEVEL != 0) {}
    } while (--len);
    IfxQspi_clearAllEventFlags(qspi);
}

void driver_qspi_write_16bit_array(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint16* data, uint32 len)
{
    Ifx_QSPI* qspi = qspi_runtime->qspi_channelhn.spiMaster->qspi;

    driver_qspi_begin_stream(qspi_runtime);
    do
    {
        if (len == 1)
        {
            driver_qspi_end_stream(qspi_runtime);
        }
        IfxQspi_writeTransmitFifo(qspi, (uint32)(*data++));
        while (qspi->STATUS.B.TXFIFOLEVEL != 0) {}
    } while (--len);
    IfxQspi_clearAllEventFlags(qspi);
}

void driver_qspi_write_32bit_array(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint32* data, uint32 len)
{
    Ifx_QSPI* qspi = qspi_runtime->qspi_channelhn.spiMaster->qspi;

    driver_qspi_begin_stream(qspi_runtime);
    do
    {
        if (len == 1)
        {
            driver_qspi_end_stream(qspi_runtime);
        }
        IfxQspi_writeTransmitFifo(qspi, (uint32)(*data++));
        while (qspi->STATUS.B.TXFIFOLEVEL != 0) {}
    } while (--len);
    IfxQspi_clearAllEventFlags(qspi);
}

void driver_qspi_write_8bit_registers(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint8 register_name, IFX_CONST uint8* data, uint32 len)
{
    Ifx_QSPI* qspi = qspi_runtime->qspi_channelhn.spiMaster->qspi;

    driver_qspi_begin_stream(qspi_runtime);
    IfxQspi_writeTransmitFifo(qspi, (uint32)register_name);
    do
    {
        if (len == 1)
        {
            driver_qspi_end_stream(qspi_runtime);
        }
        IfxQspi_writeTransmitFifo(qspi, (uint32)(*data++));
        while (qspi->STATUS.B.TXFIFOLEVEL != 0) {}
    } while (--len);
    IfxQspi_clearAllEventFlags(qspi);
}

void driver_qspi_write_16bit_registers(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint16 register_name, IFX_CONST uint16* data, uint32 len)
{
    Ifx_QSPI* qspi = qspi_runtime->qspi_channelhn.spiMaster->qspi;

    driver_qspi_begin_stream(qspi_runtime);
    IfxQspi_writeTransmitFifo(qspi, (uint32)register_name);
    do
    {
        if (len == 1)
        {
            driver_qspi_end_stream(qspi_runtime);
        }
        IfxQspi_writeTransmitFifo(qspi, (uint32)(*data++));
        while (qspi->STATUS.B.TXFIFOLEVEL != 0) {}
    } while (--len);
    IfxQspi_clearAllEventFlags(qspi);
}

void driver_qspi_write_32bit_registers(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint32 register_name, IFX_CONST uint32* data, uint32 len)
{
    Ifx_QSPI* qspi = qspi_runtime->qspi_channelhn.spiMaster->qspi;

    driver_qspi_begin_stream(qspi_runtime);
    IfxQspi_writeTransmitFifo(qspi, (uint32)register_name);
    do
    {
        if (len == 1)
        {
            driver_qspi_end_stream(qspi_runtime);
        }
        IfxQspi_writeTransmitFifo(qspi, (uint32)(*data++));
        while (qspi->STATUS.B.TXFIFOLEVEL != 0) {}
    } while (--len);
    IfxQspi_clearAllEventFlags(qspi);
}

uint8 driver_qspi_read_8bit(driver_qspi_runtime_t* qspi_runtime)
{
    Ifx_QSPI* qspi = qspi_runtime->qspi_channelhn.spiMaster->qspi;

    driver_qspi_end_stream(qspi_runtime);
    driver_qspi_clear_fifo(qspi_runtime);
    IfxQspi_writeTransmitFifo(qspi, 0);
    while (qspi->STATUS.B.RXFIFOLEVEL == 0)
    {
    }
    IfxQspi_clearAllEventFlags(qspi);
    return (uint8)IfxQspi_readReceiveFifo(qspi);
}

uint16 driver_qspi_read_16bit(driver_qspi_runtime_t* qspi_runtime)
{
    Ifx_QSPI* qspi = qspi_runtime->qspi_channelhn.spiMaster->qspi;

    driver_qspi_end_stream(qspi_runtime);
    driver_qspi_clear_fifo(qspi_runtime);
    IfxQspi_writeTransmitFifo(qspi, 0);
    while (qspi->STATUS.B.RXFIFOLEVEL == 0)
    {
    }
    IfxQspi_clearAllEventFlags(qspi);
    return (uint16)IfxQspi_readReceiveFifo(qspi);
}

uint32 driver_qspi_read_32bit(driver_qspi_runtime_t* qspi_runtime)
{
    Ifx_QSPI* qspi = qspi_runtime->qspi_channelhn.spiMaster->qspi;

    driver_qspi_end_stream(qspi_runtime);
    driver_qspi_clear_fifo(qspi_runtime);
    IfxQspi_writeTransmitFifo(qspi, 0);
    while (qspi->STATUS.B.RXFIFOLEVEL == 0)
    {
    }
    IfxQspi_clearAllEventFlags(qspi);
    return (uint32)IfxQspi_readReceiveFifo(qspi);
}

uint8 driver_qspi_read_8bit_register(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint8 register_name)
{
    Ifx_QSPI* qspi = qspi_runtime->qspi_channelhn.spiMaster->qspi;

    driver_qspi_clear_fifo(qspi_runtime);
    driver_qspi_begin_stream(qspi_runtime);
    IfxQspi_writeTransmitFifo(qspi, (uint32)register_name);
    while (qspi->STATUS.B.RXFIFOLEVEL == 0)
    {
    }
    driver_qspi_clear_fifo(qspi_runtime);
    driver_qspi_end_stream(qspi_runtime);
    IfxQspi_writeTransmitFifo(qspi, 0);
    while (qspi->STATUS.B.RXFIFOLEVEL == 0)
    {
    }
    IfxQspi_clearAllEventFlags(qspi);
    return (uint8)IfxQspi_readReceiveFifo(qspi);
}

uint16 driver_qspi_read_16bit_register(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint16 register_name)
{
    Ifx_QSPI* qspi = qspi_runtime->qspi_channelhn.spiMaster->qspi;

    driver_qspi_clear_fifo(qspi_runtime);
    driver_qspi_begin_stream(qspi_runtime);
    IfxQspi_writeTransmitFifo(qspi, (uint32)register_name);
    while (qspi->STATUS.B.RXFIFOLEVEL == 0)
    {
    }
    driver_qspi_clear_fifo(qspi_runtime);
    driver_qspi_end_stream(qspi_runtime);
    IfxQspi_writeTransmitFifo(qspi, 0);
    while (qspi->STATUS.B.RXFIFOLEVEL == 0)
    {
    }
    IfxQspi_clearAllEventFlags(qspi);
    return (uint16)IfxQspi_readReceiveFifo(qspi);
}

uint32 driver_qspi_read_32bit_register(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint32 register_name)
{
    Ifx_QSPI* qspi = qspi_runtime->qspi_channelhn.spiMaster->qspi;

    driver_qspi_clear_fifo(qspi_runtime);
    driver_qspi_begin_stream(qspi_runtime);
    IfxQspi_writeTransmitFifo(qspi, (uint32)register_name);
    while (qspi->STATUS.B.RXFIFOLEVEL == 0)
    {
    }
    driver_qspi_clear_fifo(qspi_runtime);
    driver_qspi_end_stream(qspi_runtime);
    IfxQspi_writeTransmitFifo(qspi, 0);
    while (qspi->STATUS.B.RXFIFOLEVEL == 0)
    {
    }
    IfxQspi_clearAllEventFlags(qspi);
    return (uint32)IfxQspi_readReceiveFifo(qspi);
}

void driver_qspi_read_8bit_array(driver_qspi_runtime_t* qspi_runtime, uint8* data, uint32 len)
{
    Ifx_QSPI* qspi = qspi_runtime->qspi_channelhn.spiMaster->qspi;

    driver_qspi_begin_stream(qspi_runtime);
    driver_qspi_clear_fifo(qspi_runtime);
    do
    {
        if (len == 1)
        {
            driver_qspi_end_stream(qspi_runtime);
        }
        IfxQspi_writeTransmitFifo(qspi, 0);
        while (qspi->STATUS.B.RXFIFOLEVEL == 0)
        {
        }
        *data++ = (uint8)IfxQspi_readReceiveFifo(qspi);
    } while (--len);
    IfxQspi_clearAllEventFlags(qspi);
}

void driver_qspi_read_16bit_array(driver_qspi_runtime_t* qspi_runtime, uint16* data, uint32 len)
{
    Ifx_QSPI* qspi = qspi_runtime->qspi_channelhn.spiMaster->qspi;

    driver_qspi_begin_stream(qspi_runtime);
    driver_qspi_clear_fifo(qspi_runtime);
    do
    {
        if (len == 1)
        {
            driver_qspi_end_stream(qspi_runtime);
        }
        IfxQspi_writeTransmitFifo(qspi, 0);
        while (qspi->STATUS.B.RXFIFOLEVEL == 0)
        {
        }
        *data++ = (uint16)IfxQspi_readReceiveFifo(qspi);
    } while (--len);
    IfxQspi_clearAllEventFlags(qspi);
}

void driver_qspi_read_32bit_array(driver_qspi_runtime_t* qspi_runtime, uint32* data, uint32 len)
{
    Ifx_QSPI* qspi = qspi_runtime->qspi_channelhn.spiMaster->qspi;

    driver_qspi_begin_stream(qspi_runtime);
    driver_qspi_clear_fifo(qspi_runtime);
    do
    {
        if (len == 1)
        {
            driver_qspi_end_stream(qspi_runtime);
        }
        IfxQspi_writeTransmitFifo(qspi, 0);
        while (qspi->STATUS.B.RXFIFOLEVEL == 0)
        {
        }
        *data++ = (uint32)IfxQspi_readReceiveFifo(qspi);
    } while (--len);
    IfxQspi_clearAllEventFlags(qspi);
}

void driver_qspi_read_8bit_registers(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint8 register_name, uint8* data, uint32 len)
{
    Ifx_QSPI* qspi = qspi_runtime->qspi_channelhn.spiMaster->qspi;

    driver_qspi_clear_fifo(qspi_runtime);
    driver_qspi_begin_stream(qspi_runtime);
    IfxQspi_writeTransmitFifo(qspi, (uint32)register_name);
    while (qspi->STATUS.B.RXFIFOLEVEL == 0)
    {
    }
    driver_qspi_clear_fifo(qspi_runtime);
    do
    {
        if (len == 1)
        {
            driver_qspi_end_stream(qspi_runtime);
        }
        IfxQspi_writeTransmitFifo(qspi, 0);
        while (qspi->STATUS.B.RXFIFOLEVEL == 0)
        {
        }
        *data++ = (uint8)IfxQspi_readReceiveFifo(qspi);
    } while (--len);
    IfxQspi_clearAllEventFlags(qspi);
}

void driver_qspi_read_16bit_registers(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint16 register_name, uint16* data, uint32 len)
{
    Ifx_QSPI* qspi = qspi_runtime->qspi_channelhn.spiMaster->qspi;

    driver_qspi_clear_fifo(qspi_runtime);
    driver_qspi_begin_stream(qspi_runtime);
    IfxQspi_writeTransmitFifo(qspi, (uint32)register_name);
    while (qspi->STATUS.B.RXFIFOLEVEL == 0)
    {
    }
    driver_qspi_clear_fifo(qspi_runtime);
    do
    {
        if (len == 1)
        {
            driver_qspi_end_stream(qspi_runtime);
        }
        IfxQspi_writeTransmitFifo(qspi, 0);
        while (qspi->STATUS.B.RXFIFOLEVEL == 0)
        {
        }
        *data++ = (uint16)IfxQspi_readReceiveFifo(qspi);
    } while (--len);
    IfxQspi_clearAllEventFlags(qspi);
}

void driver_qspi_read_32bit_registers(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint32 register_name, uint32* data, uint32 len)
{
    Ifx_QSPI* qspi = qspi_runtime->qspi_channelhn.spiMaster->qspi;

    driver_qspi_clear_fifo(qspi_runtime);
    driver_qspi_begin_stream(qspi_runtime);
    IfxQspi_writeTransmitFifo(qspi, (uint32)register_name);
    while (qspi->STATUS.B.RXFIFOLEVEL == 0)
    {
    }
    driver_qspi_clear_fifo(qspi_runtime);
    do
    {
        if (len == 1)
        {
            driver_qspi_end_stream(qspi_runtime);
        }
        IfxQspi_writeTransmitFifo(qspi, 0);
        while (qspi->STATUS.B.RXFIFOLEVEL == 0)
        {
        }
        *data++ = (uint32)IfxQspi_readReceiveFifo(qspi);
    } while (--len);
    IfxQspi_clearAllEventFlags(qspi);
}

void driver_qspi_transfer_8bit(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint8* write_buffer, uint8* read_buffer, uint32 len)
{
    Ifx_QSPI* qspi = qspi_runtime->qspi_channelhn.spiMaster->qspi;

    driver_qspi_begin_stream(qspi_runtime);
    driver_qspi_clear_fifo(qspi_runtime);
    do
    {
        if (len == 1)
        {
            driver_qspi_end_stream(qspi_runtime);
        }
        IfxQspi_writeTransmitFifo(qspi, (uint32)(*write_buffer++));
        if (read_buffer != NULL_PTR)
        {
            while (qspi->STATUS.B.RXFIFOLEVEL == 0)
            {
            }
            *read_buffer++ = (uint8)IfxQspi_readReceiveFifo(qspi);
        }
    } while (--len);
    IfxQspi_clearAllEventFlags(qspi);
}

void driver_qspi_transfer_16bit(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint16* write_buffer, uint16* read_buffer, uint32 len)
{
    Ifx_QSPI* qspi = qspi_runtime->qspi_channelhn.spiMaster->qspi;

    driver_qspi_begin_stream(qspi_runtime);
    driver_qspi_clear_fifo(qspi_runtime);
    do
    {
        if (len == 1)
        {
            driver_qspi_end_stream(qspi_runtime);
        }
        IfxQspi_writeTransmitFifo(qspi, (uint32)(*write_buffer++));
        if (read_buffer != NULL_PTR)
        {
            while (qspi->STATUS.B.RXFIFOLEVEL == 0)
            {
            }
            *read_buffer++ = (uint16)IfxQspi_readReceiveFifo(qspi);
        }
    } while (--len);
    IfxQspi_clearAllEventFlags(qspi);
}

void driver_qspi_transfer_32bit(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint32* write_buffer, uint32* read_buffer, uint32 len)
{
    Ifx_QSPI* qspi = qspi_runtime->qspi_channelhn.spiMaster->qspi;

    driver_qspi_begin_stream(qspi_runtime);
    driver_qspi_clear_fifo(qspi_runtime);
    do
    {
        if (len == 1)
        {
            driver_qspi_end_stream(qspi_runtime);
        }
        IfxQspi_writeTransmitFifo(qspi, (uint32)(*write_buffer++));
        if (read_buffer != NULL_PTR)
        {
            while (qspi->STATUS.B.RXFIFOLEVEL == 0)
            {
            }
            *read_buffer++ = (uint32)IfxQspi_readReceiveFifo(qspi);
        }
    } while (--len);
    IfxQspi_clearAllEventFlags(qspi);
}

IFX_INLINE void driver_qspi_keep_sls(driver_qspi_runtime_t* qspi_runtime)
{
    IfxQspi_writeBasicConfigurationBeginStream(qspi_runtime->qspi_channelhn.spiMaster->qspi, qspi_runtime->qspi_channelhn.bacon.U);
}

IFX_INLINE void driver_qspi_auto_sls(driver_qspi_runtime_t* qspi_runtime)
{
    IfxQspi_writeBasicConfigurationEndStream(qspi_runtime->qspi_channelhn.spiMaster->qspi, qspi_runtime->qspi_channelhn.bacon.U);
}

IFX_INLINE void driver_qspi_begin_stream(driver_qspi_runtime_t* qspi_runtime)
{
    IfxQspi_writeBasicConfigurationBeginStream(qspi_runtime->qspi_channelhn.spiMaster->qspi, qspi_runtime->qspi_channelhn.bacon.U);
}

IFX_INLINE void driver_qspi_end_stream(driver_qspi_runtime_t* qspi_runtime)
{
    IfxQspi_writeBasicConfigurationEndStream(qspi_runtime->qspi_channelhn.spiMaster->qspi, qspi_runtime->qspi_channelhn.bacon.U);
}

IFX_INLINE void driver_qspi_clear_fifo(driver_qspi_runtime_t* qspi_runtime)
{
    uint32 fifo_num = qspi_runtime->qspi_channelhn.spiMaster->qspi->STATUS.B.RXFIFOLEVEL;

    while (fifo_num--)
    {
        (uint8)IfxQspi_readReceiveFifo(qspi_runtime->qspi_channelhn.spiMaster->qspi);
    }
}
