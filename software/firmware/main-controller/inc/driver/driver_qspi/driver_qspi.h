#ifndef MAD_CIRCUITS_DRIVER_QSPI_H
#define MAD_CIRCUITS_DRIVER_QSPI_H

#include "IfxQspi.h"
#include "IfxQspi_SpiMaster.h"
#include "Ifx_Types.h"

typedef struct
{
    Ifx_QSPI* qspi_module;
    IfxQspi_Index qspi_index;
    IfxQspi_SpiMaster_Config qspi_modulecfg;
    IFX_CONST IfxQspi_SpiMaster_Pins qspi_pins;
    IfxQspi_SpiMaster_ChannelConfig qspi_channelcfg;
    IfxQspi_SpiMaster_InputOutput qspi_sls;
} driver_qspi_cfg_t;

typedef struct
{
    IfxQspi_SpiMaster qspi_modulehn;
    IfxQspi_SpiMaster_Channel qspi_channelhn;
} driver_qspi_runtime_t;

void driver_qspi_init(driver_qspi_cfg_t* qspi_cfg, driver_qspi_runtime_t* qspi_runtime);

void driver_qspi_write_8bit(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint8 data);
void driver_qspi_write_16bit(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint16 data);
void driver_qspi_write_32bit(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint32 data);
void driver_qspi_write_8bit_register(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint8 register_name, IFX_CONST uint8 data);
void driver_qspi_write_16bit_register(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint16 register_name, IFX_CONST uint16 data);
void driver_qspi_write_32bit_register(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint32 register_name, IFX_CONST uint32 data);
void driver_qspi_write_8bit_array(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint8* data, uint32 len);
void driver_qspi_write_16bit_array(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint16* data, uint32 len);
void driver_qspi_write_32bit_array(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint32* data, uint32 len);
void driver_qspi_write_8bit_registers(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint8 register_name, IFX_CONST uint8* data, uint32 len);
void driver_qspi_write_16bit_registers(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint16 register_name, IFX_CONST uint16* data, uint32 len);
void driver_qspi_write_32bit_registers(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint32 register_name, IFX_CONST uint32* data, uint32 len);
uint8 driver_qspi_read_8bit(driver_qspi_runtime_t* qspi_runtime);
uint16 driver_qspi_read_16bit(driver_qspi_runtime_t* qspi_runtime);
uint32 driver_qspi_read_32bit(driver_qspi_runtime_t* qspi_runtime);
uint8 driver_qspi_read_8bit_register(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint8 register_name);
uint16 driver_qspi_read_16bit_register(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint16 register_name);
uint32 driver_qspi_read_32bit_register(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint32 register_name);
void driver_qspi_read_8bit_array(driver_qspi_runtime_t* qspi_runtime, uint8* data, uint32 len);
void driver_qspi_read_16bit_array(driver_qspi_runtime_t* qspi_runtime, uint16* data, uint32 len);
void driver_qspi_read_32bit_array(driver_qspi_runtime_t* qspi_runtime, uint32* data, uint32 len);
void driver_qspi_read_8bit_registers(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint8 register_name, uint8* data, uint32 len);
void driver_qspi_read_16bit_registers(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint16 register_name, uint16* data, uint32 len);
void driver_qspi_read_32bit_registers(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint32 register_name, uint32* data, uint32 len);
void driver_qspi_transfer_8bit(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint8* write_buffer, uint8* read_buffer, uint32 len);
void driver_qspi_transfer_16bit(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint16* write_buffer, uint16* read_buffer, uint32 len);
void driver_qspi_transfer_32bit(driver_qspi_runtime_t* qspi_runtime, IFX_CONST uint32* write_buffer, uint32* read_buffer, uint32 len);

#endif
