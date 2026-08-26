/**
 * @file device_magnetic_encoder.c
 * @brief 磁编码器设备对外调用接口实现。
 */

#include "../../../inc/device/device_magnetic_encoder/device_magnetic_encoder.h"
#include "../../../config/device/device_magnetic_encoder/device_magnetic_encoder_register.h"

IFX_INLINE uint16 device_magnetic_encoder_command_build(device_magnetic_encoder_register_t register_address, boolean is_read);
IFX_INLINE uint16 device_magnetic_encoder_parity_append(uint16 value);
IFX_INLINE uint16 device_magnetic_encoder_read_register(device_magnetic_encoder_id_t magnetic_encoder_id,
                                                    device_magnetic_encoder_register_t register_address);
IFX_INLINE void device_magnetic_encoder_write_register(device_magnetic_encoder_id_t magnetic_encoder_id,
                                                   device_magnetic_encoder_register_t register_address,
                                                   uint16 value);

/**
 * @brief 初始化指定磁编码器设备。
 * @param[in] magnetic_encoder_id 磁编码器设备编号。
 * @return void
 */
void device_magnetic_encoder_init(device_magnetic_encoder_id_t magnetic_encoder_id)
{
    device_magnetic_encoder_cfg_t* magnetic_encoder_cfg = device_magnetic_encoder_cfg_table_get();
    device_magnetic_encoder_runtime_t* magnetic_encoder_runtime = device_magnetic_encoder_runtime_table_get();

    driver_gtm_atom_timer_init(&magnetic_encoder_cfg[magnetic_encoder_id].gtm_atom_timer_cfg,
                               &magnetic_encoder_runtime[magnetic_encoder_id].gtm_atom_timer_runtime);
    driver_dma_init(&magnetic_encoder_cfg[magnetic_encoder_id].tx_dma_cfg,
                    &magnetic_encoder_runtime[magnetic_encoder_id].tx_dma_runtime);
    driver_qspi_init(&magnetic_encoder_cfg[magnetic_encoder_id].qspi_cfg,
                     &magnetic_encoder_runtime[magnetic_encoder_id].qspi_runtime);
    driver_dma_init(&magnetic_encoder_cfg[magnetic_encoder_id].rx_dma_cfg,
                    &magnetic_encoder_runtime[magnetic_encoder_id].rx_dma_runtime);

    driver_dma_setSourceDestinationAddress(&magnetic_encoder_runtime[magnetic_encoder_id].tx_dma_runtime,
                                           (uint32)&magnetic_encoder_runtime[magnetic_encoder_id].dma_command_buffer,
                                           ((uint32)&magnetic_encoder_runtime[magnetic_encoder_id].qspi_runtime.qspi_channelhn.spiMaster->qspi->DATAENTRY[magnetic_encoder_runtime[magnetic_encoder_id].qspi_runtime.qspi_channelhn.channelId % 8].U));
    driver_dma_setSourceDestinationAddress(&magnetic_encoder_runtime[magnetic_encoder_id].rx_dma_runtime,
                                           ((uint32)&magnetic_encoder_runtime[magnetic_encoder_id].qspi_runtime.qspi_channelhn.spiMaster->qspi->RXEXIT),
                                           (uint32)&magnetic_encoder_runtime[magnetic_encoder_id].dma_receive_buffer);

    driver_gtm_atom_timer_start(&magnetic_encoder_runtime[magnetic_encoder_id].gtm_atom_timer_runtime);
    driver_dma_start(&magnetic_encoder_runtime[magnetic_encoder_id].tx_dma_runtime);
    driver_dma_start(&magnetic_encoder_runtime[magnetic_encoder_id].rx_dma_runtime);
}

/**
 * @brief 初始化全部已配置的磁编码器设备。
 * @param[in] void 无参数。
 * @return void
 */
void device_magnetic_encoder_init_all(void)
{
    device_magnetic_encoder_init(DEVICE_MAGNETIC_ENCODER_1);
    device_magnetic_encoder_init(DEVICE_MAGNETIC_ENCODER_2);
}

/**
 * @brief 注册指定磁编码器传输完成后的回调函数。
 * @param[in] magnetic_encoder_id 磁编码器设备编号。
 * @param[in] device_magnetic_encoder_callback 回调函数指针。
 * @return void
 */
void device_magnetic_encoder_register_callback(device_magnetic_encoder_id_t magnetic_encoder_id,
                                               void (*device_magnetic_encoder_callback) (void))
{
    device_magnetic_encoder_runtime_t* magnetic_encoder_runtime = device_magnetic_encoder_runtime_table_get();
    magnetic_encoder_runtime[magnetic_encoder_id].device_magnetic_encoder_callback = device_magnetic_encoder_callback;
}

/**
 * @brief 读取磁编码器原始角度值。
 * @param[in] magnetic_encoder_id 磁编码器设备编号。
 * @return 原始角度值。
 */
uint16 device_magnetic_encoder_read_raw_angle(device_magnetic_encoder_id_t magnetic_encoder_id)
{
    device_magnetic_encoder_angle_reg_t angle_reg;

    angle_reg.U = device_magnetic_encoder_read_register(magnetic_encoder_id,
                                                        DEVICE_MAGNETIC_ENCODER_REGISTER_ANGLE);

    return angle_reg.B.angle;
}

/**
 * @brief 读取并换算磁编码器角度值。
 * @param[in] magnetic_encoder_id 磁编码器设备编号。
 * @return 角度值，单位：度。
 */
float32 device_magnetic_encoder_read_angle_degree(device_magnetic_encoder_id_t magnetic_encoder_id)
{
    return ((float32)device_magnetic_encoder_read_raw_angle(magnetic_encoder_id)
            * DEVICE_MAGNETIC_ENCODER_ANGLE_DEGREE_PER_LSB);
}

/**
 * @brief 读取磁编码器磁场强度值。
 * @param[in] magnetic_encoder_id 磁编码器设备编号。
 * @return 磁场强度寄存器值。
 */
uint16 device_magnetic_encoder_read_magnitude(device_magnetic_encoder_id_t magnetic_encoder_id)
{
    device_magnetic_encoder_magnitude_reg_t magnitude_reg;

    magnitude_reg.U = device_magnetic_encoder_read_register(magnetic_encoder_id,
                                                            DEVICE_MAGNETIC_ENCODER_REGISTER_MAGNITUDE);

    return magnitude_reg.B.magnitude;
}

/**
 * @brief 读取磁编码器自动增益控制值。
 * @param[in] magnetic_encoder_id 磁编码器设备编号。
 * @return AGC 寄存器值。
 */
uint16 device_magnetic_encoder_read_agc(device_magnetic_encoder_id_t magnetic_encoder_id)
{
    device_magnetic_encoder_agc_reg_t agc_reg;

    agc_reg.U = device_magnetic_encoder_read_register(magnetic_encoder_id,
                                                      DEVICE_MAGNETIC_ENCODER_REGISTER_AGC);

    return agc_reg.B.agc;
}

/**
 * @brief 读取磁编码器错误寄存器值。
 * @param[in] magnetic_encoder_id 磁编码器设备编号。
 * @return 错误寄存器值。
 */
uint16 device_magnetic_encoder_read_error(device_magnetic_encoder_id_t magnetic_encoder_id)
{
    device_magnetic_encoder_error_flag_reg_t error_reg;

    error_reg.U = device_magnetic_encoder_read_register(magnetic_encoder_id,
                                          DEVICE_MAGNETIC_ENCODER_REGISTER_ERROR);
    return error_reg.U;
}

IFX_INLINE uint16 device_magnetic_encoder_command_build(device_magnetic_encoder_register_t register_address, boolean is_read)
{
    uint16 command = ((uint16)register_address & DEVICE_MAGNETIC_ENCODER_REGISTER_ADDRESS_MASK);

    if (is_read != FALSE)
    {
        command |= DEVICE_MAGNETIC_ENCODER_REGISTER_READ_MASK;
    }

    return device_magnetic_encoder_parity_append(command);
}

IFX_INLINE uint16 device_magnetic_encoder_parity_append(uint16 value)
{
    uint16 parity_source = (uint16)(value & DEVICE_MAGNETIC_ENCODER_REGISTER_PARITY_DATA_MASK);
    uint16 parity = 0u;

    while (parity_source != 0u)
    {
        parity ^= (uint16)(parity_source & 0x1u);
        parity_source >>= 1u;
    }

    if (parity != 0u)
    {
        value |= DEVICE_MAGNETIC_ENCODER_REGISTER_PARITY_MASK;
    }

    return value;
}

IFX_INLINE uint16 device_magnetic_encoder_read_register(device_magnetic_encoder_id_t magnetic_encoder_id,
                                                    device_magnetic_encoder_register_t register_address)
{
    device_magnetic_encoder_runtime_t* magnetic_encoder_runtime = device_magnetic_encoder_runtime_table_get();
    uint16 raw_value;
    device_magnetic_encoder_frame_t frame;

    driver_qspi_write_16bit(&magnetic_encoder_runtime[magnetic_encoder_id].qspi_runtime,
                            device_magnetic_encoder_command_build(register_address, TRUE));
    raw_value = driver_qspi_read_16bit(&magnetic_encoder_runtime[magnetic_encoder_id].qspi_runtime);

    magnetic_encoder_runtime[magnetic_encoder_id].last_frame = raw_value;
    frame.U = raw_value;
    return (uint16)(frame.U & DEVICE_MAGNETIC_ENCODER_REGISTER_DATA_MASK);
}

IFX_INLINE void device_magnetic_encoder_write_register(device_magnetic_encoder_id_t magnetic_encoder_id,
                                                   device_magnetic_encoder_register_t register_address,
                                                   uint16 value)
{
    device_magnetic_encoder_runtime_t* magnetic_encoder_runtime = device_magnetic_encoder_runtime_table_get();
    uint16 write_command = device_magnetic_encoder_command_build(register_address, FALSE);
    uint16 write_value = device_magnetic_encoder_parity_append((uint16)(value & DEVICE_MAGNETIC_ENCODER_REGISTER_DATA_MASK));

    driver_qspi_write_16bit_register(&magnetic_encoder_runtime[magnetic_encoder_id].qspi_runtime,
                                     write_command,
                                     write_value);
}

