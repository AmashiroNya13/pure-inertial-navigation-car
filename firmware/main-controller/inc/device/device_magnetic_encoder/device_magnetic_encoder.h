/**
 * @file device_magnetic_encoder.h
 * @brief 磁编码器设备对外调用接口。
 */

#ifndef MAD_CIRCUITS_DEVICE_MAGNETIC_ENCODER_H
#define MAD_CIRCUITS_DEVICE_MAGNETIC_ENCODER_H

#include "../../../config/device/device_magnetic_encoder/device_magnetic_encoder_cfg.h"

/**
 * @brief 初始化指定磁编码器设备。
 * @param[in] magnetic_encoder_id 磁编码器设备编号。
 * @return void
 */
void device_magnetic_encoder_init(device_magnetic_encoder_id_t magnetic_encoder_id);

/**
 * @brief 初始化全部已配置的磁编码器设备。
 * @param[in] void 无参数。
 * @return void
 */
void device_magnetic_encoder_init_all(void);

/**
 * @brief 注册指定磁编码器传输完成后的回调函数。
 * @param[in] magnetic_encoder_id 磁编码器设备编号。
 * @param[in] device_magnetic_encoder_callback 回调函数指针。
 * @return void
 */
void device_magnetic_encoder_register_callback(device_magnetic_encoder_id_t magnetic_encoder_id,
                                               void (*device_magnetic_encoder_callback) (void));

/**
 * @brief 读取磁编码器原始角度值。
 * @param[in] magnetic_encoder_id 磁编码器设备编号。
 * @return 原始角度值。
 */
uint16 device_magnetic_encoder_read_raw_angle(device_magnetic_encoder_id_t magnetic_encoder_id);

/**
 * @brief 读取并换算磁编码器角度值。
 * @param[in] magnetic_encoder_id 磁编码器设备编号。
 * @return 角度值，单位：度。
 */
float32 device_magnetic_encoder_read_angle_degree(device_magnetic_encoder_id_t magnetic_encoder_id);

/**
 * @brief 读取磁编码器磁场强度值。
 * @param[in] magnetic_encoder_id 磁编码器设备编号。
 * @return 磁场强度寄存器值。
 */
uint16 device_magnetic_encoder_read_magnitude(device_magnetic_encoder_id_t magnetic_encoder_id);

/**
 * @brief 读取磁编码器自动增益控制值。
 * @param[in] magnetic_encoder_id 磁编码器设备编号。
 * @return AGC 寄存器值。
 */
uint16 device_magnetic_encoder_read_agc(device_magnetic_encoder_id_t magnetic_encoder_id);

/**
 * @brief 读取磁编码器错误寄存器值。
 * @param[in] magnetic_encoder_id 磁编码器设备编号。
 * @return 错误寄存器值。
 */
uint16 device_magnetic_encoder_read_error(device_magnetic_encoder_id_t magnetic_encoder_id);

#endif
