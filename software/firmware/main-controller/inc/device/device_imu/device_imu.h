/**
 * @file device_imu.h
 * @brief IMU 设备对外调用接口。
 */

#ifndef MAD_CIRCUITS_DEVICE_IMU_H
#define MAD_CIRCUITS_DEVICE_IMU_H

#include "../../../config/device/device_imu/device_imu_cfg.h"

/**
 * @brief 初始化指定 IMU 设备。
 * @param[in] imu_id IMU 设备编号。
 * @return void
 */
void device_imu_init(device_imu_id_t imu_id);

/**
 * @brief 初始化全部已配置的 IMU 设备。
 * @param[in] void 无参数。
 * @return void
 */
void device_imu_init_all(void);

/**
 * @brief 注册指定 IMU 采样传输完成后的回调函数。
 * @param[in] imu_id IMU 设备编号。
 * @param[in] device_imu_callback 回调函数指针。
 * @return void
 */
void device_imu_register_callback(device_imu_id_t imu_id, void (*device_imu_callback) (void));

/**
 * @brief 读取 IMU 的 WHO_AM_I 寄存器值。
 * @param[in] imu_id IMU 设备编号。
 * @return WHO_AM_I 寄存器值。
 */
uint8 device_imu_read_whoami(device_imu_id_t imu_id);

/**
 * @brief 读取 IMU 状态寄存器值。
 * @param[in] imu_id IMU 设备编号。
 * @return 状态寄存器值。
 */
uint8 device_imu_read_status(device_imu_id_t imu_id);

uint8 device_imu_read_ctrl6(device_imu_id_t imu_id);

/**
 * @brief 读取指定 IMU 的原始温度字节数据。
 * @param[in] imu_id IMU 设备编号。
 * @param[out] buffer 温度数据输出缓冲区，单位：字节。
 * @return void
 */
void device_imu_read_temperature(device_imu_id_t imu_id, uint8* buffer);

/**
 * @brief 读取指定 IMU 的原始加速度计字节数据。
 * @param[in] imu_id IMU 设备编号。
 * @param[out] buffer 加速度计数据输出缓冲区，单位：字节。
 * @return void
 */
void device_imu_read_accelerometer(device_imu_id_t imu_id, uint8* buffer);

/**
 * @brief 读取指定 IMU 的原始陀螺仪字节数据。
 * @param[in] imu_id IMU 设备编号。
 * @param[out] buffer 陀螺仪数据输出缓冲区，单位：字节。
 * @return void
 */
void device_imu_read_gyroscope(device_imu_id_t imu_id, uint8* buffer);

/**
 * @brief 读取指定 IMU 的原始时间戳字节数据。
 * @param[in] imu_id IMU 设备编号。
 * @param[out] buffer 时间戳数据输出缓冲区，单位：字节。
 * @return void
 */
void device_imu_read_timestamp(device_imu_id_t imu_id, uint8* buffer);

boolean device_imu_sflp_game_get(device_imu_id_t imu_id,
                                 float32 quaternion[4],
                                 float32* yaw_rad);

boolean device_imu_sflp_gbias_get(device_imu_id_t imu_id, float32 gbias_dps[3]);

void device_imu_sflp_gbias_register_read(device_imu_id_t imu_id, float32 gbias_dps[3]);

void device_imu_sflp_gbias_register_write(device_imu_id_t imu_id, const float32 gbias_dps[3]);

uint16 device_imu_read_fifo_level(device_imu_id_t imu_id);

void device_imu_read_fifo_word(device_imu_id_t imu_id, uint8* buffer);

boolean device_imu_drain_fifo(device_imu_id_t imu_id);

#endif
