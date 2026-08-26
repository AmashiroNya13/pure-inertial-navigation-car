/**
 * @file vehicle_gyro_cfg.h
 * @brief 车体陀螺仪融合配置类型和常量。
 */

#ifndef MAD_CIRCUITS_VEHICLE_GYRO_CFG_H
#define MAD_CIRCUITS_VEHICLE_GYRO_CFG_H

#include "Ifx_Types.h"
#include "../../../../config/device/device_imu/device_imu_cfg.h"

#define VEHICLE_GYRO_IMU_COUNT  ((uint32)2u) /**< 用于陀螺仪融合的 IMU 数量，单位：个。 */
#define VEHICLE_GYRO_AXIS_COUNT ((uint32)3u) /**< 单个 IMU 的陀螺仪轴数量，单位：轴。 */

typedef enum
{
    VEHICLE_GYRO_AXIS_X = 0, /**< X 轴源数据索引。 */
    VEHICLE_GYRO_AXIS_Y = 1, /**< Y 轴源数据索引。 */
    VEHICLE_GYRO_AXIS_Z = 2, /**< Z 轴源数据索引。 */
} vehicle_gyro_axis_t;

typedef struct
{
    vehicle_gyro_axis_t source_axis; /**< 从 IMU 原始陀螺仪数据中选择的源轴。 */
    sint8 sign;                      /**< 安装方向转换后的轴符号，范围：-1 或 1。 */
} vehicle_gyro_axis_map_t;

typedef struct
{
    device_imu_id_t imu_id;                                      /**< 实际使用的 IMU 设备编号。 */
    vehicle_gyro_axis_map_t accel_map[VEHICLE_GYRO_AXIS_COUNT];  /**< IMU 坐标系到车体坐标系的加速度轴映射。 */
    vehicle_gyro_axis_map_t gyro_map[VEHICLE_GYRO_AXIS_COUNT];   /**< IMU 坐标系到车体坐标系的陀螺仪轴映射。 */
    float32 gyro_bias_dps[VEHICLE_GYRO_AXIS_COUNT];              /**< 车体坐标系陀螺仪固定零偏，单位：度/秒。 */
} vehicle_gyro_imu_cfg_t;

typedef enum
{
    VEHICLE_GYRO_INTERP_LATEST = 0,    /**< 不插值时使用最新一帧。 */
    VEHICLE_GYRO_INTERP_LINEAR = 1,    /**< 使用相邻两帧做线性插值。 */
    VEHICLE_GYRO_INTERP_QUADRATIC = 2, /**< 使用附近三帧做二次插值。 */
} vehicle_gyro_interp_mode_t;

typedef struct
{
    vehicle_gyro_imu_cfg_t imu[VEHICLE_GYRO_IMU_COUNT]; /**< 每个 IMU 的陀螺仪映射配置。 */
    vehicle_gyro_interp_mode_t interp_mode;             /**< 双 IMU 融合的时间对齐模式。 */
    float32 gyro_scale_dps_per_lsb;                     /**< 陀螺仪比例系数，单位：度/秒/LSB。 */
    float32 timestamp_period_s;                         /**< IMU 时间戳计数周期，单位：秒/计数。 */
    float32 gyro_z_iir_alpha;                           /**< Z 轴角速度一阶 IIR 系数，范围：0.0 到 1.0。 */
    float32 theta_iir_alpha;                            /**< yaw 角一阶 IIR 系数，范围：0.0 到 1.0。 */
    float32 imu_dt_min_s;                               /**< 有效 IMU 时间间隔下限，单位：秒。 */
    float32 imu_dt_max_s;                               /**< 有效 IMU 时间间隔上限，单位：秒。 */
} vehicle_gyro_cfg_t;

/**
 * @brief 获取只读的车体陀螺仪融合配置。
 * @param[in] void 无参数。
 * @return 车体陀螺仪融合配置指针。
 */
const vehicle_gyro_cfg_t* vehicle_gyro_cfg_get(void);

#endif
