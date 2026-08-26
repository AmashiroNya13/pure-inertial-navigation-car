#ifndef MAD_CIRCUITS_DEVICE_IMU_CFG_H
#define MAD_CIRCUITS_DEVICE_IMU_CFG_H

#include "../../../inc/driver/driver_qspi/driver_qspi.h"
#include "../../../inc/driver/driver_dma/driver_dma.h"
#include "../../../inc/driver/driver_gtm/driver_gtm_atom_timer.h"

typedef enum
{
    DEVICE_IMU_1 = 0,
    DEVICE_IMU_2 = 1,
    DEVICE_IMU_COUNT = 2,
} device_imu_id_t;

typedef struct
{
    device_imu_id_t imu_id;
    driver_gtm_atom_timer_cfg_t gtm_atom_timer_cfg;
    driver_qspi_cfg_t qspi_cfg;
} device_imu_cfg_t;

typedef struct
{
    device_imu_id_t imu_id;
    driver_gtm_atom_timer_runtime_t gtm_atom_timer_runtime;
    driver_qspi_runtime_t qspi_runtime;
    uint8 imu_temperature_buffer[2];
    uint8 imu_accelerometer_buffer[6];
    uint8 imu_gyroscope_buffer[6];
    uint8 imu_timestamp_buffer[4];
    uint8 imu_sflp_game_buffer[6];
    uint8 imu_sflp_gbias_buffer[6];
    float32 imu_sflp_game_quaternion[4];
    float32 imu_sflp_game_yaw_rad;
    boolean imu_sflp_game_valid;
    float32 imu_sflp_gbias_dps[3];
    boolean imu_sflp_gbias_valid;
    uint8 imu_fifo_word_buffer[7];
    uint16 imu_fifo_level;
    uint16 imu_fifo_level_before_drain;
    uint16 imu_fifo_level_after_drain;
    uint16 imu_fifo_last_drain_words;
    uint16 imu_fifo_drain_count;
    uint16 imu_fifo_overrun_count;
    uint16 imu_fifo_gyro_sample_count;
    uint16 imu_fifo_timestamp_sample_count;
    uint16 imu_fifo_sflp_game_sample_count;
    uint16 imu_fifo_sflp_gbias_sample_count;
    uint32 imu_raw_update_count;
    uint32 imu_sflp_game_update_count;
    uint8 imu_fifo_last_tag;
    uint8 imu_fifo_last_tag_raw;
    void (*device_imu_callback) (void);
} device_imu_runtime_t;

device_imu_cfg_t* device_imu_cfg_table_get(void);
device_imu_runtime_t* device_imu_runtime_table_get(void);

#endif
