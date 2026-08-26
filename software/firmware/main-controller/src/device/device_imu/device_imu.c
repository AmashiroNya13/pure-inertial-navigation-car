/**
 * @file device_imu.c
 * @brief IMU 设备对外调用接口实现。
 */

#include "../../../inc/device/device_imu/device_imu.h"
#include "../../../config/device/device_imu/device_imu_register.h"

#include <math.h>

#define DEVICE_IMU_FUNC_CFG_ACCESS_SHUB_MASK ((uint8)0x40u)
#define DEVICE_IMU_FUNC_CFG_ACCESS_EMB_MASK  ((uint8)0x80u)
#define DEVICE_IMU_EMB_FUNC_SFLP_GAME_MASK   ((uint8)0x02u)
#define DEVICE_IMU_EMB_FUNC_SFLP_FIFO_MASK   ((uint8)0x32u)
#define DEVICE_IMU_SFLP_GAME_ODR_MASK        ((uint8)0x38u)
#define DEVICE_IMU_PAGE_SEL_MUST_ONE_MASK    ((uint8)0x80u)
#define DEVICE_IMU_PAGE_RW_READ_MASK         ((uint8)0x20u)
#define DEVICE_IMU_PAGE_RW_WRITE_MASK        ((uint8)0x40u)
#define DEVICE_IMU_SFLP_GAME_GBIAS_PAGE      ((uint8)0x00u)

IFX_INLINE uint8 device_imu_read_register(device_imu_id_t imu_id, device_imu_register_t register_address);
IFX_INLINE void device_imu_read_registers(device_imu_id_t imu_id, device_imu_register_t register_address, uint8* buffer, uint32 length);
IFX_INLINE void device_imu_write_register(device_imu_id_t imu_id, device_imu_register_t register_address, uint8 value);
IFX_INLINE void device_imu_write_registers(device_imu_id_t imu_id, device_imu_register_t register_address, uint8* buffer, uint32 length);
IFX_INLINE void device_imu_register_init(device_imu_id_t imu_id);
IFX_INLINE void device_imu_mem_bank_set(device_imu_id_t imu_id, boolean embedded_function_access);
IFX_INLINE void device_imu_emb_page_set(device_imu_id_t imu_id, uint8 page);
IFX_INLINE uint8 device_imu_emb_page_register_read(device_imu_id_t imu_id,
                                                   uint8 page,
                                                   device_imu_emb_page_register_t register_address);
IFX_INLINE void device_imu_emb_page_register_write(device_imu_id_t imu_id,
                                                   uint8 page,
                                                   device_imu_emb_page_register_t register_address,
                                                   uint8 value);
IFX_INLINE void device_imu_fifo_word_parse(device_imu_runtime_t* imu_runtime,
                                           const uint8* fifo_word,
                                           boolean* gyro_updated);
IFX_INLINE void device_imu_sflp_game_parse(device_imu_runtime_t* imu_runtime,
                                           const uint8* fifo_word);
IFX_INLINE void device_imu_sflp_gbias_parse(device_imu_runtime_t* imu_runtime,
                                            const uint8* fifo_word);
IFX_INLINE uint16 device_imu_read_fifo_level_inner(device_imu_id_t imu_id, boolean count_overrun);
IFX_INLINE boolean device_imu_fifo_tag_is_accel(uint8 tag);
IFX_INLINE boolean device_imu_fifo_tag_is_gyro(uint8 tag);
IFX_INLINE sint16 device_imu_raw_sint16_get(const uint8* buffer);
static float32 device_imu_half_float_get(uint16 raw);
static uint16 device_imu_float_to_half_get(float32 value);

/**
 * @brief 初始化指定 IMU 设备。
 * @param[in] imu_id IMU 设备编号。
 * @return void
 */
void device_imu_init(device_imu_id_t imu_id)
{
    device_imu_cfg_t* imu_cfg = device_imu_cfg_table_get();
    device_imu_runtime_t* imu_runtime = device_imu_runtime_table_get();

    driver_gtm_atom_timer_init(&imu_cfg[imu_id].gtm_atom_timer_cfg,
                               &imu_runtime[imu_id].gtm_atom_timer_runtime);
    driver_qspi_init(&imu_cfg[imu_id].qspi_cfg, &imu_runtime[imu_id].qspi_runtime);
    imu_runtime[imu_id].imu_sflp_game_quaternion[0] = 1.0f;
    imu_runtime[imu_id].imu_sflp_game_quaternion[1] = 0.0f;
    imu_runtime[imu_id].imu_sflp_game_quaternion[2] = 0.0f;
    imu_runtime[imu_id].imu_sflp_game_quaternion[3] = 0.0f;
    imu_runtime[imu_id].imu_sflp_game_yaw_rad = 0.0f;
    imu_runtime[imu_id].imu_sflp_game_valid = FALSE;
    imu_runtime[imu_id].imu_sflp_gbias_dps[0] = 0.0f;
    imu_runtime[imu_id].imu_sflp_gbias_dps[1] = 0.0f;
    imu_runtime[imu_id].imu_sflp_gbias_dps[2] = 0.0f;
    imu_runtime[imu_id].imu_sflp_gbias_valid = FALSE;
    device_imu_register_init(imu_id);
    driver_gtm_atom_timer_start(&imu_runtime[imu_id].gtm_atom_timer_runtime);
}

/**
 * @brief 初始化全部已配置的 IMU 设备。
 * @param[in] void 无参数。
 * @return void
 */
void device_imu_init_all(void)
{
    device_imu_init(DEVICE_IMU_1);
    device_imu_init(DEVICE_IMU_2);
}

/**
 * @brief 注册指定 IMU 采样传输完成后的回调函数。
 * @param[in] imu_id IMU 设备编号。
 * @param[in] device_imu_callback 回调函数指针。
 * @return void
 */
void device_imu_register_callback(device_imu_id_t imu_id, void (*device_imu_callback) (void))
{
    device_imu_runtime_t* imu_runtime = device_imu_runtime_table_get();
    imu_runtime[imu_id].device_imu_callback = device_imu_callback;
}

/**
 * @brief 读取 IMU 的 WHO_AM_I 寄存器值。
 * @param[in] imu_id IMU 设备编号。
 * @return WHO_AM_I 寄存器值。
 */
uint8 device_imu_read_whoami(device_imu_id_t imu_id)
{
    return device_imu_read_register(imu_id, DEVICE_IMU_REGISTER_WHO_AM_I);
}

/**
 * @brief 读取 IMU 状态寄存器值。
 * @param[in] imu_id IMU 设备编号。
 * @return 状态寄存器值。
 */
uint8 device_imu_read_status(device_imu_id_t imu_id)
{
    return device_imu_read_register(imu_id, DEVICE_IMU_REGISTER_STATUS_REG);
}

uint8 device_imu_read_ctrl6(device_imu_id_t imu_id)
{
    return device_imu_read_register(imu_id, DEVICE_IMU_REGISTER_CTRL6);
}

/**
 * @brief 读取指定 IMU 的原始温度字节数据。
 * @param[in] imu_id IMU 设备编号。
 * @param[out] buffer 温度数据输出缓冲区，单位：字节。
 * @return void
 */
void device_imu_read_temperature(device_imu_id_t imu_id, uint8* buffer)
{
    device_imu_read_registers(imu_id, DEVICE_IMU_REGISTER_OUT_TEMP_L, buffer, DEVICE_IMU_TEMPERATURE_DATA_LENGTH);
}

/**
 * @brief 读取指定 IMU 的原始加速度计字节数据。
 * @param[in] imu_id IMU 设备编号。
 * @param[out] buffer 加速度计数据输出缓冲区，单位：字节。
 * @return void
 */
void device_imu_read_accelerometer(device_imu_id_t imu_id, uint8* buffer)
{
    device_imu_read_registers(imu_id, DEVICE_IMU_REGISTER_OUTX_L_A, buffer, DEVICE_IMU_ACCELEROMETER_DATA_LENGTH);
}

/**
 * @brief 读取指定 IMU 的原始陀螺仪字节数据。
 * @param[in] imu_id IMU 设备编号。
 * @param[out] buffer 陀螺仪数据输出缓冲区，单位：字节。
 * @return void
 */
void device_imu_read_gyroscope(device_imu_id_t imu_id, uint8* buffer)
{
    device_imu_read_registers(imu_id, DEVICE_IMU_REGISTER_OUTX_L_G, buffer, DEVICE_IMU_GYROSCOPE_DATA_LENGTH);
}

/**
 * @brief 读取指定 IMU 的原始时间戳字节数据。
 * @param[in] imu_id IMU 设备编号。
 * @param[out] buffer 时间戳数据输出缓冲区，单位：字节。
 * @return void
 */
void device_imu_read_timestamp(device_imu_id_t imu_id, uint8* buffer)
{
    device_imu_read_registers(imu_id, DEVICE_IMU_REGISTER_TIMESTAMP, buffer, DEVICE_IMU_TIMESTAMP_DATA_LENGTH);
}

boolean device_imu_sflp_game_get(device_imu_id_t imu_id,
                                 float32 quaternion[4],
                                 float32* yaw_rad)
{
    device_imu_runtime_t* imu_runtime = device_imu_runtime_table_get();
    uint32 index;

    if (imu_runtime[imu_id].imu_sflp_game_valid == FALSE)
    {
        return FALSE;
    }

    if (quaternion != NULL_PTR)
    {
        for (index = 0u; index < 4u; index++)
        {
            quaternion[index] = imu_runtime[imu_id].imu_sflp_game_quaternion[index];
        }
    }

    if (yaw_rad != NULL_PTR)
    {
        *yaw_rad = imu_runtime[imu_id].imu_sflp_game_yaw_rad;
    }

    return TRUE;
}

boolean device_imu_sflp_gbias_get(device_imu_id_t imu_id, float32 gbias_dps[3])
{
    device_imu_runtime_t* imu_runtime = device_imu_runtime_table_get();
    uint32 index;

    if ((imu_id >= DEVICE_IMU_COUNT)
        || (gbias_dps == NULL_PTR)
        || (imu_runtime[imu_id].imu_sflp_gbias_valid == FALSE))
    {
        return FALSE;
    }

    for (index = 0u; index < 3u; index++)
    {
        gbias_dps[index] = imu_runtime[imu_id].imu_sflp_gbias_dps[index];
    }

    return TRUE;
}

void device_imu_sflp_gbias_register_read(device_imu_id_t imu_id, float32 gbias_dps[3])
{
    uint8 buffer[6];

    if ((imu_id >= DEVICE_IMU_COUNT) || (gbias_dps == NULL_PTR))
    {
        return;
    }

    device_imu_mem_bank_set(imu_id, TRUE);
    buffer[0] = device_imu_emb_page_register_read(imu_id,
                                                  DEVICE_IMU_SFLP_GAME_GBIAS_PAGE,
                                                  DEVICE_IMU_EMB_PAGE_REGISTER_SFLP_GAME_GBIASX_L);
    buffer[1] = device_imu_emb_page_register_read(imu_id,
                                                  DEVICE_IMU_SFLP_GAME_GBIAS_PAGE,
                                                  DEVICE_IMU_EMB_PAGE_REGISTER_SFLP_GAME_GBIASX_H);
    buffer[2] = device_imu_emb_page_register_read(imu_id,
                                                  DEVICE_IMU_SFLP_GAME_GBIAS_PAGE,
                                                  DEVICE_IMU_EMB_PAGE_REGISTER_SFLP_GAME_GBIASY_L);
    buffer[3] = device_imu_emb_page_register_read(imu_id,
                                                  DEVICE_IMU_SFLP_GAME_GBIAS_PAGE,
                                                  DEVICE_IMU_EMB_PAGE_REGISTER_SFLP_GAME_GBIASY_H);
    buffer[4] = device_imu_emb_page_register_read(imu_id,
                                                  DEVICE_IMU_SFLP_GAME_GBIAS_PAGE,
                                                  DEVICE_IMU_EMB_PAGE_REGISTER_SFLP_GAME_GBIASZ_L);
    buffer[5] = device_imu_emb_page_register_read(imu_id,
                                                  DEVICE_IMU_SFLP_GAME_GBIAS_PAGE,
                                                  DEVICE_IMU_EMB_PAGE_REGISTER_SFLP_GAME_GBIASZ_H);
    device_imu_mem_bank_set(imu_id, FALSE);

    gbias_dps[0] = device_imu_half_float_get((uint16)device_imu_raw_sint16_get(&buffer[0]));
    gbias_dps[1] = device_imu_half_float_get((uint16)device_imu_raw_sint16_get(&buffer[2]));
    gbias_dps[2] = device_imu_half_float_get((uint16)device_imu_raw_sint16_get(&buffer[4]));
}

void device_imu_sflp_gbias_register_write(device_imu_id_t imu_id, const float32 gbias_dps[3])
{
    uint8 buffer[6];
    uint16 raw_x;
    uint16 raw_y;
    uint16 raw_z;

    if ((imu_id >= DEVICE_IMU_COUNT) || (gbias_dps == NULL_PTR))
    {
        return;
    }

    raw_x = device_imu_float_to_half_get(gbias_dps[0]);
    raw_y = device_imu_float_to_half_get(gbias_dps[1]);
    raw_z = device_imu_float_to_half_get(gbias_dps[2]);

    buffer[0] = (uint8)(raw_x & 0x00FFu);
    buffer[1] = (uint8)((raw_x >> 8u) & 0x00FFu);
    buffer[2] = (uint8)(raw_y & 0x00FFu);
    buffer[3] = (uint8)((raw_y >> 8u) & 0x00FFu);
    buffer[4] = (uint8)(raw_z & 0x00FFu);
    buffer[5] = (uint8)((raw_z >> 8u) & 0x00FFu);

    device_imu_mem_bank_set(imu_id, TRUE);
    device_imu_emb_page_register_write(imu_id,
                                       DEVICE_IMU_SFLP_GAME_GBIAS_PAGE,
                                       DEVICE_IMU_EMB_PAGE_REGISTER_SFLP_GAME_GBIASX_L,
                                       buffer[0]);
    device_imu_emb_page_register_write(imu_id,
                                       DEVICE_IMU_SFLP_GAME_GBIAS_PAGE,
                                       DEVICE_IMU_EMB_PAGE_REGISTER_SFLP_GAME_GBIASX_H,
                                       buffer[1]);
    device_imu_emb_page_register_write(imu_id,
                                       DEVICE_IMU_SFLP_GAME_GBIAS_PAGE,
                                       DEVICE_IMU_EMB_PAGE_REGISTER_SFLP_GAME_GBIASY_L,
                                       buffer[2]);
    device_imu_emb_page_register_write(imu_id,
                                       DEVICE_IMU_SFLP_GAME_GBIAS_PAGE,
                                       DEVICE_IMU_EMB_PAGE_REGISTER_SFLP_GAME_GBIASY_H,
                                       buffer[3]);
    device_imu_emb_page_register_write(imu_id,
                                       DEVICE_IMU_SFLP_GAME_GBIAS_PAGE,
                                       DEVICE_IMU_EMB_PAGE_REGISTER_SFLP_GAME_GBIASZ_L,
                                       buffer[4]);
    device_imu_emb_page_register_write(imu_id,
                                       DEVICE_IMU_SFLP_GAME_GBIAS_PAGE,
                                       DEVICE_IMU_EMB_PAGE_REGISTER_SFLP_GAME_GBIASZ_H,
                                       buffer[5]);
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_EMB_FUNC_INIT_A,
                              device_imu_read_register(imu_id, DEVICE_IMU_REGISTER_EMB_FUNC_INIT_A)
                              | DEVICE_IMU_EMB_FUNC_SFLP_GAME_MASK);
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_EMB_FUNC_INIT_A,
                              device_imu_read_register(imu_id, DEVICE_IMU_REGISTER_EMB_FUNC_INIT_A)
                              & (uint8)~DEVICE_IMU_EMB_FUNC_SFLP_GAME_MASK);
    device_imu_mem_bank_set(imu_id, FALSE);
}

uint16 device_imu_read_fifo_level(device_imu_id_t imu_id)
{
    return device_imu_read_fifo_level_inner(imu_id, TRUE);
}

IFX_INLINE uint16 device_imu_read_fifo_level_inner(device_imu_id_t imu_id, boolean count_overrun)
{
    uint8 status[DEVICE_IMU_FIFO_STATUS_DATA_LENGTH];
    uint16 fifo_level;
    device_imu_runtime_t* imu_runtime = device_imu_runtime_table_get();

    device_imu_read_registers(imu_id, DEVICE_IMU_REGISTER_FIFO_STATUS1, status, DEVICE_IMU_FIFO_STATUS_DATA_LENGTH);
    fifo_level = (uint16)(((uint16)(status[1] & 0x01u) << 8u) | (uint16)status[0]);
    imu_runtime[imu_id].imu_fifo_level = fifo_level;
    if (((status[1] & 0x40u) != 0u) && (count_overrun == TRUE))
    {
        imu_runtime[imu_id].imu_fifo_overrun_count++;
    }
    return fifo_level;
}

void device_imu_read_fifo_word(device_imu_id_t imu_id, uint8* buffer)
{
    device_imu_read_registers(imu_id,
                              DEVICE_IMU_REGISTER_FIFO_DATA_OUT_TAG,
                              buffer,
                              DEVICE_IMU_FIFO_DATA_WORD_LENGTH);
}

boolean device_imu_drain_fifo(device_imu_id_t imu_id)
{
    device_imu_runtime_t* imu_runtime = device_imu_runtime_table_get();
    uint16 fifo_level = device_imu_read_fifo_level(imu_id);
    uint16 index;
    boolean gyro_updated = FALSE;

    imu_runtime[imu_id].imu_fifo_gyro_sample_count = 0u;
    imu_runtime[imu_id].imu_fifo_timestamp_sample_count = 0u;
    imu_runtime[imu_id].imu_fifo_sflp_game_sample_count = 0u;
    imu_runtime[imu_id].imu_fifo_sflp_gbias_sample_count = 0u;
    imu_runtime[imu_id].imu_fifo_level_before_drain = fifo_level;
    if (fifo_level > DEVICE_IMU_FIFO_DRAIN_MAX_WORDS)
    {
        fifo_level = DEVICE_IMU_FIFO_DRAIN_MAX_WORDS;
    }
    imu_runtime[imu_id].imu_fifo_last_drain_words = fifo_level;

    for (index = 0u; index < fifo_level; index++)
    {
        device_imu_read_fifo_word(imu_id, imu_runtime[imu_id].imu_fifo_word_buffer);
        device_imu_fifo_word_parse(&imu_runtime[imu_id],
                                   imu_runtime[imu_id].imu_fifo_word_buffer,
                                   &gyro_updated);
        imu_runtime[imu_id].imu_fifo_drain_count++;
    }

    imu_runtime[imu_id].imu_fifo_level_after_drain =
        device_imu_read_fifo_level_inner(imu_id, FALSE);
    device_imu_read_accelerometer(imu_id, imu_runtime[imu_id].imu_accelerometer_buffer);
    device_imu_read_gyroscope(imu_id, imu_runtime[imu_id].imu_gyroscope_buffer);
    imu_runtime[imu_id].imu_raw_update_count++;
    gyro_updated = TRUE;

    return gyro_updated;
}

IFX_INLINE uint8 device_imu_read_register(device_imu_id_t imu_id, device_imu_register_t register_address)
{
    device_imu_runtime_t* imu_runtime = device_imu_runtime_table_get();
    return driver_qspi_read_8bit_register(&imu_runtime[imu_id].qspi_runtime,
                                          DEVICE_IMU_REGISTER_READ_MASK | register_address);
}

IFX_INLINE void device_imu_read_registers(device_imu_id_t imu_id, device_imu_register_t register_address, uint8* buffer, uint32 length)
{
    device_imu_runtime_t* imu_runtime = device_imu_runtime_table_get();
    driver_qspi_read_8bit_registers(&imu_runtime[imu_id].qspi_runtime,
                                    DEVICE_IMU_REGISTER_READ_MASK | register_address,
                                    buffer,
                                    length);
}

IFX_INLINE void device_imu_write_register(device_imu_id_t imu_id, device_imu_register_t register_address, uint8 value)
{
    device_imu_runtime_t* imu_runtime = device_imu_runtime_table_get();
    driver_qspi_write_8bit_register(&imu_runtime[imu_id].qspi_runtime,
                                    DEVICE_IMU_REGISTER_WRITE_MASK & register_address,
                                    value);
}

IFX_INLINE void device_imu_register_init(device_imu_id_t imu_id)
{
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_FIFO_CTRL4, DEVICE_IMU_FIFO_MODE_BYPASS);
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_CTRL3, device_imu_register_init_default.ctrl3.U);
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_CTRL4, device_imu_register_init_default.ctrl4.U);
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_CTRL1, device_imu_register_init_default.ctrl1.U);
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_CTRL2, device_imu_register_init_default.ctrl2.U);
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_CTRL6, device_imu_register_init_default.ctrl6.U);
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_CTRL7, device_imu_register_init_default.ctrl7.U);
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_CTRL8, device_imu_register_init_default.ctrl8.U);
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_CTRL9, device_imu_register_init_default.ctrl9.U);
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_FUNCTIONS_ENABLE, device_imu_register_init_default.functions_enable.U);
    device_imu_mem_bank_set(imu_id, TRUE);
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_SFLP_ODR,
                              (device_imu_read_register(imu_id, DEVICE_IMU_REGISTER_SFLP_ODR) & (uint8)~DEVICE_IMU_SFLP_GAME_ODR_MASK)
                              | (uint8)(DEVICE_IMU_SFLP_ODR_480HZ << 3u));
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_EMB_FUNC_FIFO_EN_A,
                              device_imu_read_register(imu_id, DEVICE_IMU_REGISTER_EMB_FUNC_FIFO_EN_A)
                              | DEVICE_IMU_EMB_FUNC_SFLP_FIFO_MASK);
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_EMB_FUNC_EN_A,
                              device_imu_read_register(imu_id, DEVICE_IMU_REGISTER_EMB_FUNC_EN_A)
                              | DEVICE_IMU_EMB_FUNC_SFLP_GAME_MASK);
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_EMB_FUNC_INIT_A,
                              device_imu_read_register(imu_id, DEVICE_IMU_REGISTER_EMB_FUNC_INIT_A)
                              | DEVICE_IMU_EMB_FUNC_SFLP_GAME_MASK);
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_EMB_FUNC_INIT_A,
                              device_imu_read_register(imu_id, DEVICE_IMU_REGISTER_EMB_FUNC_INIT_A)
                              & (uint8)~DEVICE_IMU_EMB_FUNC_SFLP_GAME_MASK);
    device_imu_mem_bank_set(imu_id, FALSE);
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_FIFO_CTRL1, device_imu_register_init_default.fifo_ctrl1.U);
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_FIFO_CTRL2, device_imu_register_init_default.fifo_ctrl2.U);
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_FIFO_CTRL3, device_imu_register_init_default.fifo_ctrl3.U);
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_FIFO_CTRL4, device_imu_register_init_default.fifo_ctrl4.U);
}

IFX_INLINE void device_imu_mem_bank_set(device_imu_id_t imu_id, boolean embedded_function_access)
{
    uint8 func_cfg_access;

    func_cfg_access = device_imu_read_register(imu_id, DEVICE_IMU_REGISTER_FUNC_CFG_ACCESS);
    func_cfg_access &= (uint8)~(DEVICE_IMU_FUNC_CFG_ACCESS_SHUB_MASK | DEVICE_IMU_FUNC_CFG_ACCESS_EMB_MASK);
    if (embedded_function_access != FALSE)
    {
        func_cfg_access |= DEVICE_IMU_FUNC_CFG_ACCESS_EMB_MASK;
    }
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_FUNC_CFG_ACCESS, func_cfg_access);
}

IFX_INLINE void device_imu_emb_page_set(device_imu_id_t imu_id, uint8 page)
{
    device_imu_write_register(imu_id,
                              DEVICE_IMU_REGISTER_PAGE_SEL,
                              DEVICE_IMU_PAGE_SEL_MUST_ONE_MASK | (page & 0x0Fu));
}

IFX_INLINE uint8 device_imu_emb_page_register_read(device_imu_id_t imu_id,
                                                   uint8 page,
                                                   device_imu_emb_page_register_t register_address)
{
    uint8 value;

    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_PAGE_RW, DEVICE_IMU_PAGE_RW_READ_MASK);
    device_imu_emb_page_set(imu_id, page);
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_PAGE_ADDRESS, (uint8)register_address);
    value = device_imu_read_register(imu_id, DEVICE_IMU_REGISTER_PAGE_VALUE);
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_PAGE_RW, 0u);

    return value;
}

IFX_INLINE void device_imu_emb_page_register_write(device_imu_id_t imu_id,
                                                   uint8 page,
                                                   device_imu_emb_page_register_t register_address,
                                                   uint8 value)
{
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_PAGE_RW, DEVICE_IMU_PAGE_RW_WRITE_MASK);
    device_imu_emb_page_set(imu_id, page);
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_PAGE_ADDRESS, (uint8)register_address);
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_PAGE_VALUE, value);
    device_imu_write_register(imu_id, DEVICE_IMU_REGISTER_PAGE_RW, 0u);
}

IFX_INLINE void device_imu_write_registers(device_imu_id_t imu_id, device_imu_register_t register_address, uint8* buffer, uint32 length)
{
    device_imu_runtime_t* imu_runtime = device_imu_runtime_table_get();
    driver_qspi_write_8bit_registers(&imu_runtime[imu_id].qspi_runtime,
                                     DEVICE_IMU_REGISTER_WRITE_MASK & register_address,
                                     buffer,
                                     length);
}

IFX_INLINE void device_imu_fifo_word_parse(device_imu_runtime_t* imu_runtime,
                                           const uint8* fifo_word,
                                           boolean* gyro_updated)
{
    uint8 tag = (fifo_word[0] >> 3u) & 0x1Fu;

    imu_runtime->imu_fifo_last_tag_raw = fifo_word[0];
    imu_runtime->imu_fifo_last_tag = tag;
    if (device_imu_fifo_tag_is_gyro(tag) == TRUE)
    {
        imu_runtime->imu_gyroscope_buffer[0] = fifo_word[1];
        imu_runtime->imu_gyroscope_buffer[1] = fifo_word[2];
        imu_runtime->imu_gyroscope_buffer[2] = fifo_word[3];
        imu_runtime->imu_gyroscope_buffer[3] = fifo_word[4];
        imu_runtime->imu_gyroscope_buffer[4] = fifo_word[5];
        imu_runtime->imu_gyroscope_buffer[5] = fifo_word[6];
        imu_runtime->imu_fifo_gyro_sample_count++;
        *gyro_updated = TRUE;
    }
    else if (device_imu_fifo_tag_is_accel(tag) == TRUE)
    {
        imu_runtime->imu_accelerometer_buffer[0] = fifo_word[1];
        imu_runtime->imu_accelerometer_buffer[1] = fifo_word[2];
        imu_runtime->imu_accelerometer_buffer[2] = fifo_word[3];
        imu_runtime->imu_accelerometer_buffer[3] = fifo_word[4];
        imu_runtime->imu_accelerometer_buffer[4] = fifo_word[5];
        imu_runtime->imu_accelerometer_buffer[5] = fifo_word[6];
    }
    else if (tag == DEVICE_IMU_FIFO_TAG_TEMPERATURE)
    {
        imu_runtime->imu_temperature_buffer[0] = fifo_word[1];
        imu_runtime->imu_temperature_buffer[1] = fifo_word[2];
    }
    else if (tag == DEVICE_IMU_FIFO_TAG_TIMESTAMP)
    {
        imu_runtime->imu_timestamp_buffer[0] = fifo_word[1];
        imu_runtime->imu_timestamp_buffer[1] = fifo_word[2];
        imu_runtime->imu_timestamp_buffer[2] = fifo_word[3];
        imu_runtime->imu_timestamp_buffer[3] = fifo_word[4];
        imu_runtime->imu_fifo_timestamp_sample_count++;
    }
    else if (tag == DEVICE_IMU_FIFO_TAG_SFLP_GAME)
    {
        device_imu_sflp_game_parse(imu_runtime, fifo_word);
        imu_runtime->imu_fifo_sflp_game_sample_count++;
        imu_runtime->imu_sflp_game_update_count++;
        *gyro_updated = TRUE;
    }
    else if (tag == DEVICE_IMU_FIFO_TAG_SFLP_GBIAS)
    {
        device_imu_sflp_gbias_parse(imu_runtime, fifo_word);
        imu_runtime->imu_fifo_sflp_gbias_sample_count++;
    }
    else
    {
        /* Unsupported FIFO tags are ignored by this raw IMU path. */
    }
}

IFX_INLINE void device_imu_sflp_game_parse(device_imu_runtime_t* imu_runtime,
                                           const uint8* fifo_word)
{
    float32 qx;
    float32 qy;
    float32 qz;
    float32 qw;
    float32 sum_square;
    float32 norm;
    float32 siny_cosp;
    float32 cosy_cosp;

    imu_runtime->imu_sflp_game_buffer[0] = fifo_word[1];
    imu_runtime->imu_sflp_game_buffer[1] = fifo_word[2];
    imu_runtime->imu_sflp_game_buffer[2] = fifo_word[3];
    imu_runtime->imu_sflp_game_buffer[3] = fifo_word[4];
    imu_runtime->imu_sflp_game_buffer[4] = fifo_word[5];
    imu_runtime->imu_sflp_game_buffer[5] = fifo_word[6];

    qx = device_imu_half_float_get((uint16)device_imu_raw_sint16_get(&fifo_word[1]));
    qy = device_imu_half_float_get((uint16)device_imu_raw_sint16_get(&fifo_word[3]));
    qz = device_imu_half_float_get((uint16)device_imu_raw_sint16_get(&fifo_word[5]));

    sum_square = (qx * qx) + (qy * qy) + (qz * qz);
    if (sum_square > 1.0f)
    {
        norm = sqrtf(sum_square);
        if (norm > 0.0f)
        {
            qx /= norm;
            qy /= norm;
            qz /= norm;
        }
        sum_square = 1.0f;
    }

    if (sum_square < 1.0f)
    {
        qw = sqrtf(1.0f - sum_square);
    }
    else
    {
        qw = 0.0f;
    }
    imu_runtime->imu_sflp_game_quaternion[0] = qw;
    imu_runtime->imu_sflp_game_quaternion[1] = qx;
    imu_runtime->imu_sflp_game_quaternion[2] = qy;
    imu_runtime->imu_sflp_game_quaternion[3] = qz;

    siny_cosp = 2.0f * ((qw * qz) + (qx * qy));
    cosy_cosp = 1.0f - (2.0f * ((qy * qy) + (qz * qz)));
    imu_runtime->imu_sflp_game_yaw_rad = atan2f(siny_cosp, cosy_cosp);
    imu_runtime->imu_sflp_game_valid = TRUE;
}

IFX_INLINE void device_imu_sflp_gbias_parse(device_imu_runtime_t* imu_runtime,
                                            const uint8* fifo_word)
{
    imu_runtime->imu_sflp_gbias_buffer[0] = fifo_word[1];
    imu_runtime->imu_sflp_gbias_buffer[1] = fifo_word[2];
    imu_runtime->imu_sflp_gbias_buffer[2] = fifo_word[3];
    imu_runtime->imu_sflp_gbias_buffer[3] = fifo_word[4];
    imu_runtime->imu_sflp_gbias_buffer[4] = fifo_word[5];
    imu_runtime->imu_sflp_gbias_buffer[5] = fifo_word[6];

    imu_runtime->imu_sflp_gbias_dps[0] =
        device_imu_half_float_get((uint16)device_imu_raw_sint16_get(&fifo_word[1]));
    imu_runtime->imu_sflp_gbias_dps[1] =
        device_imu_half_float_get((uint16)device_imu_raw_sint16_get(&fifo_word[3]));
    imu_runtime->imu_sflp_gbias_dps[2] =
        device_imu_half_float_get((uint16)device_imu_raw_sint16_get(&fifo_word[5]));
    imu_runtime->imu_sflp_gbias_valid = TRUE;
}

IFX_INLINE boolean device_imu_fifo_tag_is_accel(uint8 tag)
{
    return ((tag == DEVICE_IMU_FIFO_TAG_ACCEL)
            || (tag == DEVICE_IMU_FIFO_TAG_ACCEL_T2)
            || (tag == DEVICE_IMU_FIFO_TAG_ACCEL_T1)
            || (tag == DEVICE_IMU_FIFO_TAG_ACCEL_2XC)
            || (tag == DEVICE_IMU_FIFO_TAG_ACCEL_3XC)) ? TRUE : FALSE;
}

IFX_INLINE boolean device_imu_fifo_tag_is_gyro(uint8 tag)
{
    return ((tag == DEVICE_IMU_FIFO_TAG_GYRO)
            || (tag == DEVICE_IMU_FIFO_TAG_GYRO_T2)
            || (tag == DEVICE_IMU_FIFO_TAG_GYRO_T1)
            || (tag == DEVICE_IMU_FIFO_TAG_GYRO_2XC)
            || (tag == DEVICE_IMU_FIFO_TAG_GYRO_3XC)) ? TRUE : FALSE;
}

IFX_INLINE sint16 device_imu_raw_sint16_get(const uint8* buffer)
{
    return (sint16)((uint16)buffer[0] | ((uint16)buffer[1] << 8u));
}

static float32 device_imu_half_float_get(uint16 raw)
{
    union
    {
        uint32 u32;
        float32 f32;
    } value;
    uint32 sign = ((uint32)raw & 0x8000u) << 16u;
    uint32 exponent = ((uint32)raw >> 10u) & 0x1Fu;
    uint32 fraction = (uint32)raw & 0x03FFu;

    if (exponent == 0u)
    {
        if (fraction == 0u)
        {
            value.u32 = sign;
        }
        else
        {
            sint32 exponent_signed = -14;

            while ((fraction & 0x0400u) == 0u)
            {
                fraction <<= 1u;
                exponent_signed--;
            }
            fraction &= 0x03FFu;
            value.u32 = sign
                      | ((uint32)(exponent_signed + 127) << 23u)
                      | (fraction << 13u);
        }
    }
    else if (exponent == 0x1Fu)
    {
        value.u32 = sign | 0x7F800000u | (fraction << 13u);
    }
    else
    {
        value.u32 = sign
                  | ((exponent + 112u) << 23u)
                  | (fraction << 13u);
    }

    return value.f32;
}

static uint16 device_imu_float_to_half_get(float32 value)
{
    union
    {
        float32 f32;
        uint32 u32;
    } input;
    uint32 sign;
    sint32 exponent;
    uint32 fraction;
    uint16 result;

    input.f32 = value;
    sign = (input.u32 >> 16u) & 0x8000u;
    exponent = (sint32)((input.u32 >> 23u) & 0xFFu) - 127;
    fraction = input.u32 & 0x007FFFFFu;

    if (exponent > 15)
    {
        result = (uint16)(sign | 0x7C00u);
    }
    else if (exponent >= -14)
    {
        uint32 rounded_fraction = fraction + 0x00001000u;

        if ((rounded_fraction & 0x00800000u) != 0u)
        {
            rounded_fraction = 0u;
            exponent++;
            if (exponent > 15)
            {
                return (uint16)(sign | 0x7C00u);
            }
        }

        result = (uint16)(sign
                          | ((uint32)(exponent + 15) << 10u)
                          | (rounded_fraction >> 13u));
    }
    else if (exponent >= -24)
    {
        uint32 mantissa = fraction | 0x00800000u;
        uint32 shift = (uint32)(-exponent - 14);
        uint32 rounded = mantissa + (0x00001000u << shift);

        result = (uint16)(sign | (rounded >> (13u + shift)));
    }
    else
    {
        result = (uint16)sign;
    }

    return result;
}

