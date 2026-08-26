#include "./device_imu_register.h"

const device_imu_register_init_t device_imu_register_init_default =
{
    .fifo_ctrl1 =
    {
        .B =
        {
            .wtm = 1u,
        },
    },
    .fifo_ctrl2 =
    {
        .B =
        {
            .stop_on_wtm = 0u,
        },
    },
    .fifo_ctrl3 =
    {
        .B =
        {
            .bdr_xl = DEVICE_IMU_FIFO_BATCH_NOT_BATCHED,
            .bdr_gy = DEVICE_IMU_FIFO_BATCH_NOT_BATCHED,
        },
    },
    .fifo_ctrl4 =
    {
        .B =
        {
            .fifo_mode = DEVICE_IMU_FIFO_MODE_STREAM,
            .odr_t_batch = DEVICE_IMU_FIFO_TEMP_NOT_BATCHED,
            .dec_ts_batch = DEVICE_IMU_FIFO_TIMESTAMP_NOT_BATCHED,
        },
    },
    .ctrl1 =
    {
        .B =
        {
            .odr_xl = DEVICE_IMU_CTRL12_ODR_3840HZ,
            .op_mode_xl = DEVICE_IMU_CTRL12_OP_MODE_HIGH_PERFORMANCE,
        },
    },
    .ctrl2 =
    {
        .B =
        {
            .odr_g = DEVICE_IMU_CTRL12_ODR_3840HZ,
            .op_mode_g = DEVICE_IMU_CTRL12_OP_MODE_HIGH_PERFORMANCE,
        },
    },
    .ctrl3 =
    {
        .B =
        {
            .if_inc = 1u,
            .bdu = 1u,
        },
    },
    .ctrl4 =
    {
        .B =
        {
            .drdy_pulsed = 1u,
        },
    },
    .ctrl6 =
    {
        .B =
        {
            .lpf1_g_bw = 2u,
            .fs_g = DEVICE_IMU_CTRL6_FS_G_2000DPS,
        },
    },
    .ctrl7 =
    {
        .B =
        {
            .lpf1_g_en = 1u,
        },
    },
    .ctrl8 =
    {
        .B =
        {
            .hp_lpf2_xl_bw = 3u,
            .fs_xl = DEVICE_IMU_CTRL8_FS_XL_8G,
            .xl_dualc_en = 0u,
        },
    },
    .ctrl9 =
    {
        .B =
        {
            .lpf2_xl_en = 1u,
        },
    },
    .functions_enable =
    {
        .B =
        {
            .timestamp_en = 1u,
            .interrupts_enable = 0u,
        },
    },
    .page_sel =
    {
        .B =
        {
            .page_sel = 0u,
            .page_sel_must_one = 1u,
        },
    },
    .emb_func_en_a =
    {
        .B =
        {
            .sflp_game_en = 1u,
        },
    },
    .emb_func_fifo_en_a =
    {
        .B =
        {
            .sflp_game_fifo_en = 1u,
        },
    },
    .sflp_odr =
    {
        .B =
        {
            .sflp_game_odr = DEVICE_IMU_SFLP_ODR_480HZ,
        },
    },
    .emb_func_init_a =
    {
        .B =
        {
            .sflp_game_init = 1u,
        },
    },
};
