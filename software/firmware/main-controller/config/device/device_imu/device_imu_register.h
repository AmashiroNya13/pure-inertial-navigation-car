#ifndef MAD_CIRCUITS_DEVICE_IMU_REGISTER_H
#define MAD_CIRCUITS_DEVICE_IMU_REGISTER_H

#include "Ifx_Types.h"

#define DEVICE_IMU_REGISTER_READ_MASK  ((uint8)0x80u)
#define DEVICE_IMU_REGISTER_WRITE_MASK ((uint8)0x7Fu)

#define DEVICE_IMU_WHO_AM_I_VALUE            ((uint8)0x70u)
#define DEVICE_IMU_TEMPERATURE_DATA_LENGTH   ((uint32)2u)
#define DEVICE_IMU_ACCELEROMETER_DATA_LENGTH ((uint32)6u)
#define DEVICE_IMU_GYROSCOPE_DATA_LENGTH     ((uint32)6u)
#define DEVICE_IMU_TIMESTAMP_DATA_LENGTH     ((uint32)4u)
#define DEVICE_IMU_FIFO_STATUS_DATA_LENGTH   ((uint32)2u)
#define DEVICE_IMU_FIFO_DATA_WORD_LENGTH     ((uint32)7u)
#define DEVICE_IMU_FIFO_DRAIN_MAX_WORDS      ((uint16)64u)
#define DEVICE_IMU_SFLP_GAME_DATA_LENGTH     ((uint32)6u)

typedef enum
{
    DEVICE_IMU_REGISTER_FUNC_CFG_ACCESS   = 0x01u,
    DEVICE_IMU_REGISTER_PIN_CTRL          = 0x02u,
    DEVICE_IMU_REGISTER_IF_CFG            = 0x03u,
    DEVICE_IMU_REGISTER_ODR_TRIG_CFG      = 0x06u,
    DEVICE_IMU_REGISTER_FIFO_CTRL1        = 0x07u,
    DEVICE_IMU_REGISTER_FIFO_CTRL2        = 0x08u,
    DEVICE_IMU_REGISTER_FIFO_CTRL3        = 0x09u,
    DEVICE_IMU_REGISTER_FIFO_CTRL4        = 0x0Au,
    DEVICE_IMU_REGISTER_COUNTER_BDR_REG1  = 0x0Bu,
    DEVICE_IMU_REGISTER_COUNTER_BDR_REG2  = 0x0Cu,
    DEVICE_IMU_REGISTER_INT1_CTRL         = 0x0Du,
    DEVICE_IMU_REGISTER_INT2_CTRL         = 0x0Eu,
    DEVICE_IMU_REGISTER_WHO_AM_I          = 0x0Fu,
    DEVICE_IMU_REGISTER_PAGE_SEL          = 0x02u,
    DEVICE_IMU_REGISTER_EMB_FUNC_EN_A     = 0x04u,
    DEVICE_IMU_REGISTER_PAGE_ADDRESS      = 0x08u,
    DEVICE_IMU_REGISTER_PAGE_VALUE        = 0x09u,
    DEVICE_IMU_REGISTER_EMB_FUNC_FIFO_EN_A = 0x44u,
    DEVICE_IMU_REGISTER_SFLP_ODR          = 0x5Eu,
    DEVICE_IMU_REGISTER_EMB_FUNC_INIT_A   = 0x66u,
    DEVICE_IMU_REGISTER_CTRL1             = 0x10u,
    DEVICE_IMU_REGISTER_CTRL2             = 0x11u,
    DEVICE_IMU_REGISTER_CTRL3             = 0x12u,
    DEVICE_IMU_REGISTER_CTRL4             = 0x13u,
    DEVICE_IMU_REGISTER_CTRL5             = 0x14u,
    DEVICE_IMU_REGISTER_CTRL6             = 0x15u,
    DEVICE_IMU_REGISTER_CTRL7             = 0x16u,
    DEVICE_IMU_REGISTER_CTRL8             = 0x17u,
    DEVICE_IMU_REGISTER_PAGE_RW           = 0x17u,
    DEVICE_IMU_REGISTER_CTRL9             = 0x18u,
    DEVICE_IMU_REGISTER_CTRL10            = 0x19u,
    DEVICE_IMU_REGISTER_STATUS_REG        = 0x1Eu,
    DEVICE_IMU_REGISTER_FIFO_STATUS1      = 0x1Bu,
    DEVICE_IMU_REGISTER_FIFO_STATUS2      = 0x1Cu,
    DEVICE_IMU_REGISTER_OUT_TEMP_L        = 0x20u,
    DEVICE_IMU_REGISTER_OUT_TEMP_H        = 0x21u,
    DEVICE_IMU_REGISTER_TIMESTAMP         = 0x40u,
    DEVICE_IMU_REGISTER_TIMESTAMP1        = 0x41u,
    DEVICE_IMU_REGISTER_TIMESTAMP2        = 0x42u,
    DEVICE_IMU_REGISTER_TIMESTAMP3        = 0x43u,
    DEVICE_IMU_REGISTER_OUTX_L_G          = 0x22u,
    DEVICE_IMU_REGISTER_OUTX_H_G          = 0x23u,
    DEVICE_IMU_REGISTER_OUTY_L_G          = 0x24u,
    DEVICE_IMU_REGISTER_OUTY_H_G          = 0x25u,
    DEVICE_IMU_REGISTER_OUTZ_L_G          = 0x26u,
    DEVICE_IMU_REGISTER_OUTZ_H_G          = 0x27u,
    DEVICE_IMU_REGISTER_OUTX_L_A          = 0x28u,
    DEVICE_IMU_REGISTER_OUTX_H_A          = 0x29u,
    DEVICE_IMU_REGISTER_OUTY_L_A          = 0x2Au,
    DEVICE_IMU_REGISTER_OUTY_H_A          = 0x2Bu,
    DEVICE_IMU_REGISTER_OUTZ_L_A          = 0x2Cu,
    DEVICE_IMU_REGISTER_OUTZ_H_A          = 0x2Du,
    DEVICE_IMU_REGISTER_FIFO_DATA_OUT_TAG = 0x78u,
    DEVICE_IMU_REGISTER_TAP_CFG0          = 0x56u,
    DEVICE_IMU_REGISTER_TAP_CFG1          = 0x57u,
    DEVICE_IMU_REGISTER_TAP_CFG2          = 0x58u,
    DEVICE_IMU_REGISTER_TAP_THS_6D        = 0x59u,
    DEVICE_IMU_REGISTER_TAP_DUR           = 0x5Au,
    DEVICE_IMU_REGISTER_WAKE_UP_THS       = 0x5Bu,
    DEVICE_IMU_REGISTER_WAKE_UP_DUR       = 0x5Cu,
    DEVICE_IMU_REGISTER_FREE_FALL         = 0x5Du,
    DEVICE_IMU_REGISTER_MD1_CFG           = 0x5Eu,
    DEVICE_IMU_REGISTER_MD2_CFG           = 0x5Fu,
    DEVICE_IMU_REGISTER_HAODR_CFG         = 0x62u,
    DEVICE_IMU_REGISTER_EMB_FUNC_CFG      = 0x63u,
    DEVICE_IMU_REGISTER_UI_HANDSHAKE_CTRL = 0x64u,
    DEVICE_IMU_REGISTER_CTRL_EIS          = 0x6Bu,
    DEVICE_IMU_REGISTER_UI_INT_OIS        = 0x6Fu,
    DEVICE_IMU_REGISTER_UI_CTRL1_OIS      = 0x70u,
    DEVICE_IMU_REGISTER_UI_CTRL2_OIS      = 0x71u,
    DEVICE_IMU_REGISTER_UI_CTRL3_OIS      = 0x72u,
    DEVICE_IMU_REGISTER_X_OFS_USR         = 0x73u,
    DEVICE_IMU_REGISTER_Y_OFS_USR         = 0x74u,
    DEVICE_IMU_REGISTER_Z_OFS_USR         = 0x75u,
    DEVICE_IMU_REGISTER_FUNCTIONS_ENABLE  = 0x50u,
} device_imu_register_t;

typedef enum
{
    DEVICE_IMU_EMB_PAGE_REGISTER_SFLP_GAME_GBIASX_L = 0x6Eu,
    DEVICE_IMU_EMB_PAGE_REGISTER_SFLP_GAME_GBIASX_H = 0x6Fu,
    DEVICE_IMU_EMB_PAGE_REGISTER_SFLP_GAME_GBIASY_L = 0x70u,
    DEVICE_IMU_EMB_PAGE_REGISTER_SFLP_GAME_GBIASY_H = 0x71u,
    DEVICE_IMU_EMB_PAGE_REGISTER_SFLP_GAME_GBIASZ_L = 0x72u,
    DEVICE_IMU_EMB_PAGE_REGISTER_SFLP_GAME_GBIASZ_H = 0x73u,
} device_imu_emb_page_register_t;

typedef enum
{
    DEVICE_IMU_FIFO_TAG_EMPTY       = 0x00u,
    DEVICE_IMU_FIFO_TAG_GYRO        = 0x01u,
    DEVICE_IMU_FIFO_TAG_ACCEL       = 0x02u,
    DEVICE_IMU_FIFO_TAG_TEMPERATURE = 0x03u,
    DEVICE_IMU_FIFO_TAG_TIMESTAMP   = 0x04u,
    DEVICE_IMU_FIFO_TAG_ACCEL_T2    = 0x06u,
    DEVICE_IMU_FIFO_TAG_ACCEL_T1    = 0x07u,
    DEVICE_IMU_FIFO_TAG_ACCEL_2XC   = 0x08u,
    DEVICE_IMU_FIFO_TAG_ACCEL_3XC   = 0x09u,
    DEVICE_IMU_FIFO_TAG_GYRO_T2     = 0x0Au,
    DEVICE_IMU_FIFO_TAG_GYRO_T1     = 0x0Bu,
    DEVICE_IMU_FIFO_TAG_GYRO_2XC    = 0x0Cu,
    DEVICE_IMU_FIFO_TAG_GYRO_3XC    = 0x0Du,
    DEVICE_IMU_FIFO_TAG_SFLP_GAME   = 0x13u,
    DEVICE_IMU_FIFO_TAG_SFLP_GBIAS  = 0x16u,
    DEVICE_IMU_FIFO_TAG_SFLP_GRAVITY = 0x17u,
} device_imu_fifo_tag_t;

typedef union
{
    uint8 U;
    struct
    {
        unsigned int ois_ctrl_from_ui:1;
        unsigned int spi2_reset:1;
        unsigned int sw_por:1;
        unsigned int fsm_wr_ctrl_en:1;
        unsigned int not_used_4:2;
        unsigned int shub_reg_access:1;
        unsigned int emb_func_reg_access:1;
    } B;
} device_imu_func_cfg_access_reg_t;

typedef union
{
    uint8 U;
    struct
    {
        unsigned int page_sel:4;
        unsigned int not_used_4:3;
        unsigned int page_sel_must_one:1;
    } B;
} device_imu_page_sel_reg_t;

typedef union
{
    uint8 U;
    struct
    {
        unsigned int not_used_0:1;
        unsigned int sflp_game_en:1;
        unsigned int not_used_2:1;
        unsigned int pedo_en:1;
        unsigned int tilt_en:1;
        unsigned int sign_motion_en:1;
        unsigned int not_used_6:1;
        unsigned int mlc_before_fsm_en:1;
    } B;
} device_imu_emb_func_en_a_reg_t;

typedef union
{
    uint8 U;
    struct
    {
        unsigned int not_used_0:1;
        unsigned int sflp_game_fifo_en:1;
        unsigned int not_used_2:2;
        unsigned int sflp_gravity_fifo_en:1;
        unsigned int sflp_gbias_fifo_en:1;
        unsigned int step_counter_fifo_en:1;
        unsigned int mlc_fifo_en:1;
    } B;
} device_imu_emb_func_fifo_en_a_reg_t;

typedef enum
{
    DEVICE_IMU_SFLP_ODR_15HZ  = 0x0u,
    DEVICE_IMU_SFLP_ODR_30HZ  = 0x1u,
    DEVICE_IMU_SFLP_ODR_60HZ  = 0x2u,
    DEVICE_IMU_SFLP_ODR_120HZ = 0x3u,
    DEVICE_IMU_SFLP_ODR_240HZ = 0x4u,
    DEVICE_IMU_SFLP_ODR_480HZ = 0x5u,
} device_imu_sflp_odr_t;

typedef union
{
    uint8 U;
    struct
    {
        unsigned int not_used_0:3;
        unsigned int sflp_game_odr:3;
        unsigned int not_used_6:2;
    } B;
} device_imu_sflp_odr_reg_t;

typedef union
{
    uint8 U;
    struct
    {
        unsigned int not_used_0:1;
        unsigned int sflp_game_init:1;
        unsigned int not_used_2:1;
        unsigned int step_det_init:1;
        unsigned int tilt_init:1;
        unsigned int sig_mot_init:1;
        unsigned int not_used_6:1;
        unsigned int mlc_before_fsm_init:1;
    } B;
} device_imu_emb_func_init_a_reg_t;

typedef union
{
    uint8 U;
    struct
    {
        unsigned int wtm:8;
    } B;
} device_imu_fifo_ctrl1_reg_t;

typedef union
{
    uint8 U;
    struct
    {
        unsigned int xl_dualc_batch_from_fsm:1;
        unsigned int uncompr_rate:2;
        unsigned int not_used_3:1;
        unsigned int odr_chg_en:1;
        unsigned int not_used_5:1;
        unsigned int fifo_compr_rt_en:1;
        unsigned int stop_on_wtm:1;
    } B;
} device_imu_fifo_ctrl2_reg_t;

typedef enum
{
    DEVICE_IMU_FIFO_BATCH_NOT_BATCHED = 0x0u,
    DEVICE_IMU_FIFO_BATCH_3840HZ      = 0xBu,
} device_imu_fifo_batch_rate_t;

typedef union
{
    uint8 U;
    struct
    {
        unsigned int bdr_xl:4;
        unsigned int bdr_gy:4;
    } B;
} device_imu_fifo_ctrl3_reg_t;

typedef enum
{
    DEVICE_IMU_FIFO_TEMP_NOT_BATCHED = 0x0u,
    DEVICE_IMU_FIFO_TEMP_60HZ        = 0x3u,
} device_imu_fifo_temp_batch_rate_t;

typedef enum
{
    DEVICE_IMU_FIFO_TIMESTAMP_NOT_BATCHED = 0x0u,
    DEVICE_IMU_FIFO_TIMESTAMP_DEC_1       = 0x1u,
} device_imu_fifo_timestamp_batch_rate_t;

typedef enum
{
    DEVICE_IMU_FIFO_MODE_BYPASS = 0x0u,
    DEVICE_IMU_FIFO_MODE_STREAM = 0x6u,
} device_imu_fifo_mode_t;

typedef union
{
    uint8 U;
    struct
    {
        unsigned int fifo_mode:3;
        unsigned int g_eis_fifo_en:1;
        unsigned int odr_t_batch:2;
        unsigned int dec_ts_batch:2;
    } B;
} device_imu_fifo_ctrl4_reg_t;

typedef union
{
    uint8 U;
    struct
    {
        unsigned int sw_reset:1;
        unsigned int not_used_1:1;
        unsigned int if_inc:1;
        unsigned int not_used_3:3;
        unsigned int bdu:1;
        unsigned int boot:1;
    } B;
} device_imu_ctrl3_reg_t;

typedef union
{
    uint8 U;
    struct
    {
        unsigned int int2_on_int1:1;
        unsigned int drdy_pulsed:1;
        unsigned int int2_drdy_temp:1;
        unsigned int drdy_mask:1;
        unsigned int not_used_4:4;
    } B;
} device_imu_ctrl4_reg_t;

typedef enum
{
    DEVICE_IMU_CTRL12_OP_MODE_HIGH_PERFORMANCE = 0u,
} device_imu_ctrl12_op_mode_t;

typedef enum
{
    DEVICE_IMU_CTRL12_ODR_POWER_DOWN = 0x0u,
    DEVICE_IMU_CTRL12_ODR_3840HZ     = 0xBu,
} device_imu_ctrl12_odr_t;

typedef union
{
    uint8 U;
    struct
    {
        unsigned int odr_xl:4;
        unsigned int op_mode_xl:3;
        unsigned int not_used_7:1;
    } B;
} device_imu_ctrl1_reg_t;

typedef union
{
    uint8 U;
    struct
    {
        unsigned int odr_g:4;
        unsigned int op_mode_g:3;
        unsigned int not_used_7:1;
    } B;
} device_imu_ctrl2_reg_t;

typedef enum
{
    DEVICE_IMU_CTRL6_FS_G_125DPS  = 0x0u,
    DEVICE_IMU_CTRL6_FS_G_250DPS  = 0x1u,
    DEVICE_IMU_CTRL6_FS_G_500DPS  = 0x2u,
    DEVICE_IMU_CTRL6_FS_G_1000DPS = 0x3u,
    DEVICE_IMU_CTRL6_FS_G_2000DPS = 0x4u,
    DEVICE_IMU_CTRL6_FS_G_4000DPS = 0xCu,
} device_imu_ctrl6_fs_g_t;

typedef union
{
    uint8 U;
    struct
    {
        unsigned int fs_g:4;
        unsigned int lpf1_g_bw:3;
        unsigned int not_used_7:1;
    } B;
} device_imu_ctrl6_reg_t;

typedef union
{
    uint8 U;
    struct
    {
        unsigned int lpf1_g_en:1;
        unsigned int not_used_1_3:3;
        unsigned int ah_qvar_c_zin:2;
        unsigned int int2_drdy_ah_qvar:1;
        unsigned int ah_qvar_en:1;
    } B;
} device_imu_ctrl7_reg_t;

typedef enum
{
    DEVICE_IMU_CTRL8_FS_XL_2G  = 0x0u,
    DEVICE_IMU_CTRL8_FS_XL_4G  = 0x1u,
    DEVICE_IMU_CTRL8_FS_XL_8G  = 0x2u,
    DEVICE_IMU_CTRL8_FS_XL_16G = 0x3u,
} device_imu_ctrl8_fs_xl_t;

typedef union
{
    uint8 U;
    struct
    {
        unsigned int fs_xl:2;
        unsigned int not_used_2:1;
        unsigned int xl_dualc_en:1;
        unsigned int not_used_4:1;
        unsigned int hp_lpf2_xl_bw:3;
    } B;
} device_imu_ctrl8_reg_t;

typedef union
{
    uint8 U;
    struct
    {
        unsigned int usr_off_on_out:1;
        unsigned int usr_off_w:1;
        unsigned int not_used_2:1;
        unsigned int lpf2_xl_en:1;
        unsigned int hp_slope_xl_en:1;
        unsigned int xl_fastsettl_mode:1;
        unsigned int hp_ref_mode_xl:1;
        unsigned int not_used_7:1;
    } B;
} device_imu_ctrl9_reg_t;

typedef union
{
    uint8 U;
    struct
    {
        unsigned int drdy_xl:1;
        unsigned int drdy_gy:1;
        unsigned int drdy_temp:1;
        unsigned int drdy_ah_qvar:1;
        unsigned int drdy_eis:1;
        unsigned int drdy_ois:1;
        unsigned int gy_settling:1;
        unsigned int timestamp:1;
    } B;
} device_imu_status_reg_t;

typedef union
{
    uint8 U;
    struct
    {
        unsigned int inact_en:2;
        unsigned int not_used_2:4;
        unsigned int timestamp_en:1;
        unsigned int interrupts_enable:1;
    } B;
} device_imu_functions_enable_reg_t;

typedef struct
{
    device_imu_fifo_ctrl1_reg_t fifo_ctrl1;
    device_imu_fifo_ctrl2_reg_t fifo_ctrl2;
    device_imu_fifo_ctrl3_reg_t fifo_ctrl3;
    device_imu_fifo_ctrl4_reg_t fifo_ctrl4;
    device_imu_ctrl1_reg_t ctrl1;
    device_imu_ctrl2_reg_t ctrl2;
    device_imu_ctrl3_reg_t ctrl3;
    device_imu_ctrl4_reg_t ctrl4;
    device_imu_ctrl6_reg_t ctrl6;
    device_imu_ctrl7_reg_t ctrl7;
    device_imu_ctrl8_reg_t ctrl8;
    device_imu_ctrl9_reg_t ctrl9;
    device_imu_functions_enable_reg_t functions_enable;
    device_imu_page_sel_reg_t page_sel;
    device_imu_emb_func_en_a_reg_t emb_func_en_a;
    device_imu_emb_func_fifo_en_a_reg_t emb_func_fifo_en_a;
    device_imu_sflp_odr_reg_t sflp_odr;
    device_imu_emb_func_init_a_reg_t emb_func_init_a;
} device_imu_register_init_t;

extern const device_imu_register_init_t device_imu_register_init_default;

#endif
