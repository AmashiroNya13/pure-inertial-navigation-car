#ifndef MAD_CIRCUITS_DEVICE_MAGNETIC_ENCODER_REGISTER_H
#define MAD_CIRCUITS_DEVICE_MAGNETIC_ENCODER_REGISTER_H

#include "Ifx_Types.h"

#define DEVICE_MAGNETIC_ENCODER_REGISTER_PARITY_MASK           ((uint16)0x8000u)
#define DEVICE_MAGNETIC_ENCODER_REGISTER_READ_MASK             ((uint16)0x4000u)
#define DEVICE_MAGNETIC_ENCODER_REGISTER_ADDRESS_MASK          ((uint16)0x3FFFu)
#define DEVICE_MAGNETIC_ENCODER_REGISTER_PARITY_DATA_MASK      ((uint16)0x7FFFu)
#define DEVICE_MAGNETIC_ENCODER_REGISTER_DATA_MASK             ((uint16)0x3FFFu)
#define DEVICE_MAGNETIC_ENCODER_ANGLE_RESOLUTION               ((uint16)16384u)
#define DEVICE_MAGNETIC_ENCODER_ANGLE_DEGREE_PER_LSB           ((float32)(360.0f / 16384.0f))

typedef enum
{
    DEVICE_MAGNETIC_ENCODER_REGISTER_NOP = 0x0000u,
    DEVICE_MAGNETIC_ENCODER_REGISTER_ERROR = 0x0001u,
    DEVICE_MAGNETIC_ENCODER_REGISTER_PROG_CTRL = 0x0003u,
    DEVICE_MAGNETIC_ENCODER_REGISTER_ZERO_MSB = 0x0016u,
    DEVICE_MAGNETIC_ENCODER_REGISTER_ZERO_LSB = 0x0017u,
    DEVICE_MAGNETIC_ENCODER_REGISTER_AGC = 0x3FFDu,
    DEVICE_MAGNETIC_ENCODER_REGISTER_MAGNITUDE = 0x3FFEu,
    DEVICE_MAGNETIC_ENCODER_REGISTER_ANGLE = 0x3FFFu,
} device_magnetic_encoder_register_t;

typedef union
{
    uint16 U;
    struct
    {
        unsigned int data:14;
        unsigned int error_flag:1;
        unsigned int parity:1;
    } B;
} device_magnetic_encoder_frame_t;

typedef union
{
    uint16 U;
    struct
    {
        unsigned int framing_error:1;
        unsigned int command_invalid:1;
        unsigned int parity_error:1;
        unsigned int not_used_3_13:11;
        unsigned int error_flag:1;
        unsigned int parity:1;
    } B;
} device_magnetic_encoder_error_flag_reg_t;

typedef union
{
    uint16 U;
    struct
    {
        unsigned int prog_enable:1;
        unsigned int not_used_1_3:3;
        unsigned int burn:1;
        unsigned int not_used_5:1;
        unsigned int verify:1;
        unsigned int not_used_7_13:7;
        unsigned int not_used_14:1;
        unsigned int parity:1;
    } B;
} device_magnetic_encoder_program_control_reg_t;

typedef union
{
    uint16 U;
    struct
    {
        unsigned int zero_lsb_part:6;
        unsigned int zero_msb:8;
        unsigned int not_used_14:1;
        unsigned int parity:1;
    } B;
} device_magnetic_encoder_zero_position_high_reg_t;

typedef union
{
    uint16 U;
    struct
    {
        unsigned int zero_low6:6;
        unsigned int not_used_6_13:8;
        unsigned int not_used_14:1;
        unsigned int parity:1;
    } B;
} device_magnetic_encoder_zero_position_low_reg_t;

typedef union
{
    uint16 U;
    struct
    {
        unsigned int agc:8;
        unsigned int offset_compensation_finished:1;
        unsigned int comp_low:1;
        unsigned int comp_high:1;
        unsigned int not_used_11:1;
        unsigned int not_used_12_13:2;
        unsigned int not_used_14:1;
        unsigned int parity:1;
    } B;
} device_magnetic_encoder_agc_reg_t;

typedef union
{
    uint16 U;
    struct
    {
        unsigned int agc:8;
        unsigned int offset_compensation_finished:1;
        unsigned int comp_low:1;
        unsigned int comp_high:1;
        unsigned int not_used_11:1;
        unsigned int not_used_12_13:2;
        unsigned int not_used_14:1;
        unsigned int parity:1;
    } B;
} device_magnetic_encoder_diagnostic_reg_t;

typedef union
{
    uint16 U;
    struct
    {
        unsigned int magnitude:14;
        unsigned int not_used_14:1;
        unsigned int parity:1;
    } B;
} device_magnetic_encoder_magnitude_reg_t;

typedef union
{
    uint16 U;
    struct
    {
        unsigned int angle:14;
        unsigned int not_used_14:1;
        unsigned int parity:1;
    } B;
} device_magnetic_encoder_angle_reg_t;

#endif
