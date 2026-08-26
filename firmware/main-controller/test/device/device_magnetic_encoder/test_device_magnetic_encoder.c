#include "test_device_magnetic_encoder.h"
#include "device_magnetic_encoder.h"
#include "device_magnetic_encoder_register.h"
#include "tools_print.h"
#include "driver_qspi.h"

static void magnetic_encoder_print_data(device_magnetic_encoder_id_t id, const uint8* label)
{
    uint16 raw_angle, magnitude, agc, error_frame;

    error_frame = device_magnetic_encoder_read_error(id);
    agc         = device_magnetic_encoder_read_agc(id);
    magnitude   = device_magnetic_encoder_read_magnitude(id);
    raw_angle   = device_magnetic_encoder_read_raw_angle(id);

    tools_printf("[%s] ANG:%5u  MAG:%5u  AGC:%3u  ERR:0x%04X\r\n",
                 label,
                 (unsigned int)raw_angle,
                 (unsigned int)magnitude,
                 (unsigned int)agc,
                 (unsigned int)error_frame);
}

void test_device_magnetic_encoder_read(void)
{
    magnetic_encoder_print_data(DEVICE_MAGNETIC_ENCODER_1, (const uint8*)"ENC1");
    magnetic_encoder_print_data(DEVICE_MAGNETIC_ENCODER_2, (const uint8*)"ENC2");
}

void test_device_encoder1_rx_callback(void)
{
    device_magnetic_encoder_runtime_t *runtime = device_magnetic_encoder_runtime_table_get();
    device_magnetic_encoder_angle_reg_t angle_reg;
    float32 degree;
    uint16 angle_raw = runtime[DEVICE_MAGNETIC_ENCODER_1].dma_receive_buffer;

    angle_reg.U = (uint16)(angle_raw & DEVICE_MAGNETIC_ENCODER_REGISTER_DATA_MASK);
    degree = (float32)angle_reg.B.angle * DEVICE_MAGNETIC_ENCODER_ANGLE_DEGREE_PER_LSB;
    tools_printf("[ENC1] angle:%.2f deg\r\n", (double)degree);
}

void test_device_encoder2_rx_callback(void)
{
    device_magnetic_encoder_runtime_t *runtime = device_magnetic_encoder_runtime_table_get();
    device_magnetic_encoder_angle_reg_t angle_reg;
    float32 degree;
    uint16 angle_raw = runtime[DEVICE_MAGNETIC_ENCODER_2].dma_receive_buffer;

    angle_reg.U = (uint16)(angle_raw & DEVICE_MAGNETIC_ENCODER_REGISTER_DATA_MASK);
    degree = (float32)angle_reg.B.angle * DEVICE_MAGNETIC_ENCODER_ANGLE_DEGREE_PER_LSB;
    tools_printf("[ENC2] angle:%.2f deg\r\n", (double)degree);
}

void test_device_encoder1_rx_empty(void)
{
}

void test_device_encoder2_rx_empty(void)
{
}
