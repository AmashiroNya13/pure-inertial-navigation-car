#include "test_device_imu.h"
#include "device_imu.h"
#include "device_imu_register.h"
#include "tools_print.h"

static void test_device_imu_print_buffer(device_imu_id_t imu_id, const uint8* label)
{
    device_imu_runtime_t* imu_runtime = device_imu_runtime_table_get();
    sint16 acc_x = (sint16)((((uint16)imu_runtime[imu_id].imu_accelerometer_buffer[1]) << 8u)
                            | ((uint16)imu_runtime[imu_id].imu_accelerometer_buffer[0]));
    sint16 acc_y = (sint16)((((uint16)imu_runtime[imu_id].imu_accelerometer_buffer[3]) << 8u)
                            | ((uint16)imu_runtime[imu_id].imu_accelerometer_buffer[2]));
    sint16 acc_z = (sint16)((((uint16)imu_runtime[imu_id].imu_accelerometer_buffer[5]) << 8u)
                            | ((uint16)imu_runtime[imu_id].imu_accelerometer_buffer[4]));
    sint16 gyr_x = (sint16)((((uint16)imu_runtime[imu_id].imu_gyroscope_buffer[1]) << 8u)
                            | ((uint16)imu_runtime[imu_id].imu_gyroscope_buffer[0]));
    sint16 gyr_y = (sint16)((((uint16)imu_runtime[imu_id].imu_gyroscope_buffer[3]) << 8u)
                            | ((uint16)imu_runtime[imu_id].imu_gyroscope_buffer[2]));
    sint16 gyr_z = (sint16)((((uint16)imu_runtime[imu_id].imu_gyroscope_buffer[5]) << 8u)
                            | ((uint16)imu_runtime[imu_id].imu_gyroscope_buffer[4]));
    sint16 temp = (sint16)((((uint16)imu_runtime[imu_id].imu_temperature_buffer[1]) << 8u)
                           | ((uint16)imu_runtime[imu_id].imu_temperature_buffer[0]));
    uint32 timestamp = (((uint32)imu_runtime[imu_id].imu_timestamp_buffer[3]) << 24u)
                     | (((uint32)imu_runtime[imu_id].imu_timestamp_buffer[2]) << 16u)
                     | (((uint32)imu_runtime[imu_id].imu_timestamp_buffer[1]) << 8u)
                     | ((uint32)imu_runtime[imu_id].imu_timestamp_buffer[0]);

    tools_printf("[%s] ACC:%d %d %d  GYR:%d %d %d  TEMP:%d  TS:%lu\r\n",
                 label,
                 (int)acc_x,
                 (int)acc_y,
                 (int)acc_z,
                 (int)gyr_x,
                 (int)gyr_y,
                 (int)gyr_z,
                 (int)temp,
                 (unsigned long)timestamp);
}

void test_device_imu_read(void)
{
    static boolean imu_ready = FALSE;
    uint8 imu1_whoami;
    uint8 imu2_whoami;
    device_imu_status_reg_t imu1_status;
    device_imu_status_reg_t imu2_status;
    device_imu_runtime_t* imu_runtime = device_imu_runtime_table_get();

    if (imu_ready == FALSE)
    {
        imu1_whoami = device_imu_read_whoami(DEVICE_IMU_1);
        imu2_whoami = device_imu_read_whoami(DEVICE_IMU_2);
        imu1_status.U = device_imu_read_status(DEVICE_IMU_1);
        imu2_status.U = device_imu_read_status(DEVICE_IMU_2);

        tools_printf("[IMU1] WHOAMI:0x%02X STATUS:0x%02X\r\n",
                     (unsigned int)imu1_whoami,
                     (unsigned int)imu1_status.U);
        tools_printf("[IMU2] WHOAMI:0x%02X STATUS:0x%02X\r\n",
                     (unsigned int)imu2_whoami,
                     (unsigned int)imu2_status.U);

        if ((imu1_whoami == DEVICE_IMU_WHO_AM_I_VALUE)
            && (imu2_whoami == DEVICE_IMU_WHO_AM_I_VALUE))
        {
            imu_ready = TRUE;
        }
        else
        {
            return;
        }
    }

    device_imu_read_accelerometer(DEVICE_IMU_1, imu_runtime[DEVICE_IMU_1].imu_accelerometer_buffer);
    device_imu_read_gyroscope(DEVICE_IMU_1, imu_runtime[DEVICE_IMU_1].imu_gyroscope_buffer);
    device_imu_read_temperature(DEVICE_IMU_1, imu_runtime[DEVICE_IMU_1].imu_temperature_buffer);
    device_imu_read_timestamp(DEVICE_IMU_1, imu_runtime[DEVICE_IMU_1].imu_timestamp_buffer);
    device_imu_read_accelerometer(DEVICE_IMU_2, imu_runtime[DEVICE_IMU_2].imu_accelerometer_buffer);
    device_imu_read_gyroscope(DEVICE_IMU_2, imu_runtime[DEVICE_IMU_2].imu_gyroscope_buffer);
    device_imu_read_temperature(DEVICE_IMU_2, imu_runtime[DEVICE_IMU_2].imu_temperature_buffer);
    device_imu_read_timestamp(DEVICE_IMU_2, imu_runtime[DEVICE_IMU_2].imu_timestamp_buffer);

    test_device_imu_print_buffer(DEVICE_IMU_1, (const uint8*)"IMU1");
    test_device_imu_print_buffer(DEVICE_IMU_2, (const uint8*)"IMU2");
}

void test_device_imu1_rx_callback(void)
{
    test_device_imu_print_buffer(DEVICE_IMU_1, (const uint8*)"IMU1");
}

void test_device_imu2_rx_callback(void)
{
    test_device_imu_print_buffer(DEVICE_IMU_2, (const uint8*)"IMU2");
}
