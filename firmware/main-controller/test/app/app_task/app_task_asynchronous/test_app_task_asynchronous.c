#include "test_app_task_asynchronous.h"

#include "../../../../inc/middleware/tools/tools_print/tools_print.h"

void app_test_app_task_asynchronous_imu_callback(void) { tools_printf("IMU_ASYNC "); }
void app_test_app_task_asynchronous_encoder_callback(void) { tools_printf("ENC_ASYNC "); }
void app_test_app_task_asynchronous_phototube_callback(void) { tools_printf("PT_ASYNC "); }
