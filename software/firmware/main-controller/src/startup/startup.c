/**
 * @file startup.c
 * @brief 系统启动初始化与运行入口实现。
 */

#include "../../inc/startup/startup.h"

#include "../../inc/app/core/app_task_manager/app_task_manager_asynchronous.h"
#include "../../inc/app/core/app_task_manager/app_task_manager_resolution.h"
#include "../../inc/app/core/app_task_manager/app_task_manager_scheduler.h"
#include "../../inc/app/core/app_task_manager/app_task_manager_service.h"
#include "../../inc/app/gui/gui.h"
#include "../../inc/app/app_task/app_task_asynchronous/app_task_phototube.h"
#include "../../inc/app/module/module_vehicle_control/module_vehicle_control.h"
#include "../../inc/app/module/module_vehicle_encoder/module_vehicle_encoder.h"
#include "../../inc/app/module/module_vehicle_esc/module_vehicle_esc.h"
#include "../../inc/app/module/module_vehicle_gyro/module_vehicle_gyro.h"
#include "../../inc/app/module/module_vehicle_path/module_vehicle_path.h"
#include "../../inc/app/module/module_vehicle_phototube/module_vehicle_phototube.h"
#include "../../inc/app/module/module_vehicle_pose_fusion/module_vehicle_pose_fusion.h"
#include "../../inc/app/service/service_storage/service_storage.h"
#include "../../inc/app/app_task/app_task_asynchronous/app_task_vehicle.h"
#include "../../inc/app/app_task/app_task_resolution/app_task_resolution.h"
#include "../../inc/app/app_task/app_task_scheduler/app_task_gui.h"
#include "../../inc/app/app_task/app_task_service/app_task_storage.h"

#include "../../inc/middleware/gtm_preinit/gtm_preinit/gtm_preinit.h"
#include "../../inc/middleware/gtm_preinit/gtm_cmu_gclk_preinit/gtm_cmu_gclk_preinit.h"
#include "../../inc/middleware/gtm_preinit/gtm_cmu_clk_preinit/gtm_cmu_clk_preinit.h"
#include "../../inc/middleware/gtm_preinit/gtm_tbu_preinit/gtm_tbu_preinit.h"
#include "../../inc/device/device_key/device_key.h"
#include "../../inc/device/device_led/device_led.h"
#include "../../inc/device/device_debug/device_debug.h"
#include "../../inc/device/device_imu/device_imu.h"
#include "../../inc/device/device_magnetic_encoder/device_magnetic_encoder.h"
#include "../../inc/device/device_esc/device_esc.h"
#include "../../inc/device/device_phototube/device_phototube.h"
//#include "../../inc/device/device_ext_flash/device_ext_flash.h"
#include "../../inc/device/device_int_flash/device_int_flash.h"
#include "../../inc/device/device_carrier/device_carrier.h"
#include "../../inc/middleware/task/task.h"
#include "../../inc/middleware/sysTick/sysTick.h"
#include "../../inc/middleware/tools/tools_host/tools_host.h"
#include "../../inc/middleware/tools/tools_log/tools_log.h"
#include "../../inc/middleware/tools/tools_timing/tools_timing.h"
#include "../../test/test.h"

/**
 * @brief 初始化系统所需底层设备、中间件、服务与应用模块。
 * @param[in] void 无参数。
 * @return void
 */
static void startup_empty(void)
{
}

void startup_init_all(void)
{
    app_task_resolution_enable(FALSE);

    app_task_manager_app_task_resolution_imu_register(DEVICE_IMU_1, app_task_resolution_imu_1_decide);
    app_task_manager_app_task_resolution_imu_register(DEVICE_IMU_2, app_task_resolution_imu_2_decide);

    app_task_manager_app_task_resolution_magnetic_encoder_register(DEVICE_MAGNETIC_ENCODER_1,
                                                                   app_task_resolution_magnetic_encoder_1_decide);
    app_task_manager_app_task_resolution_magnetic_encoder_register(DEVICE_MAGNETIC_ENCODER_2,
                                                                   app_task_resolution_magnetic_encoder_2_decide);

    /*
    app_task_manager_app_task_resolution_esc_register(DEVICE_ESC_1, startup_empty);
    app_task_manager_app_task_resolution_esc_register(DEVICE_ESC_2, startup_empty);
    app_task_manager_app_task_resolution_esc_register(DEVICE_ESC_3, startup_empty);
    */

    
    app_task_manager_app_task_resolution_phototube_group_register(DEVICE_PHOTOTUBE_GROUP_0, app_task_resolution_phototube_group_0_decide);
    app_task_manager_app_task_resolution_phototube_group_register(DEVICE_PHOTOTUBE_GROUP_1, app_task_resolution_phototube_group_1_decide);
    app_task_manager_app_task_resolution_phototube_group_register(DEVICE_PHOTOTUBE_GROUP_2, app_task_resolution_phototube_group_2_decide);

    
    /*
    app_task_manager_app_task_resolution_carrier_register(DEVICE_CARRIER_1, startup_empty);
    app_task_manager_app_task_resolution_carrier_register(DEVICE_CARRIER_2, startup_empty);
    app_task_manager_app_task_resolution_carrier_register(DEVICE_CARRIER_3, startup_empty);
    app_task_manager_app_task_resolution_carrier_register(DEVICE_CARRIER_4, startup_empty);
    app_task_manager_app_task_resolution_carrier_register(DEVICE_CARRIER_5, startup_empty);
    app_task_manager_app_task_resolution_carrier_register(DEVICE_CARRIER_6, startup_empty);
    app_task_manager_app_task_resolution_carrier_register(DEVICE_CARRIER_7, startup_empty);
    app_task_manager_app_task_resolution_carrier_register(DEVICE_CARRIER_8, startup_empty);
    */


    /*
    app_task_manager_app_task_resolution_imu_register(DEVICE_IMU_1, app_test_app_task_resolution_imu_1_callback);
    app_task_manager_app_task_resolution_imu_register(DEVICE_IMU_2, app_test_app_task_resolution_imu_2_callback);

    app_task_manager_app_task_resolution_magnetic_encoder_register(DEVICE_MAGNETIC_ENCODER_1,
                                                                     app_test_app_task_resolution_magnetic_encoder_1_callback);
    app_task_manager_app_task_resolution_magnetic_encoder_register(DEVICE_MAGNETIC_ENCODER_2,
                                                                     app_test_app_task_resolution_magnetic_encoder_2_callback);

    app_task_manager_app_task_resolution_esc_register(DEVICE_ESC_1, app_test_app_task_resolution_esc_1_callback);
    app_task_manager_app_task_resolution_esc_register(DEVICE_ESC_2, app_test_app_task_resolution_esc_2_callback);
    app_task_manager_app_task_resolution_esc_register(DEVICE_ESC_3, app_test_app_task_resolution_esc_3_callback);

    app_task_manager_app_task_resolution_phototube_group_register(DEVICE_PHOTOTUBE_GROUP_0, app_task_resolution_phototube_group_0_decide);
    app_task_manager_app_task_resolution_phototube_group_register(DEVICE_PHOTOTUBE_GROUP_1, app_task_resolution_phototube_group_1_decide);
    app_task_manager_app_task_resolution_phototube_group_register(DEVICE_PHOTOTUBE_GROUP_2, app_task_resolution_phototube_group_2_decide);

    app_task_manager_app_task_resolution_carrier_register(DEVICE_CARRIER_1, app_test_app_task_resolution_carrier_1_callback);
    app_task_manager_app_task_resolution_carrier_register(DEVICE_CARRIER_2, app_test_app_task_resolution_carrier_2_callback);
    app_task_manager_app_task_resolution_carrier_register(DEVICE_CARRIER_3, app_test_app_task_resolution_carrier_3_callback);
    app_task_manager_app_task_resolution_carrier_register(DEVICE_CARRIER_4, app_test_app_task_resolution_carrier_4_callback);
    app_task_manager_app_task_resolution_carrier_register(DEVICE_CARRIER_5, app_test_app_task_resolution_carrier_5_callback);
    app_task_manager_app_task_resolution_carrier_register(DEVICE_CARRIER_6, app_test_app_task_resolution_carrier_6_callback);
    app_task_manager_app_task_resolution_carrier_register(DEVICE_CARRIER_7, app_test_app_task_resolution_carrier_7_callback);
    app_task_manager_app_task_resolution_carrier_register(DEVICE_CARRIER_8, app_test_app_task_resolution_carrier_8_callback);
    */

    //app_task_manager_app_task_asynchronous_register(TASK2, app_test_app_task_asynchronous_phototube_print_all);
    //app_task_manager_app_task_asynchronous_register(TASK1, app_test_app_task_asynchronous_encoder_callback);
    //app_task_manager_app_task_asynchronous_register(TASK2, app_test_app_task_asynchronous_imu_callback);
    //app_task_manager_app_task_asynchronous_register(TASK3, app_test_app_task_asynchronous_phototube_callback);
    app_task_manager_app_task_asynchronous_register(TASK1, app_task_asynchronous_vehicle_fast_run);
    app_task_manager_app_task_asynchronous_register(TASK2, app_task_asynchronous_phototube_run);
    app_task_manager_app_task_asynchronous_register(TASK3, startup_empty);
    app_task_manager_app_task_asynchronous_register(TASK4, startup_empty);
    app_task_manager_app_task_asynchronous_register(TASK5, startup_empty);
    app_task_manager_app_task_asynchronous_register(TASK6, startup_empty);
    app_task_manager_app_task_asynchronous_register(TASK7, startup_empty);
    app_task_manager_app_task_asynchronous_register(TASK8, startup_empty);

    app_task_manager_app_task_scheduler_register(SYSTICK1, startup_empty);
    //app_task_manager_app_task_scheduler_register(SYSTICK1, app_test_app_task_scheduler_tick_1_callback);
    //app_task_manager_app_task_scheduler_register(SYSTICK1, app_test_app_task_scheduler_tick_2_callback);

    app_task_manager_app_task_service_register(app_task_asynchronous_vehicle_service_run);
    //app_task_manager_app_task_service_register(app_test_app_task_service_a_callback);
    //app_task_manager_app_task_service_register(app_test_app_task_service_b_callback);

    device_debug_register_callback(DEVICE_DEBUG_1, tools_host_rx_callback);

    gtm_preinit();
    gtm_cmu_gclk_preinit();
    gtm_cmu_clk_preinit();
    gtm_tbu_preinit();

    device_debug_init_all();
    device_imu_init_all();
    device_key_init_all();
    device_led_init_all();
    device_magnetic_encoder_init_all();
    device_phototube_init_all();
//    device_ext_flash_init_all();
    device_int_flash_init_all();
    device_esc_init_all();
    device_carrier_init_all();

    service_storage_init_all();

    task_init_all();
    sysTick_init_all();
    tools_host_init_all();
    tools_log_init_all();
    tools_timing_init_all();

    module_vehicle_phototube_init();
    module_vehicle_gyro_init();
    module_vehicle_encoder_init();
    module_vehicle_esc_init();
    module_vehicle_pose_fusion_init();
    module_vehicle_path_init();
    vehicle_control_init();

    (void)tools_host_register((const uint8*)"ptcal", module_vehicle_phototube_calibration_command);
    (void)tools_host_register((const uint8*)"ptline", module_vehicle_phototube_line_command);
    (void)tools_host_register((const uint8*)"ptturn", module_vehicle_phototube_turn_command);
    (void)tools_host_register((const uint8*)"ptmap", module_vehicle_phototube_map_command);
    (void)tools_host_register((const uint8*)"ptadc", module_vehicle_phototube_adc_command);
    (void)tools_host_register((const uint8*)"ptpower", module_vehicle_phototube_power_command);
    (void)tools_host_register((const uint8*)"ptcorr", module_vehicle_phototube_correction_command);
    (void)tools_host_register((const uint8*)"vspd", vehicle_control_speed_test_target_command);
    (void)tools_host_register((const uint8*)"vpwm", vehicle_control_drive_pwm_command);
    (void)tools_host_register((const uint8*)"vpid", vehicle_control_speed_test_pid_command);
    (void)tools_host_register((const uint8*)"vaff", vehicle_control_speed_accel_feedforward_command);
    (void)tools_host_register((const uint8*)"vangpid", vehicle_control_angle_pid_command);
    (void)tools_host_register((const uint8*)"vang2d", vehicle_control_angle_2dof_command);
    (void)tools_host_register((const uint8*)"vlaunchpid", vehicle_control_launch_angle_pid_command);
    (void)tools_host_register((const uint8*)"vlaunch", vehicle_control_launch_gate_command);
    (void)tools_host_register((const uint8*)"vff", vehicle_control_speed_test_feedforward_command);
    (void)tools_host_register((const uint8*)"vilim", vehicle_control_speed_integral_limit_command);
    (void)tools_host_register((const uint8*)"vprot", vehicle_control_speed_protection_command);
    (void)tools_host_register((const uint8*)"vstatus", vehicle_control_status_command);
    (void)tools_host_register((const uint8*)"vrspd", vehicle_control_replay_speed_command);
    (void)tools_host_register((const uint8*)"vwcirc", vehicle_control_wheel_circumference_command);
    (void)tools_host_register((const uint8*)"vslipacc", vehicle_control_accel_slip_command);
    (void)tools_host_register((const uint8*)"vtelemetry", vehicle_control_replay_telemetry_command);
    (void)tools_host_register((const uint8*)"vsuc", vehicle_control_suction_command);
    (void)tools_host_register((const uint8*)"vmusic", vehicle_control_music_command);
    (void)tools_host_register((const uint8*)"vautosuc", vehicle_control_auto_suction_command);
    (void)tools_host_register((const uint8*)"vesc", module_vehicle_esc_status_command);
    (void)tools_host_register((const uint8*)"gprint", module_vehicle_gyro_print_command);
    (void)tools_host_register((const uint8*)"gnotch", module_vehicle_gyro_notch_command);
    (void)tools_host_register((const uint8*)"vload", vehicle_control_load_test_command);
    (void)tools_host_register((const uint8*)"vang", vehicle_control_angle_test_command);
    (void)tools_host_register((const uint8*)"vturn", vehicle_control_turn_command);
    (void)tools_host_register((const uint8*)"vpath", app_task_asynchronous_vehicle_path_command);
    (void)tools_host_register((const uint8*)"vstop", app_task_asynchronous_vehicle_stop_command);
    (void)tools_host_register((const uint8*)"stop", app_task_asynchronous_vehicle_stop_command);
    tools_host_binary_register(app_task_asynchronous_vehicle_path_binary_frame);

    gui_init_all();
    app_task_asynchronous_vehicle_init();
    app_task_resolution_enable(TRUE);
}

/**
 * @brief 执行一次系统运行循环。
 * @param[in] void 无参数。
 * @return void
 */
void startup_run(void)
{
    app_task_manager_app_task_service_run();
    //test_device_led_blink();
    //test_device_led_on();
    //test_device_led_off();
    //test_device_key_scanner();
    //test_middleware_tools_print_printf();
    //test_middleware_tools_print_putc();
    //test_middleware_tools_print_print();
    //test_middleware_tools_print_println();
    //test_middleware_tools_print_assert();
    //test_middleware_tools_print_panic();
    //test_middleware_tools_timing_pair();
    //test_middleware_tools_timing_benchmark();
    //test_middleware_tools_log();
    //test_middleware_tools_log_multi();
    //test_middleware_tools_log_edge();
    //test_middleware_tools_log_power_loss();
    //test_middleware_sysTick_work_func_overhead();
    //test_device_carrier_16bit_run();
    //test_device_debug_send();
    //test_device_imu_read();
    //test_device_imu_print_bacon();
    //test_device_magnetic_encoder_read();
    //test_device_esc_run()
}
