/**
 * @file app_task_vehicle.h
 * @brief 整车最终业务异步任务接口。
 */

#ifndef MAD_CIRCUITS_APP_TASK_ASYNCHRONOUS_VEHICLE_H
#define MAD_CIRCUITS_APP_TASK_ASYNCHRONOUS_VEHICLE_H

#include "Ifx_Types.h"

/**
 * @brief 初始化整车异步任务运行状态。
 * @param[in] void 无参数。
 * @return void
 */
void app_task_asynchronous_vehicle_init(void);

/**
 * @brief 周期运行整车异步业务、观测更新和速度测试打印。
 * @param[in] void 无参数。
 * @return void
 */
void app_task_asynchronous_vehicle_run(void);

/**
 * @brief Run only the high-rate vehicle observation and record path update.
 * @param[in] void No parameter.
 * @return void
 */
void app_task_asynchronous_vehicle_fast_run(void);

/**
 * @brief Run low-rate vehicle service work such as keys, state timers, printing and suction ramp.
 * @param[in] void No parameter.
 * @return void
 */
void app_task_asynchronous_vehicle_service_run(void);

/**
 * @brief Parse path record and replay serial command.
 * @param[in] argc Command argument count.
 * @param[in] argv Command argument string array.
 * @return void
 */
void app_task_asynchronous_vehicle_path_command(uint8 argc, uint8* argv[]);

/**
 * @brief Process one binary path upload frame.
 * @param[in] frame Complete frame including header and CRC32.
 * @param[in] length Frame length in bytes.
 * @return void
 */
void app_task_asynchronous_vehicle_path_binary_frame(const uint8* frame, uint16 length);

/**
 * @brief Parse global vehicle stop command.
 * @param[in] argc Command argument count.
 * @param[in] argv Command argument string array.
 * @return void
 */
void app_task_asynchronous_vehicle_stop_command(uint8 argc, uint8* argv[]);

#endif
