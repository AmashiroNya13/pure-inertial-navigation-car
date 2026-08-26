/**
 * @file app_task_resolution.c
 * @brief 设备数据完成事件到应用任务的分辨调度实现。
 */

#include "../../../../inc/app/app_task/app_task_resolution/app_task_resolution.h"

#include "../../../../inc/app/module/module_vehicle_control/module_vehicle_control.h"
#include "../../../../inc/app/module/module_vehicle_encoder/module_vehicle_encoder.h"
#include "../../../../inc/app/module/module_vehicle_gyro/module_vehicle_gyro.h"
#include "../../../../inc/app/module/module_vehicle_pose_fusion/module_vehicle_pose_fusion.h"
#include "../../../../inc/middleware/task/task.h"

#define APP_TASK_RESOLUTION_ENCODER_DT_S (0.0005208333f)

#define APP_TASK_RESOLUTION_TASK1_ENCODER_READY_MASK \
    (((uint32)1u << APP_TASK_RESOLUTION_DEVICE_MAGNETIC_ENCODER_1) \
     | ((uint32)1u << APP_TASK_RESOLUTION_DEVICE_MAGNETIC_ENCODER_2))

#define APP_TASK_RESOLUTION_TASK2_PHOTOTUBE_READY_MASK \
    (((uint32)1u << 0u) \
     | ((uint32)1u << 1u) \
     | ((uint32)1u << 2u))

typedef struct
{
    uint32 ready_mask;
} app_task_resolution_task1_runtime_t;

typedef struct
{
    uint32 ready_mask;
} app_task_resolution_task2_runtime_t;

static app_task_resolution_task1_runtime_t app_task_resolution_task1_runtime;
static app_task_resolution_task2_runtime_t app_task_resolution_task2_runtime;
static boolean app_task_resolution_enabled = FALSE;

static void app_task_resolution_observation_update(void);

void app_task_resolution_enable(boolean enable)
{
    app_task_resolution_enabled = (enable != FALSE) ? TRUE : FALSE;

    if (app_task_resolution_enabled == FALSE)
    {
        app_task_resolution_task1_runtime.ready_mask = 0u;
        app_task_resolution_task2_runtime.ready_mask = 0u;
    }
}

/**
 * @brief 在速度环测试模式下推进一次编码器速度解算和速度环 PID。
 * @param[in] void 无参数。
 * @return void
 */
static void app_task_resolution_encoder_control_update(void)
{
    const vehicle_control_state_t* control_state;

    app_task_resolution_observation_update();
    vehicle_control_command_process();

    if (vehicle_control_speed_test_is_enabled() != FALSE)
    {
        vehicle_control_speed_test_update(APP_TASK_RESOLUTION_ENCODER_DT_S);
    }
    else
    {
        control_state = vehicle_control_state_get();
        if (control_state->enabled != FALSE)
        {
            vehicle_control_closed_loop_update(APP_TASK_RESOLUTION_ENCODER_DT_S);
        }
        else
        {
            vehicle_control_suction_process();
        }
    }
}

static void app_task_resolution_observation_update(void)
{
    module_vehicle_gyro_update(APP_TASK_RESOLUTION_ENCODER_DT_S);
    module_vehicle_pose_fusion_calculate();
}

static void app_task_resolution_encoder_sample_update(app_task_resolution_id_t resolution_id)
{
    if (resolution_id == APP_TASK_RESOLUTION_DEVICE_MAGNETIC_ENCODER_1)
    {
        module_vehicle_encoder_sample(DEVICE_MAGNETIC_ENCODER_1,
                                      APP_TASK_RESOLUTION_ENCODER_DT_S);
    }
    else if (resolution_id == APP_TASK_RESOLUTION_DEVICE_MAGNETIC_ENCODER_2)
    {
        module_vehicle_encoder_sample(DEVICE_MAGNETIC_ENCODER_2,
                                      APP_TASK_RESOLUTION_ENCODER_DT_S);
    }
}

/**
 * @brief 根据 TASK1 相关设备完成标志决定是否触发 TASK1 或速度环测试更新。
 * @param[in] resolution_id 本次完成的数据源编号，取值见 app_task_resolution_id_t。
 * @return void
 */
static void app_task_resolution_task1_decide(app_task_resolution_id_t resolution_id)
{
    if (app_task_resolution_enabled == FALSE)
    {
        return;
    }

    app_task_resolution_task1_runtime.ready_mask |= (uint32)1u << resolution_id;
    app_task_resolution_encoder_sample_update(resolution_id);

    if ((app_task_resolution_task1_runtime.ready_mask & APP_TASK_RESOLUTION_TASK1_ENCODER_READY_MASK)
        != APP_TASK_RESOLUTION_TASK1_ENCODER_READY_MASK)
    {
        return;
    }

    app_task_resolution_task1_runtime.ready_mask &= ~APP_TASK_RESOLUTION_TASK1_ENCODER_READY_MASK;
    app_task_resolution_encoder_control_update();
    task_trap(TASK1);
}

/**
 * @brief 根据光电管组完成标志决定是否触发 TASK2。
 * @param[in] group_index 光电管组索引，单位：组，取值范围：0 到 2。
 * @return void
 */
static void app_task_resolution_task2_decide(uint32 group_index)
{
    if (app_task_resolution_enabled == FALSE)
    {
        return;
    }

    app_task_resolution_task2_runtime.ready_mask |= (uint32)1u << group_index;

    if ((app_task_resolution_task2_runtime.ready_mask & APP_TASK_RESOLUTION_TASK2_PHOTOTUBE_READY_MASK)
        != APP_TASK_RESOLUTION_TASK2_PHOTOTUBE_READY_MASK)
    {
        return;
    }

    app_task_resolution_task2_runtime.ready_mask = 0u;
    task_trap(TASK2);
}

/**
 * @brief IMU1 数据完成后转发到 TASK1 分辨逻辑。
 * @param[in] void 无参数。
 * @return void
 */
void app_task_resolution_imu_1_decide(void)
{
    app_task_resolution_task1_decide(APP_TASK_RESOLUTION_DEVICE_IMU_1);
}

/**
 * @brief IMU2 数据完成后转发到 TASK1 分辨逻辑。
 * @param[in] void 无参数。
 * @return void
 */
void app_task_resolution_imu_2_decide(void)
{
    app_task_resolution_task1_decide(APP_TASK_RESOLUTION_DEVICE_IMU_2);
}

/**
 * @brief 磁编码器1数据完成后转发到 TASK1 分辨逻辑。
 * @param[in] void 无参数。
 * @return void
 */
void app_task_resolution_magnetic_encoder_1_decide(void)
{
    app_task_resolution_task1_decide(APP_TASK_RESOLUTION_DEVICE_MAGNETIC_ENCODER_1);
}

/**
 * @brief 磁编码器2数据完成后转发到 TASK1 分辨逻辑。
 * @param[in] void 无参数。
 * @return void
 */
void app_task_resolution_magnetic_encoder_2_decide(void)
{
    app_task_resolution_task1_decide(APP_TASK_RESOLUTION_DEVICE_MAGNETIC_ENCODER_2);
}

/**
 * @brief 光电管组0数据完成后转发到 TASK2 分辨逻辑。
 * @param[in] void 无参数。
 * @return void
 */
void app_task_resolution_phototube_group_0_decide(void)
{
    app_task_resolution_task2_decide(0u);
}

/**
 * @brief 光电管组1数据完成后转发到 TASK2 分辨逻辑。
 * @param[in] void 无参数。
 * @return void
 */
void app_task_resolution_phototube_group_1_decide(void)
{
    app_task_resolution_task2_decide(1u);
}

/**
 * @brief 光电管组2数据完成后转发到 TASK2 分辨逻辑。
 * @param[in] void 无参数。
 * @return void
 */
void app_task_resolution_phototube_group_2_decide(void)
{
    app_task_resolution_task2_decide(2u);
}
