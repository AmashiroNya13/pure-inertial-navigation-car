/**
 * @file module_vehicle_control.h
 * @brief Vehicle drive control module public interface.
 */

#ifndef MAD_CIRCUITS_APP_MODULE_MODULE_VEHICLE_CONTROL_H
#define MAD_CIRCUITS_APP_MODULE_MODULE_VEHICLE_CONTROL_H

#include "../../../../config/app/module/module_vehicle_control/vehicle_control_cfg.h"
#include "../module_vehicle_path/module_vehicle_path.h"

typedef enum
{
    VEHICLE_CONTROL_MODE_IDLE = 0,
    VEHICLE_CONTROL_MODE_MANUAL = 1,
    VEHICLE_CONTROL_MODE_RECORD = 2,
    VEHICLE_CONTROL_MODE_REPLAY_WAIT = 3,
    VEHICLE_CONTROL_MODE_REPLAY = 4,
} vehicle_control_mode_t;

typedef algorithm_control_pid_state_t vehicle_control_pid_state_t;

typedef struct
{
    vehicle_control_mode_t mode;
    boolean enabled;
    boolean replay_target_valid;
    uint32 replay_delay_cnt;
    float32 target_speed_mm_s;
    float32 target_theta_rad;
    float32 target_left_speed_mm_s;
    float32 target_right_speed_mm_s;
    float32 yaw_speed_correction_mm_s;
    sint32 left_duty;
    sint32 right_duty;
    uint32 suction_duty;
    vehicle_path_replay_target_t replay_target;
    vehicle_control_pid_state_t angle_pid;
    vehicle_control_pid_state_t left_speed_pid;
    vehicle_control_pid_state_t right_speed_pid;
} vehicle_control_state_t;

typedef enum
{
    VEHICLE_CONTROL_COMMAND_NONE = 0,
    VEHICLE_CONTROL_COMMAND_ENABLE = 1,
    VEHICLE_CONTROL_COMMAND_SET_SPEED = 2,
    VEHICLE_CONTROL_COMMAND_SET_THETA = 3,
    VEHICLE_CONTROL_COMMAND_SET_SUCTION = 4,
    VEHICLE_CONTROL_COMMAND_STOP_ALL = 5,
    VEHICLE_CONTROL_COMMAND_DRIVE_STOP_KEEP_SUCTION = 6,
    VEHICLE_CONTROL_COMMAND_REPLAY_START = 7,
    VEHICLE_CONTROL_COMMAND_REPLAY_STOP = 8,
    VEHICLE_CONTROL_COMMAND_SPEED_TEST_TARGET = 9,
    VEHICLE_CONTROL_COMMAND_SPEED_TEST_STOP = 10,
    VEHICLE_CONTROL_COMMAND_SPEED_TEST_ENABLE = 11,
    VEHICLE_CONTROL_COMMAND_LOAD_TEST_START = 12,
    VEHICLE_CONTROL_COMMAND_LOAD_TEST_STOP = 13,
    VEHICLE_CONTROL_COMMAND_ANGLE_TEST_START = 14,
    VEHICLE_CONTROL_COMMAND_ANGLE_TEST_STOP = 15,
    VEHICLE_CONTROL_COMMAND_TURN_START = 16,
    VEHICLE_CONTROL_COMMAND_TURN_STOP = 17,
    VEHICLE_CONTROL_COMMAND_SPEED_TEST_HEADING_TARGET = 18,
    VEHICLE_CONTROL_COMMAND_DRIVE_PWM = 19,
} vehicle_control_command_type_t;

typedef struct
{
    vehicle_control_command_type_t type;
    boolean enable;
    float32 speed_mm_s;
    float32 theta_rad;
    float32 left_speed_mm_s;
    float32 right_speed_mm_s;
    float32 step_deg;
    sint32 left_duty;
    sint32 right_duty;
    uint32 suction_duty;
} vehicle_control_command_t;

/**
 * @brief 初始化车体控制模块状态和默认输出。
 * @param[in] void 无参数。
 * @return void
 */
void vehicle_control_init(void);

/**
 * @brief Post one control command from service/key/serial context.
 * @param[in] command Command payload copied into the CPU1-side queue.
 * @return TRUE if queued, FALSE if the queue is full or command is invalid.
 */
boolean vehicle_control_command_post(const vehicle_control_command_t* command);

/**
 * @brief Execute queued control commands. Call only from the control-frame owner.
 * @param[in] void No parameter.
 * @return void
 */
void vehicle_control_command_process(void);

/**
 * @brief 使能或关闭车体闭环控制。
 * @param[in] enable TRUE 表示使能控制，FALSE 表示关闭控制。
 * @return void
 */
void vehicle_control_enable(boolean enable);

/**
 * @brief 推进车体控制模式和路径记录回放状态机。
 * @param[in] void 无参数。
 * @return void
 */
void vehicle_control_process(void);

/**
 * @brief 推进位置外环并得到目标角速度。
 * @param[in] dt_s 控制周期，单位：秒。
 * @return 目标角速度，单位：弧度/秒。
 */
float32 vehicle_control_angle_loop_update(float32 dt_s);

/**
 * @brief 推进角速度环并得到差速速度修正量。
 * @param[in] dt_s 控制周期，单位：秒。
 * @return 左右轮差速速度修正量，单位：毫米/秒。
 */

/**
 * @brief 推进左右轮速度环并更新左右驱动占空命令。
 * @param[in] dt_s 控制周期，单位：秒。
 * @return void
 */
void vehicle_control_speed_loop_update(float32 dt_s);

/**
 * @brief Update normal closed-loop drive control after one encoder sample.
 * @param[in] dt_s Encoder sample period, unit: s.
 * @return void
 */
void vehicle_control_closed_loop_update(float32 dt_s);

/**
 * @brief 设置车体目标直线速度。
 * @param[in] speed_mm_s 目标直线速度，单位：毫米/秒。
 * @return void
 */
void vehicle_control_set_speed(float32 speed_mm_s);

/**
 * @brief 设置车体目标航向角。
 * @param[in] theta_rad 目标航向角，单位：弧度。
 * @return void
 */
void vehicle_control_set_theta(float32 theta_rad);

/**
 * @brief 使能或关闭速度环单独测试模式。
 * @param[in] enable TRUE 表示使能速度环测试，FALSE 表示关闭速度环测试。
 * @return void
 */
void vehicle_control_speed_test_enable(boolean enable);

/**
 * @brief 获取速度环单独测试模式是否已使能。
 * @param[in] void 无参数。
 * @return 已使能返回 TRUE，否则返回 FALSE。
 */
boolean vehicle_control_speed_test_is_enabled(void);

/**
 * @brief Return whether the shared speed/angle test launch sequence is active.
 * @return TRUE while launch drag or heading stabilization is running.
 */
boolean vehicle_control_test_launch_active_get(void);

/**
 * @brief 设置速度环测试模式下左右轮目标速度。
 * @param[in] left_speed_mm_s 左轮目标速度，单位：毫米/秒。
 * @param[in] right_speed_mm_s 右轮目标速度，单位：毫米/秒。
 * @return void
 */
void vehicle_control_speed_test_set_target(float32 left_speed_mm_s, float32 right_speed_mm_s);

/**
 * @brief 设置速度环测试模式下左右轮共用的 PID 参数并清空速度环状态。
 * @param[in] kp 比例系数，单位：PWM ticks/(毫米/秒)。
 * @param[in] ki 积分系数，单位：PWM ticks/(毫米/秒)/秒。
 * @param[in] kd 微分系数，单位：PWM ticks/(毫米/秒)*秒。
 * @return void
 */
void vehicle_control_speed_test_set_pid(float32 kp, float32 ki, float32 kd);

/**
 * @brief 推进一次速度环测试闭环并立即输出左右驱动电调占空命令。
 * @param[in] dt_s 本次速度采样和 PID 更新周期，单位：秒。
 * @return void
 */
void vehicle_control_speed_test_update(float32 dt_s);

/**
 * @brief 将速度环测试缓存帧按 VOFA/FireWater 文本格式输出。
 * @param[in] void 无参数。
 * @return void
 */
void vehicle_control_speed_test_print_process(void);

/**
 * @brief Output cached angle-loop step-test frame in VOFA/FireWater text format.
 * @param[in] void No parameter.
 * @return void
 */
void vehicle_control_angle_test_print_process(void);
void vehicle_control_replay_debug_print_process(void);

/**
 * @brief 解析速度环测试目标速度串口命令。
 * @param[in] argc 命令参数个数，单位：个。
 * @param[in] argv 命令参数字符串指针数组。
 * @return void
 */
void vehicle_control_speed_test_target_command(uint8 argc, uint8* argv[]);

/**
 * @brief Directly set left and right drive ESC PWM ticks for bench testing.
 * @param[in] argc Command argument count.
 * @param[in] argv Command argument strings.
 * @return void
 */
void vehicle_control_drive_pwm_command(uint8 argc, uint8* argv[]);

/**
 * @brief 解析速度环测试 PID 参数串口命令。
 * @param[in] argc 命令参数个数，单位：个。
 * @param[in] argv 命令参数字符串指针数组。
 * @return void
 */
void vehicle_control_speed_test_pid_command(uint8 argc, uint8* argv[]);

/**
 * @brief Configure common-mode speed acceleration feedforward.
 * @param[in] argc Command argument count.
 * @param[in] argv Command argument string array.
 * @return void
 */
void vehicle_control_speed_accel_feedforward_command(uint8 argc, uint8* argv[]);

/**
 * @brief Parse angle-loop PID command.
 * @param[in] argc Command argument count.
 * @param[in] argv Command argument string array.
 * @return void
 */
void vehicle_control_angle_pid_command(uint8 argc, uint8* argv[]);

/**
 * @brief Configure the nonlinear two-degree-of-freedom heading controller.
 * @param[in] argc Command argument count.
 * @param[in] argv Command argument strings.
 * @return void
 */
void vehicle_control_angle_2dof_command(uint8 argc, uint8* argv[]);

/**
 * @brief Parse launch correction angle-loop PID command.
 * @param[in] argc Command argument count.
 * @param[in] argv Command argument string array.
 * @return void
 */
void vehicle_control_launch_angle_pid_command(uint8 argc, uint8* argv[]);

/**
 * @brief Enable or bypass replay and test launch drag/correction phases.
 * @param[in] argc Command argument count.
 * @param[in] argv Command argument string array.
 * @return void
 */
void vehicle_control_launch_gate_command(uint8 argc, uint8* argv[]);

/**
 * @brief Parse speed-loop feedforward table command.
 * @param[in] argc Command argument count.
 * @param[in] argv Command argument string array.
 * @return void
 */
void vehicle_control_speed_test_feedforward_command(uint8 argc, uint8* argv[]);

/**
 * @brief Parse speed-loop integral limit command.
 * @param[in] argc Command argument count.
 * @param[in] argv Command argument string array.
 * @return void
 */
void vehicle_control_speed_integral_limit_command(uint8 argc, uint8* argv[]);

/**
 * @brief Parse speed target protection command.
 * @param[in] argc Command argument count.
 * @param[in] argv Command argument string array.
 * @return void
 */
void vehicle_control_speed_protection_command(uint8 argc, uint8* argv[]);

/**
 * @brief Parse replay base speed command.
 * @param[in] argc Command argument count.
 * @param[in] argv Command argument string array.
 * @return void
 */
void vehicle_control_replay_speed_command(uint8 argc, uint8* argv[]);

void vehicle_control_wheel_circumference_command(uint8 argc, uint8* argv[]);

void vehicle_control_accel_slip_command(uint8 argc, uint8* argv[]);

void vehicle_control_replay_telemetry_command(uint8 argc, uint8* argv[]);

/**
 * @brief Print a read-only snapshot of control, path and speed-plan runtime parameters.
 * @param[in] argc Command argument count.
 * @param[in] argv Command argument string array.
 * @return void
 */
void vehicle_control_status_command(uint8 argc, uint8* argv[]);

/**
 * @brief Parse suction ESC duty command.
 * @param[in] argc Command argument count.
 * @param[in] argv Command argument string array.
 * @return void
 */
void vehicle_control_suction_command(uint8 argc, uint8* argv[]);

/**
 * @brief Parse motor music command and forward it to the UART ESC board.
 * @param[in] argc Command argument count.
 * @param[in] argv Command argument string array.
 * @return void
 */
void vehicle_control_music_command(uint8 argc, uint8* argv[]);

/**
 * @brief Get whether automatic suction is allowed for test/replay flows.
 * @param[in] void No parameter.
 * @return TRUE if automatic suction is enabled.
 */
boolean vehicle_control_auto_suction_enabled_get(void);

/**
 * @brief Get whether phototube correction is allowed in the current replay state.
 * @param[in] void No parameter.
 * @return TRUE if phototube correction may update fused XY position.
 */
boolean vehicle_control_phototube_correction_allowed(void);

/**
 * @brief Parse automatic suction enable command for test/replay flows.
 * @param[in] argc Command argument count.
 * @param[in] argv Command argument string array.
 * @return void
 */
void vehicle_control_auto_suction_command(uint8 argc, uint8* argv[]);

/**
 * @brief Parse real-load speed-loop test command.
 * @param[in] argc Command argument count.
 * @param[in] argv Command argument string array.
 * @return void
 */
void vehicle_control_load_test_command(uint8 argc, uint8* argv[]);

/**
 * @brief Parse angle-loop step test command.
 * @param[in] argc Command argument count.
 * @param[in] argv Command argument string array.
 * @return void
 */
void vehicle_control_angle_test_command(uint8 argc, uint8* argv[]);

/**
 * @brief Parse direct zero-speed turn command.
 * @param[in] argc Command argument count.
 * @param[in] argv Command argument string array.
 * @return void
 */
void vehicle_control_turn_command(uint8 argc, uint8* argv[]);

/**
 * @brief Set cached left drive ESC duty without applying it to hardware.
 * @param[in] duty_cycle Target duty command, unit: timer ticks.
 * @return void
 */
void vehicle_control_set_left_duty(sint32 duty_cycle);

/**
 * @brief Set cached right drive ESC duty without applying it to hardware.
 * @param[in] duty_cycle Target duty command, unit: timer ticks.
 * @return void
 */
void vehicle_control_set_right_duty(sint32 duty_cycle);

/**
 * @brief Set cached suction ESC duty without applying it to hardware.
 * @param[in] duty_cycle Target duty command, unit: timer ticks.
 * @return void
 */
void vehicle_control_set_suction_duty(uint32 duty_cycle);

/**
 * @brief Set cached left drive, right drive and suction ESC duties.
 * @param[in] left_duty Left drive duty command, unit: timer ticks.
 * @param[in] right_duty Right drive duty command, unit: timer ticks.
 * @param[in] suction_duty Suction duty command, unit: timer ticks.
 * @return void
 */
void vehicle_control_set_duties(sint32 left_duty, sint32 right_duty, uint32 suction_duty);

/**
 * @brief Apply cached ESC duties to left drive, right drive and suction hardware.
 * @param[in] void No parameter.
 * @return void
 */
void vehicle_control_apply_outputs(void);

/**
 * @brief Advance suction ESC output one slew-limited step toward cached target duty.
 * @param[in] void No parameter.
 * @return void
 */
void vehicle_control_suction_process(void);

/**
 * @brief 开始路径记录并切换到记录模式。
 * @param[in] void 无参数。
 * @return 启动成功返回 TRUE，否则返回 FALSE。
 */
boolean vehicle_control_record_start(void);

/**
 * @brief 停止路径记录并恢复空闲或手动模式。
 * @param[in] void 无参数。
 * @return void
 */
void vehicle_control_record_stop(void);

/**
 * @brief 开始路径回放等待或回放流程。
 * @param[in] void 无参数。
 * @return 启动成功返回 TRUE，否则返回 FALSE。
 */
boolean vehicle_control_replay_start(void);

/**
 * @brief 停止路径回放并恢复空闲或手动模式。
 * @param[in] void 无参数。
 * @return void
 */
void vehicle_control_replay_stop(void);

/**
 * @brief 获取车体控制模块只读状态。
 * @param[in] void 无参数。
 * @return 车体控制模块只读状态指针。
 */
const vehicle_control_state_t* vehicle_control_state_get(void);

#endif
