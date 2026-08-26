/**
 * @file module_vehicle_control.c
 * @brief Vehicle drive control module implementation.
 */

#include "../../../../inc/app/module/module_vehicle_control/module_vehicle_control.h"

#include "../../../../inc/app/module/module_vehicle_encoder/module_vehicle_encoder.h"
#include "../../../../inc/app/module/module_vehicle_esc/module_vehicle_esc.h"
#include "../../../../inc/app/module/module_vehicle_gyro/module_vehicle_gyro.h"
#include "../../../../inc/app/module/module_vehicle_pose_fusion/module_vehicle_pose_fusion.h"
#include "../../../../inc/middleware/algorithm/algorithm_attitude.h"
#include "../../../../inc/middleware/algorithm/algorithm_control.h"
#include "../../../../inc/middleware/sysTick/sysTick.h"
#include "../../../../inc/middleware/tools/tools_print/tools_print.h"

#define VEHICLE_CONTROL_SPEED_TEST_CORE_PRINT_DIVIDER ((uint32)15u)
#define VEHICLE_CONTROL_SPEED_TEST_ENCODER_PRINT_DIVIDER ((uint32)50u)
#define VEHICLE_CONTROL_SPEED_TEST_DIAG_PRINT_DIVIDER ((uint32)100u)
#define VEHICLE_CONTROL_REPLAY_DEBUG_PRINT_DIVIDER ((uint32)50u)
#define VEHICLE_CONTROL_REPLAY_SLIP_PRINT_DIVIDER ((uint32)4u)
#define VEHICLE_CONTROL_REPLAY_EXTENDED_DEBUG_ENABLE (0u)
#define VEHICLE_CONTROL_SPEED_TEST_FEEDFORWARD_KS_TICK (0.0f)
#define VEHICLE_CONTROL_SPEED_TEST_FEEDFORWARD_KV_TICK_PER_MM_S (0.0f)
#define VEHICLE_CONTROL_SPEED_FEEDFORWARD_POINT_COUNT ((uint32)25u)
#define VEHICLE_CONTROL_DEG_TO_RAD (0.017453292519943295f)
#define VEHICLE_CONTROL_DRIVE_DUTY_SLEW_LIMIT_TICK_PER_UPDATE ((sint32)40)
#define VEHICLE_CONTROL_DRIVE_DUTY_FALL_SLEW_LIMIT_TICK_PER_UPDATE ((sint32)30)
#define VEHICLE_CONTROL_DRIVE_LAUNCH_DUTY_SLEW_LIMIT_TICK_PER_UPDATE ((sint32)7)
#define VEHICLE_CONTROL_DRIVE_LAUNCH_CORRECTION_DUTY_SLEW_LIMIT_TICK_PER_UPDATE ((sint32)10)
#define VEHICLE_CONTROL_DRIVE_CLOSED_LOOP_KEEPALIVE_DUTY_TICK ((sint32)400)
#define VEHICLE_CONTROL_DRIVE_STOP_TARGET_EPSILON_MM_S (0.5f)
#define VEHICLE_CONTROL_DRIVE_DUTY_LIMIT_TICK ((sint32)14500)
#define VEHICLE_CONTROL_DRIVE_START_ASSIST_DUTY_TICK (0.0f)
#define VEHICLE_CONTROL_DRIVE_START_ASSIST_SPEED_MM_S (120.0f)
#define VEHICLE_CONTROL_START_INTEGRAL_HOLD_S (0.60f)
#define VEHICLE_CONTROL_START_INTEGRAL_TARGET_THRESHOLD_MM_S (50.0f)
#define VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_TIME_S (0.40f)
#define VEHICLE_CONTROL_LAUNCH_DRAG_DUTY_TICK ((sint32)750)
#define VEHICLE_CONTROL_REPLAY_LAUNCH_LOW_SPEED_HOLD_TIME_S (0.30f)
#define VEHICLE_CONTROL_REPLAY_LAUNCH_FIXED_FEEDFORWARD_TIME_S (0.0f)
#define VEHICLE_CONTROL_REPLAY_LAUNCH_CROSS_TRACK_STABLE_TIME_S (0.30f)
#define VEHICLE_CONTROL_REPLAY_LAUNCH_CROSS_TRACK_LIMIT_MM (15.0f)
#define VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_SPEED_MM_S (400.0f)
#define VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S (400.0f)
#define VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_ACCEL_LIMIT_MM_S2 (500.0f)
#define VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_FEEDFORWARD_SCALE (0.5f)
#define VEHICLE_CONTROL_REPLAY_LAUNCH_WHEEL_READY_SPEED_MM_S (80.0f)
#define VEHICLE_CONTROL_REPLAY_LAUNCH_WHEEL_READY_TIME_S (0.10f)
#define VEHICLE_CONTROL_REPLAY_LAUNCH_HEADING_STABLE_LIMIT_RAD (8.0f * VEHICLE_CONTROL_DEG_TO_RAD)
#define VEHICLE_CONTROL_REPLAY_LAUNCH_CORRECTION_STABLE_LIMIT_MM_S (300.0f)
#define VEHICLE_CONTROL_REPLAY_LAUNCH_CORRECTION_LIMIT_MM_S (200.0f)
#define VEHICLE_CONTROL_REPLAY_LAUNCH_YAW_CORRECTION_SLEW_LIMIT_MM_S2 (3000.0f)
#define VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_YAW_DUTY_LIMIT_TICK ((sint32)80)
#define VEHICLE_CONTROL_SPEED_INTEGRAL_LIMIT_DEFAULT (800.0f)
#define VEHICLE_CONTROL_SPEED_INTEGRAL_ERROR_LIMIT_MM_S (200.0f)
#define VEHICLE_CONTROL_SPEED_INTEGRAL_ERROR_RATIO (0.10f)
#define VEHICLE_CONTROL_SPEED_INTEGRAL_RECOVERY_MIN_RATIO (0.75f)
#define VEHICLE_CONTROL_SPEED_INTEGRAL_RECOVERY_MAX_RATIO (1.15f)
#define VEHICLE_CONTROL_SPEED_INTEGRAL_TARGET_SLEW_LIMIT_MM_S2 (20000.0f)
#define VEHICLE_CONTROL_TARGET_ACCEL_LIMIT_DEFAULT_MM_S2 (35000.0f)
#define VEHICLE_CONTROL_TARGET_DECEL_LIMIT_DEFAULT_MM_S2 (35000.0f)
#define VEHICLE_CONTROL_START_ACCEL_LIMIT_DEFAULT_MM_S2 (5000.0f)
#define VEHICLE_CONTROL_ACCEL_LIMIT_RAMP_DEFAULT_MM_S3 (86666.664f)
#define VEHICLE_CONTROL_WHEEL_TARGET_ACCEL_LEAD_LIMIT_MM_S (900.0f)
#define VEHICLE_CONTROL_REPLAY_WHEEL_TARGET_DECEL_LEAD_LIMIT_MM_S (0.0f)
#define VEHICLE_CONTROL_YAW_CORRECTION_SLEW_LIMIT_MM_S2 (14000.0f)
#define VEHICLE_CONTROL_D_YAW_RATE_LPF_CUTOFF_HZ (30.0f)
#define VEHICLE_CONTROL_TWO_PI (6.283185307179586f)
#define VEHICLE_CONTROL_REVERSE_GUARD_SPEED_DEFAULT_MM_S (150.0f)
#define VEHICLE_CONTROL_REVERSE_GUARD_TARGET_THRESHOLD_MM_S (1000.0f)
#define VEHICLE_CONTROL_REVERSE_GUARD_CAP_TIME_S (0.03f)
#define VEHICLE_CONTROL_REVERSE_GUARD_CAP_HOLD_S (0.20f)
#define VEHICLE_CONTROL_REVERSE_GUARD_FAULT_TIME_S (0.10f)
#define VEHICLE_CONTROL_REVERSE_GUARD_BASE_CAP_MM_S (500.0f)
#define VEHICLE_CONTROL_ANGLE_DIFF_LIMIT_DEFAULT_MM_S (3000.0f)
#define VEHICLE_CONTROL_ANGLE_CORRECTION_MIN_WHEEL_SPEED_MM_S (300.0f)
#define VEHICLE_CONTROL_WHEEL_BASE_MM (140.0f)
#define VEHICLE_CONTROL_WHEEL_CIRCUMFERENCE_MIN_MM (50.0f)
#define VEHICLE_CONTROL_WHEEL_CIRCUMFERENCE_MAX_MM (150.0f)
#define VEHICLE_CONTROL_ACCEL_SLIP_LPF_HZ (20.0f)
#define VEHICLE_CONTROL_ACCEL_SLIP_ENCODER_MIN_MM_S2 (5000.0f)
#define VEHICLE_CONTROL_ACCEL_SLIP_RESIDUAL_MIN_MM_S2 (4000.0f)
#define VEHICLE_CONTROL_ACCEL_SLIP_RATIO_MAX (0.75f)
#define VEHICLE_CONTROL_ACCEL_SLIP_CONFIRM_S (0.040f)
#define VEHICLE_CONTROL_ACCEL_SLIP_RELEASE_S (0.060f)
#define VEHICLE_CONTROL_ACCEL_SLIP_YAW_RATE_MAX_RAD_S (1.50f)
#define VEHICLE_CONTROL_REPLAY_BINARY_SYNC_0 ((uint8)0xA5u)
#define VEHICLE_CONTROL_REPLAY_BINARY_SYNC_1 ((uint8)0x5Au)
#define VEHICLE_CONTROL_REPLAY_BINARY_VERSION ((uint8)1u)
#define VEHICLE_CONTROL_REPLAY_BINARY_TYPE_SAMPLE ((uint8)1u)
#define VEHICLE_CONTROL_REPLAY_BINARY_TYPE_SLIP ((uint8)2u)
#define VEHICLE_CONTROL_REPLAY_BINARY_PAYLOAD_SIZE ((uint16)128u)
#define VEHICLE_CONTROL_REPLAY_BINARY_SLIP_PAYLOAD_SIZE ((uint16)40u)
#define VEHICLE_CONTROL_REPLAY_BINARY_FRAME_SIZE ((uint16)138u)
#define VEHICLE_CONTROL_SUCTION_DUTY_SLEW_LIMIT_TICK_PER_UPDATE (50u)
#define VEHICLE_CONTROL_LOAD_TEST_START_DELAY_S (1.0f)
#define VEHICLE_CONTROL_LOAD_TEST_SUCTION_HOLD_S (2.0f)
#define VEHICLE_CONTROL_LOAD_TEST_DRAG_TIME_S (0.40f)
#define VEHICLE_CONTROL_LOAD_TEST_RUN_S (0.75f)
#define VEHICLE_CONTROL_LOAD_TEST_CAPTURE_START_S (0.50f)
#define VEHICLE_CONTROL_LOAD_TEST_RESULT_SPEED_ERROR_MIN_MM_S (100.0f)
#define VEHICLE_CONTROL_LOAD_TEST_RESULT_SPEED_ERROR_RATIO (0.05f)
#define VEHICLE_CONTROL_LOAD_TEST_FINISH_SPEED_MM_S (1200.0f)
#define VEHICLE_CONTROL_LOAD_TEST_FINISH_HOLD_S (0.5f)
#define VEHICLE_CONTROL_LOAD_TEST_STOP_HOLD_S (1.0f)
#define VEHICLE_CONTROL_LOAD_TEST_SUCTION_DUTY VEHICLE_CONTROL_DEFAULT_RUN_SUCTION_DUTY
#define VEHICLE_CONTROL_STEER_LOOP_DIVIDER ((uint32)4u)
#define VEHICLE_CONTROL_ANGLE_TEST_PRINT_DIVIDER ((uint32)40u)
#define VEHICLE_CONTROL_ANGLE_TEST_STRAIGHT_TIME_S (1.0f)
#define VEHICLE_CONTROL_ANGLE_TEST_TURN_TIME_S (1.0f)
#define VEHICLE_CONTROL_ANGLE_TEST_SUCTION_DUTY VEHICLE_CONTROL_DEFAULT_RUN_SUCTION_DUTY
#define VEHICLE_CONTROL_TEST_LAUNCH_HEADING_STABLE_LIMIT_RAD (3.0f * VEHICLE_CONTROL_DEG_TO_RAD)
#define VEHICLE_CONTROL_MUSIC_COUNT ((uint32)6u)
#define VEHICLE_CONTROL_COMMAND_QUEUE_LENGTH ((uint8)16u)

typedef enum
{
    VEHICLE_CONTROL_LOAD_TEST_STEP_IDLE = 0,
    VEHICLE_CONTROL_LOAD_TEST_STEP_WAIT_START = 1,
    VEHICLE_CONTROL_LOAD_TEST_STEP_SUCTION_HOLD = 2,
    VEHICLE_CONTROL_LOAD_TEST_STEP_DRAG = 3,
    VEHICLE_CONTROL_LOAD_TEST_STEP_LOW_SPEED_HOLD = 4,
    VEHICLE_CONTROL_LOAD_TEST_STEP_RUN = 5,
    VEHICLE_CONTROL_LOAD_TEST_STEP_FINISH_HOLD = 6,
    VEHICLE_CONTROL_LOAD_TEST_STEP_STOP_HOLD = 7,
} vehicle_control_load_test_step_t;

typedef enum
{
    VEHICLE_CONTROL_ANGLE_TEST_STEP_IDLE = 0,
    VEHICLE_CONTROL_ANGLE_TEST_STEP_WAIT_SUCTION = 1,
    VEHICLE_CONTROL_ANGLE_TEST_STEP_WAIT_ANGLE = 2,
    VEHICLE_CONTROL_ANGLE_TEST_STEP_RUN = 3,
} vehicle_control_angle_test_step_t;

typedef enum
{
    VEHICLE_CONTROL_REPLAY_LAUNCH_PHASE_OFF = 0,
    VEHICLE_CONTROL_REPLAY_LAUNCH_PHASE_DRAG_TIME = 1,
    VEHICLE_CONTROL_REPLAY_LAUNCH_PHASE_CROSS_TRACK_STABILIZE = 2,
} vehicle_control_replay_launch_phase_t;

typedef struct
{
    float32 speed_mm_s;
    float32 duty_tick;
    boolean valid;
} vehicle_control_speed_feedforward_point_t;

typedef struct
{
    vehicle_control_load_test_step_t step;
    float32 timer_s;
    float32 left_speed_mm_s;
    float32 right_speed_mm_s;
    float32 left_speed_sum_mm_s;
    float32 right_speed_sum_mm_s;
    float32 left_duty_sum_tick;
    float32 right_duty_sum_tick;
    uint32 capture_count;
} vehicle_control_load_test_state_t;

typedef struct
{
    vehicle_control_angle_test_step_t step;
    float32 timer_s;
    float32 step_deg;
    float32 speed_mm_s;
    float32 reference_theta_rad;
    float32 target_theta_rad;
    boolean suction_off_done;
} vehicle_control_angle_test_state_t;

typedef struct
{
    float32 target_theta_deg;
    float32 current_theta_deg;
    float32 yaw_speed_correction_mm_s;
    float32 left_target_mm_s;
    float32 right_target_mm_s;
    sint32 left_duty;
    sint32 right_duty;
    uint32 suction_duty;
    uint32 step;
    float32 sample_dt_ms;
    uint8 left_esc_state;
    uint8 left_esc_fault;
    uint8 right_esc_state;
    uint8 right_esc_fault;
} vehicle_control_angle_test_print_frame_t;

typedef struct
{
    vehicle_control_command_t buffer[VEHICLE_CONTROL_COMMAND_QUEUE_LENGTH];
    volatile uint8 head;
    volatile uint8 tail;
    volatile uint32 dropped_count;
} vehicle_control_command_queue_t;

static vehicle_control_state_t vehicle_control_state;
static vehicle_control_command_queue_t vehicle_control_command_queue;
static sint32 vehicle_control_left_output_duty;
static sint32 vehicle_control_right_output_duty;
static uint32 vehicle_control_suction_output_duty;
static algorithm_control_pid_cfg_t vehicle_control_angle_pid_cfg;
static algorithm_control_pid_cfg_t vehicle_control_launch_angle_pid_cfg;
static algorithm_control_pid_cfg_t vehicle_control_left_speed_pid_cfg;
static algorithm_control_pid_cfg_t vehicle_control_right_speed_pid_cfg;
static algorithm_control_pid_state_t vehicle_control_launch_angle_pid;
static algorithm_control_pid_cfg_t vehicle_control_speed_test_left_pid_cfg =
{
    /* Position PID speed loop units:
     * kp: PWM ticks/(mm/s), ki: PWM ticks/mm, kd: PWM ticks/(mm/s^2). */
    .kp = 5.0f,
    .ki = 10.0f,
    .kd = 0.0f,
    .output_min = -3000.0f,
    .output_max = 8000.0f,
    .integral_min = -VEHICLE_CONTROL_SPEED_INTEGRAL_LIMIT_DEFAULT,
    .integral_max = VEHICLE_CONTROL_SPEED_INTEGRAL_LIMIT_DEFAULT,
};
static algorithm_control_pid_cfg_t vehicle_control_speed_test_right_pid_cfg =
{
    /* Position PID speed loop units:
     * kp: PWM ticks/(mm/s), ki: PWM ticks/mm, kd: PWM ticks/(mm/s^2). */
    .kp = 5.0f,
    .ki = 10.0f,
    .kd = 0.0f,
    .output_min = -3000.0f,
    .output_max = 8000.0f,
    .integral_min = -VEHICLE_CONTROL_SPEED_INTEGRAL_LIMIT_DEFAULT,
    .integral_max = VEHICLE_CONTROL_SPEED_INTEGRAL_LIMIT_DEFAULT,
};
typedef struct
{
    float32 left_target_mm_s;
    float32 right_target_mm_s;
    float32 left_speed_mm_s;
    float32 right_speed_mm_s;
    float32 left_integral;
    float32 right_integral;
    sint32 left_duty;
    sint32 right_duty;
    float32 sample_dt_ms;
    float32 target_theta_deg;
    float32 current_theta_deg;
    float32 yaw_speed_correction_mm_s;
    uint8 left_esc_state;
    uint8 left_esc_fault;
    uint8 right_esc_state;
    uint8 right_esc_fault;
    uint16 left_raw_angle;
    uint16 right_raw_angle;
    sint16 left_raw_delta_count;
    sint16 right_raw_delta_count;
    sint16 left_delta_count;
    sint16 right_delta_count;
    float32 left_raw_speed_mm_s;
    float32 right_raw_speed_mm_s;
    float32 left_sample_dt_ms;
    float32 right_sample_dt_ms;
    uint32 left_sample_count;
    uint32 right_sample_count;
    uint32 encoder_drop_count;
    uint32 left_invalid_sample_count;
    uint32 right_invalid_sample_count;
    uint32 left_invalid_streak_count;
    uint32 right_invalid_streak_count;
    uint16 left_rejected_raw_angle;
    uint16 right_rejected_raw_angle;
} vehicle_control_speed_test_print_frame_t;

static boolean vehicle_control_speed_test_enabled;
static boolean vehicle_control_speed_test_stop_requested;
static boolean vehicle_control_speed_test_heading_enabled;
static float32 vehicle_control_speed_test_left_target_mm_s;
static float32 vehicle_control_speed_test_right_target_mm_s;
static uint32 vehicle_control_speed_test_core_print_divider;
static uint32 vehicle_control_speed_test_encoder_print_divider;
static uint32 vehicle_control_speed_test_diag_print_divider;
static vehicle_control_speed_test_print_frame_t vehicle_control_speed_test_print_live_frame;
static volatile boolean vehicle_control_speed_test_core_print_live_pending;
static volatile boolean vehicle_control_speed_test_encoder_print_live_pending;
static volatile boolean vehicle_control_speed_test_diag_print_live_pending;
static uint64 vehicle_control_speed_test_capture_last_tick;
static boolean vehicle_control_speed_test_capture_last_tick_valid;
static vehicle_control_speed_feedforward_point_t
    vehicle_control_left_speed_feedforward_table[VEHICLE_CONTROL_SPEED_FEEDFORWARD_POINT_COUNT];
static vehicle_control_speed_feedforward_point_t
    vehicle_control_right_speed_feedforward_table[VEHICLE_CONTROL_SPEED_FEEDFORWARD_POINT_COUNT];
static boolean vehicle_control_speed_feedforward_enabled = TRUE;
static float32 vehicle_control_speed_accel_feedforward_gain;
static float32 vehicle_control_speed_decel_feedforward_gain;
static float32 vehicle_control_speed_accel_feedforward_limit;
static float32 vehicle_control_speed_accel_feedforward_previous_target_mm_s;
static float32 vehicle_control_speed_accel_feedforward_output;
static boolean vehicle_control_speed_accel_feedforward_ready;
static boolean vehicle_control_speed_integral_limit_enabled = TRUE;
static float32 vehicle_control_speed_integral_limit_abs = VEHICLE_CONTROL_SPEED_INTEGRAL_LIMIT_DEFAULT;
static boolean vehicle_control_speed_protection_enabled = TRUE;
static float32 vehicle_control_target_accel_limit_mm_s2 = VEHICLE_CONTROL_TARGET_ACCEL_LIMIT_DEFAULT_MM_S2;
static float32 vehicle_control_target_decel_limit_mm_s2 = VEHICLE_CONTROL_TARGET_DECEL_LIMIT_DEFAULT_MM_S2;
static float32 vehicle_control_start_accel_limit_mm_s2 = VEHICLE_CONTROL_START_ACCEL_LIMIT_DEFAULT_MM_S2;
static float32 vehicle_control_reverse_guard_speed_mm_s = VEHICLE_CONTROL_REVERSE_GUARD_SPEED_DEFAULT_MM_S;
static float32 vehicle_control_angle_diff_limit_mm_s = VEHICLE_CONTROL_ANGLE_DIFF_LIMIT_DEFAULT_MM_S;
static float32 vehicle_control_yaw_feedback_correction_mm_s;
static float32 vehicle_control_effective_yaw_speed_correction_mm_s;
static float32 vehicle_control_d_yaw_rate_lpf_rad_s;
static boolean vehicle_control_d_yaw_rate_lpf_ready;
static boolean vehicle_control_angle_2dof_enabled;
static float32 vehicle_control_angle_kp2;
static float32 vehicle_control_target_yaw_rate_feedforward_gain_mm;
static float32 vehicle_control_target_yaw_rate_lpf_cutoff_hz;
static float32 vehicle_control_target_yaw_rate_limit_rad_s;
static float32 vehicle_control_target_yaw_rate_rad_s;
static float32 vehicle_control_target_theta_previous_rad;
static boolean vehicle_control_target_yaw_rate_ready;
static float32 vehicle_control_angle_linear_output_mm_s;
static float32 vehicle_control_angle_nonlinear_output_mm_s;
static float32 vehicle_control_angle_rate_output_mm_s;
static float32 vehicle_control_angle_feedforward_output_mm_s;
static float32 vehicle_control_base_effective_target_mm_s;
static boolean vehicle_control_base_start_accel_active;
static float32 vehicle_control_base_accel_limit_mm_s2;
static float32 vehicle_control_left_effective_target_mm_s;
static float32 vehicle_control_right_effective_target_mm_s;
static float32 vehicle_control_left_speed_pid_p_output;
static float32 vehicle_control_right_speed_pid_p_output;
static float32 vehicle_control_left_speed_pid_i_output;
static float32 vehicle_control_right_speed_pid_i_output;
static float32 vehicle_control_left_speed_pid_d_output;
static float32 vehicle_control_right_speed_pid_d_output;
static float32 vehicle_control_left_speed_feedback_output;
static float32 vehicle_control_right_speed_feedback_output;
static float32 vehicle_control_left_speed_feedforward_output;
static float32 vehicle_control_right_speed_feedforward_output;
static float32 vehicle_control_left_speed_assist_output;
static float32 vehicle_control_right_speed_assist_output;
static float32 vehicle_control_left_speed_total_output;
static float32 vehicle_control_right_speed_total_output;
static boolean vehicle_control_left_speed_integral_enabled;
static boolean vehicle_control_right_speed_integral_enabled;
static float32 vehicle_control_tracking_limit_left_before_mm_s;
static float32 vehicle_control_tracking_limit_right_before_mm_s;
static float32 vehicle_control_tracking_limit_left_after_mm_s;
static float32 vehicle_control_tracking_limit_right_after_mm_s;
static boolean vehicle_control_tracking_limit_active;
static uint32 vehicle_control_tracking_limit_event_count;
static boolean vehicle_control_left_start_accel_active;
static boolean vehicle_control_right_start_accel_active;
static float32 vehicle_control_left_accel_limit_mm_s2;
static float32 vehicle_control_right_accel_limit_mm_s2;
static float32 vehicle_control_start_integral_hold_timer_s;
static boolean vehicle_control_start_integral_hold_moving;
static float32 vehicle_control_left_reverse_feedback_timer_s;
static float32 vehicle_control_right_reverse_feedback_timer_s;
static float32 vehicle_control_reverse_feedback_cap_hold_timer_s;
static boolean vehicle_control_reverse_feedback_cap_active;
static boolean vehicle_control_reverse_feedback_fault_latched;
static uint32 vehicle_control_reverse_feedback_event_count;
static float32 vehicle_control_reverse_guard_left_before_mm_s;
static float32 vehicle_control_reverse_guard_right_before_mm_s;
static float32 vehicle_control_reverse_guard_left_after_mm_s;
static float32 vehicle_control_reverse_guard_right_after_mm_s;
static float32 vehicle_control_reverse_guard_base_cap_mm_s;
static boolean vehicle_control_replay_launch_gate_active;
static vehicle_control_replay_launch_phase_t vehicle_control_replay_launch_phase;
static float32 vehicle_control_replay_launch_gate_timer_s;
static float32 vehicle_control_replay_launch_gate_stable_timer_s;
static float32 vehicle_control_replay_launch_wheel_ready_timer_s;
static boolean vehicle_control_replay_launch_wheel_ready;
static float32 vehicle_control_replay_launch_reference_theta_rad;
static boolean vehicle_control_test_launch_gate_active;
static vehicle_control_replay_launch_phase_t vehicle_control_test_launch_phase;
static float32 vehicle_control_test_launch_gate_timer_s;
static float32 vehicle_control_test_launch_gate_stable_timer_s;
static float32 vehicle_control_test_launch_wheel_ready_timer_s;
static boolean vehicle_control_test_launch_wheel_ready;
static float32 vehicle_control_test_launch_reference_theta_rad;
static boolean vehicle_control_launch_drag_enabled = TRUE;
static boolean vehicle_control_launch_correction_enabled = TRUE;
static boolean vehicle_control_launch_drag_correction_enabled = FALSE;
static boolean vehicle_control_auto_suction_enabled = TRUE;
static boolean vehicle_control_accel_slip_enabled = FALSE;
static boolean vehicle_control_replay_binary_telemetry_enabled = TRUE;
static uint16 vehicle_control_replay_binary_sequence;
static uint8 vehicle_control_replay_binary_frame[VEHICLE_CONTROL_REPLAY_BINARY_FRAME_SIZE];
static boolean vehicle_control_accel_slip_active;
static boolean vehicle_control_accel_slip_speed_ready;
static float32 vehicle_control_accel_slip_previous_speed_mm_s;
static float32 vehicle_control_accel_slip_encoder_accel_mm_s2;
static float32 vehicle_control_accel_slip_imu_accel_mm_s2;
static float32 vehicle_control_accel_slip_ratio = 1.0f;
static float32 vehicle_control_accel_slip_confirm_timer_s;
static float32 vehicle_control_accel_slip_release_timer_s;
static uint32 vehicle_control_accel_slip_event_id;
static uint8 vehicle_control_accel_slip_pending_state;
static uint32 vehicle_control_accel_slip_pending_path_index;
static float32 vehicle_control_accel_slip_pending_x_mm;
static float32 vehicle_control_accel_slip_pending_y_mm;
static float32 vehicle_control_accel_slip_pending_encoder_accel_mm_s2;
static float32 vehicle_control_accel_slip_pending_imu_accel_mm_s2;
static float32 vehicle_control_accel_slip_pending_ratio;
static boolean vehicle_control_turn_slip_previous_confirmed;
static boolean vehicle_control_turn_slip_event_pending;
static uint32 vehicle_control_turn_slip_event_id;
static uint32 vehicle_control_turn_slip_pending_path_index;
static float32 vehicle_control_turn_slip_pending_x_mm;
static float32 vehicle_control_turn_slip_pending_y_mm;
static float32 vehicle_control_turn_slip_pending_ratio;
static float32 vehicle_control_turn_slip_pending_encoder_turn_deg;
static float32 vehicle_control_turn_slip_pending_imu_turn_deg;
static float32 vehicle_control_turn_slip_pending_correction_mm;
static vehicle_control_load_test_state_t vehicle_control_load_test_state;
static boolean vehicle_control_load_test_feedforward_enabled = TRUE;
static vehicle_control_angle_test_state_t vehicle_control_angle_test_state;
static vehicle_control_angle_test_print_frame_t vehicle_control_angle_test_print_live_frame;
static volatile boolean vehicle_control_angle_test_print_live_pending;
static uint64 vehicle_control_angle_test_capture_last_tick;
static boolean vehicle_control_angle_test_capture_last_tick_valid;
static uint32 vehicle_control_angle_test_print_divider;
static uint32 vehicle_control_steer_loop_divider;
static const char* vehicle_control_music_name_table[VEHICLE_CONTROL_MUSIC_COUNT] =
{
    "lifeline",
    "shinkai_shoujo",
    "bokura_no_kioku",
    "miku_disappearance",
    "domestic_na_kawaki",
    "sirius_no_shinzou",
};

/**
 * @brief 清空车体控制模块内所有 PID 运行状态。
 * @param[in] void 无参数。
 * @return void
 */
static void vehicle_control_pid_reset_all(void);

static void vehicle_control_speed_accel_feedforward_reset(void);

static float32 vehicle_control_speed_accel_feedforward_update(float32 target_mm_s, float32 dt_s);

static void vehicle_control_d_yaw_rate_lpf_reset(void);

static float32 vehicle_control_d_yaw_rate_lpf_update(float32 input_rad_s, float32 dt_s);

static void vehicle_control_target_yaw_rate_reset(void);

static boolean vehicle_control_angle_2dof_runtime_active(void);

static float32 vehicle_control_target_yaw_rate_update(float32 dt_s);

static void vehicle_control_accel_slip_reset(void);
static void vehicle_control_accel_slip_update(float32 dt_s);
static void vehicle_control_slip_event_print(void);
static void vehicle_control_turn_slip_event_capture(void);
static void vehicle_control_replay_binary_sample_print(
    uint64 dt_us,
    const module_vehicle_pose_fusion_observation_t* pose_observation,
    const module_vehicle_encoder_observation_t* encoder_observation,
    const vehicle_path_state_t* path_state,
    const module_vehicle_esc_role_status_t* left_esc_status,
    const module_vehicle_esc_role_status_t* right_esc_status,
    float32 angle_error_deg,
    float32 yaw_rate_deg_s,
    float32 target_yaw_rate_deg_s);
static void vehicle_control_binary_u16_put(uint8* buffer, uint16* offset, uint16 value);
static void vehicle_control_binary_u32_put(uint8* buffer, uint16* offset, uint32 value);
static void vehicle_control_binary_s16_put(uint8* buffer, uint16* offset, sint16 value);
static void vehicle_control_binary_f32_put(uint8* buffer, uint16* offset, float32 value);
static uint16 vehicle_control_binary_crc16(const uint8* data, uint16 length);
static void vehicle_control_replay_binary_slip_print(
    const module_vehicle_pose_fusion_replay_correction_t* replay_correction);

static void vehicle_control_angle_2dof_status_print(void);

static float32 vehicle_control_angle_correction_update(float32 dt_s);

/**
 * @brief 根据路径回放模块更新当前回放目标。
 * @param[in] void 无参数。
 * @return void
 */
static void vehicle_control_replay_update_target(void);

/**
 * @brief 将当前缓存的空闲占空命令写入电调模块。
 * @param[in] void 无参数。
 * @return void
 */
static void vehicle_control_apply_idle_outputs(void);

static sint32 vehicle_control_drive_duty_slew_step(sint32 current, sint32 target);

static void vehicle_control_drive_launch_duty_slew_pair_step(sint32 current_left,
                                                             sint32 current_right,
                                                             sint32 target_left,
                                                             sint32 target_right,
                                                             sint32* next_left,
                                                             sint32* next_right);

static sint32 vehicle_control_drive_keepalive_duty(sint32 duty, float32 target_speed_mm_s);

static float32 vehicle_control_drive_start_assist(float32 target_speed_mm_s, float32 feedback_speed_mm_s);

static void vehicle_control_speed_integral_limit_clamp_state(algorithm_control_pid_state_t* pid);

static boolean vehicle_control_speed_integral_ready(float32 target_mm_s,
                                                    float32 feedback_mm_s,
                                                    float32 previous_target_mm_s,
                                                    float32 dt_s);

static void vehicle_control_speed_target_runtime_reset(void);

static void vehicle_control_speed_integral_clear(void);

static void vehicle_control_start_integral_hold_update(float32 left_target_mm_s,
                                                       float32 right_target_mm_s,
                                                       float32 dt_s);

static boolean vehicle_control_start_integral_hold_active(void);

static void vehicle_control_replay_launch_gate_start(void);

static void vehicle_control_replay_launch_gate_stop(void);

static void vehicle_control_replay_launch_complete(void);

static boolean vehicle_control_replay_launch_gate_update(float32 dt_s);

static boolean vehicle_control_replay_launch_drag_active(void);

static boolean vehicle_control_replay_launch_fixed_feedforward_active(void);

static boolean vehicle_control_replay_launch_wheel_wait_active(void);

static boolean vehicle_control_replay_launch_angle_hold_active(void);

static boolean vehicle_control_launch_drag_active(void);

static void vehicle_control_replay_path_lookahead_speed_cap_update(void);

static void vehicle_control_replay_launch_target_limit(vehicle_path_replay_target_t* replay_target);

static void vehicle_control_test_launch_gate_start(float32 reference_theta_rad);

static void vehicle_control_test_launch_gate_stop(void);

static void vehicle_control_test_launch_complete(void);

static boolean vehicle_control_test_launch_gate_update(float32 dt_s);

static boolean vehicle_control_test_launch_drag_active(void);

static boolean vehicle_control_launch_correction_active(void);

static boolean vehicle_control_launch_low_speed_active(void);

static boolean vehicle_control_test_launch_moving_requested(void);

static boolean vehicle_control_test_launch_pair_requested(void);

static float32 vehicle_control_speed_target_slew_step(float32 current_target_mm_s,
                                                      float32 desired_target_mm_s,
                                                      float32 dt_s,
                                                      boolean* start_accel_active,
                                                      float32* accel_limit_runtime_mm_s2);

static float32 vehicle_control_base_target_update(float32 desired_base_mm_s, float32 dt_s);

static float32 vehicle_control_yaw_correction_available_get(float32 base_target_mm_s);

static float32 vehicle_control_yaw_rate_available_get(float32 base_target_mm_s);

static float32 vehicle_control_yaw_correction_slew_step(float32 current_mm_s,
                                                        float32 target_mm_s,
                                                        float32 dt_s,
                                                        float32 slew_limit_mm_s2);

static void vehicle_control_wheel_target_accel_limit_apply(float32* left_target_mm_s,
                                                          float32* right_target_mm_s,
                                                          float32 left_feedback_mm_s,
                                                          float32 right_feedback_mm_s,
                                                          float32 dt_s);
static void vehicle_control_wheel_target_tracking_limit_apply(float32* left_target_mm_s,
                                                              float32* right_target_mm_s,
                                                              float32 left_feedback_mm_s,
                                                              float32 right_feedback_mm_s);

static float32 vehicle_control_speed_no_reverse_target(float32 target_mm_s);

static float32 vehicle_control_reverse_guard_target(float32 desired_target_mm_s,
                                                    float32 feedback_speed_mm_s);
static void vehicle_control_reverse_feedback_guard_reset(void);
static void vehicle_control_reverse_feedback_guard_update(float32 left_target_mm_s,
                                                         float32 right_target_mm_s,
                                                         float32 left_feedback_mm_s,
                                                         float32 right_feedback_mm_s,
                                                         float32 dt_s);

static void vehicle_control_speed_target_pair_update(float32 desired_left_mm_s,
                                                     float32 desired_right_mm_s,
                                                     float32 left_feedback_mm_s,
                                                     float32 right_feedback_mm_s,
                                                     float32 dt_s,
                                                     float32* left_target_mm_s,
                                                     float32* right_target_mm_s);

/**
 * @brief 将 PID 输出增量叠加到基准占空命令并转换为非负 PWM ticks。
 * @param[in] idle_duty 基准占空命令，单位：GTM PWM ticks。
 * @param[in] duty_delta PID 输出增量，单位：GTM PWM ticks。
 * @return 非负 PWM 占空命令，单位：GTM PWM ticks。
 */
static sint32 vehicle_control_duty_from_delta(sint32 idle_duty, float32 duty_delta);

static sint32 vehicle_control_drive_clamp_duty(vehicle_esc_role_t role, sint32 duty_cycle);

/**
 * @brief Reset speed feedforward table to the mandatory zero point.
 * @param[in] void No parameter.
 * @return void
 */
static void vehicle_control_speed_feedforward_table_reset(void);

static void vehicle_control_speed_feedforward_table_reset_one(
    vehicle_control_speed_feedforward_point_t* table);

static void vehicle_control_speed_feedforward_table_print(
    const char* side,
    const vehicle_control_speed_feedforward_point_t* table);

/**
 * @brief Add or update one speed feedforward calibration point.
 * @param[in,out] table Target feedforward table.
 * @param[in] speed_mm_s Calibrated wheel speed, unit: mm/s.
 * @param[in] duty_tick Feedforward duty command, unit: GTM PWM ticks.
 * @return TRUE if the point was stored, FALSE if the table is full.
 */
static boolean vehicle_control_speed_feedforward_point_set(
    vehicle_control_speed_feedforward_point_t* table,
    float32 speed_mm_s,
    float32 duty_tick);

/**
 * @brief Calculate open-loop speed feedforward for speed test.
 * @param[in] table Target feedforward table.
 * @param[in] target_speed_mm_s Target wheel speed, unit: mm/s.
 * @return Feedforward output, unit: GTM PWM ticks.
 */
static float32 vehicle_control_speed_feedforward_calculate(
    const vehicle_control_speed_feedforward_point_t* table,
    float32 target_speed_mm_s);

static float32 vehicle_control_abs_f32(float32 value);

/**
 * @brief Calculate suction duty from percent command.
 * @param[in] percent Suction throttle percent, unit: %.
 * @return Suction ESC duty command, unit: GTM PWM ticks.
 */
static uint32 vehicle_control_suction_duty_from_percent(float32 percent);

/**
 * @brief Set suction target duty and advance the hardware output one slew-limited step.
 * @param[in] duty_cycle Target suction duty command, unit: GTM PWM ticks.
 * @return void
 */
static void vehicle_control_suction_apply_duty(uint32 duty_cycle);

/**
 * @brief Advance one suction ESC duty step toward the cached target.
 * @param[in] void No parameter.
 * @return void
 */
void vehicle_control_suction_process(void);

/**
 * @brief Cancel running load-test sequence without changing ESC output.
 * @param[in] void No parameter.
 * @return void
 */
static void vehicle_control_load_test_cancel(void);

static void vehicle_control_load_test_capture_reset(void);

static void vehicle_control_load_test_drag_begin(void);

static void vehicle_control_load_test_low_speed_hold_begin(void);

static void vehicle_control_load_test_closed_loop_begin(void);

/**
 * @brief Advance speed-loop real-load test sequence.
 * @param[in] dt_s Control period, unit: s.
 * @return void
 */
static void vehicle_control_load_test_update(float32 dt_s);

/**
 * @brief Cancel running angle-loop step test without changing normal control mode.
 * @param[in] void No parameter.
 * @return void
 */
static void vehicle_control_angle_test_cancel(void);

/**
 * @brief Advance angle-loop step test sequence.
 * @param[in] dt_s Control period, unit: s.
 * @return void
 */
static void vehicle_control_angle_test_update(float32 dt_s);

/**
 * @brief Capture one angle-loop test print frame at limited rate.
 * @param[in] void No parameter.
 * @return void
 */
static void vehicle_control_angle_test_print_capture(void);
static void vehicle_control_angle_test_print_reset(void);
static void vehicle_control_angle_test_print_emit(
    const vehicle_control_angle_test_print_frame_t* frame);

static void vehicle_control_command_queue_reset(void);
static boolean vehicle_control_command_post_checked(const vehicle_control_command_t* command);
static boolean vehicle_control_command_pop(vehicle_control_command_t* command);
static void vehicle_control_command_execute(const vehicle_control_command_t* command);

/**
 * @brief Compare a command argument with a constant ASCII word.
 * @param[in] text Command argument.
 * @param[in] word Expected word.
 * @return TRUE if identical, otherwise FALSE.
 */
static boolean vehicle_control_command_word_is(const uint8* text, const char* word);

/**
 * @brief Incremental PID for yaw-rate loop without output or integral clamp.
 * @param[in] cfg PID configuration.
 * @param[in,out] pid PID runtime state.
 * @param[in] target Target yaw rate, unit: rad/s.
 * @param[in] feedback Measured yaw rate, unit: rad/s.
 * @param[in] dt_s Control period, unit: s.
 * @return Yaw differential speed correction, unit: mm/s.
 */
/**
 * @brief Position PID for wheel speed.
 * @param[in] cfg PID configuration.
 * @param[in,out] pid PID runtime state.
 * @param[in] target Target wheel speed, unit: mm/s.
 * @param[in] feedback Measured wheel speed, unit: mm/s.
 * @param[in] dt_s Control period, unit: s.
 * @return PID output, unit: GTM PWM ticks.
 */
static float32 vehicle_control_speed_position_pid_update(const algorithm_control_pid_cfg_t* cfg,
                                                        algorithm_control_pid_state_t* pid,
                                                        float32 target,
                                                        float32 feedback,
                                                        float32 dt_s,
                                                        boolean integral_enable);

/**
 * @brief 将串口命令参数字符串解析为浮点数。
 * @param[in] text 命令参数字符串指针。
 * @return 解析得到的浮点数。
 */
static float32 vehicle_control_command_float_get(const uint8* text);

/**
 * @brief 判断串口命令参数是否表示零速度。
 * @param[in] text 命令参数字符串指针。
 * @return 近似为零返回 TRUE，否则返回 FALSE。
 */
static boolean vehicle_control_command_is_zero(const uint8* text);
static void vehicle_control_status_help_print(void);
static void vehicle_control_status_check_print(void);

/**
 * @brief 按打印分频捕获一次速度环测试目标、反馈和输出占空命令。
 * @param[in] encoder_observation 编码器速度观测指针，速度单位：毫米/秒。
 * @return void
 */
static void vehicle_control_speed_test_print_capture(const module_vehicle_encoder_observation_t* encoder_observation);
static void vehicle_control_speed_test_print_reset(void);
static void vehicle_control_speed_test_core_print_emit(
    const vehicle_control_speed_test_print_frame_t* frame);
static void vehicle_control_speed_test_encoder_print_emit(
    const vehicle_control_speed_test_print_frame_t* frame);
static void vehicle_control_speed_test_diag_print_emit(
    const vehicle_control_speed_test_print_frame_t* frame);

/**
 * @brief 初始化车体控制模块状态和默认输出。
 * @param[in] void 无参数。
 * @return void
 */
void vehicle_control_init(void)
{
    const vehicle_control_cfg_t* cfg = vehicle_control_cfg_get();

    vehicle_control_angle_pid_cfg = cfg->angle_pid;
    vehicle_control_launch_angle_pid_cfg = vehicle_control_angle_pid_cfg;
    vehicle_control_left_speed_pid_cfg = cfg->left_speed_pid;
    vehicle_control_right_speed_pid_cfg = cfg->right_speed_pid;
    vehicle_control_speed_test_left_pid_cfg = vehicle_control_left_speed_pid_cfg;
    vehicle_control_speed_test_right_pid_cfg = vehicle_control_right_speed_pid_cfg;
    vehicle_control_angle_2dof_enabled = TRUE;
    vehicle_control_angle_kp2 = cfg->angle_kp2;
    vehicle_control_target_yaw_rate_feedforward_gain_mm =
        cfg->target_yaw_rate_feedforward_gain_mm;
    vehicle_control_target_yaw_rate_lpf_cutoff_hz =
        cfg->target_yaw_rate_lpf_cutoff_hz;
    vehicle_control_target_yaw_rate_limit_rad_s =
        cfg->target_yaw_rate_limit_rad_s;
    vehicle_control_speed_accel_feedforward_gain = cfg->speed_accel_feedforward_gain;
    vehicle_control_speed_decel_feedforward_gain = cfg->speed_decel_feedforward_gain;
    vehicle_control_speed_accel_feedforward_limit = cfg->speed_accel_feedforward_limit;
    vehicle_control_state.mode = VEHICLE_CONTROL_MODE_IDLE;
    vehicle_control_state.enabled = FALSE;
    vehicle_control_state.replay_target_valid = FALSE;
    vehicle_control_state.replay_delay_cnt = 0u;
    vehicle_control_state.target_speed_mm_s = 0.0f;
    vehicle_control_state.target_theta_rad = 0.0f;
    vehicle_control_state.target_left_speed_mm_s = 0.0f;
    vehicle_control_state.target_right_speed_mm_s = 0.0f;
    vehicle_control_pid_reset_all();
    vehicle_control_speed_target_runtime_reset();
    vehicle_control_speed_accel_feedforward_reset();
    vehicle_control_state.yaw_speed_correction_mm_s = 0.0f;
    vehicle_control_yaw_feedback_correction_mm_s = 0.0f;
    vehicle_control_effective_yaw_speed_correction_mm_s = 0.0f;
    vehicle_control_state.left_duty = module_vehicle_esc_clamp_duty(VEHICLE_ESC_ROLE_LEFT_DRIVE, cfg->drive_idle_duty);
    vehicle_control_state.right_duty = module_vehicle_esc_clamp_duty(VEHICLE_ESC_ROLE_RIGHT_DRIVE, cfg->drive_idle_duty);
    vehicle_control_state.suction_duty = module_vehicle_esc_clamp_duty(VEHICLE_ESC_ROLE_SUCTION, cfg->suction_idle_duty);
    vehicle_control_left_output_duty = vehicle_control_state.left_duty;
    vehicle_control_right_output_duty = vehicle_control_state.right_duty;
    vehicle_control_suction_output_duty = vehicle_control_state.suction_duty;
    vehicle_control_speed_test_enabled = FALSE;
    vehicle_control_speed_test_stop_requested = FALSE;
    vehicle_control_speed_test_heading_enabled = FALSE;
    vehicle_control_speed_test_left_target_mm_s = 0.0f;
    vehicle_control_speed_test_right_target_mm_s = 0.0f;
    vehicle_control_speed_test_print_reset();
    vehicle_control_speed_feedforward_table_reset();
    vehicle_control_tracking_limit_left_before_mm_s = 0.0f;
    vehicle_control_tracking_limit_right_before_mm_s = 0.0f;
    vehicle_control_tracking_limit_left_after_mm_s = 0.0f;
    vehicle_control_tracking_limit_right_after_mm_s = 0.0f;
    vehicle_control_tracking_limit_active = FALSE;
    vehicle_control_tracking_limit_event_count = 0u;
    vehicle_control_load_test_cancel();
    vehicle_control_angle_test_cancel();
    vehicle_control_steer_loop_divider = 0u;
    vehicle_control_command_queue_reset();
}

boolean vehicle_control_command_post(const vehicle_control_command_t* command)
{
    uint8 tail;
    uint8 next_tail;

    if ((command == NULL_PTR) || (command->type == VEHICLE_CONTROL_COMMAND_NONE))
    {
        return FALSE;
    }

    tail = vehicle_control_command_queue.tail;
    next_tail = tail + 1u;
    if (next_tail >= VEHICLE_CONTROL_COMMAND_QUEUE_LENGTH)
    {
        next_tail = 0u;
    }

    if (next_tail == vehicle_control_command_queue.head)
    {
        vehicle_control_command_queue.dropped_count++;
        return FALSE;
    }

    vehicle_control_command_queue.buffer[tail] = *command;
    vehicle_control_command_queue.tail = next_tail;
    return TRUE;
}

void vehicle_control_command_process(void)
{
    vehicle_control_command_t command;

    while (vehicle_control_command_pop(&command) != FALSE)
    {
        vehicle_control_command_execute(&command);
    }
}

/**
 * @brief 使能或关闭车体闭环控制。
 * @param[in] enable TRUE 表示使能控制，FALSE 表示关闭控制。
 * @return void
 */
void vehicle_control_enable(boolean enable)
{
    const vehicle_control_cfg_t* cfg = vehicle_control_cfg_get();

    vehicle_control_state.enabled = enable;

    if (enable == FALSE)
    {
        vehicle_control_pid_reset_all();
        vehicle_control_state.mode = VEHICLE_CONTROL_MODE_IDLE;
        vehicle_control_state.replay_target_valid = FALSE;
        vehicle_control_state.replay_delay_cnt = 0u;
        vehicle_control_state.target_speed_mm_s = 0.0f;
        vehicle_control_state.target_left_speed_mm_s = 0.0f;
        vehicle_control_state.target_right_speed_mm_s = 0.0f;
        vehicle_control_speed_test_enabled = FALSE;
        vehicle_control_speed_test_stop_requested = FALSE;
        vehicle_control_speed_test_left_target_mm_s = 0.0f;
        vehicle_control_speed_test_right_target_mm_s = 0.0f;
        vehicle_control_speed_target_runtime_reset();
        vehicle_control_replay_launch_gate_stop();
        vehicle_control_test_launch_gate_stop();
        module_vehicle_path_replay_lookahead_speed_cap_set(0.0f);
        vehicle_control_state.yaw_speed_correction_mm_s = 0.0f;
        vehicle_control_yaw_feedback_correction_mm_s = 0.0f;
        vehicle_control_effective_yaw_speed_correction_mm_s = 0.0f;
        vehicle_control_state.left_duty = module_vehicle_esc_clamp_duty(VEHICLE_ESC_ROLE_LEFT_DRIVE, cfg->drive_idle_duty);
        vehicle_control_state.right_duty = module_vehicle_esc_clamp_duty(VEHICLE_ESC_ROLE_RIGHT_DRIVE, cfg->drive_idle_duty);
        vehicle_control_state.suction_duty = module_vehicle_esc_clamp_duty(VEHICLE_ESC_ROLE_SUCTION, cfg->suction_idle_duty);
        module_vehicle_path_replay_stop();
        module_vehicle_path_stop();
        vehicle_control_load_test_cancel();
        vehicle_control_angle_test_cancel();
        vehicle_control_apply_idle_outputs();
    }
    else
    {
        vehicle_control_state.mode = VEHICLE_CONTROL_MODE_MANUAL;
        vehicle_control_state.target_theta_rad = module_vehicle_pose_fusion_heading_get();
        vehicle_control_apply_outputs();
    }
}

/**
 * @brief 推进车体控制模式和路径记录回放状态机。
 * @param[in] void 无参数。
 * @return void
 */
void vehicle_control_process(void)
{
    if (vehicle_control_state.enabled == FALSE)
    {
        return;
    }

    if (vehicle_control_state.mode == VEHICLE_CONTROL_MODE_REPLAY)
    {
        vehicle_control_replay_update_target();
    }

    if (vehicle_control_state.mode == VEHICLE_CONTROL_MODE_RECORD)
    {
        (void)module_vehicle_path_update();
    }
}

/**
 * @brief 推进位置外环并得到目标角速度。
 * @param[in] dt_s 控制周期，单位：秒。
 * @return 目标角速度，单位：弧度/秒。
 */
float32 vehicle_control_angle_loop_update(float32 dt_s)
{
    if ((vehicle_control_state.enabled == FALSE) || (dt_s <= 0.0f))
    {
        return vehicle_control_state.yaw_speed_correction_mm_s;
    }

    return vehicle_control_angle_correction_update(dt_s);
}

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
void vehicle_control_speed_loop_update(float32 dt_s)
{
    const vehicle_control_cfg_t* cfg = vehicle_control_cfg_get();
    const module_vehicle_encoder_observation_t* encoder_observation = module_vehicle_encoder_observation_get();
    float32 correction_mm_s;
    float32 correction_limit_mm_s;
    float32 desired_base_mm_s;
    float32 base_target_mm_s;
    float32 left_target_mm_s;
    float32 right_target_mm_s;
    float32 left_feedback_output;
    float32 right_feedback_output;
    float32 left_feedforward_output;
    float32 right_feedforward_output;
    float32 acceleration_feedforward_output;
    float32 left_output;
    float32 right_output;
    sint32 left_target_duty;
    sint32 right_target_duty;
    boolean left_integral_hard_enable;
    boolean right_integral_hard_enable;
    boolean left_integral_enable;
    boolean right_integral_enable;

    if ((vehicle_control_state.enabled == FALSE) || (dt_s <= 0.0f))
    {
        return;
    }

    if (vehicle_control_replay_launch_drag_active() != FALSE)
    {
        float32 drag_correction_ratio;
        float32 left_drag_feedforward;
        float32 right_drag_feedforward;
        float32 left_drag_target_mm_s;
        float32 right_drag_target_mm_s;
        sint32 drag_yaw_duty;
        sint32 left_drag_duty;
        sint32 right_drag_duty;

        vehicle_control_speed_integral_clear();
        drag_correction_ratio = (vehicle_control_launch_drag_correction_enabled != FALSE)
            ? algorithm_control_clamp_f32(
                  vehicle_control_state.yaw_speed_correction_mm_s
                      / VEHICLE_CONTROL_REPLAY_LAUNCH_CORRECTION_LIMIT_MM_S,
                  -1.0f,
                  1.0f)
            : 0.0f;
        drag_yaw_duty = (sint32)(drag_correction_ratio
                                 * (float32)VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_YAW_DUTY_LIMIT_TICK);
        left_drag_feedforward = (float32)VEHICLE_CONTROL_LAUNCH_DRAG_DUTY_TICK;
        right_drag_feedforward = (float32)VEHICLE_CONTROL_LAUNCH_DRAG_DUTY_TICK;
        if (vehicle_control_launch_drag_correction_enabled != FALSE)
        {
            left_drag_target_mm_s = algorithm_control_clamp_f32(
                VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_SPEED_MM_S
                    - vehicle_control_state.yaw_speed_correction_mm_s,
                0.0f,
                cfg->target_speed_max_mm_s);
            right_drag_target_mm_s = algorithm_control_clamp_f32(
                VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_SPEED_MM_S
                    + vehicle_control_state.yaw_speed_correction_mm_s,
                0.0f,
                cfg->target_speed_max_mm_s);
        }
        else
        {
            left_drag_target_mm_s = 0.0f;
            right_drag_target_mm_s = 0.0f;
            vehicle_control_state.yaw_speed_correction_mm_s = 0.0f;
            vehicle_control_yaw_feedback_correction_mm_s = 0.0f;
            vehicle_control_effective_yaw_speed_correction_mm_s = 0.0f;
        }
        left_drag_duty = vehicle_control_duty_from_delta(
            cfg->drive_idle_duty,
            left_drag_feedforward) - drag_yaw_duty;
        right_drag_duty = vehicle_control_duty_from_delta(
            cfg->drive_idle_duty,
            right_drag_feedforward) + drag_yaw_duty;

        vehicle_control_state.target_left_speed_mm_s = left_drag_target_mm_s;
        vehicle_control_state.target_right_speed_mm_s = right_drag_target_mm_s;
        vehicle_control_left_effective_target_mm_s = left_drag_target_mm_s;
        vehicle_control_right_effective_target_mm_s = right_drag_target_mm_s;
        vehicle_control_base_effective_target_mm_s =
            (vehicle_control_launch_drag_correction_enabled != FALSE)
                ? VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_SPEED_MM_S
                : 0.0f;
        vehicle_control_effective_yaw_speed_correction_mm_s =
            (vehicle_control_launch_drag_correction_enabled != FALSE)
                ? vehicle_control_state.yaw_speed_correction_mm_s
                : 0.0f;
        vehicle_control_left_speed_feedback_output = 0.0f;
        vehicle_control_right_speed_feedback_output = 0.0f;
        vehicle_control_left_speed_feedforward_output = left_drag_feedforward;
        vehicle_control_right_speed_feedforward_output = right_drag_feedforward;
        vehicle_control_left_speed_assist_output = 0.0f;
        vehicle_control_right_speed_assist_output = 0.0f;
        vehicle_control_left_speed_total_output =
            left_drag_feedforward - (float32)drag_yaw_duty;
        vehicle_control_right_speed_total_output =
            right_drag_feedforward + (float32)drag_yaw_duty;
        vehicle_control_left_speed_integral_enabled = FALSE;
        vehicle_control_right_speed_integral_enabled = FALSE;
        vehicle_control_state.left_duty = vehicle_control_drive_clamp_duty(
            VEHICLE_ESC_ROLE_LEFT_DRIVE,
            left_drag_duty);
        vehicle_control_state.right_duty = vehicle_control_drive_clamp_duty(
            VEHICLE_ESC_ROLE_RIGHT_DRIVE,
            right_drag_duty);
        return;
    }

    desired_base_mm_s = vehicle_control_state.target_speed_mm_s;
    if ((vehicle_control_state.mode == VEHICLE_CONTROL_MODE_REPLAY)
        && (vehicle_control_reverse_feedback_cap_active != FALSE)
        && (desired_base_mm_s > VEHICLE_CONTROL_REVERSE_GUARD_BASE_CAP_MM_S))
    {
        desired_base_mm_s = VEHICLE_CONTROL_REVERSE_GUARD_BASE_CAP_MM_S;
    }

    /* Slew only the common base speed, then apply yaw split so steering is not swallowed at launch. */
    base_target_mm_s =
        vehicle_control_base_target_update(desired_base_mm_s, dt_s);

    correction_mm_s = algorithm_control_clamp_f32(vehicle_control_state.yaw_speed_correction_mm_s,
                                                  cfg->yaw_speed_correction_min_mm_s,
                                                  cfg->yaw_speed_correction_max_mm_s);
    if (vehicle_control_speed_protection_enabled != FALSE)
    {
        correction_mm_s = algorithm_control_clamp_f32(correction_mm_s,
                                                      -(vehicle_control_angle_diff_limit_mm_s * 0.5f),
                                                      vehicle_control_angle_diff_limit_mm_s * 0.5f);
    }
    if ((vehicle_control_state.mode == VEHICLE_CONTROL_MODE_REPLAY)
        && (vehicle_control_replay_launch_drag_active() != FALSE)
        && (base_target_mm_s >= 0.0f))
    {
        correction_limit_mm_s = base_target_mm_s;
        correction_mm_s = algorithm_control_clamp_f32(correction_mm_s,
                                                      -correction_limit_mm_s,
                                                      correction_limit_mm_s);
    }
    else if (vehicle_control_launch_low_speed_active() != FALSE)
    {
        correction_mm_s = algorithm_control_clamp_f32(
            correction_mm_s,
            -VEHICLE_CONTROL_REPLAY_LAUNCH_CORRECTION_LIMIT_MM_S,
            VEHICLE_CONTROL_REPLAY_LAUNCH_CORRECTION_LIMIT_MM_S);
    }
    else if (base_target_mm_s > VEHICLE_CONTROL_ANGLE_CORRECTION_MIN_WHEEL_SPEED_MM_S)
    {
        correction_limit_mm_s = vehicle_control_yaw_correction_available_get(base_target_mm_s);
        correction_mm_s = algorithm_control_clamp_f32(correction_mm_s,
                                                      -correction_limit_mm_s,
                                                      correction_limit_mm_s);
    }
    else
    {
        correction_mm_s = 0.0f;
    }
    left_target_mm_s = base_target_mm_s - correction_mm_s;
    right_target_mm_s = base_target_mm_s + correction_mm_s;
    vehicle_control_wheel_target_accel_limit_apply(&left_target_mm_s,
                                                   &right_target_mm_s,
                                                   encoder_observation->left_speed_mm_s,
                                                   encoder_observation->right_speed_mm_s,
                                                   dt_s);
    vehicle_control_wheel_target_tracking_limit_apply(&left_target_mm_s,
                                                      &right_target_mm_s,
                                                      encoder_observation->left_speed_mm_s,
                                                      encoder_observation->right_speed_mm_s);
    left_target_mm_s =
        vehicle_control_speed_no_reverse_target(algorithm_control_clamp_f32(left_target_mm_s,
                                                                            cfg->target_speed_min_mm_s,
                                                                            cfg->target_speed_max_mm_s));
    right_target_mm_s =
        vehicle_control_speed_no_reverse_target(algorithm_control_clamp_f32(right_target_mm_s,
                                                                             cfg->target_speed_min_mm_s,
                                                                             cfg->target_speed_max_mm_s));
    correction_mm_s = 0.5f * (right_target_mm_s - left_target_mm_s);
    vehicle_control_effective_yaw_speed_correction_mm_s = correction_mm_s;
    vehicle_control_base_effective_target_mm_s = 0.5f * (left_target_mm_s + right_target_mm_s);
    vehicle_control_reverse_feedback_guard_update(left_target_mm_s,
                                                  right_target_mm_s,
                                                  encoder_observation->left_speed_mm_s,
                                                  encoder_observation->right_speed_mm_s,
                                                  dt_s);
    vehicle_control_left_effective_target_mm_s = left_target_mm_s;
    vehicle_control_right_effective_target_mm_s = right_target_mm_s;
    vehicle_control_start_integral_hold_update(left_target_mm_s, right_target_mm_s, dt_s);
    left_integral_hard_enable =
        (module_vehicle_esc_role_closed_loop(VEHICLE_ESC_ROLE_LEFT_DRIVE)
         && (vehicle_control_start_integral_hold_active() == FALSE)
         && (vehicle_control_base_start_accel_active == FALSE)
         && (vehicle_control_reverse_feedback_cap_active == FALSE)
         && (vehicle_control_replay_launch_drag_active() == FALSE)
         && (vehicle_control_replay_launch_wheel_wait_active() == FALSE)
         && (vehicle_control_test_launch_drag_active() == FALSE));
    left_integral_enable =
        (left_integral_hard_enable != FALSE)
        && (vehicle_control_speed_integral_ready(left_target_mm_s,
                                                encoder_observation->left_speed_mm_s,
                                                vehicle_control_state.left_speed_pid.target,
                                                dt_s) != FALSE);
    right_integral_hard_enable =
        (module_vehicle_esc_role_closed_loop(VEHICLE_ESC_ROLE_RIGHT_DRIVE)
         && (vehicle_control_start_integral_hold_active() == FALSE)
         && (vehicle_control_base_start_accel_active == FALSE)
         && (vehicle_control_reverse_feedback_cap_active == FALSE)
         && (vehicle_control_replay_launch_drag_active() == FALSE)
         && (vehicle_control_replay_launch_wheel_wait_active() == FALSE)
         && (vehicle_control_test_launch_drag_active() == FALSE));
    right_integral_enable =
        (right_integral_hard_enable != FALSE)
        && (vehicle_control_speed_integral_ready(right_target_mm_s,
                                                encoder_observation->right_speed_mm_s,
                                                vehicle_control_state.right_speed_pid.target,
                                                dt_s) != FALSE);
    if (left_integral_hard_enable == FALSE)
    {
        vehicle_control_state.left_speed_pid.integral = 0.0f;
    }
    if (right_integral_hard_enable == FALSE)
    {
        vehicle_control_state.right_speed_pid.integral = 0.0f;
    }

    if (vehicle_control_replay_launch_fixed_feedforward_active() != FALSE)
    {
        vehicle_control_speed_integral_clear();
        left_feedback_output = 0.0f;
        right_feedback_output = 0.0f;
        left_feedforward_output = vehicle_control_speed_feedforward_calculate(
            vehicle_control_left_speed_feedforward_table,
            VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_SPEED_MM_S);
        right_feedforward_output = vehicle_control_speed_feedforward_calculate(
            vehicle_control_right_speed_feedforward_table,
            VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_SPEED_MM_S);
    }
    else
    {
        left_feedback_output = vehicle_control_speed_position_pid_update(
            &vehicle_control_left_speed_pid_cfg,
            &vehicle_control_state.left_speed_pid,
            left_target_mm_s,
            encoder_observation->left_speed_mm_s,
            dt_s,
            left_integral_enable);
        right_feedback_output = vehicle_control_speed_position_pid_update(
            &vehicle_control_right_speed_pid_cfg,
            &vehicle_control_state.right_speed_pid,
            right_target_mm_s,
            encoder_observation->right_speed_mm_s,
            dt_s,
            right_integral_enable);
        left_feedforward_output = vehicle_control_speed_feedforward_calculate(
            vehicle_control_left_speed_feedforward_table,
            left_target_mm_s);
        right_feedforward_output = vehicle_control_speed_feedforward_calculate(
            vehicle_control_right_speed_feedforward_table,
            right_target_mm_s);
    }
    if ((vehicle_control_launch_drag_active() != FALSE)
        && (vehicle_control_replay_launch_fixed_feedforward_active() == FALSE))
    {
        left_feedforward_output *= VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_FEEDFORWARD_SCALE;
        right_feedforward_output *= VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_FEEDFORWARD_SCALE;
    }
    acceleration_feedforward_output =
        (vehicle_control_replay_launch_fixed_feedforward_active() != FALSE)
            ? 0.0f
            : vehicle_control_speed_accel_feedforward_update(
                  vehicle_control_base_effective_target_mm_s,
                  dt_s);
    vehicle_control_left_speed_pid_p_output =
        vehicle_control_left_speed_pid_cfg.kp * vehicle_control_state.left_speed_pid.error;
    vehicle_control_right_speed_pid_p_output =
        vehicle_control_right_speed_pid_cfg.kp * vehicle_control_state.right_speed_pid.error;
    vehicle_control_left_speed_pid_i_output =
        vehicle_control_left_speed_pid_cfg.ki * vehicle_control_state.left_speed_pid.integral;
    vehicle_control_right_speed_pid_i_output =
        vehicle_control_right_speed_pid_cfg.ki * vehicle_control_state.right_speed_pid.integral;
    vehicle_control_left_speed_pid_d_output =
        (dt_s > 0.0f)
        ? (vehicle_control_left_speed_pid_cfg.kd
           * ((vehicle_control_state.left_speed_pid.error
               - vehicle_control_state.left_speed_pid.last_error) / dt_s))
        : 0.0f;
    vehicle_control_right_speed_pid_d_output =
        (dt_s > 0.0f)
        ? (vehicle_control_right_speed_pid_cfg.kd
           * ((vehicle_control_state.right_speed_pid.error
               - vehicle_control_state.right_speed_pid.last_error) / dt_s))
        : 0.0f;
    vehicle_control_left_speed_feedback_output = left_feedback_output;
    vehicle_control_right_speed_feedback_output = right_feedback_output;
    vehicle_control_left_speed_feedforward_output = left_feedforward_output;
    vehicle_control_right_speed_feedforward_output = right_feedforward_output;
    vehicle_control_left_speed_assist_output =
        (vehicle_control_replay_launch_fixed_feedforward_active() != FALSE)
            ? 0.0f
            : vehicle_control_drive_start_assist(left_target_mm_s,
                                                 encoder_observation->left_speed_mm_s);
    vehicle_control_right_speed_assist_output =
        (vehicle_control_replay_launch_fixed_feedforward_active() != FALSE)
            ? 0.0f
            : vehicle_control_drive_start_assist(right_target_mm_s,
                                                 encoder_observation->right_speed_mm_s);
    vehicle_control_left_speed_integral_enabled = left_integral_enable;
    vehicle_control_right_speed_integral_enabled = right_integral_enable;
    left_output = left_feedforward_output
                  + acceleration_feedforward_output
                  + left_feedback_output
                  + vehicle_control_left_speed_assist_output;
    right_output = right_feedforward_output
                   + acceleration_feedforward_output
                   + right_feedback_output
                   + vehicle_control_right_speed_assist_output;
    vehicle_control_left_speed_total_output = left_output;
    vehicle_control_right_speed_total_output = right_output;

    vehicle_control_state.target_left_speed_mm_s = left_target_mm_s;
    vehicle_control_state.target_right_speed_mm_s = right_target_mm_s;
    left_target_duty = vehicle_control_drive_clamp_duty(VEHICLE_ESC_ROLE_LEFT_DRIVE,
                                                         vehicle_control_duty_from_delta(cfg->drive_idle_duty,
                                                                                         left_output));
    right_target_duty = vehicle_control_drive_clamp_duty(VEHICLE_ESC_ROLE_RIGHT_DRIVE,
                                                          vehicle_control_duty_from_delta(cfg->drive_idle_duty,
                                                                                          right_output));
    left_target_duty = vehicle_control_drive_keepalive_duty(left_target_duty, left_target_mm_s);
    right_target_duty = vehicle_control_drive_keepalive_duty(right_target_duty, right_target_mm_s);
    vehicle_control_state.left_duty = left_target_duty;
    vehicle_control_state.right_duty = right_target_duty;
}

void vehicle_control_closed_loop_update(float32 dt_s)
{
    float32 steer_dt_s;

    if ((vehicle_control_state.enabled == FALSE)
        || (vehicle_control_speed_test_enabled != FALSE)
        || (dt_s <= 0.0f))
    {
        return;
    }

    vehicle_control_angle_test_update(dt_s);
    vehicle_control_steer_loop_divider++;

    if (vehicle_control_steer_loop_divider >= VEHICLE_CONTROL_STEER_LOOP_DIVIDER)
    {
        vehicle_control_steer_loop_divider = 0u;
        vehicle_control_process();
        steer_dt_s = dt_s * (float32)VEHICLE_CONTROL_STEER_LOOP_DIVIDER;
        (void)vehicle_control_angle_loop_update(steer_dt_s);
    }

    vehicle_control_speed_loop_update(dt_s);
    vehicle_control_accel_slip_update(dt_s);
    vehicle_control_turn_slip_event_capture();
    vehicle_control_angle_test_print_capture();
    vehicle_control_apply_outputs();
}

/**
 * @brief 设置车体目标直线速度。
 * @param[in] speed_mm_s 目标直线速度，单位：毫米/秒。
 * @return void
 */
void vehicle_control_set_speed(float32 speed_mm_s)
{
    const vehicle_control_cfg_t* cfg = vehicle_control_cfg_get();

    vehicle_control_state.target_speed_mm_s =
        vehicle_control_speed_no_reverse_target(algorithm_control_clamp_f32(speed_mm_s,
                                                                            cfg->target_speed_min_mm_s,
                                                                            cfg->target_speed_max_mm_s));
}

/**
 * @brief 设置车体目标航向角。
 * @param[in] theta_rad 目标航向角，单位：弧度。
 * @return void
 */
void vehicle_control_set_theta(float32 theta_rad)
{
    vehicle_control_state.target_theta_rad = algorithm_attitude_wrap_pi(theta_rad);
}

/**
 * @brief 使能或关闭速度环单独测试模式。
 * @param[in] enable TRUE 表示使能速度环测试，FALSE 表示关闭速度环测试。
 * @return void
 */
void vehicle_control_speed_test_enable(boolean enable)
{
    boolean next_enabled = (enable != FALSE) ? TRUE : FALSE;
    boolean already_enabled = vehicle_control_speed_test_enabled;

    if ((next_enabled != FALSE) && (already_enabled != FALSE))
    {
        return;
    }

    vehicle_control_speed_test_enabled = next_enabled;
    vehicle_control_speed_test_stop_requested = FALSE;
    if (already_enabled != next_enabled)
    {
        vehicle_control_speed_test_print_reset();
    }
    algorithm_control_pid_reset(&vehicle_control_state.left_speed_pid);
    algorithm_control_pid_reset(&vehicle_control_state.right_speed_pid);
    vehicle_control_state.yaw_speed_correction_mm_s = 0.0f;
    vehicle_control_yaw_feedback_correction_mm_s = 0.0f;
    vehicle_control_effective_yaw_speed_correction_mm_s = 0.0f;
    vehicle_control_d_yaw_rate_lpf_reset();
    vehicle_control_target_yaw_rate_reset();
    vehicle_control_steer_loop_divider = 0u;
    vehicle_control_speed_target_runtime_reset();
    vehicle_control_replay_launch_gate_stop();
    vehicle_control_test_launch_gate_stop();

    if (next_enabled != FALSE)
    {
        vehicle_control_angle_test_cancel();
        vehicle_control_angle_test_print_reset();
        vehicle_control_state.target_theta_rad = module_vehicle_pose_fusion_heading_get();
        if (vehicle_control_test_launch_pair_requested() != FALSE)
        {
            vehicle_control_test_launch_gate_start(vehicle_control_state.target_theta_rad);
        }
    }

    if (next_enabled == FALSE)
    {
        vehicle_control_speed_test_heading_enabled = FALSE;
        vehicle_control_load_test_cancel();
        vehicle_control_speed_test_left_target_mm_s = 0.0f;
        vehicle_control_speed_test_right_target_mm_s = 0.0f;
        vehicle_control_state.target_left_speed_mm_s = 0.0f;
        vehicle_control_state.target_right_speed_mm_s = 0.0f;
        vehicle_control_state.left_duty = module_vehicle_esc_clamp_duty(VEHICLE_ESC_ROLE_LEFT_DRIVE, 0u);
        vehicle_control_state.right_duty = module_vehicle_esc_clamp_duty(VEHICLE_ESC_ROLE_RIGHT_DRIVE, 0u);
        vehicle_control_state.suction_duty = module_vehicle_esc_clamp_duty(VEHICLE_ESC_ROLE_SUCTION, 0u);
        vehicle_control_apply_idle_outputs();
    }
}

/**
 * @brief 获取速度环单独测试模式是否已使能。
 * @param[in] void 无参数。
 * @return 已使能返回 TRUE，否则返回 FALSE。
 */
boolean vehicle_control_speed_test_is_enabled(void)
{
    return vehicle_control_speed_test_enabled;
}

boolean vehicle_control_test_launch_active_get(void)
{
    return vehicle_control_test_launch_gate_active;
}

/**
 * @brief 设置速度环测试模式下左右轮目标速度。
 * @param[in] left_speed_mm_s 左轮目标速度，单位：毫米/秒。
 * @param[in] right_speed_mm_s 右轮目标速度，单位：毫米/秒。
 * @return void
 */
void vehicle_control_speed_test_set_target(float32 left_speed_mm_s, float32 right_speed_mm_s)
{
    const vehicle_control_cfg_t* cfg = vehicle_control_cfg_get();
    boolean moving_before = vehicle_control_test_launch_moving_requested();
    boolean moving_after;

    vehicle_control_speed_test_left_target_mm_s =
        vehicle_control_speed_no_reverse_target(algorithm_control_clamp_f32(left_speed_mm_s,
                                                                            cfg->target_speed_min_mm_s,
                                                                            cfg->target_speed_max_mm_s));
    vehicle_control_speed_test_right_target_mm_s =
        vehicle_control_speed_no_reverse_target(algorithm_control_clamp_f32(right_speed_mm_s,
                                                                            cfg->target_speed_min_mm_s,
                                                                            cfg->target_speed_max_mm_s));
    moving_after = vehicle_control_test_launch_moving_requested();
    if ((vehicle_control_speed_test_left_target_mm_s > 0.5f)
        || (vehicle_control_speed_test_left_target_mm_s < -0.5f)
        || (vehicle_control_speed_test_right_target_mm_s > 0.5f)
        || (vehicle_control_speed_test_right_target_mm_s < -0.5f))
    {
        vehicle_control_speed_test_stop_requested = FALSE;
    }
    if ((moving_after == FALSE)
        || (vehicle_control_test_launch_pair_requested() == FALSE))
    {
        vehicle_control_test_launch_gate_stop();
    }
    else if ((vehicle_control_speed_test_enabled != FALSE) && (moving_before == FALSE))
    {
        if (vehicle_control_load_test_state.step != VEHICLE_CONTROL_LOAD_TEST_STEP_IDLE)
        {
            vehicle_control_test_launch_gate_stop();
        }
        else
        {
            vehicle_control_test_launch_gate_start(module_vehicle_pose_fusion_heading_get());
        }
    }
}

/**
 * @brief 设置速度环测试模式下左右轮共用的 PID 参数并清空速度环状态。
 * @param[in] kp 比例系数，单位：PWM ticks/(毫米/秒)。
 * @param[in] ki 积分系数，单位：PWM ticks/(毫米/秒)/秒。
 * @param[in] kd 微分系数，单位：PWM ticks/(毫米/秒)*秒。
 * @return void
 */
void vehicle_control_speed_test_set_pid(float32 kp, float32 ki, float32 kd)
{
    vehicle_control_left_speed_pid_cfg.kp = kp;
    vehicle_control_left_speed_pid_cfg.ki = ki;
    vehicle_control_left_speed_pid_cfg.kd = kd;
    vehicle_control_right_speed_pid_cfg.kp = kp;
    vehicle_control_right_speed_pid_cfg.ki = ki;
    vehicle_control_right_speed_pid_cfg.kd = kd;
    vehicle_control_speed_test_left_pid_cfg.kp = kp;
    vehicle_control_speed_test_left_pid_cfg.ki = ki;
    vehicle_control_speed_test_left_pid_cfg.kd = kd;
    vehicle_control_speed_test_right_pid_cfg.kp = kp;
    vehicle_control_speed_test_right_pid_cfg.ki = ki;
    vehicle_control_speed_test_right_pid_cfg.kd = kd;
    algorithm_control_pid_reset(&vehicle_control_state.left_speed_pid);
    algorithm_control_pid_reset(&vehicle_control_state.right_speed_pid);
}

/**
 * @brief 推进一次速度环测试闭环并立即输出左右驱动电调占空命令。
 * @param[in] dt_s 本次速度采样和 PID 更新周期，单位：秒。
 * @return void
 */
void vehicle_control_speed_test_update(float32 dt_s)
{
    const vehicle_control_cfg_t* cfg = vehicle_control_cfg_get();
    const module_vehicle_encoder_observation_t* encoder_observation = module_vehicle_encoder_observation_get();
    float32 desired_left_mm_s;
    float32 desired_right_mm_s;
    float32 left_target_mm_s;
    float32 right_target_mm_s;
    float32 correction_mm_s;
    float32 correction_limit_mm_s;
    float32 steer_dt_s;
    float32 left_feedback_output;
    float32 right_feedback_output;
    float32 left_feedforward_output;
    float32 right_feedforward_output;
    float32 acceleration_feedforward_output;
    float32 left_output;
    float32 right_output;
    float32 drag_correction_ratio;
    sint32 left_target_duty;
    sint32 right_target_duty;
    sint32 drag_yaw_duty;
    boolean left_integral_hard_enable;
    boolean right_integral_hard_enable;
    boolean left_integral_enable;
    boolean right_integral_enable;

    if ((vehicle_control_speed_test_enabled == FALSE) || (dt_s <= 0.0f))
    {
        return;
    }

    vehicle_control_load_test_update(dt_s);

    if (vehicle_control_speed_test_enabled == FALSE)
    {
        return;
    }

    if (vehicle_control_test_launch_gate_active != FALSE)
    {
        (void)vehicle_control_test_launch_gate_update(dt_s);
    }

    if (((vehicle_control_test_launch_gate_active != FALSE)
         && ((vehicle_control_test_launch_drag_active() == FALSE)
             || (vehicle_control_launch_drag_correction_enabled != FALSE)))
        || (vehicle_control_speed_test_heading_enabled != FALSE))
    {
        vehicle_control_state.target_theta_rad = vehicle_control_test_launch_reference_theta_rad;
        vehicle_control_steer_loop_divider++;
        if (vehicle_control_steer_loop_divider >= VEHICLE_CONTROL_STEER_LOOP_DIVIDER)
        {
            vehicle_control_steer_loop_divider = 0u;
            steer_dt_s = dt_s * (float32)VEHICLE_CONTROL_STEER_LOOP_DIVIDER;
            (void)vehicle_control_angle_correction_update(steer_dt_s);
        }
    }
    else
    {
        vehicle_control_state.yaw_speed_correction_mm_s = 0.0f;
        vehicle_control_yaw_feedback_correction_mm_s = 0.0f;
        vehicle_control_effective_yaw_speed_correction_mm_s = 0.0f;
        algorithm_control_pid_reset(&vehicle_control_state.angle_pid);
        vehicle_control_steer_loop_divider = 0u;
    }

    if ((vehicle_control_load_test_state.step == VEHICLE_CONTROL_LOAD_TEST_STEP_DRAG)
        || (vehicle_control_test_launch_drag_active() != FALSE))
    {
        vehicle_control_state.target_left_speed_mm_s = 0.0f;
        vehicle_control_state.target_right_speed_mm_s = 0.0f;
        vehicle_control_left_effective_target_mm_s = 0.0f;
        vehicle_control_right_effective_target_mm_s = 0.0f;
        vehicle_control_base_effective_target_mm_s = 0.0f;
        drag_correction_ratio = ((vehicle_control_load_test_state.step
                                  != VEHICLE_CONTROL_LOAD_TEST_STEP_DRAG)
                                 && (vehicle_control_launch_drag_correction_enabled != FALSE))
            ? algorithm_control_clamp_f32(
                  vehicle_control_state.yaw_speed_correction_mm_s
                      / VEHICLE_CONTROL_REPLAY_LAUNCH_CORRECTION_LIMIT_MM_S,
                  -1.0f,
                  1.0f)
            : 0.0f;
        drag_yaw_duty = (sint32)(drag_correction_ratio
                                 * (float32)VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_YAW_DUTY_LIMIT_TICK);
        vehicle_control_effective_yaw_speed_correction_mm_s =
            vehicle_control_state.yaw_speed_correction_mm_s;
        vehicle_control_left_speed_feedback_output = 0.0f;
        vehicle_control_right_speed_feedback_output = 0.0f;
        vehicle_control_left_speed_feedforward_output = 0.0f;
        vehicle_control_right_speed_feedforward_output = 0.0f;
        vehicle_control_left_speed_pid_p_output = 0.0f;
        vehicle_control_right_speed_pid_p_output = 0.0f;
        vehicle_control_left_speed_pid_i_output = 0.0f;
        vehicle_control_right_speed_pid_i_output = 0.0f;
        vehicle_control_left_speed_pid_d_output = 0.0f;
        vehicle_control_right_speed_pid_d_output = 0.0f;
        vehicle_control_left_speed_total_output =
            (float32)(VEHICLE_CONTROL_LAUNCH_DRAG_DUTY_TICK - drag_yaw_duty);
        vehicle_control_right_speed_total_output =
            (float32)(VEHICLE_CONTROL_LAUNCH_DRAG_DUTY_TICK + drag_yaw_duty);
        vehicle_control_left_speed_integral_enabled = FALSE;
        vehicle_control_right_speed_integral_enabled = FALSE;
        vehicle_control_state.left_duty = vehicle_control_drive_clamp_duty(
            VEHICLE_ESC_ROLE_LEFT_DRIVE,
            VEHICLE_CONTROL_LAUNCH_DRAG_DUTY_TICK - drag_yaw_duty);
        vehicle_control_state.right_duty = vehicle_control_drive_clamp_duty(
            VEHICLE_ESC_ROLE_RIGHT_DRIVE,
            VEHICLE_CONTROL_LAUNCH_DRAG_DUTY_TICK + drag_yaw_duty);
        vehicle_control_apply_outputs();
        vehicle_control_speed_test_print_capture(encoder_observation);
        return;
    }

    if (vehicle_control_test_launch_gate_active != FALSE)
    {
        if (vehicle_control_test_launch_drag_active() != FALSE)
        {
            desired_left_mm_s = VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_SPEED_MM_S;
            desired_right_mm_s = VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_SPEED_MM_S;
        }
        else
        {
            desired_left_mm_s = VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S;
            desired_right_mm_s = VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S;
        }
    }
    else
    {
        desired_left_mm_s = vehicle_control_speed_test_left_target_mm_s;
        desired_right_mm_s = vehicle_control_speed_test_right_target_mm_s;
    }

    left_target_mm_s = desired_left_mm_s;
    right_target_mm_s = desired_right_mm_s;

    correction_mm_s = algorithm_control_clamp_f32(vehicle_control_state.yaw_speed_correction_mm_s,
                                                  cfg->yaw_speed_correction_min_mm_s,
                                                  cfg->yaw_speed_correction_max_mm_s);
    if (vehicle_control_speed_protection_enabled != FALSE)
    {
        correction_mm_s = algorithm_control_clamp_f32(correction_mm_s,
                                                      -(vehicle_control_angle_diff_limit_mm_s * 0.5f),
                                                      vehicle_control_angle_diff_limit_mm_s * 0.5f);
    }
    if ((vehicle_control_test_launch_drag_active() != FALSE)
        && (left_target_mm_s >= 0.0f)
        && (right_target_mm_s >= 0.0f))
    {
        correction_limit_mm_s = left_target_mm_s;
        if (right_target_mm_s < correction_limit_mm_s)
        {
            correction_limit_mm_s = right_target_mm_s;
        }
        correction_mm_s = algorithm_control_clamp_f32(correction_mm_s,
                                                      -correction_limit_mm_s,
                                                      correction_limit_mm_s);
    }
    else if (vehicle_control_launch_low_speed_active() != FALSE)
    {
        correction_mm_s = algorithm_control_clamp_f32(
            correction_mm_s,
            -VEHICLE_CONTROL_REPLAY_LAUNCH_CORRECTION_LIMIT_MM_S,
            VEHICLE_CONTROL_REPLAY_LAUNCH_CORRECTION_LIMIT_MM_S);
    }
    else if (((vehicle_control_test_launch_gate_active != FALSE)
              || (vehicle_control_speed_test_heading_enabled != FALSE))
             && (left_target_mm_s > VEHICLE_CONTROL_ANGLE_CORRECTION_MIN_WHEEL_SPEED_MM_S)
             && (right_target_mm_s > VEHICLE_CONTROL_ANGLE_CORRECTION_MIN_WHEEL_SPEED_MM_S))
    {
        correction_limit_mm_s = left_target_mm_s;
        if (right_target_mm_s < correction_limit_mm_s)
        {
            correction_limit_mm_s = right_target_mm_s;
        }
        correction_limit_mm_s = vehicle_control_yaw_correction_available_get(correction_limit_mm_s);
        correction_mm_s = algorithm_control_clamp_f32(correction_mm_s,
                                                      -correction_limit_mm_s,
                                                      correction_limit_mm_s);
    }
    else
    {
        correction_mm_s = 0.0f;
    }
    left_target_mm_s -= correction_mm_s;
    right_target_mm_s += correction_mm_s;
    vehicle_control_wheel_target_accel_limit_apply(&left_target_mm_s,
                                                   &right_target_mm_s,
                                                   encoder_observation->left_speed_mm_s,
                                                   encoder_observation->right_speed_mm_s,
                                                   dt_s);
    if (vehicle_control_load_test_state.step != VEHICLE_CONTROL_LOAD_TEST_STEP_IDLE)
    {
        vehicle_control_wheel_target_tracking_limit_apply(&left_target_mm_s,
                                                          &right_target_mm_s,
                                                          encoder_observation->left_speed_mm_s,
                                                          encoder_observation->right_speed_mm_s);
    }
    if ((vehicle_control_test_launch_gate_active != FALSE)
        || (vehicle_control_speed_test_heading_enabled != FALSE))
    {
        left_target_mm_s =
            vehicle_control_speed_no_reverse_target(algorithm_control_clamp_f32(
                left_target_mm_s,
                cfg->target_speed_min_mm_s,
                cfg->target_speed_max_mm_s));
        right_target_mm_s =
            vehicle_control_speed_no_reverse_target(algorithm_control_clamp_f32(
                right_target_mm_s,
                cfg->target_speed_min_mm_s,
                cfg->target_speed_max_mm_s));
        vehicle_control_reverse_feedback_guard_update(
            left_target_mm_s,
            right_target_mm_s,
            encoder_observation->left_speed_mm_s,
            encoder_observation->right_speed_mm_s,
            dt_s);
    }
    vehicle_control_effective_yaw_speed_correction_mm_s =
        0.5f * (right_target_mm_s - left_target_mm_s);
    vehicle_control_base_effective_target_mm_s =
        0.5f * (left_target_mm_s + right_target_mm_s);
    vehicle_control_left_effective_target_mm_s = left_target_mm_s;
    vehicle_control_right_effective_target_mm_s = right_target_mm_s;

    vehicle_control_start_integral_hold_update(left_target_mm_s, right_target_mm_s, dt_s);
    left_integral_hard_enable =
        (module_vehicle_esc_role_closed_loop(VEHICLE_ESC_ROLE_LEFT_DRIVE)
         && ((vehicle_control_load_test_state.step != VEHICLE_CONTROL_LOAD_TEST_STEP_IDLE)
             || (vehicle_control_start_integral_hold_active() == FALSE))
         && (vehicle_control_left_start_accel_active == FALSE)
         && (vehicle_control_test_launch_drag_active() == FALSE));
    left_integral_enable =
        (left_integral_hard_enable != FALSE)
        && (vehicle_control_speed_integral_ready(left_target_mm_s,
                                                encoder_observation->left_speed_mm_s,
                                                vehicle_control_state.left_speed_pid.target,
                                                dt_s) != FALSE);
    right_integral_hard_enable =
        (module_vehicle_esc_role_closed_loop(VEHICLE_ESC_ROLE_RIGHT_DRIVE)
         && ((vehicle_control_load_test_state.step != VEHICLE_CONTROL_LOAD_TEST_STEP_IDLE)
             || (vehicle_control_start_integral_hold_active() == FALSE))
         && (vehicle_control_right_start_accel_active == FALSE)
         && (vehicle_control_test_launch_drag_active() == FALSE));
    right_integral_enable =
        (right_integral_hard_enable != FALSE)
        && (vehicle_control_speed_integral_ready(right_target_mm_s,
                                                encoder_observation->right_speed_mm_s,
                                                vehicle_control_state.right_speed_pid.target,
                                                dt_s) != FALSE);
    if (left_integral_hard_enable == FALSE)
    {
        vehicle_control_state.left_speed_pid.integral = 0.0f;
    }
    if (right_integral_hard_enable == FALSE)
    {
        vehicle_control_state.right_speed_pid.integral = 0.0f;
    }

    if ((left_target_mm_s > 0.5f)
        || (left_target_mm_s < -0.5f))
    {
        left_feedback_output =
            vehicle_control_speed_position_pid_update(&vehicle_control_speed_test_left_pid_cfg,
                                                      &vehicle_control_state.left_speed_pid,
                                                      left_target_mm_s,
                                                      encoder_observation->left_speed_mm_s,
                                                      dt_s,
                                                      left_integral_enable);
        left_feedforward_output =
            vehicle_control_speed_feedforward_calculate(vehicle_control_left_speed_feedforward_table,
                                                        left_target_mm_s);
    }
    else
    {
        algorithm_control_pid_reset(&vehicle_control_state.left_speed_pid);
        left_feedback_output = 0.0f;
        left_feedforward_output = 0.0f;
    }

    if ((right_target_mm_s > 0.5f)
        || (right_target_mm_s < -0.5f))
    {
        right_feedback_output =
            vehicle_control_speed_position_pid_update(&vehicle_control_speed_test_right_pid_cfg,
                                                      &vehicle_control_state.right_speed_pid,
                                                      right_target_mm_s,
                                                      encoder_observation->right_speed_mm_s,
                                                      dt_s,
                                                      right_integral_enable);
        right_feedforward_output =
            vehicle_control_speed_feedforward_calculate(vehicle_control_right_speed_feedforward_table,
                                                        right_target_mm_s);
    }
    else
    {
        algorithm_control_pid_reset(&vehicle_control_state.right_speed_pid);
        right_feedback_output = 0.0f;
        right_feedforward_output = 0.0f;
    }

    if (vehicle_control_launch_drag_active() != FALSE)
    {
        left_feedforward_output *= VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_FEEDFORWARD_SCALE;
        right_feedforward_output *= VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_FEEDFORWARD_SCALE;
    }

    if ((vehicle_control_load_test_state.step != VEHICLE_CONTROL_LOAD_TEST_STEP_IDLE)
        && (vehicle_control_load_test_feedforward_enabled == FALSE))
    {
        left_feedforward_output = 0.0f;
        right_feedforward_output = 0.0f;
    }

    acceleration_feedforward_output =
        (vehicle_control_load_test_state.step != VEHICLE_CONTROL_LOAD_TEST_STEP_IDLE)
            ? 0.0f
            : vehicle_control_speed_accel_feedforward_update(
                  vehicle_control_base_effective_target_mm_s,
                  dt_s);

    left_output = left_feedforward_output
                  + (((left_target_mm_s > 0.5f) || (left_target_mm_s < -0.5f))
                         ? acceleration_feedforward_output
                         : 0.0f)
                  + left_feedback_output
                  + vehicle_control_drive_start_assist(left_target_mm_s,
                                                       encoder_observation->left_speed_mm_s);
    right_output = right_feedforward_output
                   + (((right_target_mm_s > 0.5f) || (right_target_mm_s < -0.5f))
                          ? acceleration_feedforward_output
                          : 0.0f)
                   + right_feedback_output
                   + vehicle_control_drive_start_assist(right_target_mm_s,
                                                        encoder_observation->right_speed_mm_s);

    vehicle_control_state.target_left_speed_mm_s = left_target_mm_s;
    vehicle_control_state.target_right_speed_mm_s = right_target_mm_s;
    left_target_duty = vehicle_control_drive_clamp_duty(VEHICLE_ESC_ROLE_LEFT_DRIVE,
                                                         vehicle_control_duty_from_delta(0u,
                                                                                         left_output));
    right_target_duty = vehicle_control_drive_clamp_duty(VEHICLE_ESC_ROLE_RIGHT_DRIVE,
                                                          vehicle_control_duty_from_delta(0u,
                                                                                          right_output));
    left_target_duty = vehicle_control_drive_keepalive_duty(left_target_duty, left_target_mm_s);
    right_target_duty = vehicle_control_drive_keepalive_duty(right_target_duty, right_target_mm_s);
    vehicle_control_state.left_duty = left_target_duty;
    vehicle_control_state.right_duty = right_target_duty;
    vehicle_control_apply_outputs();
    vehicle_control_speed_test_print_capture(encoder_observation);

    if ((vehicle_control_speed_test_stop_requested != FALSE)
        && (left_target_mm_s > -0.5f)
        && (left_target_mm_s < 0.5f)
        && (right_target_mm_s > -0.5f)
        && (right_target_mm_s < 0.5f))
    {
        vehicle_control_speed_test_enable(FALSE);
    }
}

/**
 * @brief 将速度环测试缓存帧按 VOFA/FireWater 文本格式输出。
 * @param[in] void 无参数。
 * @return void
 */
static void vehicle_control_speed_test_print_reset(void)
{
    vehicle_control_speed_test_core_print_divider = 0u;
    vehicle_control_speed_test_encoder_print_divider = 0u;
    vehicle_control_speed_test_diag_print_divider = 0u;
    vehicle_control_speed_test_core_print_live_pending = FALSE;
    vehicle_control_speed_test_encoder_print_live_pending = FALSE;
    vehicle_control_speed_test_diag_print_live_pending = FALSE;
    vehicle_control_speed_test_capture_last_tick = 0u;
    vehicle_control_speed_test_capture_last_tick_valid = FALSE;
}

static void vehicle_control_speed_test_core_print_emit(
    const vehicle_control_speed_test_print_frame_t* frame)
{
    tools_printf("{vspd}%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%.3f,%.3f,%.3f,%.3f\r\n",
                 (double)frame->left_target_mm_s,
                 (double)frame->right_target_mm_s,
                 (double)frame->left_speed_mm_s,
                 (double)frame->right_speed_mm_s,
                 (double)frame->left_integral,
                 (double)frame->right_integral,
                 (int)frame->left_duty,
                 (int)frame->right_duty,
                 (double)frame->sample_dt_ms,
                 (double)frame->target_theta_deg,
                 (double)frame->current_theta_deg,
                 (double)frame->yaw_speed_correction_mm_s);
}

static void vehicle_control_speed_test_encoder_print_emit(
    const vehicle_control_speed_test_print_frame_t* frame)
{
    tools_printf("{vspdenc}%u,%u,%d,%d,%d,%d,%.0f,%.0f\r\n",
                 (unsigned int)frame->left_raw_angle,
                 (unsigned int)frame->right_raw_angle,
                 (int)frame->left_raw_delta_count,
                 (int)frame->right_raw_delta_count,
                 (int)frame->left_delta_count,
                 (int)frame->right_delta_count,
                 (double)frame->left_raw_speed_mm_s,
                 (double)frame->right_raw_speed_mm_s);
}

static void vehicle_control_speed_test_diag_print_emit(
    const vehicle_control_speed_test_print_frame_t* frame)
{
    tools_printf("{vspddiag}%u,%u,%u,%u,%.3f,%.3f,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u,%u\r\n",
                 (unsigned int)frame->left_esc_state,
                 (unsigned int)frame->left_esc_fault,
                 (unsigned int)frame->right_esc_state,
                 (unsigned int)frame->right_esc_fault,
                 (double)frame->left_sample_dt_ms,
                 (double)frame->right_sample_dt_ms,
                 (unsigned long)frame->left_sample_count,
                 (unsigned long)frame->right_sample_count,
                 (unsigned long)frame->encoder_drop_count,
                 (unsigned long)frame->left_invalid_sample_count,
                 (unsigned long)frame->right_invalid_sample_count,
                 (unsigned long)frame->left_invalid_streak_count,
                 (unsigned long)frame->right_invalid_streak_count,
                 (unsigned int)frame->left_rejected_raw_angle,
                 (unsigned int)frame->right_rejected_raw_angle);
}

void vehicle_control_speed_test_print_process(void)
{
    vehicle_control_speed_test_print_frame_t frame;

    if (vehicle_control_speed_test_core_print_live_pending != FALSE)
    {
        vehicle_control_speed_test_core_print_live_pending = FALSE;
        frame = vehicle_control_speed_test_print_live_frame;
        vehicle_control_speed_test_core_print_emit(&frame);
    }

    if (vehicle_control_speed_test_encoder_print_live_pending != FALSE)
    {
        vehicle_control_speed_test_encoder_print_live_pending = FALSE;
        frame = vehicle_control_speed_test_print_live_frame;
        vehicle_control_speed_test_encoder_print_emit(&frame);
    }

    if (vehicle_control_speed_test_diag_print_live_pending != FALSE)
    {
        vehicle_control_speed_test_diag_print_live_pending = FALSE;
        frame = vehicle_control_speed_test_print_live_frame;
        vehicle_control_speed_test_diag_print_emit(&frame);
    }
}

static void vehicle_control_angle_test_print_reset(void)
{
    vehicle_control_angle_test_print_divider = 0u;
    vehicle_control_angle_test_print_live_pending = FALSE;
    vehicle_control_angle_test_capture_last_tick = 0u;
    vehicle_control_angle_test_capture_last_tick_valid = FALSE;
}

static void vehicle_control_angle_test_print_emit(
    const vehicle_control_angle_test_print_frame_t* frame)
{
    tools_printf("{vang}%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%u,%u,%.3f,%u,%u,%u,%u\r\n",
                 (double)frame->target_theta_deg,
                 (double)frame->current_theta_deg,
                 (double)frame->yaw_speed_correction_mm_s,
                 (double)frame->left_target_mm_s,
                 (double)frame->right_target_mm_s,
                 (int)frame->left_duty,
                 (int)frame->right_duty,
                 (unsigned int)frame->suction_duty,
                 (unsigned int)frame->step,
                 (double)frame->sample_dt_ms,
                 (unsigned int)frame->left_esc_state,
                 (unsigned int)frame->left_esc_fault,
                 (unsigned int)frame->right_esc_state,
                 (unsigned int)frame->right_esc_fault);
}

void vehicle_control_angle_test_print_process(void)
{
    vehicle_control_angle_test_print_frame_t frame;

    if (vehicle_control_angle_test_print_live_pending == FALSE)
    {
        return;
    }

    frame = vehicle_control_angle_test_print_live_frame;
    vehicle_control_angle_test_print_live_pending = FALSE;
    vehicle_control_angle_test_print_emit(&frame);
}

void vehicle_control_suction_command(uint8 argc, uint8* argv[])
{
    vehicle_control_command_t command;
    uint32 duty;

    if (argc < 2u)
    {
        return;
    }

    command.type = VEHICLE_CONTROL_COMMAND_SET_SUCTION;
    command.enable = FALSE;
    command.speed_mm_s = 0.0f;
    command.theta_rad = 0.0f;
    command.left_speed_mm_s = 0.0f;
    command.right_speed_mm_s = 0.0f;
    command.step_deg = 0.0f;
    command.left_duty = 0;
    command.right_duty = 0;

    if ((vehicle_control_command_word_is(argv[1], "off") != FALSE)
        || (vehicle_control_command_is_zero(argv[1]) != FALSE))
    {
        command.suction_duty = 0u;
        (void)vehicle_control_command_post_checked(&command);
        tools_printf("{vsuc}off,0\r\n");
        return;
    }

    if (vehicle_control_command_word_is(argv[1], "on") != FALSE)
    {
        duty = VEHICLE_CONTROL_LOAD_TEST_SUCTION_DUTY;
        command.suction_duty = duty;
        (void)vehicle_control_command_post_checked(&command);
        tools_printf("{vsuc}%u\r\n", (unsigned int)duty);
        return;
    }

    if ((argc >= 3u) && (vehicle_control_command_word_is(argv[1], "pct") != FALSE))
    {
        duty = vehicle_control_suction_duty_from_percent(vehicle_control_command_float_get(argv[2]));
    }
    else
    {
        duty = (uint32)(vehicle_control_command_float_get(argv[1]) + 0.5f);
        duty = module_vehicle_esc_clamp_duty(VEHICLE_ESC_ROLE_SUCTION, duty);
    }

    command.suction_duty = duty;
    (void)vehicle_control_command_post_checked(&command);
    tools_printf("{vsuc}%u\r\n", (unsigned int)duty);
}

void vehicle_control_music_command(uint8 argc, uint8* argv[])
{
    uint32 song_id;
    uint32 index;

    if ((argc < 2u)
        || (vehicle_control_command_word_is(argv[1], "list") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "status") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "s") != FALSE))
    {
        for (index = 0u; index < VEHICLE_CONTROL_MUSIC_COUNT; index++)
        {
            tools_printf("{vmusic}list,%u,%s\r\n",
                         (unsigned int)index,
                         vehicle_control_music_name_table[index]);
        }
        return;
    }

    if ((vehicle_control_command_word_is(argv[1], "stop") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "off") != FALSE))
    {
        module_vehicle_esc_stop_music();
        tools_printf("{vmusic}stop\r\n");
        return;
    }

    song_id = (uint32)(vehicle_control_command_float_get(argv[1]) + 0.5f);
    if (song_id >= VEHICLE_CONTROL_MUSIC_COUNT)
    {
        tools_printf("{vmusic}err,%u\r\n", (unsigned int)song_id);
        return;
    }

    module_vehicle_esc_play_music((uint8)song_id);
    tools_printf("{vmusic}play,%u,%s\r\n",
                 (unsigned int)song_id,
                 vehicle_control_music_name_table[song_id]);
}

boolean vehicle_control_auto_suction_enabled_get(void)
{
    return vehicle_control_auto_suction_enabled;
}

void vehicle_control_auto_suction_command(uint8 argc, uint8* argv[])
{
    if (argc < 2u)
    {
        tools_printf("{vautosuc}enable,%u\r\n",
                     (unsigned int)((vehicle_control_auto_suction_enabled != FALSE) ? 1u : 0u));
        return;
    }

    if ((vehicle_control_command_word_is(argv[1], "on") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "enable") != FALSE))
    {
        vehicle_control_auto_suction_enabled = TRUE;
        tools_printf("{vautosuc}enable,1\r\n");
        return;
    }

    if ((vehicle_control_command_word_is(argv[1], "off") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "disable") != FALSE)
        || (vehicle_control_command_is_zero(argv[1]) != FALSE))
    {
        vehicle_control_auto_suction_enabled = FALSE;
        tools_printf("{vautosuc}enable,0\r\n");
        return;
    }

    if ((vehicle_control_command_word_is(argv[1], "status") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "s") != FALSE))
    {
        tools_printf("{vautosuc}enable,%u\r\n",
                     (unsigned int)((vehicle_control_auto_suction_enabled != FALSE) ? 1u : 0u));
    }
}

void vehicle_control_load_test_command(uint8 argc, uint8* argv[])
{
    vehicle_control_command_t command;
    float32 left_speed_mm_s;
    float32 right_speed_mm_s;

    if (argc < 2u)
    {
        tools_printf("{vload}ff,%u,accff,0\r\n",
                     (unsigned int)((vehicle_control_load_test_feedforward_enabled != FALSE) ? 1u : 0u));
        return;
    }

    if (vehicle_control_command_word_is(argv[1], "ff") != FALSE)
    {
        if ((argc < 3u)
            || (vehicle_control_command_word_is(argv[2], "status") != FALSE)
            || (vehicle_control_command_word_is(argv[2], "s") != FALSE))
        {
            tools_printf("{vload}ff,%u,accff,0\r\n",
                         (unsigned int)((vehicle_control_load_test_feedforward_enabled != FALSE) ? 1u : 0u));
            return;
        }

        if (vehicle_control_load_test_state.step != VEHICLE_CONTROL_LOAD_TEST_STEP_IDLE)
        {
            tools_printf("{vload}ff,busy,next_test_unchanged\r\n");
            return;
        }

        if ((vehicle_control_command_word_is(argv[2], "on") != FALSE)
            || (vehicle_control_command_word_is(argv[2], "enable") != FALSE))
        {
            vehicle_control_load_test_feedforward_enabled = TRUE;
            tools_printf("{vload}ff,1,accff,0\r\n");
            return;
        }

        if ((vehicle_control_command_word_is(argv[2], "off") != FALSE)
            || (vehicle_control_command_word_is(argv[2], "disable") != FALSE)
            || (vehicle_control_command_is_zero(argv[2]) != FALSE))
        {
            vehicle_control_load_test_feedforward_enabled = FALSE;
            tools_printf("{vload}ff,0,accff,0\r\n");
            return;
        }

        tools_printf("{vload}usage,ff on|off|status\r\n");
        return;
    }

    command.type = VEHICLE_CONTROL_COMMAND_LOAD_TEST_STOP;
    command.enable = FALSE;
    command.speed_mm_s = 0.0f;
    command.theta_rad = 0.0f;
    command.left_speed_mm_s = 0.0f;
    command.right_speed_mm_s = 0.0f;
    command.step_deg = 0.0f;
    command.left_duty = 0;
    command.right_duty = 0;
    command.suction_duty = 0u;

    if ((vehicle_control_command_word_is(argv[1], "off") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "stop") != FALSE)
        || (vehicle_control_command_is_zero(argv[1]) != FALSE))
    {
        (void)vehicle_control_command_post_checked(&command);
        tools_printf("{vload}off\r\n");
        return;
    }

    left_speed_mm_s = vehicle_control_command_float_get(argv[1]);
    right_speed_mm_s = (argc >= 3u) ? vehicle_control_command_float_get(argv[2]) : left_speed_mm_s;
    command.type = VEHICLE_CONTROL_COMMAND_LOAD_TEST_START;
    command.enable = TRUE;
    command.left_speed_mm_s = left_speed_mm_s;
    command.right_speed_mm_s = right_speed_mm_s;
    (void)vehicle_control_command_post_checked(&command);
    tools_printf("{vload}start,%.3f,%.3f\r\n", (double)left_speed_mm_s, (double)right_speed_mm_s);
}

void vehicle_control_angle_test_command(uint8 argc, uint8* argv[])
{
    vehicle_control_command_t command;
    float32 step_deg;
    float32 speed_mm_s = 0.0f;
    float32 current_heading_rad;

    if (argc < 2u)
    {
        return;
    }

    command.type = VEHICLE_CONTROL_COMMAND_ANGLE_TEST_STOP;
    command.enable = FALSE;
    command.speed_mm_s = 0.0f;
    command.theta_rad = 0.0f;
    command.left_speed_mm_s = 0.0f;
    command.right_speed_mm_s = 0.0f;
    command.step_deg = 0.0f;
    command.left_duty = 0;
    command.right_duty = 0;
    command.suction_duty = 0u;

    if ((vehicle_control_command_word_is(argv[1], "off") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "stop") != FALSE))
    {
        (void)vehicle_control_command_post_checked(&command);
        tools_printf("{vang}off\r\n");
        return;
    }

    step_deg = vehicle_control_command_float_get(argv[1]);
    if (argc >= 3u)
    {
        speed_mm_s = vehicle_control_command_float_get(argv[2]);
    }
    current_heading_rad = module_vehicle_pose_fusion_heading_get();

    command.type = VEHICLE_CONTROL_COMMAND_ANGLE_TEST_START;
    command.enable = TRUE;
    command.speed_mm_s = speed_mm_s;
    command.theta_rad = current_heading_rad;
    command.step_deg = step_deg;
    if (vehicle_control_auto_suction_enabled != FALSE)
    {
        command.suction_duty = VEHICLE_CONTROL_ANGLE_TEST_SUCTION_DUTY;
    }
    else
    {
        command.suction_duty = 0u;
    }
    (void)vehicle_control_command_post_checked(&command);
    tools_printf("{vang}start,%.3f,%.3f\r\n", (double)step_deg, (double)speed_mm_s);
}

/**
 * @brief 解析速度环测试目标速度串口命令。
 * @param[in] argc 命令参数个数，单位：个。
 * @param[in] argv 命令参数字符串指针数组。
 * @return void
 */
void vehicle_control_turn_command(uint8 argc, uint8* argv[])
{
    vehicle_control_command_t command;
    float32 step_deg;
    float32 current_heading_rad;
    float32 target_heading_rad;

    if (argc < 2u)
    {
        return;
    }

    command.type = VEHICLE_CONTROL_COMMAND_TURN_STOP;
    command.enable = FALSE;
    command.speed_mm_s = 0.0f;
    command.theta_rad = 0.0f;
    command.left_speed_mm_s = 0.0f;
    command.right_speed_mm_s = 0.0f;
    command.step_deg = 0.0f;
    command.left_duty = 0;
    command.right_duty = 0;
    command.suction_duty = 0u;

    if ((vehicle_control_command_word_is(argv[1], "off") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "stop") != FALSE))
    {
        (void)vehicle_control_command_post_checked(&command);
        tools_printf("{vturn}off\r\n");
        return;
    }

    step_deg = vehicle_control_command_float_get(argv[1]);
    current_heading_rad = module_vehicle_pose_fusion_heading_get();
    target_heading_rad = algorithm_attitude_wrap_pi(current_heading_rad
                                                    + (step_deg * VEHICLE_CONTROL_DEG_TO_RAD));

    command.type = VEHICLE_CONTROL_COMMAND_TURN_START;
    command.enable = TRUE;
    command.theta_rad = target_heading_rad;
    command.step_deg = step_deg;
    (void)vehicle_control_command_post_checked(&command);

    tools_printf("{vturn}start,%.3f,%.3f,%.3f\r\n",
                 (double)step_deg,
                 (double)(current_heading_rad / VEHICLE_CONTROL_DEG_TO_RAD),
                 (double)(target_heading_rad / VEHICLE_CONTROL_DEG_TO_RAD));
}

void vehicle_control_speed_test_target_command(uint8 argc, uint8* argv[])
{
    vehicle_control_command_t command;

    if (argc < 2u)
    {
        return;
    }

    command.type = VEHICLE_CONTROL_COMMAND_SPEED_TEST_TARGET;
    command.enable = TRUE;
    command.speed_mm_s = 0.0f;
    command.theta_rad = 0.0f;
    command.left_speed_mm_s = 0.0f;
    command.right_speed_mm_s = 0.0f;
    command.step_deg = 0.0f;
    command.left_duty = 0;
    command.right_duty = 0;
    command.suction_duty = 0u;

    if (((argc == 2u) && (vehicle_control_command_is_zero(argv[1]) != FALSE))
        || ((argc >= 3u)
            && (vehicle_control_command_is_zero(argv[1]) != FALSE)
            && (vehicle_control_command_is_zero(argv[2]) != FALSE)))
    {
        command.type = VEHICLE_CONTROL_COMMAND_SPEED_TEST_STOP;
        command.enable = FALSE;
        (void)vehicle_control_command_post_checked(&command);
        return;
    }

    if (argc >= 3u)
    {
        command.left_speed_mm_s = vehicle_control_command_float_get(argv[1]);
        command.right_speed_mm_s = vehicle_control_command_float_get(argv[2]);
    }
    else
    {
        float32 speed_mm_s = vehicle_control_command_float_get(argv[1]);

        command.left_speed_mm_s = speed_mm_s;
        command.right_speed_mm_s = speed_mm_s;
    }

    (void)vehicle_control_command_post_checked(&command);
}

void vehicle_control_drive_pwm_command(uint8 argc, uint8* argv[])
{
    vehicle_control_command_t command;

    if ((argc < 2u)
        || (vehicle_control_command_word_is(argv[1], "status") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "s") != FALSE))
    {
        tools_printf("{vpwm}%d,%d\r\n",
                     (int)vehicle_control_state.left_duty,
                     (int)vehicle_control_state.right_duty);
        return;
    }

    command.type = VEHICLE_CONTROL_COMMAND_DRIVE_PWM;
    command.enable = FALSE;
    command.speed_mm_s = 0.0f;
    command.theta_rad = 0.0f;
    command.left_speed_mm_s = 0.0f;
    command.right_speed_mm_s = 0.0f;
    command.step_deg = 0.0f;
    command.suction_duty = 0u;

    if ((vehicle_control_command_word_is(argv[1], "off") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "stop") != FALSE))
    {
        command.left_duty = 0;
        command.right_duty = 0;
    }
    else if (argc >= 3u)
    {
        command.left_duty = (sint32)vehicle_control_command_float_get(argv[1]);
        command.right_duty = (sint32)vehicle_control_command_float_get(argv[2]);
    }
    else
    {
        command.left_duty = (sint32)vehicle_control_command_float_get(argv[1]);
        command.right_duty = command.left_duty;
    }

    command.left_duty = vehicle_control_drive_clamp_duty(VEHICLE_ESC_ROLE_LEFT_DRIVE,
                                                          command.left_duty);
    command.right_duty = vehicle_control_drive_clamp_duty(VEHICLE_ESC_ROLE_RIGHT_DRIVE,
                                                           command.right_duty);
    if (vehicle_control_command_post_checked(&command) != FALSE)
    {
        tools_printf("{vpwm}set,%d,%d\r\n",
                     (int)command.left_duty,
                     (int)command.right_duty);
    }
}

/**
 * @brief 解析速度环测试 PID 参数串口命令。
 * @param[in] argc 命令参数个数，单位：个。
 * @param[in] argv 命令参数字符串指针数组。
 * @return void
 */
static void vehicle_control_speed_pid_status_print(void)
{
    tools_printf("{vpid}left,%.3f,%.3f,%.3f,right,%.3f,%.3f,%.3f\r\n",
                 (double)vehicle_control_left_speed_pid_cfg.kp,
                 (double)vehicle_control_left_speed_pid_cfg.ki,
                 (double)vehicle_control_left_speed_pid_cfg.kd,
                 (double)vehicle_control_right_speed_pid_cfg.kp,
                 (double)vehicle_control_right_speed_pid_cfg.ki,
                 (double)vehicle_control_right_speed_pid_cfg.kd);
}

void vehicle_control_speed_test_pid_command(uint8 argc, uint8* argv[])
{
    if ((argc < 2u)
        || (vehicle_control_command_word_is(argv[1], "status") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "s") != FALSE))
    {
        vehicle_control_speed_pid_status_print();
        return;
    }

    if ((argv[1][0] == (uint8)'l') || (argv[1][0] == (uint8)'L'))
    {
        if (argc < 5u)
        {
            tools_printf("{vpid}usage:vpid <kp> <ki> <kd>|l <kp> <ki> <kd>|r <kp> <ki> <kd>|status\r\n");
            return;
        }
        vehicle_control_left_speed_pid_cfg.kp = vehicle_control_command_float_get(argv[2]);
        vehicle_control_left_speed_pid_cfg.ki = vehicle_control_command_float_get(argv[3]);
        vehicle_control_left_speed_pid_cfg.kd = vehicle_control_command_float_get(argv[4]);
        vehicle_control_speed_test_left_pid_cfg.kp = vehicle_control_command_float_get(argv[2]);
        vehicle_control_speed_test_left_pid_cfg.ki = vehicle_control_command_float_get(argv[3]);
        vehicle_control_speed_test_left_pid_cfg.kd = vehicle_control_command_float_get(argv[4]);
        algorithm_control_pid_reset(&vehicle_control_state.left_speed_pid);
    }
    else if ((argv[1][0] == (uint8)'r') || (argv[1][0] == (uint8)'R'))
    {
        if (argc < 5u)
        {
            tools_printf("{vpid}usage:vpid <kp> <ki> <kd>|l <kp> <ki> <kd>|r <kp> <ki> <kd>|status\r\n");
            return;
        }
        vehicle_control_right_speed_pid_cfg.kp = vehicle_control_command_float_get(argv[2]);
        vehicle_control_right_speed_pid_cfg.ki = vehicle_control_command_float_get(argv[3]);
        vehicle_control_right_speed_pid_cfg.kd = vehicle_control_command_float_get(argv[4]);
        vehicle_control_speed_test_right_pid_cfg.kp = vehicle_control_command_float_get(argv[2]);
        vehicle_control_speed_test_right_pid_cfg.ki = vehicle_control_command_float_get(argv[3]);
        vehicle_control_speed_test_right_pid_cfg.kd = vehicle_control_command_float_get(argv[4]);
        algorithm_control_pid_reset(&vehicle_control_state.right_speed_pid);
    }
    else
    {
        if (argc < 4u)
        {
            tools_printf("{vpid}usage:vpid <kp> <ki> <kd>|l <kp> <ki> <kd>|r <kp> <ki> <kd>|status\r\n");
            return;
        }
        vehicle_control_speed_test_set_pid(vehicle_control_command_float_get(argv[1]),
                                           vehicle_control_command_float_get(argv[2]),
                                           vehicle_control_command_float_get(argv[3]));
    }
    vehicle_control_speed_pid_status_print();
}

void vehicle_control_speed_accel_feedforward_command(uint8 argc, uint8* argv[])
{
    float32 value;

    if ((argc < 2u)
        || (vehicle_control_command_word_is(argv[1], "status") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "s") != FALSE))
    {
        tools_printf("{vaff}accel_gain,%.4f,decel_gain,%.4f,limit_pwm,%.1f,output,%.1f\r\n",
                     (double)vehicle_control_speed_accel_feedforward_gain,
                     (double)vehicle_control_speed_decel_feedforward_gain,
                     (double)vehicle_control_speed_accel_feedforward_limit,
                     (double)vehicle_control_speed_accel_feedforward_output);
        return;
    }

    if (argc < 4u)
    {
        tools_printf("{vaff}usage:vaff <accel_gain> <decel_gain> <limit_pwm>|status\r\n");
        return;
    }

    value = vehicle_control_command_float_get(argv[1]);
    vehicle_control_speed_accel_feedforward_gain = (value > 0.0f) ? value : 0.0f;
    value = vehicle_control_command_float_get(argv[2]);
    vehicle_control_speed_decel_feedforward_gain = (value > 0.0f) ? value : 0.0f;
    value = vehicle_control_command_float_get(argv[3]);
    vehicle_control_speed_accel_feedforward_limit = (value > 0.0f) ? value : 0.0f;
    vehicle_control_speed_accel_feedforward_reset();
    tools_printf("{vaff}accel_gain,%.4f,decel_gain,%.4f,limit_pwm,%.1f,output,%.1f\r\n",
                 (double)vehicle_control_speed_accel_feedforward_gain,
                 (double)vehicle_control_speed_decel_feedforward_gain,
                 (double)vehicle_control_speed_accel_feedforward_limit,
                 (double)vehicle_control_speed_accel_feedforward_output);
}

void vehicle_control_angle_pid_command(uint8 argc, uint8* argv[])
{
    if ((argc < 2u)
        || (vehicle_control_command_word_is(argv[1], "status") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "s") != FALSE))
    {
        tools_printf("{vangpid}%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
                     (double)vehicle_control_angle_pid_cfg.kp,
                     (double)vehicle_control_angle_pid_cfg.ki,
                     (double)vehicle_control_angle_pid_cfg.kd,
                     (double)vehicle_control_angle_pid_cfg.output_min,
                     (double)vehicle_control_angle_pid_cfg.output_max,
                     (double)vehicle_control_angle_pid_cfg.integral_min,
                     (double)vehicle_control_angle_pid_cfg.integral_max);
        return;
    }

    if (argc < 4u)
    {
        return;
    }

    vehicle_control_angle_pid_cfg.kp = vehicle_control_command_float_get(argv[1]);
    vehicle_control_angle_pid_cfg.ki = vehicle_control_command_float_get(argv[2]);
    vehicle_control_angle_pid_cfg.kd = vehicle_control_command_float_get(argv[3]);
    vehicle_control_launch_angle_pid_cfg = vehicle_control_angle_pid_cfg;
    algorithm_control_pid_reset(&vehicle_control_state.angle_pid);
    algorithm_control_pid_reset(&vehicle_control_launch_angle_pid);
    vehicle_control_d_yaw_rate_lpf_reset();
    vehicle_control_target_yaw_rate_reset();

    tools_printf("{vangpid}%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
                 (double)vehicle_control_angle_pid_cfg.kp,
                 (double)vehicle_control_angle_pid_cfg.ki,
                 (double)vehicle_control_angle_pid_cfg.kd,
                 (double)vehicle_control_angle_pid_cfg.output_min,
                 (double)vehicle_control_angle_pid_cfg.output_max,
                 (double)vehicle_control_angle_pid_cfg.integral_min,
                 (double)vehicle_control_angle_pid_cfg.integral_max);
}

static void vehicle_control_angle_2dof_status_print(void)
{
    tools_printf("{vang2d}enabled,%u,kp2,%.3f,target_rate_gain_mm,%.3f,lpf_hz,%.3f,limit_rad_s,%.3f,omega_ref,%.3f,p,%.1f,p2,%.1f,rate_feedback,%.1f,target_rate,%.1f\r\n",
                 (unsigned int)((vehicle_control_angle_2dof_enabled != FALSE) ? 1u : 0u),
                 (double)vehicle_control_angle_kp2,
                 (double)vehicle_control_target_yaw_rate_feedforward_gain_mm,
                 (double)vehicle_control_target_yaw_rate_lpf_cutoff_hz,
                 (double)vehicle_control_target_yaw_rate_limit_rad_s,
                 (double)vehicle_control_target_yaw_rate_rad_s,
                 (double)vehicle_control_angle_linear_output_mm_s,
                 (double)vehicle_control_angle_nonlinear_output_mm_s,
                 (double)vehicle_control_angle_rate_output_mm_s,
                 (double)vehicle_control_angle_feedforward_output_mm_s);
}

void vehicle_control_angle_2dof_command(uint8 argc, uint8* argv[])
{
    float32 value;

    if ((argc < 2u)
        || (vehicle_control_command_word_is(argv[1], "status") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "s") != FALSE))
    {
        vehicle_control_angle_2dof_status_print();
        return;
    }

    if (vehicle_control_command_word_is(argv[1], "on") != FALSE)
    {
        vehicle_control_angle_2dof_enabled = TRUE;
        vehicle_control_target_yaw_rate_reset();
        vehicle_control_angle_2dof_status_print();
        return;
    }
    if (vehicle_control_command_word_is(argv[1], "off") != FALSE)
    {
        vehicle_control_angle_2dof_enabled = FALSE;
        vehicle_control_target_yaw_rate_reset();
        vehicle_control_angle_2dof_status_print();
        return;
    }

    if (argc < 5u)
    {
        tools_printf("{vang2d}usage:vang2d on|off|status|<kp2> <target_rate_gain_mm> <lpf_hz> <limit_rad_s>\r\n");
        return;
    }

    value = vehicle_control_command_float_get(argv[1]);
    vehicle_control_angle_kp2 = (value > 0.0f) ? value : 0.0f;
    value = vehicle_control_command_float_get(argv[2]);
    vehicle_control_target_yaw_rate_feedforward_gain_mm = (value > 0.0f) ? value : 0.0f;
    value = vehicle_control_command_float_get(argv[3]);
    vehicle_control_target_yaw_rate_lpf_cutoff_hz = (value > 0.0f) ? value : 0.0f;
    value = vehicle_control_command_float_get(argv[4]);
    vehicle_control_target_yaw_rate_limit_rad_s = (value > 0.0f) ? value : 0.0f;
    vehicle_control_target_yaw_rate_reset();
    vehicle_control_angle_2dof_status_print();
}

void vehicle_control_launch_angle_pid_command(uint8 argc, uint8* argv[])
{
    if ((argc < 2u)
        || (vehicle_control_command_word_is(argv[1], "status") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "s") != FALSE))
    {
        tools_printf("{vlaunchpid}%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
                     (double)vehicle_control_launch_angle_pid_cfg.kp,
                     (double)vehicle_control_launch_angle_pid_cfg.ki,
                     (double)vehicle_control_launch_angle_pid_cfg.kd,
                     (double)vehicle_control_launch_angle_pid_cfg.output_min,
                     (double)vehicle_control_launch_angle_pid_cfg.output_max,
                     (double)vehicle_control_launch_angle_pid_cfg.integral_min,
                     (double)vehicle_control_launch_angle_pid_cfg.integral_max);
        return;
    }

    if (argc < 4u)
    {
        return;
    }

    vehicle_control_launch_angle_pid_cfg.kp = vehicle_control_command_float_get(argv[1]);
    vehicle_control_launch_angle_pid_cfg.ki = vehicle_control_command_float_get(argv[2]);
    vehicle_control_launch_angle_pid_cfg.kd = vehicle_control_command_float_get(argv[3]);
    algorithm_control_pid_reset(&vehicle_control_launch_angle_pid);

    tools_printf("{vlaunchpid}%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
                 (double)vehicle_control_launch_angle_pid_cfg.kp,
                 (double)vehicle_control_launch_angle_pid_cfg.ki,
                 (double)vehicle_control_launch_angle_pid_cfg.kd,
                 (double)vehicle_control_launch_angle_pid_cfg.output_min,
                 (double)vehicle_control_launch_angle_pid_cfg.output_max,
                 (double)vehicle_control_launch_angle_pid_cfg.integral_min,
                 (double)vehicle_control_launch_angle_pid_cfg.integral_max);
}


/**
 * @brief Parse speed feedforward table command.
 * @param[in] argc Command argument count.
 * @param[in] argv Command argument string array.
 * @return void
 */
void vehicle_control_speed_test_feedforward_command(uint8 argc, uint8* argv[])
{
    float32 speed_mm_s;
    float32 duty_tick;
    boolean stored = TRUE;
    boolean left_selected = FALSE;
    boolean right_selected = FALSE;
    boolean side_command = FALSE;
    const char* side_name = "both";

    if (argc < 2u)
    {
        return;
    }

    if ((vehicle_control_command_word_is(argv[1], "on") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "enable") != FALSE))
    {
        vehicle_control_speed_feedforward_enabled = TRUE;
        tools_printf("{vff}enable,1\r\n");
        return;
    }

    if ((vehicle_control_command_word_is(argv[1], "off") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "disable") != FALSE))
    {
        vehicle_control_speed_feedforward_enabled = FALSE;
        tools_printf("{vff}enable,0\r\n");
        return;
    }

    if ((vehicle_control_command_word_is(argv[1], "status") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "s") != FALSE))
    {
        tools_printf("{vff}enable,%u\r\n",
                     (unsigned int)((vehicle_control_speed_feedforward_enabled != FALSE) ? 1u : 0u));
        return;
    }

    if ((argv[1][0] == (uint8)'c') || (argv[1][0] == (uint8)'C'))
    {
        vehicle_control_speed_feedforward_table_reset();
        tools_printf("{vff}clear,both\r\n");
        return;
    }

    if ((vehicle_control_command_word_is(argv[1], "list") != FALSE)
        || (((argv[1][0] == (uint8)'l') || (argv[1][0] == (uint8)'L'))
            && (argv[1][1] == (uint8)'\0')))
    {
        tools_printf("{vff}enable,%u\r\n",
                     (unsigned int)((vehicle_control_speed_feedforward_enabled != FALSE) ? 1u : 0u));
        vehicle_control_speed_feedforward_table_print("left",
                                                      vehicle_control_left_speed_feedforward_table);
        vehicle_control_speed_feedforward_table_print("right",
                                                      vehicle_control_right_speed_feedforward_table);
        return;
    }

    if (vehicle_control_command_word_is(argv[1], "left") != FALSE)
    {
        left_selected = TRUE;
        side_command = TRUE;
        side_name = "left";
    }
    else if ((vehicle_control_command_word_is(argv[1], "right") != FALSE)
             || (((argv[1][0] == (uint8)'r') || (argv[1][0] == (uint8)'R'))
                 && (argv[1][1] == (uint8)'\0')))
    {
        right_selected = TRUE;
        side_command = TRUE;
        side_name = "right";
    }
    else if ((vehicle_control_command_word_is(argv[1], "both") != FALSE)
             || (vehicle_control_command_word_is(argv[1], "all") != FALSE))
    {
        left_selected = TRUE;
        right_selected = TRUE;
        side_command = TRUE;
        side_name = "both";
    }

    if (side_command != FALSE)
    {
        if ((argc < 3u)
            || (vehicle_control_command_word_is(argv[2], "list") != FALSE)
            || (vehicle_control_command_word_is(argv[2], "status") != FALSE)
            || (((argv[2][0] == (uint8)'l') || (argv[2][0] == (uint8)'L'))
                && (argv[2][1] == (uint8)'\0')))
        {
            tools_printf("{vff}enable,%u\r\n",
                         (unsigned int)((vehicle_control_speed_feedforward_enabled != FALSE) ? 1u : 0u));
            if (left_selected != FALSE)
            {
                vehicle_control_speed_feedforward_table_print("left",
                                                              vehicle_control_left_speed_feedforward_table);
            }
            if (right_selected != FALSE)
            {
                vehicle_control_speed_feedforward_table_print("right",
                                                              vehicle_control_right_speed_feedforward_table);
            }
            return;
        }

        if ((argv[2][0] == (uint8)'c') || (argv[2][0] == (uint8)'C'))
        {
            if (left_selected != FALSE)
            {
                vehicle_control_speed_feedforward_table_reset_one(vehicle_control_left_speed_feedforward_table);
            }
            if (right_selected != FALSE)
            {
                vehicle_control_speed_feedforward_table_reset_one(vehicle_control_right_speed_feedforward_table);
            }
            tools_printf("{vff}clear,%s\r\n", side_name);
            return;
        }

        if (argc < 4u)
        {
            return;
        }

        speed_mm_s = vehicle_control_command_float_get(argv[2]);
        duty_tick = vehicle_control_command_float_get(argv[3]);
        if (left_selected != FALSE)
        {
            stored = vehicle_control_speed_feedforward_point_set(vehicle_control_left_speed_feedforward_table,
                                                                 speed_mm_s,
                                                                 duty_tick);
        }
        if ((right_selected != FALSE) && (stored != FALSE))
        {
            stored = vehicle_control_speed_feedforward_point_set(vehicle_control_right_speed_feedforward_table,
                                                                 speed_mm_s,
                                                                 duty_tick);
        }

        if (stored != FALSE)
        {
            tools_printf("{vff}set,%s,%.3f,%.3f\r\n",
                         side_name,
                         (double)speed_mm_s,
                         (double)duty_tick);
        }
        else
        {
            tools_printf("{vff}full,%s\r\n", side_name);
        }
        return;
    }

    if (argc < 3u)
    {
        return;
    }

    speed_mm_s = vehicle_control_command_float_get(argv[1]);
    duty_tick = vehicle_control_command_float_get(argv[2]);
    stored = vehicle_control_speed_feedforward_point_set(vehicle_control_left_speed_feedforward_table,
                                                         speed_mm_s,
                                                         duty_tick);
    if (stored != FALSE)
    {
        stored = vehicle_control_speed_feedforward_point_set(vehicle_control_right_speed_feedforward_table,
                                                            speed_mm_s,
                                                            duty_tick);
    }

    if (stored != FALSE)
    {
        tools_printf("{vff}set,both,%.3f,%.3f\r\n", (double)speed_mm_s, (double)duty_tick);
    }
    else
    {
        tools_printf("{vff}full,both\r\n");
    }
}

void vehicle_control_speed_integral_limit_command(uint8 argc, uint8* argv[])
{
    float32 limit_abs;

    if (argc < 2u)
    {
        tools_printf("{vilim}enable,%u,%.3f\r\n",
                     (unsigned int)((vehicle_control_speed_integral_limit_enabled != FALSE) ? 1u : 0u),
                     (double)vehicle_control_speed_integral_limit_abs);
        return;
    }

    if ((vehicle_control_command_word_is(argv[1], "on") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "enable") != FALSE))
    {
        if (argc >= 3u)
        {
            limit_abs = vehicle_control_command_float_get(argv[2]);
            if (limit_abs < 0.0f)
            {
                limit_abs = -limit_abs;
            }
            vehicle_control_speed_integral_limit_abs = limit_abs;
        }
        vehicle_control_speed_integral_limit_enabled = TRUE;
        vehicle_control_speed_integral_limit_clamp_state(&vehicle_control_state.left_speed_pid);
        vehicle_control_speed_integral_limit_clamp_state(&vehicle_control_state.right_speed_pid);
        tools_printf("{vilim}enable,1,%.3f\r\n", (double)vehicle_control_speed_integral_limit_abs);
        return;
    }

    if ((vehicle_control_command_word_is(argv[1], "off") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "disable") != FALSE))
    {
        vehicle_control_speed_integral_limit_enabled = FALSE;
        tools_printf("{vilim}enable,0,%.3f\r\n", (double)vehicle_control_speed_integral_limit_abs);
        return;
    }

    if ((vehicle_control_command_word_is(argv[1], "status") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "s") != FALSE))
    {
        tools_printf("{vilim}enable,%u,%.3f\r\n",
                     (unsigned int)((vehicle_control_speed_integral_limit_enabled != FALSE) ? 1u : 0u),
                     (double)vehicle_control_speed_integral_limit_abs);
        return;
    }

    limit_abs = vehicle_control_command_float_get(argv[1]);
    if (limit_abs < 0.0f)
    {
        limit_abs = -limit_abs;
    }

    vehicle_control_speed_integral_limit_abs = limit_abs;
    vehicle_control_speed_integral_limit_enabled = TRUE;
    vehicle_control_speed_integral_limit_clamp_state(&vehicle_control_state.left_speed_pid);
    vehicle_control_speed_integral_limit_clamp_state(&vehicle_control_state.right_speed_pid);
    tools_printf("{vilim}enable,1,%.3f\r\n", (double)vehicle_control_speed_integral_limit_abs);
}

void vehicle_control_speed_protection_command(uint8 argc, uint8* argv[])
{
    float32 value;

    if ((argc < 2u)
        || (vehicle_control_command_word_is(argv[1], "status") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "s") != FALSE))
    {
        tools_printf("{vprot}enable,%u,acc,%.3f,dec,%.3f,startacc,%.3f,diff,%.3f,rev,%.3f\r\n",
                     (unsigned int)((vehicle_control_speed_protection_enabled != FALSE) ? 1u : 0u),
                     (double)(vehicle_control_target_accel_limit_mm_s2 * 0.001f),
                     (double)(vehicle_control_target_decel_limit_mm_s2 * 0.001f),
                     (double)(vehicle_control_start_accel_limit_mm_s2 * 0.001f),
                     (double)(vehicle_control_angle_diff_limit_mm_s * 0.001f),
                     (double)(vehicle_control_reverse_guard_speed_mm_s * 0.001f));
        return;
    }

    if ((vehicle_control_command_word_is(argv[1], "on") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "enable") != FALSE))
    {
        vehicle_control_speed_protection_enabled = TRUE;
        vehicle_control_speed_target_runtime_reset();
        tools_printf("{vprot}enable,1\r\n");
        return;
    }

    if ((vehicle_control_command_word_is(argv[1], "off") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "disable") != FALSE))
    {
        vehicle_control_speed_protection_enabled = FALSE;
        vehicle_control_speed_target_runtime_reset();
        tools_printf("{vprot}enable,0\r\n");
        return;
    }

    if ((vehicle_control_command_word_is(argv[1], "acc") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "accel") != FALSE))
    {
        if (argc >= 3u)
        {
            value = vehicle_control_command_float_get(argv[2]);
            if (value < 0.0f)
            {
                value = -value;
            }
            vehicle_control_target_accel_limit_mm_s2 = value * 1000.0f;
        }
    }
    else if ((vehicle_control_command_word_is(argv[1], "dec") != FALSE)
             || (vehicle_control_command_word_is(argv[1], "decel") != FALSE))
    {
        if (argc >= 3u)
        {
            value = vehicle_control_command_float_get(argv[2]);
            if (value < 0.0f)
            {
                value = -value;
            }
            vehicle_control_target_decel_limit_mm_s2 = value * 1000.0f;
        }
    }
    else if ((vehicle_control_command_word_is(argv[1], "startacc") != FALSE)
             || (vehicle_control_command_word_is(argv[1], "sacc") != FALSE))
    {
        if (argc >= 3u)
        {
            value = vehicle_control_command_float_get(argv[2]);
            if (value < 0.0f)
            {
                value = -value;
            }
            vehicle_control_start_accel_limit_mm_s2 = value * 1000.0f;
        }
    }
    else if ((vehicle_control_command_word_is(argv[1], "diff") != FALSE)
             || (vehicle_control_command_word_is(argv[1], "angle") != FALSE))
    {
        if (argc >= 3u)
        {
            value = vehicle_control_command_float_get(argv[2]);
            if (value < 0.0f)
            {
                value = -value;
            }
            vehicle_control_angle_diff_limit_mm_s = value * 1000.0f;
        }
    }
    else if ((vehicle_control_command_word_is(argv[1], "rev") != FALSE)
             || (vehicle_control_command_word_is(argv[1], "reverse") != FALSE))
    {
        if (argc >= 3u)
        {
            value = vehicle_control_command_float_get(argv[2]);
            if (value < 0.0f)
            {
                value = -value;
            }
            vehicle_control_reverse_guard_speed_mm_s = value * 1000.0f;
        }
    }
    else if (argc >= 4u)
    {
        vehicle_control_target_accel_limit_mm_s2 = vehicle_control_command_float_get(argv[1]) * 1000.0f;
        vehicle_control_angle_diff_limit_mm_s = vehicle_control_command_float_get(argv[2]) * 1000.0f;
        vehicle_control_reverse_guard_speed_mm_s = vehicle_control_command_float_get(argv[3]) * 1000.0f;
        if (argc >= 5u)
        {
            vehicle_control_start_accel_limit_mm_s2 = vehicle_control_command_float_get(argv[4]) * 1000.0f;
        }
        if (vehicle_control_target_accel_limit_mm_s2 < 0.0f)
        {
            vehicle_control_target_accel_limit_mm_s2 = -vehicle_control_target_accel_limit_mm_s2;
        }
        if (vehicle_control_start_accel_limit_mm_s2 < 0.0f)
        {
            vehicle_control_start_accel_limit_mm_s2 = -vehicle_control_start_accel_limit_mm_s2;
        }
        if (vehicle_control_angle_diff_limit_mm_s < 0.0f)
        {
            vehicle_control_angle_diff_limit_mm_s = -vehicle_control_angle_diff_limit_mm_s;
        }
        if (vehicle_control_reverse_guard_speed_mm_s < 0.0f)
        {
            vehicle_control_reverse_guard_speed_mm_s = -vehicle_control_reverse_guard_speed_mm_s;
        }
    }

    tools_printf("{vprot}enable,%u,acc,%.3f,dec,%.3f,startacc,%.3f,diff,%.3f,rev,%.3f\r\n",
                 (unsigned int)((vehicle_control_speed_protection_enabled != FALSE) ? 1u : 0u),
                 (double)(vehicle_control_target_accel_limit_mm_s2 * 0.001f),
                 (double)(vehicle_control_target_decel_limit_mm_s2 * 0.001f),
                 (double)(vehicle_control_start_accel_limit_mm_s2 * 0.001f),
                 (double)(vehicle_control_angle_diff_limit_mm_s * 0.001f),
                 (double)(vehicle_control_reverse_guard_speed_mm_s * 0.001f));
}

void vehicle_control_replay_speed_command(uint8 argc, uint8* argv[])
{
    float32 speed_m_s;
    float32 speed_mm_s;

    if ((argc < 2u)
        || (vehicle_control_command_word_is(argv[1], "status") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "s") != FALSE))
    {
        speed_mm_s = module_vehicle_path_replay_speed_get();
        tools_printf("{vrspd}%.3f,%.0f\r\n",
                     (double)(speed_mm_s * 0.001f),
                     (double)speed_mm_s);
        return;
    }

    if ((vehicle_control_command_word_is(argv[1], "reset") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "default") != FALSE))
    {
        module_vehicle_path_replay_speed_set(VEHICLE_PATH_DEFAULT_REPLAY_SPEED_MM_S);
    }
    else
    {
        speed_m_s = vehicle_control_command_float_get(argv[1]);
        if (speed_m_s < 0.0f)
        {
            speed_m_s = 0.0f;
        }
        module_vehicle_path_replay_speed_set(speed_m_s * 1000.0f);
    }

    speed_mm_s = module_vehicle_path_replay_speed_get();
    tools_printf("{vrspd}%.3f,%.0f\r\n",
                 (double)(speed_mm_s * 0.001f),
                 (double)speed_mm_s);
}

void vehicle_control_status_command(uint8 argc, uint8* argv[])
{
    const vehicle_control_cfg_t* cfg = vehicle_control_cfg_get();
    const vehicle_path_cfg_t* path_cfg = vehicle_path_cfg_get();
    const vehicle_path_state_t* path_state = module_vehicle_path_state_get();
    const module_vehicle_encoder_observation_t* encoder_observation =
        module_vehicle_encoder_observation_get();
    float32 replay_lookahead_mm = 0.0f;
    float32 replay_tangent_mm = 0.0f;
    float32 replay_speed_mm_s = module_vehicle_path_replay_speed_get();
    float32 replay_speed_cap_mm_s = 0.0f;
    uint32 planned_speed_point_cnt = module_vehicle_path_replay_planned_speed_point_count_get();
    uint32 planned_speed_source_point_cnt =
        module_vehicle_path_replay_planned_speed_source_point_count_get();
    float32 planned_speed_source_length_mm =
        module_vehicle_path_replay_planned_speed_source_length_get();
    float32 planned_speed_step_mm =
        module_vehicle_path_replay_planned_speed_step_get();
    boolean planned_speed_meta_valid =
        module_vehicle_path_replay_planned_speed_meta_valid_get();
    boolean planned_speed_available =
        module_vehicle_path_replay_planned_speed_available_get();
    boolean print_check = FALSE;

    if ((argc >= 2u)
        && ((vehicle_control_command_word_is(argv[1], "help") != FALSE)
            || (vehicle_control_command_word_is(argv[1], "usage") != FALSE)
            || (vehicle_control_command_word_is(argv[1], "?") != FALSE)))
    {
        vehicle_control_status_help_print();
        return;
    }
    if ((argc >= 2u)
        && ((vehicle_control_command_word_is(argv[1], "check") != FALSE)
            || (vehicle_control_command_word_is(argv[1], "chk") != FALSE)))
    {
        vehicle_control_status_check_print();
        return;
    }
    if ((argc >= 2u)
        && ((vehicle_control_command_word_is(argv[1], "all") != FALSE)
            || (vehicle_control_command_word_is(argv[1], "full") != FALSE)))
    {
        print_check = TRUE;
    }

    module_vehicle_path_replay_ahead_get(&replay_lookahead_mm, &replay_tangent_mm);
    if ((vehicle_control_state.mode == VEHICLE_CONTROL_MODE_REPLAY)
        && (vehicle_control_replay_launch_gate_active != FALSE))
    {
        if (vehicle_control_replay_launch_drag_active() != FALSE)
        {
            replay_speed_cap_mm_s = (vehicle_control_launch_drag_correction_enabled != FALSE)
                                        ? VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_SPEED_MM_S
                                        : 0.0f;
        }
        else
        {
            replay_speed_cap_mm_s = VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S;
        }
    }

    tools_printf("{vstat0}mode,%u,en,%u,spdtest,%u,gate,%u,phase,%u,gate_ms,%.0f,stable_ms,%.0f,ready,%u,hold_ms,%.0f\r\n",
                 (unsigned int)vehicle_control_state.mode,
                 (unsigned int)((vehicle_control_state.enabled != FALSE) ? 1u : 0u),
                 (unsigned int)((vehicle_control_speed_test_enabled != FALSE) ? 1u : 0u),
                 (unsigned int)(((vehicle_control_replay_launch_gate_active != FALSE)
                                 || (vehicle_control_test_launch_gate_active != FALSE)) ? 1u : 0u),
                 (unsigned int)((vehicle_control_replay_launch_gate_active != FALSE)
                                    ? vehicle_control_replay_launch_phase
                                    : vehicle_control_test_launch_phase),
                 (double)(((vehicle_control_replay_launch_gate_active != FALSE)
                               ? vehicle_control_replay_launch_gate_timer_s
                               : vehicle_control_test_launch_gate_timer_s) * 1000.0f),
                 (double)(((vehicle_control_replay_launch_gate_active != FALSE)
                               ? vehicle_control_replay_launch_gate_stable_timer_s
                               : vehicle_control_test_launch_gate_stable_timer_s) * 1000.0f),
                 (unsigned int)(((vehicle_control_replay_launch_gate_active != FALSE)
                                     ? vehicle_control_replay_launch_wheel_ready
                                     : vehicle_control_test_launch_wheel_ready) != FALSE ? 1u : 0u),
                 (double)(vehicle_control_start_integral_hold_timer_s * 1000.0f));
    tools_printf("{vstat1}target,%.0f,base_eff,%.0f,left_t,%.0f,right_t,%.0f,left_v,%.0f,right_v,%.0f,yaw_cmd,%.0f,yaw_fb,%.0f,yaw_eff,%.0f\r\n",
                 (double)vehicle_control_state.target_speed_mm_s,
                 (double)vehicle_control_base_effective_target_mm_s,
                 (double)vehicle_control_state.target_left_speed_mm_s,
                 (double)vehicle_control_state.target_right_speed_mm_s,
                 (double)encoder_observation->left_speed_mm_s,
                 (double)encoder_observation->right_speed_mm_s,
                 (double)vehicle_control_state.yaw_speed_correction_mm_s,
                 (double)vehicle_control_yaw_feedback_correction_mm_s,
                 (double)vehicle_control_effective_yaw_speed_correction_mm_s);
    tools_printf("{vstat2}angle,kp,%.3f,ki,%.3f,kd,%.3f,out,%.0f,%.0f,launch_kp,%.3f,launch_ki,%.3f,launch_kd,%.3f\r\n",
                 (double)vehicle_control_angle_pid_cfg.kp,
                 (double)vehicle_control_angle_pid_cfg.ki,
                 (double)vehicle_control_angle_pid_cfg.kd,
                 (double)vehicle_control_angle_pid_cfg.output_min,
                 (double)vehicle_control_angle_pid_cfg.output_max,
                 (double)vehicle_control_launch_angle_pid_cfg.kp,
                 (double)vehicle_control_launch_angle_pid_cfg.ki,
                 (double)vehicle_control_launch_angle_pid_cfg.kd);
    tools_printf("{vstat3}speedpid,lkp,%.3f,lki,%.3f,lkd,%.3f,rkp,%.3f,rki,%.3f,rkd,%.3f,ff,%u,ilim,%u,ilim_abs,%.0f\r\n",
                 (double)vehicle_control_left_speed_pid_cfg.kp,
                 (double)vehicle_control_left_speed_pid_cfg.ki,
                 (double)vehicle_control_left_speed_pid_cfg.kd,
                 (double)vehicle_control_right_speed_pid_cfg.kp,
                 (double)vehicle_control_right_speed_pid_cfg.ki,
                 (double)vehicle_control_right_speed_pid_cfg.kd,
                 (unsigned int)((vehicle_control_speed_feedforward_enabled != FALSE) ? 1u : 0u),
                 (unsigned int)((vehicle_control_speed_integral_limit_enabled != FALSE) ? 1u : 0u),
                 (double)vehicle_control_speed_integral_limit_abs);
    tools_printf("{vstat4}limit,duty,%d,slew,%d,fall_slew,%d,launch_slew,%d,acc,%.0f,dec,%.0f,startacc,%.0f,baseacc,%.0f,baseacc_on,%u,diff,%.0f,yaw_slew,%.0f,track,%u,track_evt,%u,rg_cap,%u,rg_fault,%u,rg_evt,%u,rg_base_cap,%.0f,track_acc_lead,%.0f,track_dec_lead,%.0f,rg_spd,%.0f,rg_target,%.0f\r\n",
                 (int)VEHICLE_CONTROL_DRIVE_DUTY_LIMIT_TICK,
                 (int)VEHICLE_CONTROL_DRIVE_DUTY_SLEW_LIMIT_TICK_PER_UPDATE,
                 (int)VEHICLE_CONTROL_DRIVE_DUTY_FALL_SLEW_LIMIT_TICK_PER_UPDATE,
                 (int)VEHICLE_CONTROL_DRIVE_LAUNCH_DUTY_SLEW_LIMIT_TICK_PER_UPDATE,
                 (double)vehicle_control_target_accel_limit_mm_s2,
                 (double)vehicle_control_target_decel_limit_mm_s2,
                 (double)vehicle_control_start_accel_limit_mm_s2,
                 (double)vehicle_control_base_accel_limit_mm_s2,
                 (unsigned int)((vehicle_control_base_start_accel_active != FALSE) ? 1u : 0u),
                 (double)vehicle_control_angle_diff_limit_mm_s,
                 (double)VEHICLE_CONTROL_YAW_CORRECTION_SLEW_LIMIT_MM_S2,
                 (unsigned int)((vehicle_control_tracking_limit_active != FALSE) ? 1u : 0u),
                 (unsigned int)vehicle_control_tracking_limit_event_count,
                 (unsigned int)((vehicle_control_reverse_feedback_cap_active != FALSE) ? 1u : 0u),
                 (unsigned int)((vehicle_control_reverse_feedback_fault_latched != FALSE) ? 1u : 0u),
                 (unsigned int)vehicle_control_reverse_feedback_event_count,
                 (double)vehicle_control_reverse_guard_base_cap_mm_s,
                 (double)VEHICLE_CONTROL_WHEEL_TARGET_ACCEL_LEAD_LIMIT_MM_S,
                 (double)VEHICLE_CONTROL_REPLAY_WHEEL_TARGET_DECEL_LEAD_LIMIT_MM_S,
                 (double)vehicle_control_reverse_guard_speed_mm_s,
                 (double)VEHICLE_CONTROL_REVERSE_GUARD_TARGET_THRESHOLD_MM_S);
    tools_printf("{vstat5}path,status,%u,rec,%u,rpl,%u,tvalid,%u,cnt,%u,cursor,%u,tidx,%u,last,%.1f,start,%.1f,look,%.0f,tan,%.0f,vrspd,%.0f\r\n",
                 (unsigned int)path_state->status,
                 (unsigned int)((path_state->recording != FALSE) ? 1u : 0u),
                 (unsigned int)((path_state->replaying != FALSE) ? 1u : 0u),
                 (unsigned int)((path_state->replay_target_valid != FALSE) ? 1u : 0u),
                 (unsigned int)path_state->point_cnt,
                 (unsigned int)path_state->replay_cursor_cnt,
                 (unsigned int)path_state->replay_target_index_cnt,
                 (double)path_state->last_distance_mm,
                 (double)path_state->replay_start_distance_mm,
                 (double)replay_lookahead_mm,
                 (double)replay_tangent_mm,
                 (double)replay_speed_mm_s);
    tools_printf("{vstat6}plan,min,%.0f,max,%.0f,lat,%.0f,curve_lat,%.0f,acc,%.0f,dec,%.0f,long_acc,%.0f,long_dec,%.0f,fric,%u,geom,%u,bspline,%u,curvesmooth,%u,clothoid,%u,radius,%u,gap,%u,step,%.1f,start_hold,%.0f,%.0f\r\n",
                 (double)path_cfg->plan_min_speed_mm_s,
                 (double)path_cfg->plan_max_speed_mm_s,
                 (double)path_cfg->plan_lateral_accel_mm_s2,
                 (double)path_cfg->plan_curve_lateral_accel_mm_s2,
                 (double)path_cfg->plan_accel_mm_s2,
                 (double)path_cfg->plan_decel_mm_s2,
                 (double)path_cfg->plan_long_accel_mm_s2,
                 (double)path_cfg->plan_long_decel_mm_s2,
                 (unsigned int)((path_cfg->plan_friction_circle_enable != FALSE) ? 1u : 0u),
                 (unsigned int)((path_cfg->plan_geometry_enable != FALSE) ? 1u : 0u),
                 (unsigned int)((path_cfg->plan_bspline_enable != FALSE) ? 1u : 0u),
                 (unsigned int)((path_cfg->plan_curvature_smooth_enable != FALSE) ? 1u : 0u),
                 (unsigned int)((path_cfg->plan_clothoid_enable != FALSE) ? 1u : 0u),
                 (unsigned int)((path_cfg->plan_radius_lock_enable != FALSE) ? 1u : 0u),
                 (unsigned int)path_cfg->plan_curvature_gap_cnt,
                 (double)path_cfg->min_distance_step_mm,
                 (double)VEHICLE_PATH_DEFAULT_PLAN_START_HOLD_DISTANCE_MM,
                 (double)VEHICLE_PATH_DEFAULT_PLAN_START_HOLD_SPEED_MM_S);
    tools_printf("{vstat7}speedpath,planned,%u,available,%u,raw,%.0f,preview,%.0f,final,%.0f,gate_cap,%.0f,preview_mode,direct_point,preview_mm,%.0f,enddecel,%.0f,endbrake,%.0f,pcnt,%u,rawcnt,%u,meta,%u,scnt,%u,slen,%.1f,pstep,%.3f\r\n",
                 (unsigned int)((module_vehicle_path_replay_planned_speed_active_get() != FALSE) ? 1u : 0u),
                 (unsigned int)((planned_speed_available != FALSE) ? 1u : 0u),
                 (double)((vehicle_control_state.replay_target_valid != FALSE)
                          ? vehicle_control_state.replay_target.raw_plan_speed_mm_s
                          : 0.0f),
                 (double)((vehicle_control_state.replay_target_valid != FALSE)
                          ? vehicle_control_state.replay_target.speed_preview_limit_mm_s
                          : 0.0f),
                 (double)((vehicle_control_state.replay_target_valid != FALSE)
                          ? vehicle_control_state.replay_target.target_speed_mm_s
                          : 0.0f),
                 (double)replay_speed_cap_mm_s,
                 (double)VEHICLE_PATH_DEFAULT_REPLAY_SPEED_PREVIEW_DISTANCE_MM,
                 (double)VEHICLE_PATH_DEFAULT_END_DECEL_DISTANCE_MM,
                 (double)VEHICLE_PATH_DEFAULT_REPLAY_END_BRAKE_DISTANCE_MM,
                 (unsigned int)planned_speed_point_cnt,
                 (unsigned int)path_state->point_cnt,
                 (unsigned int)((planned_speed_meta_valid != FALSE) ? 1u : 0u),
                 (unsigned int)planned_speed_source_point_cnt,
                 (double)planned_speed_source_length_mm,
                 (double)planned_speed_step_mm);
    if (print_check != FALSE)
    {
        vehicle_control_status_check_print();
    }
}

static void vehicle_control_status_help_print(void)
{
    tools_printf("{vstatus}usage:vstatus|vstatus check|vstatus all\r\n");
    tools_printf("{vhelp0}mode,enable,speedtest,launch_gate,phase,timers\r\n");
    tools_printf("{vhelp1}speed_targets,wheel_feedback,yaw_outputs\r\n");
    tools_printf("{vhelp2}angle_pid,launch_angle_pid,angle_2dof(vang2d),speed_accel_ff(vaff)\r\n");
    tools_printf("{vhelp3}speed_pid,feedforward,integral_limit\r\n");
    tools_printf("{vhelp4}duty_slew,accel_limits,diff_limit,yaw_slew,wheel_tracking_limit,reverse_guard,limit_params\r\n");
    tools_printf("{vhelp5}path_state,lookahead,tangent,fixed_replay_speed\r\n");
    tools_printf("{vhelp6}speed_plan_config,start_hold,geometry_flags\r\n");
    tools_printf("{vhelp7}planned_speed_chain:active,available,raw,preview,final,gate_cap,endbrake,counts,meta,source_length,plan_step\r\n");
}

static void vehicle_control_status_check_print(void)
{
    const vehicle_control_cfg_t* cfg = vehicle_control_cfg_get();
    const vehicle_path_cfg_t* path_cfg = vehicle_path_cfg_get();
    const vehicle_path_state_t* path_state = module_vehicle_path_state_get();
    float32 replay_lookahead_mm = 0.0f;
    float32 replay_tangent_mm = 0.0f;
    uint32 planned_speed_point_cnt = module_vehicle_path_replay_planned_speed_point_count_get();
    float32 planned_speed_source_length_mm =
        module_vehicle_path_replay_planned_speed_source_length_get();
    float32 planned_speed_step_mm =
        module_vehicle_path_replay_planned_speed_step_get();
    boolean planned_speed_meta_valid =
        module_vehicle_path_replay_planned_speed_meta_valid_get();
    uint32 warn_count = 0u;

    module_vehicle_path_replay_ahead_get(&replay_lookahead_mm, &replay_tangent_mm);

    if (path_cfg->min_distance_step_mm <= 0.0f)
    {
        tools_printf("{vcheck}warn,path_step_invalid,%.3f\r\n",
                     (double)path_cfg->min_distance_step_mm);
        warn_count++;
    }
    if (path_cfg->plan_max_speed_mm_s < path_cfg->plan_min_speed_mm_s)
    {
        tools_printf("{vcheck}warn,plan_speed_range,%.0f,%.0f\r\n",
                     (double)path_cfg->plan_min_speed_mm_s,
                     (double)path_cfg->plan_max_speed_mm_s);
        warn_count++;
    }
    if ((VEHICLE_PATH_DEFAULT_PLAN_START_HOLD_SPEED_MM_S < path_cfg->plan_min_speed_mm_s)
        || (VEHICLE_PATH_DEFAULT_PLAN_START_HOLD_SPEED_MM_S > path_cfg->plan_max_speed_mm_s))
    {
        tools_printf("{vcheck}warn,start_hold_speed_clamped,%.0f,%.0f,%.0f\r\n",
                     (double)VEHICLE_PATH_DEFAULT_PLAN_START_HOLD_SPEED_MM_S,
                     (double)path_cfg->plan_min_speed_mm_s,
                     (double)path_cfg->plan_max_speed_mm_s);
        warn_count++;
    }
    if (VEHICLE_PATH_DEFAULT_PLAN_START_HOLD_DISTANCE_MM < replay_lookahead_mm)
    {
        tools_printf("{vcheck}warn,start_hold_shorter_than_lookahead,%.0f,%.0f\r\n",
                     (double)VEHICLE_PATH_DEFAULT_PLAN_START_HOLD_DISTANCE_MM,
                     (double)replay_lookahead_mm);
        warn_count++;
    }
    if (VEHICLE_PATH_DEFAULT_PLAN_START_HOLD_SPEED_MM_S != VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S)
    {
        tools_printf("{vcheck}warn,start_hold_vs_launch_speed,%.0f,%.0f\r\n",
                     (double)VEHICLE_PATH_DEFAULT_PLAN_START_HOLD_SPEED_MM_S,
                     (double)VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S);
        warn_count++;
    }
    if (VEHICLE_PATH_DEFAULT_REPLAY_SPEED_PREVIEW_DISTANCE_MM < 0.0f)
    {
        tools_printf("{vcheck}warn,speed_preview_distance_invalid,%.0f\r\n",
                     (double)VEHICLE_PATH_DEFAULT_REPLAY_SPEED_PREVIEW_DISTANCE_MM);
        warn_count++;
    }
    if (VEHICLE_PATH_DEFAULT_REPLAY_END_BRAKE_DISTANCE_MM <= 0.0f)
    {
        tools_printf("{vcheck}warn,end_brake_distance_invalid,%.0f\r\n",
                     (double)VEHICLE_PATH_DEFAULT_REPLAY_END_BRAKE_DISTANCE_MM);
        warn_count++;
    }
    if (VEHICLE_PATH_DEFAULT_END_DECEL_DISTANCE_MM
        <= VEHICLE_PATH_DEFAULT_REPLAY_END_BRAKE_DISTANCE_MM)
    {
        tools_printf("{vcheck}warn,end_decel_distance_invalid,%.0f,%.0f\r\n",
                     (double)VEHICLE_PATH_DEFAULT_END_DECEL_DISTANCE_MM,
                     (double)VEHICLE_PATH_DEFAULT_REPLAY_END_BRAKE_DISTANCE_MM);
        warn_count++;
    }
    if ((path_cfg->plan_geometry_enable == FALSE)
        && ((path_cfg->plan_bspline_enable != FALSE)
            || (path_cfg->plan_clothoid_enable != FALSE)
            || (path_cfg->plan_curvature_smooth_enable != FALSE)
            || (path_cfg->plan_radius_lock_enable != FALSE)))
    {
        tools_printf("{vcheck}warn,geometry_subfeature_gated,geom,%u,bspline,%u,clothoid,%u,curvesmooth,%u,radius,%u\r\n",
                     (unsigned int)((path_cfg->plan_geometry_enable != FALSE) ? 1u : 0u),
                     (unsigned int)((path_cfg->plan_bspline_enable != FALSE) ? 1u : 0u),
                     (unsigned int)((path_cfg->plan_clothoid_enable != FALSE) ? 1u : 0u),
                     (unsigned int)((path_cfg->plan_curvature_smooth_enable != FALSE) ? 1u : 0u),
                     (unsigned int)((path_cfg->plan_radius_lock_enable != FALSE) ? 1u : 0u));
        warn_count++;
    }
    if ((path_cfg->plan_friction_circle_enable != FALSE)
        && ((path_cfg->plan_lateral_accel_mm_s2 <= 0.0f)
            || (path_cfg->plan_long_accel_mm_s2 <= 0.0f)
            || (path_cfg->plan_long_decel_mm_s2 <= 0.0f)))
    {
        tools_printf("{vcheck}warn,friction_circle_param,lat,%.0f,long_acc,%.0f,long_dec,%.0f\r\n",
                     (double)path_cfg->plan_lateral_accel_mm_s2,
                     (double)path_cfg->plan_long_accel_mm_s2,
                     (double)path_cfg->plan_long_decel_mm_s2);
        warn_count++;
    }
    if (vehicle_control_target_accel_limit_mm_s2 < path_cfg->plan_long_accel_mm_s2)
    {
        tools_printf("{vcheck}warn,runtime_acc_below_plan,%.0f,%.0f\r\n",
                     (double)vehicle_control_target_accel_limit_mm_s2,
                     (double)path_cfg->plan_long_accel_mm_s2);
        warn_count++;
    }
    if (vehicle_control_target_decel_limit_mm_s2 < path_cfg->plan_long_decel_mm_s2)
    {
        tools_printf("{vcheck}warn,runtime_decel_below_plan,%.0f,%.0f\r\n",
                     (double)vehicle_control_target_decel_limit_mm_s2,
                     (double)path_cfg->plan_long_decel_mm_s2);
        warn_count++;
    }
    if (vehicle_control_speed_protection_enabled == FALSE)
    {
        tools_printf("{vcheck}warn,speed_protection_off\r\n");
        warn_count++;
    }
    if (vehicle_control_tracking_limit_active != FALSE)
    {
        tools_printf("{vcheck}warn,wheel_target_tracking_limited,%.0f,%.0f,%.0f,%.0f,evt,%u\r\n",
                     (double)vehicle_control_tracking_limit_left_before_mm_s,
                     (double)vehicle_control_tracking_limit_right_before_mm_s,
                     (double)vehicle_control_tracking_limit_left_after_mm_s,
                     (double)vehicle_control_tracking_limit_right_after_mm_s,
                     (unsigned int)vehicle_control_tracking_limit_event_count);
        warn_count++;
    }
    if (vehicle_control_reverse_feedback_cap_active != FALSE)
    {
        tools_printf("{vcheck}warn,reverse_feedback_guard_cap,base_cap,%.0f,evt,%u,fault,%u\r\n",
                     (double)vehicle_control_reverse_guard_base_cap_mm_s,
                     (unsigned int)vehicle_control_reverse_feedback_event_count,
                     (unsigned int)((vehicle_control_reverse_feedback_fault_latched != FALSE) ? 1u : 0u));
        warn_count++;
    }
    if ((module_vehicle_path_replay_planned_speed_active_get() != FALSE)
        && (planned_speed_meta_valid == FALSE))
    {
        tools_printf("{vcheck}warn,planned_speed_meta_missing,pcnt,%u,rawcnt,%u\r\n",
                     (unsigned int)planned_speed_point_cnt,
                     (unsigned int)path_state->point_cnt);
        warn_count++;
    }
    if ((module_vehicle_path_replay_planned_speed_active_get() != FALSE)
        && (planned_speed_meta_valid == FALSE)
        && (planned_speed_point_cnt != (uint32)path_state->point_cnt))
    {
        tools_printf("{vcheck}warn,planned_speed_index_scaled,pcnt,%u,rawcnt,%u\r\n",
                     (unsigned int)planned_speed_point_cnt,
                     (unsigned int)path_state->point_cnt);
        warn_count++;
    }
    if ((planned_speed_meta_valid != FALSE)
        && ((planned_speed_source_length_mm <= 0.0f)
            || (planned_speed_step_mm <= 0.0f)))
    {
        tools_printf("{vcheck}warn,planned_speed_distance_meta_invalid,slen,%.1f,pstep,%.3f\r\n",
                     (double)planned_speed_source_length_mm,
                     (double)planned_speed_step_mm);
        warn_count++;
    }
    if (cfg->yaw_speed_correction_max_mm_s > (vehicle_control_angle_diff_limit_mm_s * 0.5f))
    {
        tools_printf("{vcheck}warn,yaw_output_hidden_by_diff_limit,%.0f,%.0f\r\n",
                     (double)cfg->yaw_speed_correction_max_mm_s,
                     (double)(vehicle_control_angle_diff_limit_mm_s * 0.5f));
        warn_count++;
    }
    if ((replay_lookahead_mm <= 0.0f) || (replay_tangent_mm <= 0.0f))
    {
        tools_printf("{vcheck}warn,replay_ahead_invalid,%.0f,%.0f\r\n",
                     (double)replay_lookahead_mm,
                     (double)replay_tangent_mm);
        warn_count++;
    }
    if (replay_tangent_mm >= replay_lookahead_mm)
    {
        tools_printf("{vcheck}warn,tangent_not_less_than_lookahead,%.0f,%.0f\r\n",
                     (double)replay_tangent_mm,
                     (double)replay_lookahead_mm);
        warn_count++;
    }
    if ((vehicle_control_state.mode == VEHICLE_CONTROL_MODE_REPLAY)
        && (path_state->replaying == FALSE))
    {
        tools_printf("{vcheck}warn,control_replay_without_path\r\n");
        warn_count++;
    }
    if ((path_state->replaying != FALSE)
        && (vehicle_control_state.mode != VEHICLE_CONTROL_MODE_REPLAY))
    {
        tools_printf("{vcheck}warn,path_replay_without_control,%u\r\n",
                     (unsigned int)vehicle_control_state.mode);
        warn_count++;
    }
    if ((vehicle_control_state.mode == VEHICLE_CONTROL_MODE_REPLAY)
        && (vehicle_control_replay_launch_gate_active == FALSE)
        && (vehicle_control_state.replay_target_valid == FALSE))
    {
        tools_printf("{vcheck}warn,replay_no_target_after_gate\r\n");
        warn_count++;
    }

    tools_printf("{vcheck}summary,warn,%u\r\n", (unsigned int)warn_count);
}

void vehicle_control_set_left_duty(sint32 duty_cycle)
{
    vehicle_control_state.left_duty = vehicle_control_drive_clamp_duty(VEHICLE_ESC_ROLE_LEFT_DRIVE, duty_cycle);
}

/**
 * @brief 设置缓存的右驱动电调占空命令但不立即写入硬件。
 * @param[in] duty_cycle 目标占空命令，单位：GTM PWM ticks。
 * @return void
 */
void vehicle_control_set_right_duty(sint32 duty_cycle)
{
    vehicle_control_state.right_duty = vehicle_control_drive_clamp_duty(VEHICLE_ESC_ROLE_RIGHT_DRIVE, duty_cycle);
}

/**
 * @brief 设置缓存的负压电调占空命令但不立即写入硬件。
 * @param[in] duty_cycle 目标占空命令，单位：GTM PWM ticks。
 * @return void
 */
void vehicle_control_set_suction_duty(uint32 duty_cycle)
{
    vehicle_control_state.suction_duty = module_vehicle_esc_clamp_duty(VEHICLE_ESC_ROLE_SUCTION, duty_cycle);
}

/**
 * @brief 同时设置缓存的左驱动、右驱动和负压电调占空命令。
 * @param[in] left_duty 左驱动占空命令，单位：GTM PWM ticks。
 * @param[in] right_duty 右驱动占空命令，单位：GTM PWM ticks。
 * @param[in] suction_duty 负压占空命令，单位：GTM PWM ticks。
 * @return void
 */
void vehicle_control_set_duties(sint32 left_duty, sint32 right_duty, uint32 suction_duty)
{
    vehicle_control_set_left_duty(left_duty);
    vehicle_control_set_right_duty(right_duty);
    vehicle_control_set_suction_duty(suction_duty);
}

/**
 * @brief 开始路径记录并切换到记录模式。
 * @param[in] void 无参数。
 * @return 启动成功返回 TRUE，否则返回 FALSE。
 */
boolean vehicle_control_record_start(void)
{
    if (vehicle_control_state.mode == VEHICLE_CONTROL_MODE_REPLAY)
    {
        module_vehicle_path_replay_stop();
        vehicle_control_state.replay_target_valid = FALSE;
    }

    if (module_vehicle_path_start() == FALSE)
    {
        return FALSE;
    }

    vehicle_control_state.mode = VEHICLE_CONTROL_MODE_RECORD;
    return TRUE;
}

/**
 * @brief 停止路径记录并恢复空闲或手动模式。
 * @param[in] void 无参数。
 * @return void
 */
void vehicle_control_record_stop(void)
{
    module_vehicle_path_stop();

    if (vehicle_control_state.enabled != FALSE)
    {
        vehicle_control_state.mode = VEHICLE_CONTROL_MODE_MANUAL;
    }
    else
    {
        vehicle_control_state.mode = VEHICLE_CONTROL_MODE_IDLE;
    }
}

/**
 * @brief 开始路径回放等待或回放流程。
 * @param[in] void 无参数。
 * @return 启动成功返回 TRUE，否则返回 FALSE。
 */
boolean vehicle_control_replay_start(void)
{
    vehicle_path_replay_target_t replay_target;

    if (vehicle_control_state.mode == VEHICLE_CONTROL_MODE_RECORD)
    {
        module_vehicle_path_stop();
    }

    vehicle_control_pid_reset_all();
    vehicle_control_state.replay_target_valid = FALSE;
    vehicle_control_state.replay_delay_cnt = 0u;
    vehicle_control_state.target_speed_mm_s = 0.0f;
    vehicle_control_state.target_left_speed_mm_s = 0.0f;
    vehicle_control_state.target_right_speed_mm_s = 0.0f;
    vehicle_control_state.yaw_speed_correction_mm_s = 0.0f;
    vehicle_control_yaw_feedback_correction_mm_s = 0.0f;
    vehicle_control_effective_yaw_speed_correction_mm_s = 0.0f;
    vehicle_control_state.target_theta_rad = module_vehicle_pose_fusion_heading_get();
    vehicle_control_speed_target_runtime_reset();
    vehicle_control_state.mode = VEHICLE_CONTROL_MODE_REPLAY;
    vehicle_control_replay_launch_gate_start();
    vehicle_control_replay_path_lookahead_speed_cap_update();

    if (module_vehicle_path_replay_start() == FALSE)
    {
        module_vehicle_path_replay_lookahead_speed_cap_set(0.0f);
        vehicle_control_replay_launch_gate_stop();
        vehicle_control_state.mode = (vehicle_control_state.enabled != FALSE) ? VEHICLE_CONTROL_MODE_MANUAL
                                                                              : VEHICLE_CONTROL_MODE_IDLE;
        return FALSE;
    }

    /* Path replay resets the fused pose to the imported path origin. Capture
     * the launch heading after that reset so the drag correction cannot use
     * the heading left by the previous run. */
    vehicle_control_replay_launch_reference_theta_rad =
        module_vehicle_pose_fusion_observation_get()->theta_accum_rad;
    vehicle_control_state.target_theta_rad =
        vehicle_control_replay_launch_reference_theta_rad;

    if (module_vehicle_path_replay_target_get(&replay_target) != FALSE)
    {
        vehicle_control_replay_launch_target_limit(&replay_target);
        vehicle_control_state.replay_target = replay_target;
        vehicle_control_state.replay_target_valid = TRUE;
        vehicle_control_state.target_theta_rad = replay_target.target_theta_rad;
        vehicle_control_set_speed(replay_target.target_speed_mm_s);
    }

    return TRUE;
}

/**
 * @brief 停止路径回放并恢复空闲或手动模式。
 * @param[in] void 无参数。
 * @return void
 */
void vehicle_control_replay_stop(void)
{
    module_vehicle_path_replay_stop();
    module_vehicle_path_replay_lookahead_speed_cap_set(0.0f);
    vehicle_control_state.replay_target_valid = FALSE;
    vehicle_control_replay_launch_gate_stop();
    vehicle_control_state.yaw_speed_correction_mm_s = 0.0f;
    vehicle_control_yaw_feedback_correction_mm_s = 0.0f;
    vehicle_control_effective_yaw_speed_correction_mm_s = 0.0f;
    vehicle_control_tracking_limit_left_before_mm_s = 0.0f;
    vehicle_control_tracking_limit_right_before_mm_s = 0.0f;
    vehicle_control_tracking_limit_left_after_mm_s = 0.0f;
    vehicle_control_tracking_limit_right_after_mm_s = 0.0f;
    vehicle_control_tracking_limit_active = FALSE;

    if (vehicle_control_state.enabled != FALSE)
    {
        vehicle_control_state.mode = VEHICLE_CONTROL_MODE_MANUAL;
    }
    else
    {
        vehicle_control_state.mode = VEHICLE_CONTROL_MODE_IDLE;
    }
}

/**
 * @brief 获取车体控制模块只读状态。
 * @param[in] void 无参数。
 * @return 车体控制模块只读状态指针。
 */
void vehicle_control_replay_debug_print_process(void)
{
    static uint32 print_divider = 0u;
    static uint32 slip_print_divider = 0u;
    static uint64 last_print_tick = 0u;
    static boolean last_print_tick_valid = FALSE;
    static vehicle_control_replay_launch_phase_t last_launch_phase =
        VEHICLE_CONTROL_REPLAY_LAUNCH_PHASE_OFF;
    static boolean last_launch_phase_valid = FALSE;
    const module_vehicle_pose_fusion_observation_t* pose_observation;
    const module_vehicle_pose_fusion_replay_correction_t* replay_correction;
    const module_vehicle_encoder_observation_t* encoder_observation;
    const vehicle_path_state_t* path_state;
    const module_vehicle_esc_role_status_t* left_esc_status;
    const module_vehicle_esc_role_status_t* right_esc_status;
    uint64 now_tick;
    uint64 dt_us;
    float32 angle_error_deg;
    float32 yaw_rate_deg_s;
    float32 target_yaw_rate_deg_s;

    if ((vehicle_control_state.mode != VEHICLE_CONTROL_MODE_REPLAY)
        || (vehicle_control_state.replay_target_valid == FALSE))
    {
        print_divider = 0u;
        slip_print_divider = 0u;
        last_print_tick_valid = FALSE;
        last_launch_phase_valid = FALSE;
        return;
    }

    print_divider++;
    if (print_divider < VEHICLE_CONTROL_REPLAY_DEBUG_PRINT_DIVIDER)
    {
        return;
    }
    print_divider = 0u;

    pose_observation = module_vehicle_pose_fusion_observation_get();
    replay_correction = module_vehicle_pose_fusion_replay_correction_get();
    encoder_observation = module_vehicle_encoder_observation_get();
    path_state = module_vehicle_path_state_get();
    left_esc_status = module_vehicle_esc_role_status_get(VEHICLE_ESC_ROLE_LEFT_DRIVE);
    right_esc_status = module_vehicle_esc_role_status_get(VEHICLE_ESC_ROLE_RIGHT_DRIVE);
    now_tick = sysTick_getTick(SYSTICK1);
    dt_us = (last_print_tick_valid != FALSE)
          ? sysTick_ticksToMicroseconds(SYSTICK1, now_tick - last_print_tick)
          : 0u;
    last_print_tick = now_tick;
    last_print_tick_valid = TRUE;
    angle_error_deg =
        algorithm_attitude_wrap_pi(vehicle_control_state.target_theta_rad - pose_observation->theta_accum_rad)
        / VEHICLE_CONTROL_DEG_TO_RAD;
    yaw_rate_deg_s = module_vehicle_gyro_yaw_rate_get() / VEHICLE_CONTROL_DEG_TO_RAD;
    target_yaw_rate_deg_s =
        vehicle_control_state.replay_target.target_yaw_rate_rad_s / VEHICLE_CONTROL_DEG_TO_RAD;

    if (vehicle_control_turn_slip_event_pending != FALSE)
    {
        tools_printf("{vslipevt}%u,%u,%.1f,%.1f,%.3f,%.2f,%.2f,%.2f,%.2f\r\n",
                     (unsigned int)vehicle_control_turn_slip_event_id,
                     (unsigned int)vehicle_control_turn_slip_pending_path_index,
                     (double)vehicle_control_turn_slip_pending_x_mm,
                     (double)vehicle_control_turn_slip_pending_y_mm,
                     (double)vehicle_control_turn_slip_pending_ratio,
                     (double)vehicle_control_turn_slip_pending_encoder_turn_deg,
                     (double)vehicle_control_turn_slip_pending_imu_turn_deg,
                     (double)vehicle_control_turn_slip_pending_correction_mm,
                     (double)replay_correction->total_correction_mm);
        vehicle_control_turn_slip_event_pending = FALSE;
    }
    vehicle_control_slip_event_print();

    if (vehicle_control_replay_binary_telemetry_enabled != FALSE)
    {
        vehicle_control_replay_binary_sample_print(dt_us,
                                                   pose_observation,
                                                   encoder_observation,
                                                   path_state,
                                                   left_esc_status,
                                                   right_esc_status,
                                                   angle_error_deg,
                                                   yaw_rate_deg_s,
                                                   target_yaw_rate_deg_s);
    }
    else
    {
        tools_printf("{vrpl0}%llu,%u,%u,%.0f,%u,%u,%u\r\n",
                 (unsigned long long)dt_us,
                 (unsigned int)path_state->replay_cursor_cnt,
                 (unsigned int)vehicle_control_state.replay_target.target_index_cnt,
                 (double)vehicle_control_state.replay_target.lookahead_distance_mm,
                 (unsigned int)vehicle_control_state.replay_target.curve_section_active,
                 (unsigned int)path_state->point_cnt,
                 (unsigned int)vehicle_control_state.replay_target.track_mode);
        tools_printf("{vrpl1}%.1f,%.1f,%.2f,%.1f,%.1f,%.2f,%.1f,%.1f,%.1f,%.1f\r\n",
                 (double)pose_observation->x_mm,
                 (double)pose_observation->y_mm,
                 (double)(pose_observation->theta_accum_rad / VEHICLE_CONTROL_DEG_TO_RAD),
                 (double)vehicle_control_state.replay_target.base_point.x_mm,
                 (double)vehicle_control_state.replay_target.base_point.y_mm,
                 (double)(vehicle_control_state.replay_target.base_point.theta_rad / VEHICLE_CONTROL_DEG_TO_RAD),
                 (double)vehicle_control_state.replay_target.cross_track_error_mm,
                 (double)vehicle_control_state.replay_target.along_track_error_mm,
                 (double)vehicle_control_state.replay_target.target_point.x_mm,
                 (double)vehicle_control_state.replay_target.target_point.y_mm);
        tools_printf("{vrpl2}%.2f,%.2f,%.2f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.1f,%.1f,%.0f,%.0f,%.0f,%.0f\r\n",
                 (double)(vehicle_control_state.target_theta_rad / VEHICLE_CONTROL_DEG_TO_RAD),
                 (double)(vehicle_control_state.replay_target.feedforward_theta_rad / VEHICLE_CONTROL_DEG_TO_RAD),
                 (double)angle_error_deg,
                 (double)vehicle_control_state.replay_target.target_speed_mm_s,
                 (double)vehicle_control_state.target_left_speed_mm_s,
                 (double)vehicle_control_state.target_right_speed_mm_s,
                 (double)encoder_observation->left_speed_mm_s,
                 (double)encoder_observation->right_speed_mm_s,
                 (double)vehicle_control_state.yaw_speed_correction_mm_s,
                 (double)yaw_rate_deg_s,
                 (double)target_yaw_rate_deg_s,
                 (double)vehicle_control_state.replay_target.yaw_ff_mm_s,
                 (double)vehicle_control_effective_yaw_speed_correction_mm_s,
                 (double)vehicle_control_state.replay_target.raw_plan_speed_mm_s,
                 (double)vehicle_control_state.replay_target.speed_preview_limit_mm_s);
        tools_printf("{vrpl3}%d,%d,%d,%d,%u,%u,%u,%u\r\n",
                 (int)vehicle_control_state.left_duty,
                 (int)vehicle_control_state.right_duty,
                 (int)vehicle_control_left_output_duty,
                 (int)vehicle_control_right_output_duty,
                 (unsigned int)left_esc_status->state,
                 (unsigned int)left_esc_status->fault,
                 (unsigned int)right_esc_status->state,
                 (unsigned int)right_esc_status->fault);
    }
    slip_print_divider++;
    if (slip_print_divider >= VEHICLE_CONTROL_REPLAY_SLIP_PRINT_DIVIDER)
    {
        slip_print_divider = 0u;
        if (vehicle_control_replay_binary_telemetry_enabled != FALSE)
        {
            vehicle_control_replay_binary_slip_print(replay_correction);
        }
        else
        {
            tools_printf("{vslip}%u,%u,%u,%.2f,%.2f,%.3f,%.2f,%.2f,%.2f,%.2f,%u,%u,%u,%u,%.0f,%d\r\n",
                     (unsigned int)((replay_correction->active != FALSE) ? 1u : 0u),
                     (unsigned int)((replay_correction->slip_confirmed != FALSE) ? 1u : 0u),
                     (unsigned int)((replay_correction->straight_release_ready != FALSE) ? 1u : 0u),
                     (double)(replay_correction->encoder_turn_rad / VEHICLE_CONTROL_DEG_TO_RAD),
                     (double)(replay_correction->imu_turn_rad / VEHICLE_CONTROL_DEG_TO_RAD),
                     (double)replay_correction->yaw_realization_ratio,
                     (double)replay_correction->pending_correction_mm,
                     (double)replay_correction->total_correction_mm,
                     (double)replay_correction->correction_x_mm,
                     (double)replay_correction->correction_y_mm,
                     (unsigned int)((replay_correction->slip_window_ready != FALSE) ? 1u : 0u),
                     (unsigned int)replay_correction->slip_window_bucket_count,
                     (unsigned int)replay_correction->slip_window_valid_count,
                     (unsigned int)replay_correction->slip_reject_reason,
                     (double)(replay_correction->slip_window_duration_s * 1000.0f),
                     (int)replay_correction->slip_turn_sign);
        }
    }
#if (VEHICLE_CONTROL_REPLAY_EXTENDED_DEBUG_ENABLE != 0u)
    tools_printf("{vrpl4}%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.0f,%.0f\r\n",
                 (double)vehicle_control_state.replay_target.base_index_float,
                 (double)vehicle_control_state.replay_target.target_index_float,
                 (double)vehicle_control_state.replay_target.tangent_index_float,
                 (double)vehicle_control_state.replay_target.base_point.x_mm,
                 (double)vehicle_control_state.replay_target.base_point.y_mm,
                 (double)vehicle_control_state.replay_target.target_point.x_mm,
                 (double)vehicle_control_state.replay_target.target_point.y_mm,
                 (double)vehicle_control_state.replay_target.tangent_point.x_mm,
                 (double)vehicle_control_state.replay_target.tangent_point.y_mm,
                 (double)vehicle_control_state.replay_target.lookahead_distance_mm,
                 (double)vehicle_control_state.replay_target.tangent_distance_mm);
    tools_printf("{vrpl5}%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%u,%u,%u,%.0f,%.0f,%.0f,%.0f,%.0f,%u,%u\r\n",
                 (double)vehicle_control_reverse_guard_left_before_mm_s,
                 (double)vehicle_control_reverse_guard_right_before_mm_s,
                 (double)vehicle_control_reverse_guard_left_after_mm_s,
                 (double)vehicle_control_reverse_guard_right_after_mm_s,
                 (double)(vehicle_control_left_reverse_feedback_timer_s * 1000.0f),
                 (double)(vehicle_control_right_reverse_feedback_timer_s * 1000.0f),
                 (double)(vehicle_control_reverse_feedback_cap_hold_timer_s * 1000.0f),
                 (unsigned int)((vehicle_control_reverse_feedback_cap_active != FALSE) ? 1u : 0u),
                 (unsigned int)((vehicle_control_reverse_feedback_fault_latched != FALSE) ? 1u : 0u),
                 (unsigned int)vehicle_control_reverse_feedback_event_count,
                 (double)vehicle_control_reverse_guard_base_cap_mm_s,
                 (double)vehicle_control_tracking_limit_left_before_mm_s,
                 (double)vehicle_control_tracking_limit_right_before_mm_s,
                 (double)vehicle_control_tracking_limit_left_after_mm_s,
                 (double)vehicle_control_tracking_limit_right_after_mm_s,
                 (unsigned int)((vehicle_control_tracking_limit_active != FALSE) ? 1u : 0u),
                 (unsigned int)vehicle_control_tracking_limit_event_count);
    tools_printf("{vrpl6}%d,%d,%d,%d,%u,%u,%.3f,%.3f,%.3f,%.3f,%.0f,%.0f,%.0f,%.0f,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,%d\r\n",
                 (int)encoder_observation->left_delta_count,
                 (int)encoder_observation->right_delta_count,
                 (int)encoder_observation->left_raw_delta_count,
                 (int)encoder_observation->right_raw_delta_count,
                 (unsigned int)encoder_observation->left_raw_angle,
                 (unsigned int)encoder_observation->right_raw_angle,
                 (double)encoder_observation->left_sample_dt_s,
                 (double)encoder_observation->right_sample_dt_s,
                 (double)encoder_observation->left_distance_mm,
                 (double)encoder_observation->right_distance_mm,
                 (double)encoder_observation->left_raw_speed_mm_s,
                 (double)encoder_observation->right_raw_speed_mm_s,
                 (double)encoder_observation->left_speed_mm_s,
                 (double)encoder_observation->right_speed_mm_s,
                 (unsigned int)encoder_observation->left_invalid_sample_count,
                 (unsigned int)encoder_observation->right_invalid_sample_count,
                 (unsigned int)encoder_observation->left_invalid_streak_count,
                 (unsigned int)encoder_observation->right_invalid_streak_count,
                 (unsigned int)encoder_observation->left_rejected_raw_angle,
                 (unsigned int)encoder_observation->right_rejected_raw_angle,
                 (unsigned int)encoder_observation->left_sample_count,
                 (unsigned int)encoder_observation->right_sample_count,
                 (unsigned int)encoder_observation->update_count,
                 (unsigned int)encoder_observation->encoder_drop_count,
                 (int)encoder_observation->left_total_count,
                 (int)encoder_observation->right_total_count);
    tools_printf("{vrpl7}%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%u,%u\r\n",
                 (double)vehicle_control_left_speed_pid_p_output,
                 (double)vehicle_control_right_speed_pid_p_output,
                 (double)vehicle_control_left_speed_pid_i_output,
                 (double)vehicle_control_right_speed_pid_i_output,
                 (double)vehicle_control_left_speed_pid_d_output,
                 (double)vehicle_control_right_speed_pid_d_output,
                 (double)vehicle_control_left_speed_feedback_output,
                 (double)vehicle_control_right_speed_feedback_output,
                 (double)vehicle_control_left_speed_feedforward_output,
                 (double)vehicle_control_right_speed_feedforward_output,
                 (double)vehicle_control_left_speed_assist_output,
                 (double)vehicle_control_right_speed_assist_output,
                 (double)vehicle_control_left_speed_total_output,
                 (double)vehicle_control_right_speed_total_output,
                 (unsigned int)((vehicle_control_left_speed_integral_enabled != FALSE) ? 1u : 0u),
                 (unsigned int)((vehicle_control_right_speed_integral_enabled != FALSE) ? 1u : 0u));
#endif

    if ((vehicle_control_replay_launch_phase != VEHICLE_CONTROL_REPLAY_LAUNCH_PHASE_OFF)
        || (last_launch_phase_valid == FALSE)
        || (vehicle_control_replay_launch_phase != last_launch_phase))
    {
        tools_printf("{vrplg}%u,%.0f,%.0f,%.1f,%.0f,%u,%.0f\r\n",
                     (unsigned int)vehicle_control_replay_launch_phase,
                     (double)(vehicle_control_replay_launch_gate_timer_s * 1000.0f),
                     (double)(vehicle_control_replay_launch_gate_stable_timer_s * 1000.0f),
                     (double)vehicle_control_state.replay_target.cross_track_error_mm,
                     (double)vehicle_control_state.replay_target.target_speed_mm_s,
                     (unsigned int)vehicle_control_replay_launch_wheel_ready,
                     (double)(vehicle_control_replay_launch_wheel_ready_timer_s * 1000.0f));
    }
    last_launch_phase = vehicle_control_replay_launch_phase;
    last_launch_phase_valid = TRUE;

#if (VEHICLE_CONTROL_REPLAY_EXTENDED_DEBUG_ENABLE != 0u)
    tools_printf("{vrplm}%u,%u,%u,%.3f,%.1f,%.1f,%.2f,%.2f\r\n",
                 (unsigned int)vehicle_control_state.replay_target.track_mode,
                 (unsigned int)vehicle_control_state.replay_target.marker_segment_cnt,
                 (unsigned int)vehicle_control_state.replay_target.marker_next_cnt,
                 (double)vehicle_control_state.replay_target.marker_blend_ratio,
                 (double)vehicle_control_state.replay_target.marker_cross_track_error_mm,
                 (double)vehicle_control_state.replay_target.dense_cross_track_error_mm,
                 (double)(vehicle_control_state.replay_target.marker_target_theta_rad / VEHICLE_CONTROL_DEG_TO_RAD),
                 (double)(vehicle_control_state.replay_target.dense_target_theta_rad / VEHICLE_CONTROL_DEG_TO_RAD));
#endif
}

void vehicle_control_wheel_circumference_command(uint8 argc, uint8* argv[])
{
    const module_vehicle_pose_fusion_observation_t* pose_observation;
    float32 circumference_mm;
    float32 pose_x_mm;
    float32 pose_y_mm;
    float32 pose_theta_rad;

    if ((argc < 2u)
        || (vehicle_control_command_word_is(argv[1], "status") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "s") != FALSE))
    {
        tools_printf("{vwcirc}%.3f,default,%.3f,min,%.1f,max,%.1f\r\n",
                     (double)module_vehicle_encoder_wheel_circumference_get(),
                     (double)vehicle_encoder_cfg_get()->wheel_circumference_mm,
                     (double)VEHICLE_CONTROL_WHEEL_CIRCUMFERENCE_MIN_MM,
                     (double)VEHICLE_CONTROL_WHEEL_CIRCUMFERENCE_MAX_MM);
        return;
    }

    if ((vehicle_control_state.mode != VEHICLE_CONTROL_MODE_IDLE)
        || (vehicle_control_speed_test_enabled != FALSE)
        || (vehicle_control_load_test_state.step != VEHICLE_CONTROL_LOAD_TEST_STEP_IDLE))
    {
        tools_printf("{vwcirc}busy,stop_vehicle_first\r\n");
        return;
    }

    circumference_mm = vehicle_control_command_float_get(argv[1]);
    if ((circumference_mm < VEHICLE_CONTROL_WHEEL_CIRCUMFERENCE_MIN_MM)
        || (circumference_mm > VEHICLE_CONTROL_WHEEL_CIRCUMFERENCE_MAX_MM))
    {
        tools_printf("{vwcirc}range,%.1f,%.1f\r\n",
                     (double)VEHICLE_CONTROL_WHEEL_CIRCUMFERENCE_MIN_MM,
                     (double)VEHICLE_CONTROL_WHEEL_CIRCUMFERENCE_MAX_MM);
        return;
    }

    pose_observation = module_vehicle_pose_fusion_observation_get();
    pose_x_mm = pose_observation->x_mm;
    pose_y_mm = pose_observation->y_mm;
    pose_theta_rad = pose_observation->theta_rad;
    module_vehicle_encoder_wheel_circumference_set(circumference_mm);
    module_vehicle_encoder_reset_baseline();
    module_vehicle_pose_fusion_reset(pose_x_mm, pose_y_mm, pose_theta_rad);
    vehicle_control_pid_reset_all();
    vehicle_control_speed_target_runtime_reset();
    tools_printf("{vwcirc}ok,%.3f,volatile,1\r\n", (double)circumference_mm);
}

void vehicle_control_accel_slip_command(uint8 argc, uint8* argv[])
{
    if ((argc < 2u)
        || (vehicle_control_command_word_is(argv[1], "status") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "s") != FALSE))
    {
        tools_printf("{vslipacc}enable,%u,gravity,%u,active,%u,enc,%.1f,imu,%.1f,ratio,%.3f\r\n",
                     (unsigned int)((vehicle_control_accel_slip_enabled != FALSE) ? 1u : 0u),
                     (unsigned int)((module_vehicle_gyro_observation_get()->gravity_ready != FALSE) ? 1u : 0u),
                     (unsigned int)((vehicle_control_accel_slip_active != FALSE) ? 1u : 0u),
                     (double)vehicle_control_accel_slip_encoder_accel_mm_s2,
                     (double)vehicle_control_accel_slip_imu_accel_mm_s2,
                     (double)vehicle_control_accel_slip_ratio);
        return;
    }
    if ((vehicle_control_command_word_is(argv[1], "on") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "enable") != FALSE))
    {
        vehicle_control_accel_slip_enabled = TRUE;
        vehicle_control_accel_slip_reset();
        tools_printf("{vslipacc}enable,1\r\n");
        return;
    }
    if ((vehicle_control_command_word_is(argv[1], "off") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "disable") != FALSE))
    {
        vehicle_control_accel_slip_enabled = FALSE;
        vehicle_control_accel_slip_reset();
        tools_printf("{vslipacc}enable,0\r\n");
        return;
    }
    tools_printf("{vslipacc}usage:on|off|status\r\n");
}

void vehicle_control_replay_telemetry_command(uint8 argc, uint8* argv[])
{
    if ((argc < 2u)
        || (vehicle_control_command_word_is(argv[1], "status") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "s") != FALSE))
    {
        tools_printf("{vtelemetry}mode,%s,version,%u,payload,%u,frame,%u\r\n",
                     (vehicle_control_replay_binary_telemetry_enabled != FALSE)
                         ? "binary" : "text",
                     (unsigned int)VEHICLE_CONTROL_REPLAY_BINARY_VERSION,
                     (unsigned int)VEHICLE_CONTROL_REPLAY_BINARY_PAYLOAD_SIZE,
                     (unsigned int)VEHICLE_CONTROL_REPLAY_BINARY_FRAME_SIZE);
        return;
    }
    if ((vehicle_control_command_word_is(argv[1], "binary") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "bin") != FALSE))
    {
        vehicle_control_replay_binary_telemetry_enabled = TRUE;
        tools_printf("{vtelemetry}mode,binary\r\n");
        return;
    }
    if ((vehicle_control_command_word_is(argv[1], "text") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "ascii") != FALSE))
    {
        vehicle_control_replay_binary_telemetry_enabled = FALSE;
        tools_printf("{vtelemetry}mode,text\r\n");
        return;
    }
    tools_printf("{vtelemetry}usage:binary|text|status\r\n");
}

void vehicle_control_launch_gate_command(uint8 argc, uint8* argv[])
{
    if ((argc < 2u)
        || (vehicle_control_command_word_is(argv[1], "status") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "s") != FALSE))
    {
        tools_printf("{vlaunch}drag,%u,dragcorr,%u,correction,%u,replay_active,%u,test_active,%u\r\n",
                     (unsigned int)((vehicle_control_launch_drag_enabled != FALSE) ? 1u : 0u),
                     (unsigned int)((vehicle_control_launch_drag_correction_enabled != FALSE) ? 1u : 0u),
                     (unsigned int)((vehicle_control_launch_correction_enabled != FALSE) ? 1u : 0u),
                     (unsigned int)((vehicle_control_replay_launch_gate_active != FALSE) ? 1u : 0u),
                     (unsigned int)((vehicle_control_test_launch_gate_active != FALSE) ? 1u : 0u));
        return;
    }

    if ((vehicle_control_command_word_is(argv[1], "on") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "enable") != FALSE))
    {
        vehicle_control_launch_drag_enabled = TRUE;
        vehicle_control_launch_drag_correction_enabled = FALSE;
        vehicle_control_launch_correction_enabled = TRUE;
        tools_printf("{vlaunch}drag,1,dragcorr,0,correction,1,replay_active,%u,test_active,%u\r\n",
                     (unsigned int)((vehicle_control_replay_launch_gate_active != FALSE) ? 1u : 0u),
                     (unsigned int)((vehicle_control_test_launch_gate_active != FALSE) ? 1u : 0u));
        return;
    }

    if ((vehicle_control_command_word_is(argv[1], "off") != FALSE)
        || (vehicle_control_command_word_is(argv[1], "disable") != FALSE)
        || (vehicle_control_command_is_zero(argv[1]) != FALSE))
    {
        vehicle_control_launch_drag_enabled = FALSE;
        vehicle_control_launch_drag_correction_enabled = FALSE;
        vehicle_control_launch_correction_enabled = FALSE;
        vehicle_control_replay_launch_gate_stop();
        vehicle_control_test_launch_gate_stop();
        tools_printf("{vlaunch}drag,0,dragcorr,0,correction,0,replay_active,0,test_active,0\r\n");
        return;
    }

    if ((vehicle_control_command_word_is(argv[1], "drag") != FALSE)
        && (argc >= 3u))
    {
        if ((vehicle_control_command_word_is(argv[2], "on") != FALSE)
            || (vehicle_control_command_word_is(argv[2], "enable") != FALSE))
        {
            vehicle_control_launch_drag_enabled = TRUE;
        }
        else if ((vehicle_control_command_word_is(argv[2], "off") != FALSE)
                 || (vehicle_control_command_word_is(argv[2], "disable") != FALSE)
                 || (vehicle_control_command_is_zero(argv[2]) != FALSE))
        {
            vehicle_control_launch_drag_enabled = FALSE;
            if (vehicle_control_replay_launch_drag_active() != FALSE)
            {
                vehicle_control_replay_launch_gate_stop();
            }
            if (vehicle_control_test_launch_drag_active() != FALSE)
            {
                vehicle_control_test_launch_gate_stop();
            }
        }
        else
        {
            tools_printf("{vlaunch}usage:vlaunch drag on|off\r\n");
            return;
        }

        tools_printf("{vlaunch}drag,%u,dragcorr,%u,correction,%u\r\n",
                     (unsigned int)((vehicle_control_launch_drag_enabled != FALSE) ? 1u : 0u),
                     (unsigned int)((vehicle_control_launch_drag_correction_enabled != FALSE) ? 1u : 0u),
                     (unsigned int)((vehicle_control_launch_correction_enabled != FALSE) ? 1u : 0u));
        return;
    }

    if (((vehicle_control_command_word_is(argv[1], "dragcorr") != FALSE)
         || (vehicle_control_command_word_is(argv[1], "dragcorrect") != FALSE))
        && (argc >= 3u))
    {
        if ((vehicle_control_command_word_is(argv[2], "on") != FALSE)
            || (vehicle_control_command_word_is(argv[2], "enable") != FALSE))
        {
            vehicle_control_launch_drag_correction_enabled = TRUE;
        }
        else if ((vehicle_control_command_word_is(argv[2], "off") != FALSE)
                 || (vehicle_control_command_word_is(argv[2], "disable") != FALSE)
                 || (vehicle_control_command_is_zero(argv[2]) != FALSE))
        {
            vehicle_control_launch_drag_correction_enabled = FALSE;
            vehicle_control_state.yaw_speed_correction_mm_s = 0.0f;
            vehicle_control_yaw_feedback_correction_mm_s = 0.0f;
            vehicle_control_effective_yaw_speed_correction_mm_s = 0.0f;
            algorithm_control_pid_reset(&vehicle_control_launch_angle_pid);
        }
        else
        {
            tools_printf("{vlaunch}usage:vlaunch dragcorr on|off\r\n");
            return;
        }

        tools_printf("{vlaunch}drag,%u,dragcorr,%u,correction,%u\r\n",
                     (unsigned int)((vehicle_control_launch_drag_enabled != FALSE) ? 1u : 0u),
                     (unsigned int)((vehicle_control_launch_drag_correction_enabled != FALSE) ? 1u : 0u),
                     (unsigned int)((vehicle_control_launch_correction_enabled != FALSE) ? 1u : 0u));
        return;
    }

    if (((vehicle_control_command_word_is(argv[1], "correction") != FALSE)
         || (vehicle_control_command_word_is(argv[1], "correct") != FALSE)
         || (vehicle_control_command_word_is(argv[1], "corr") != FALSE))
        && (argc >= 3u))
    {
        if ((vehicle_control_command_word_is(argv[2], "on") != FALSE)
            || (vehicle_control_command_word_is(argv[2], "enable") != FALSE))
        {
            vehicle_control_launch_correction_enabled = TRUE;
        }
        else if ((vehicle_control_command_word_is(argv[2], "off") != FALSE)
                 || (vehicle_control_command_word_is(argv[2], "disable") != FALSE)
                 || (vehicle_control_command_is_zero(argv[2]) != FALSE))
        {
            vehicle_control_launch_correction_enabled = FALSE;
            if (vehicle_control_launch_correction_active() != FALSE)
            {
                vehicle_control_replay_launch_gate_stop();
                vehicle_control_test_launch_gate_stop();
            }
        }
        else
        {
            tools_printf("{vlaunch}usage:vlaunch correction on|off\r\n");
            return;
        }

        tools_printf("{vlaunch}drag,%u,dragcorr,%u,correction,%u\r\n",
                     (unsigned int)((vehicle_control_launch_drag_enabled != FALSE) ? 1u : 0u),
                     (unsigned int)((vehicle_control_launch_drag_correction_enabled != FALSE) ? 1u : 0u),
                     (unsigned int)((vehicle_control_launch_correction_enabled != FALSE) ? 1u : 0u));
        return;
    }

    tools_printf("{vlaunch}usage:vlaunch on|off|status|drag on|off|dragcorr on|off|correction on|off\r\n");
}

const vehicle_control_state_t* vehicle_control_state_get(void)
{
    return &vehicle_control_state;
}

boolean vehicle_control_phototube_correction_allowed(void)
{
    const vehicle_path_state_t* path_state = module_vehicle_path_state_get();

    if (vehicle_control_state.enabled == FALSE)
    {
        return FALSE;
    }
    if (vehicle_control_state.mode != VEHICLE_CONTROL_MODE_REPLAY)
    {
        return FALSE;
    }
    if (vehicle_control_state.replay_target_valid == FALSE)
    {
        return FALSE;
    }
    if (vehicle_control_replay_launch_gate_active != FALSE)
    {
        return FALSE;
    }
    if ((path_state == NULL_PTR) || (path_state->status != VEHICLE_PATH_STATUS_REPLAY))
    {
        return FALSE;
    }

    return TRUE;
}

static void vehicle_control_command_queue_reset(void)
{
    vehicle_control_command_queue.head = 0u;
    vehicle_control_command_queue.tail = 0u;
    vehicle_control_command_queue.dropped_count = 0u;
}

static boolean vehicle_control_command_post_checked(const vehicle_control_command_t* command)
{
    boolean result = vehicle_control_command_post(command);

    if (result == FALSE)
    {
        uint32 command_type = (command != NULL_PTR) ? (uint32)command->type : 0u;
        tools_printf("{vctrl}queue_full,%u\r\n", (unsigned int)command_type);
    }

    return result;
}

static boolean vehicle_control_command_pop(vehicle_control_command_t* command)
{
    uint8 head;
    uint8 next_head;

    if (command == NULL_PTR)
    {
        return FALSE;
    }

    head = vehicle_control_command_queue.head;
    if (head == vehicle_control_command_queue.tail)
    {
        return FALSE;
    }

    *command = vehicle_control_command_queue.buffer[head];
    next_head = head + 1u;
    if (next_head >= VEHICLE_CONTROL_COMMAND_QUEUE_LENGTH)
    {
        next_head = 0u;
    }
    vehicle_control_command_queue.head = next_head;
    return TRUE;
}

static void vehicle_control_command_execute(const vehicle_control_command_t* command)
{
    if (command == NULL_PTR)
    {
        return;
    }

    switch (command->type)
    {
        case VEHICLE_CONTROL_COMMAND_ENABLE:
            vehicle_control_enable(command->enable);
            break;

        case VEHICLE_CONTROL_COMMAND_SET_SPEED:
            vehicle_control_set_speed(command->speed_mm_s);
            break;

        case VEHICLE_CONTROL_COMMAND_SET_THETA:
            vehicle_control_set_theta(command->theta_rad);
            break;

        case VEHICLE_CONTROL_COMMAND_SET_SUCTION:
            vehicle_control_suction_apply_duty(command->suction_duty);
            break;

        case VEHICLE_CONTROL_COMMAND_STOP_ALL:
            vehicle_control_enable(FALSE);
            vehicle_control_set_speed(command->speed_mm_s);
            vehicle_control_set_suction_duty(command->suction_duty);
            vehicle_control_apply_idle_outputs();
            (void)module_vehicle_esc_stop(VEHICLE_ESC_ROLE_LEFT_DRIVE);
            (void)module_vehicle_esc_stop(VEHICLE_ESC_ROLE_RIGHT_DRIVE);
            break;

        case VEHICLE_CONTROL_COMMAND_DRIVE_STOP_KEEP_SUCTION:
            vehicle_control_enable(FALSE);
            vehicle_control_set_left_duty(command->left_duty);
            vehicle_control_set_right_duty(command->right_duty);
            vehicle_control_set_suction_duty(command->suction_duty);
            vehicle_control_apply_idle_outputs();
            break;

        case VEHICLE_CONTROL_COMMAND_REPLAY_START:
            vehicle_control_speed_test_enable(FALSE);
            vehicle_control_enable(TRUE);
            vehicle_control_set_suction_duty(command->suction_duty);
            vehicle_control_apply_outputs();
            (void)vehicle_control_replay_start();
            vehicle_control_apply_outputs();
            break;

        case VEHICLE_CONTROL_COMMAND_REPLAY_STOP:
            vehicle_control_replay_stop();
            break;

        case VEHICLE_CONTROL_COMMAND_SPEED_TEST_TARGET:
            vehicle_control_load_test_cancel();
            vehicle_control_speed_test_heading_enabled = FALSE;
            vehicle_control_speed_test_set_target(command->left_speed_mm_s,
                                                   command->right_speed_mm_s);
            vehicle_control_speed_test_enable(TRUE);
            break;

        case VEHICLE_CONTROL_COMMAND_SPEED_TEST_HEADING_TARGET:
            vehicle_control_speed_test_set_target(command->speed_mm_s,
                                                  command->speed_mm_s);
            vehicle_control_speed_test_enable(TRUE);
            if (vehicle_control_speed_test_heading_enabled == FALSE)
            {
                algorithm_control_pid_reset(&vehicle_control_state.angle_pid);
                vehicle_control_d_yaw_rate_lpf_reset();
                vehicle_control_target_yaw_rate_reset();
                vehicle_control_steer_loop_divider = 0u;
            }
            vehicle_control_test_launch_gate_stop();
            vehicle_control_set_theta(command->theta_rad);
            vehicle_control_speed_test_heading_enabled = TRUE;
            break;

        case VEHICLE_CONTROL_COMMAND_DRIVE_PWM:
            vehicle_control_speed_test_enabled = FALSE;
            vehicle_control_speed_test_stop_requested = FALSE;
            vehicle_control_speed_test_heading_enabled = FALSE;
            vehicle_control_speed_test_left_target_mm_s = 0.0f;
            vehicle_control_speed_test_right_target_mm_s = 0.0f;
            vehicle_control_state.enabled = FALSE;
            vehicle_control_state.mode = VEHICLE_CONTROL_MODE_IDLE;
            vehicle_control_state.replay_target_valid = FALSE;
            vehicle_control_load_test_cancel();
            vehicle_control_angle_test_cancel();
            vehicle_control_replay_launch_gate_stop();
            vehicle_control_test_launch_gate_stop();
            module_vehicle_path_replay_stop();
            module_vehicle_path_stop();
            vehicle_control_pid_reset_all();
            vehicle_control_speed_target_runtime_reset();
            vehicle_control_speed_accel_feedforward_reset();
            vehicle_control_set_left_duty(command->left_duty);
            vehicle_control_set_right_duty(command->right_duty);
            vehicle_control_apply_idle_outputs();
            break;

        case VEHICLE_CONTROL_COMMAND_SPEED_TEST_STOP:
            vehicle_control_speed_test_set_target(0.0f, 0.0f);
            vehicle_control_speed_test_stop_requested = TRUE;
            break;

        case VEHICLE_CONTROL_COMMAND_SPEED_TEST_ENABLE:
            vehicle_control_speed_test_enable(command->enable);
            break;

        case VEHICLE_CONTROL_COMMAND_LOAD_TEST_START:
            vehicle_control_speed_test_enable(TRUE);
            vehicle_control_speed_test_heading_enabled = FALSE;
            vehicle_control_test_launch_gate_stop();
            vehicle_control_test_launch_reference_theta_rad =
                module_vehicle_pose_fusion_heading_get();
            vehicle_control_state.yaw_speed_correction_mm_s = 0.0f;
            vehicle_control_yaw_feedback_correction_mm_s = 0.0f;
            vehicle_control_effective_yaw_speed_correction_mm_s = 0.0f;
            vehicle_control_pid_reset_all();
            vehicle_control_speed_target_runtime_reset();
            vehicle_control_speed_accel_feedforward_reset();
            vehicle_control_speed_test_set_target(0.0f, 0.0f);
            vehicle_control_suction_apply_duty(command->suction_duty);
            vehicle_control_load_test_state.step = VEHICLE_CONTROL_LOAD_TEST_STEP_WAIT_START;
            vehicle_control_load_test_state.timer_s = 0.0f;
            vehicle_control_load_test_state.left_speed_mm_s = command->left_speed_mm_s;
            vehicle_control_load_test_state.right_speed_mm_s = command->right_speed_mm_s;
            break;

        case VEHICLE_CONTROL_COMMAND_LOAD_TEST_STOP:
            vehicle_control_load_test_cancel();
            vehicle_control_suction_apply_duty(command->suction_duty);
            vehicle_control_speed_test_enable(FALSE);
            break;

        case VEHICLE_CONTROL_COMMAND_ANGLE_TEST_START:
            vehicle_control_speed_test_enable(FALSE);
            vehicle_control_enable(TRUE);
            vehicle_control_pid_reset_all();
            vehicle_control_set_speed(VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_SPEED_MM_S);
            vehicle_control_set_theta(command->theta_rad);
            vehicle_control_suction_apply_duty(command->suction_duty);
            vehicle_control_steer_loop_divider = 0u;
            vehicle_control_angle_test_print_reset();
            vehicle_control_angle_test_state.step = VEHICLE_CONTROL_ANGLE_TEST_STEP_WAIT_ANGLE;
            vehicle_control_angle_test_state.timer_s = 0.0f;
            vehicle_control_angle_test_state.step_deg = command->step_deg;
            vehicle_control_angle_test_state.speed_mm_s = command->speed_mm_s;
            vehicle_control_angle_test_state.suction_off_done = FALSE;
            vehicle_control_angle_test_state.reference_theta_rad = command->theta_rad;
            vehicle_control_angle_test_state.target_theta_rad = command->theta_rad;
            vehicle_control_test_launch_gate_start(command->theta_rad);
            break;

        case VEHICLE_CONTROL_COMMAND_ANGLE_TEST_STOP:
            vehicle_control_angle_test_cancel();
            vehicle_control_set_speed(0.0f);
            vehicle_control_suction_apply_duty(command->suction_duty);
            vehicle_control_enable(FALSE);
            break;

        case VEHICLE_CONTROL_COMMAND_TURN_START:
            vehicle_control_angle_test_cancel();
            vehicle_control_speed_test_enable(FALSE);
            vehicle_control_enable(TRUE);
            vehicle_control_pid_reset_all();
            vehicle_control_set_speed(0.0f);
            vehicle_control_set_theta(command->theta_rad);
            vehicle_control_suction_apply_duty(command->suction_duty);
            vehicle_control_steer_loop_divider = 0u;
            break;

        case VEHICLE_CONTROL_COMMAND_TURN_STOP:
            vehicle_control_angle_test_cancel();
            vehicle_control_speed_test_enable(FALSE);
            vehicle_control_set_speed(0.0f);
            vehicle_control_suction_apply_duty(command->suction_duty);
            vehicle_control_enable(FALSE);
            break;

        default:
            break;
    }
}

/**
 * @brief 清空车体控制模块内所有 PID 运行状态。
 * @param[in] void 无参数。
 * @return void
 */
static void vehicle_control_pid_reset_all(void)
{
    algorithm_control_pid_reset(&vehicle_control_state.angle_pid);
    algorithm_control_pid_reset(&vehicle_control_launch_angle_pid);
    algorithm_control_pid_reset(&vehicle_control_state.left_speed_pid);
    algorithm_control_pid_reset(&vehicle_control_state.right_speed_pid);
    vehicle_control_yaw_feedback_correction_mm_s = 0.0f;
    vehicle_control_effective_yaw_speed_correction_mm_s = 0.0f;
    vehicle_control_d_yaw_rate_lpf_reset();
    vehicle_control_target_yaw_rate_reset();
    vehicle_control_speed_accel_feedforward_reset();
}

static void vehicle_control_speed_accel_feedforward_reset(void)
{
    vehicle_control_speed_accel_feedforward_previous_target_mm_s = 0.0f;
    vehicle_control_speed_accel_feedforward_output = 0.0f;
    vehicle_control_speed_accel_feedforward_ready = FALSE;
}

static float32 vehicle_control_speed_accel_feedforward_update(float32 target_mm_s, float32 dt_s)
{
    float32 target_accel_mm_s2;
    float32 gain;

    if ((dt_s <= 0.0f)
        || (vehicle_control_replay_launch_gate_active != FALSE)
        || (vehicle_control_test_launch_gate_active != FALSE))
    {
        vehicle_control_speed_accel_feedforward_reset();
        return 0.0f;
    }

    if (vehicle_control_speed_accel_feedforward_ready == FALSE)
    {
        vehicle_control_speed_accel_feedforward_previous_target_mm_s = target_mm_s;
        vehicle_control_speed_accel_feedforward_ready = TRUE;
        return 0.0f;
    }

    target_accel_mm_s2 =
        (target_mm_s - vehicle_control_speed_accel_feedforward_previous_target_mm_s) / dt_s;
    vehicle_control_speed_accel_feedforward_previous_target_mm_s = target_mm_s;
    gain = (target_accel_mm_s2 >= 0.0f)
        ? vehicle_control_speed_accel_feedforward_gain
        : vehicle_control_speed_decel_feedforward_gain;
    vehicle_control_speed_accel_feedforward_output = algorithm_control_clamp_f32(
        gain * target_accel_mm_s2,
        -vehicle_control_speed_accel_feedforward_limit,
        vehicle_control_speed_accel_feedforward_limit);
    return vehicle_control_speed_accel_feedforward_output;
}

static void vehicle_control_accel_slip_reset(void)
{
    vehicle_control_accel_slip_active = FALSE;
    vehicle_control_accel_slip_speed_ready = FALSE;
    vehicle_control_accel_slip_previous_speed_mm_s = 0.0f;
    vehicle_control_accel_slip_encoder_accel_mm_s2 = 0.0f;
    vehicle_control_accel_slip_imu_accel_mm_s2 = 0.0f;
    vehicle_control_accel_slip_ratio = 1.0f;
    vehicle_control_accel_slip_confirm_timer_s = 0.0f;
    vehicle_control_accel_slip_release_timer_s = 0.0f;
    vehicle_control_accel_slip_pending_state = 0u;
}

static void vehicle_control_accel_slip_update(float32 dt_s)
{
    const module_vehicle_encoder_observation_t* encoder = module_vehicle_encoder_observation_get();
    const module_vehicle_gyro_observation_t* gyro = module_vehicle_gyro_observation_get();
    const module_vehicle_pose_fusion_observation_t* pose = module_vehicle_pose_fusion_observation_get();
    const vehicle_path_state_t* path_state = module_vehicle_path_state_get();
    float32 speed_mm_s;
    float32 raw_encoder_accel_mm_s2;
    float32 alpha;
    boolean candidate;

    if ((vehicle_control_accel_slip_enabled == FALSE)
        || (vehicle_control_state.mode != VEHICLE_CONTROL_MODE_REPLAY)
        || (vehicle_control_state.replay_target_valid == FALSE)
        || (vehicle_control_replay_launch_gate_active != FALSE)
        || (gyro->gravity_ready == FALSE)
        || (dt_s <= 0.0f))
    {
        vehicle_control_accel_slip_reset();
        return;
    }

    speed_mm_s = 0.5f * (encoder->left_speed_mm_s + encoder->right_speed_mm_s);
    if (vehicle_control_accel_slip_speed_ready == FALSE)
    {
        vehicle_control_accel_slip_previous_speed_mm_s = speed_mm_s;
        vehicle_control_accel_slip_speed_ready = TRUE;
        return;
    }

    raw_encoder_accel_mm_s2 =
        (speed_mm_s - vehicle_control_accel_slip_previous_speed_mm_s) / dt_s;
    vehicle_control_accel_slip_previous_speed_mm_s = speed_mm_s;
    alpha = algorithm_control_clamp_f32(
        dt_s / ((1.0f / (VEHICLE_CONTROL_TWO_PI * VEHICLE_CONTROL_ACCEL_SLIP_LPF_HZ)) + dt_s),
        0.0f,
        1.0f);
    vehicle_control_accel_slip_encoder_accel_mm_s2 +=
        alpha * (raw_encoder_accel_mm_s2 - vehicle_control_accel_slip_encoder_accel_mm_s2);
    vehicle_control_accel_slip_imu_accel_mm_s2 +=
        alpha * (gyro->linear_accel_mm_s2.x - vehicle_control_accel_slip_imu_accel_mm_s2);
    vehicle_control_accel_slip_ratio =
        (vehicle_control_accel_slip_encoder_accel_mm_s2
         > VEHICLE_CONTROL_ACCEL_SLIP_ENCODER_MIN_MM_S2)
        ? (vehicle_control_accel_slip_imu_accel_mm_s2
           / vehicle_control_accel_slip_encoder_accel_mm_s2)
        : 1.0f;

    candidate = ((vehicle_control_accel_slip_encoder_accel_mm_s2
                  >= VEHICLE_CONTROL_ACCEL_SLIP_ENCODER_MIN_MM_S2)
                 && ((vehicle_control_accel_slip_encoder_accel_mm_s2
                      - vehicle_control_accel_slip_imu_accel_mm_s2)
                     >= VEHICLE_CONTROL_ACCEL_SLIP_RESIDUAL_MIN_MM_S2)
                 && (vehicle_control_accel_slip_ratio
                     <= VEHICLE_CONTROL_ACCEL_SLIP_RATIO_MAX)
                 && (fabsf(gyro->gyro_z_rad_s)
                     <= VEHICLE_CONTROL_ACCEL_SLIP_YAW_RATE_MAX_RAD_S)
                 && (module_vehicle_pose_fusion_replay_path_straight_get() != FALSE));
    if (candidate != FALSE)
    {
        vehicle_control_accel_slip_release_timer_s = 0.0f;
        vehicle_control_accel_slip_confirm_timer_s += dt_s;
        if ((vehicle_control_accel_slip_active == FALSE)
            && (vehicle_control_accel_slip_confirm_timer_s
                >= VEHICLE_CONTROL_ACCEL_SLIP_CONFIRM_S))
        {
            vehicle_control_accel_slip_active = TRUE;
            vehicle_control_accel_slip_event_id++;
            vehicle_control_accel_slip_pending_state = 1u;
            vehicle_control_accel_slip_pending_path_index = path_state->replay_cursor_cnt;
            vehicle_control_accel_slip_pending_x_mm = pose->x_mm;
            vehicle_control_accel_slip_pending_y_mm = pose->y_mm;
            vehicle_control_accel_slip_pending_encoder_accel_mm_s2 =
                vehicle_control_accel_slip_encoder_accel_mm_s2;
            vehicle_control_accel_slip_pending_imu_accel_mm_s2 =
                vehicle_control_accel_slip_imu_accel_mm_s2;
            vehicle_control_accel_slip_pending_ratio = vehicle_control_accel_slip_ratio;
        }
        return;
    }

    vehicle_control_accel_slip_confirm_timer_s = 0.0f;
    if (vehicle_control_accel_slip_active != FALSE)
    {
        vehicle_control_accel_slip_release_timer_s += dt_s;
        if (vehicle_control_accel_slip_release_timer_s
            >= VEHICLE_CONTROL_ACCEL_SLIP_RELEASE_S)
        {
            vehicle_control_accel_slip_active = FALSE;
            vehicle_control_accel_slip_pending_state = 2u;
            vehicle_control_accel_slip_pending_path_index = path_state->replay_cursor_cnt;
            vehicle_control_accel_slip_pending_x_mm = pose->x_mm;
            vehicle_control_accel_slip_pending_y_mm = pose->y_mm;
            vehicle_control_accel_slip_pending_encoder_accel_mm_s2 =
                vehicle_control_accel_slip_encoder_accel_mm_s2;
            vehicle_control_accel_slip_pending_imu_accel_mm_s2 =
                vehicle_control_accel_slip_imu_accel_mm_s2;
            vehicle_control_accel_slip_pending_ratio = vehicle_control_accel_slip_ratio;
        }
    }
}

static void vehicle_control_slip_event_print(void)
{
    if (vehicle_control_accel_slip_pending_state == 0u)
    {
        return;
    }
    tools_printf("{vslipacc}%u,%u,%u,%.1f,%.1f,%.1f,%.1f,%.3f\r\n",
                 (unsigned int)vehicle_control_accel_slip_event_id,
                 (unsigned int)vehicle_control_accel_slip_pending_state,
                 (unsigned int)vehicle_control_accel_slip_pending_path_index,
                 (double)vehicle_control_accel_slip_pending_x_mm,
                 (double)vehicle_control_accel_slip_pending_y_mm,
                 (double)vehicle_control_accel_slip_pending_encoder_accel_mm_s2,
                 (double)vehicle_control_accel_slip_pending_imu_accel_mm_s2,
                 (double)vehicle_control_accel_slip_pending_ratio);
    vehicle_control_accel_slip_pending_state = 0u;
}

static void vehicle_control_turn_slip_event_capture(void)
{
    const module_vehicle_pose_fusion_replay_correction_t* correction =
        module_vehicle_pose_fusion_replay_correction_get();

    if ((vehicle_control_state.mode != VEHICLE_CONTROL_MODE_REPLAY)
        || (vehicle_control_state.replay_target_valid == FALSE))
    {
        vehicle_control_turn_slip_previous_confirmed = FALSE;
        return;
    }
    if ((correction->slip_confirmed != FALSE)
        && (vehicle_control_turn_slip_previous_confirmed == FALSE))
    {
        const module_vehicle_pose_fusion_observation_t* pose =
            module_vehicle_pose_fusion_observation_get();
        const vehicle_path_state_t* path_state = module_vehicle_path_state_get();

        vehicle_control_turn_slip_event_id++;
        vehicle_control_turn_slip_event_pending = TRUE;
        vehicle_control_turn_slip_pending_path_index = path_state->replay_cursor_cnt;
        vehicle_control_turn_slip_pending_x_mm = pose->x_mm;
        vehicle_control_turn_slip_pending_y_mm = pose->y_mm;
        vehicle_control_turn_slip_pending_ratio = correction->yaw_realization_ratio;
        vehicle_control_turn_slip_pending_encoder_turn_deg =
            correction->encoder_turn_rad / VEHICLE_CONTROL_DEG_TO_RAD;
        vehicle_control_turn_slip_pending_imu_turn_deg =
            correction->imu_turn_rad / VEHICLE_CONTROL_DEG_TO_RAD;
        vehicle_control_turn_slip_pending_correction_mm = correction->pending_correction_mm;
    }
    vehicle_control_turn_slip_previous_confirmed = correction->slip_confirmed;
}

static void vehicle_control_replay_binary_sample_print(
    uint64 dt_us,
    const module_vehicle_pose_fusion_observation_t* pose_observation,
    const module_vehicle_encoder_observation_t* encoder_observation,
    const vehicle_path_state_t* path_state,
    const module_vehicle_esc_role_status_t* left_esc_status,
    const module_vehicle_esc_role_status_t* right_esc_status,
    float32 angle_error_deg,
    float32 yaw_rate_deg_s,
    float32 target_yaw_rate_deg_s)
{
    uint8* frame = vehicle_control_replay_binary_frame;
    uint16 offset = 0u;
    uint16 crc;

    frame[offset++] = VEHICLE_CONTROL_REPLAY_BINARY_SYNC_0;
    frame[offset++] = VEHICLE_CONTROL_REPLAY_BINARY_SYNC_1;
    frame[offset++] = VEHICLE_CONTROL_REPLAY_BINARY_VERSION;
    frame[offset++] = VEHICLE_CONTROL_REPLAY_BINARY_TYPE_SAMPLE;
    vehicle_control_binary_u16_put(frame, &offset, vehicle_control_replay_binary_sequence++);
    vehicle_control_binary_u16_put(frame, &offset, VEHICLE_CONTROL_REPLAY_BINARY_PAYLOAD_SIZE);

    vehicle_control_binary_u32_put(frame, &offset, (uint32)dt_us);
    vehicle_control_binary_u16_put(frame, &offset, (uint16)path_state->replay_cursor_cnt);
    vehicle_control_binary_u16_put(frame, &offset,
                                   (uint16)vehicle_control_state.replay_target.target_index_cnt);
    vehicle_control_binary_f32_put(frame, &offset,
                                   vehicle_control_state.replay_target.lookahead_distance_mm);
    frame[offset++] = (uint8)((vehicle_control_state.replay_target.curve_section_active != FALSE) ? 1u : 0u);
    vehicle_control_binary_u16_put(frame, &offset, (uint16)path_state->point_cnt);
    frame[offset++] = (uint8)vehicle_control_state.replay_target.track_mode;

    vehicle_control_binary_f32_put(frame, &offset, pose_observation->x_mm);
    vehicle_control_binary_f32_put(frame, &offset, pose_observation->y_mm);
    vehicle_control_binary_f32_put(frame, &offset,
                                   pose_observation->theta_accum_rad / VEHICLE_CONTROL_DEG_TO_RAD);
    vehicle_control_binary_f32_put(frame, &offset, vehicle_control_state.replay_target.base_point.x_mm);
    vehicle_control_binary_f32_put(frame, &offset, vehicle_control_state.replay_target.base_point.y_mm);
    vehicle_control_binary_f32_put(frame, &offset,
                                   vehicle_control_state.replay_target.base_point.theta_rad
                                   / VEHICLE_CONTROL_DEG_TO_RAD);
    vehicle_control_binary_f32_put(frame, &offset,
                                   vehicle_control_state.replay_target.cross_track_error_mm);
    vehicle_control_binary_f32_put(frame, &offset,
                                   vehicle_control_state.replay_target.along_track_error_mm);
    vehicle_control_binary_f32_put(frame, &offset, vehicle_control_state.replay_target.target_point.x_mm);
    vehicle_control_binary_f32_put(frame, &offset, vehicle_control_state.replay_target.target_point.y_mm);

    vehicle_control_binary_f32_put(frame, &offset,
                                   vehicle_control_state.target_theta_rad
                                   / VEHICLE_CONTROL_DEG_TO_RAD);
    vehicle_control_binary_f32_put(frame, &offset,
                                   vehicle_control_state.replay_target.feedforward_theta_rad
                                   / VEHICLE_CONTROL_DEG_TO_RAD);
    vehicle_control_binary_f32_put(frame, &offset, angle_error_deg);
    vehicle_control_binary_f32_put(frame, &offset, vehicle_control_state.replay_target.target_speed_mm_s);
    vehicle_control_binary_f32_put(frame, &offset, vehicle_control_state.target_left_speed_mm_s);
    vehicle_control_binary_f32_put(frame, &offset, vehicle_control_state.target_right_speed_mm_s);
    vehicle_control_binary_f32_put(frame, &offset, encoder_observation->left_speed_mm_s);
    vehicle_control_binary_f32_put(frame, &offset, encoder_observation->right_speed_mm_s);
    vehicle_control_binary_f32_put(frame, &offset, vehicle_control_state.yaw_speed_correction_mm_s);
    vehicle_control_binary_f32_put(frame, &offset, yaw_rate_deg_s);
    vehicle_control_binary_f32_put(frame, &offset, target_yaw_rate_deg_s);
    vehicle_control_binary_f32_put(frame, &offset, vehicle_control_state.replay_target.yaw_ff_mm_s);
    vehicle_control_binary_f32_put(frame, &offset,
                                   vehicle_control_effective_yaw_speed_correction_mm_s);
    vehicle_control_binary_f32_put(frame, &offset,
                                   vehicle_control_state.replay_target.raw_plan_speed_mm_s);
    vehicle_control_binary_f32_put(frame, &offset,
                                   vehicle_control_state.replay_target.speed_preview_limit_mm_s);

    vehicle_control_binary_s16_put(frame, &offset, (sint16)vehicle_control_state.left_duty);
    vehicle_control_binary_s16_put(frame, &offset, (sint16)vehicle_control_state.right_duty);
    vehicle_control_binary_s16_put(frame, &offset, (sint16)vehicle_control_left_output_duty);
    vehicle_control_binary_s16_put(frame, &offset, (sint16)vehicle_control_right_output_duty);
    frame[offset++] = (uint8)left_esc_status->state;
    frame[offset++] = (uint8)left_esc_status->fault;
    frame[offset++] = (uint8)right_esc_status->state;
    frame[offset++] = (uint8)right_esc_status->fault;

    crc = vehicle_control_binary_crc16(&frame[2u], (uint16)(offset - 2u));
    vehicle_control_binary_u16_put(frame, &offset, crc);
    if (offset == VEHICLE_CONTROL_REPLAY_BINARY_FRAME_SIZE)
    {
        tools_print_write_buffer(frame, offset);
    }
}

static void vehicle_control_binary_u16_put(uint8* buffer, uint16* offset, uint16 value)
{
    buffer[(*offset)++] = (uint8)(value & 0xFFu);
    buffer[(*offset)++] = (uint8)((value >> 8u) & 0xFFu);
}

static void vehicle_control_binary_u32_put(uint8* buffer, uint16* offset, uint32 value)
{
    buffer[(*offset)++] = (uint8)(value & 0xFFu);
    buffer[(*offset)++] = (uint8)((value >> 8u) & 0xFFu);
    buffer[(*offset)++] = (uint8)((value >> 16u) & 0xFFu);
    buffer[(*offset)++] = (uint8)((value >> 24u) & 0xFFu);
}

static void vehicle_control_binary_s16_put(uint8* buffer, uint16* offset, sint16 value)
{
    vehicle_control_binary_u16_put(buffer, offset, (uint16)value);
}

static void vehicle_control_binary_f32_put(uint8* buffer, uint16* offset, float32 value)
{
    union
    {
        float32 value;
        uint32 bits;
    } converted;

    converted.value = value;
    vehicle_control_binary_u32_put(buffer, offset, converted.bits);
}

static uint16 vehicle_control_binary_crc16(const uint8* data, uint16 length)
{
    uint16 crc = 0xFFFFu;
    uint16 index;
    uint8 bit;

    for (index = 0u; index < length; index++)
    {
        crc ^= (uint16)data[index];
        for (bit = 0u; bit < 8u; bit++)
        {
            crc = ((crc & 1u) != 0u)
                ? (uint16)((crc >> 1u) ^ 0xA001u)
                : (uint16)(crc >> 1u);
        }
    }
    return crc;
}

static void vehicle_control_replay_binary_slip_print(
    const module_vehicle_pose_fusion_replay_correction_t* replay_correction)
{
    uint8* frame = vehicle_control_replay_binary_frame;
    uint16 offset = 0u;
    uint16 crc;

    frame[offset++] = VEHICLE_CONTROL_REPLAY_BINARY_SYNC_0;
    frame[offset++] = VEHICLE_CONTROL_REPLAY_BINARY_SYNC_1;
    frame[offset++] = VEHICLE_CONTROL_REPLAY_BINARY_VERSION;
    frame[offset++] = VEHICLE_CONTROL_REPLAY_BINARY_TYPE_SLIP;
    vehicle_control_binary_u16_put(frame, &offset, vehicle_control_replay_binary_sequence++);
    vehicle_control_binary_u16_put(frame, &offset,
                                   VEHICLE_CONTROL_REPLAY_BINARY_SLIP_PAYLOAD_SIZE);
    frame[offset++] = (uint8)((replay_correction->active != FALSE) ? 1u : 0u);
    frame[offset++] = (uint8)((replay_correction->slip_confirmed != FALSE) ? 1u : 0u);
    frame[offset++] = (uint8)((replay_correction->straight_release_ready != FALSE) ? 1u : 0u);
    frame[offset++] = (uint8)((replay_correction->slip_window_ready != FALSE) ? 1u : 0u);
    vehicle_control_binary_f32_put(frame, &offset,
                                   replay_correction->encoder_turn_rad / VEHICLE_CONTROL_DEG_TO_RAD);
    vehicle_control_binary_f32_put(frame, &offset,
                                   replay_correction->imu_turn_rad / VEHICLE_CONTROL_DEG_TO_RAD);
    vehicle_control_binary_f32_put(frame, &offset, replay_correction->yaw_realization_ratio);
    vehicle_control_binary_f32_put(frame, &offset, replay_correction->pending_correction_mm);
    vehicle_control_binary_f32_put(frame, &offset, replay_correction->total_correction_mm);
    vehicle_control_binary_f32_put(frame, &offset, replay_correction->correction_x_mm);
    vehicle_control_binary_f32_put(frame, &offset, replay_correction->correction_y_mm);
    frame[offset++] = (uint8)replay_correction->slip_window_bucket_count;
    frame[offset++] = (uint8)replay_correction->slip_window_valid_count;
    frame[offset++] = (uint8)replay_correction->slip_reject_reason;
    frame[offset++] = (uint8)((sint8)replay_correction->slip_turn_sign);
    vehicle_control_binary_f32_put(frame, &offset,
                                   replay_correction->slip_window_duration_s * 1000.0f);
    crc = vehicle_control_binary_crc16(&frame[2u], (uint16)(offset - 2u));
    vehicle_control_binary_u16_put(frame, &offset, crc);
    tools_print_write_buffer(frame, offset);
}

static void vehicle_control_d_yaw_rate_lpf_reset(void)
{
    vehicle_control_d_yaw_rate_lpf_rad_s = 0.0f;
    vehicle_control_d_yaw_rate_lpf_ready = FALSE;
}

static float32 vehicle_control_d_yaw_rate_lpf_update(float32 input_rad_s, float32 dt_s)
{
    float32 tau_s;
    float32 alpha;

    if (dt_s <= 0.0f)
    {
        vehicle_control_d_yaw_rate_lpf_rad_s = input_rad_s;
        vehicle_control_d_yaw_rate_lpf_ready = TRUE;
        return input_rad_s;
    }

    if (vehicle_control_d_yaw_rate_lpf_ready == FALSE)
    {
        vehicle_control_d_yaw_rate_lpf_rad_s = input_rad_s;
        vehicle_control_d_yaw_rate_lpf_ready = TRUE;
        return input_rad_s;
    }

    tau_s = 1.0f / (VEHICLE_CONTROL_TWO_PI * VEHICLE_CONTROL_D_YAW_RATE_LPF_CUTOFF_HZ);
    alpha = dt_s / (tau_s + dt_s);
    alpha = algorithm_control_clamp_f32(alpha, 0.0f, 1.0f);
    vehicle_control_d_yaw_rate_lpf_rad_s +=
        alpha * (input_rad_s - vehicle_control_d_yaw_rate_lpf_rad_s);
    return vehicle_control_d_yaw_rate_lpf_rad_s;
}

static void vehicle_control_target_yaw_rate_reset(void)
{
    vehicle_control_target_yaw_rate_rad_s = 0.0f;
    vehicle_control_target_theta_previous_rad = vehicle_control_state.target_theta_rad;
    vehicle_control_target_yaw_rate_ready = FALSE;
    vehicle_control_angle_linear_output_mm_s = 0.0f;
    vehicle_control_angle_nonlinear_output_mm_s = 0.0f;
    vehicle_control_angle_rate_output_mm_s = 0.0f;
    vehicle_control_angle_feedforward_output_mm_s = 0.0f;
}

static boolean vehicle_control_angle_2dof_runtime_active(void)
{
    return ((vehicle_control_angle_2dof_enabled != FALSE)
            && (vehicle_control_state.mode == VEHICLE_CONTROL_MODE_REPLAY)
            && (vehicle_control_state.replay_target_valid != FALSE)
            && (vehicle_control_replay_launch_gate_active == FALSE))
               ? TRUE
               : FALSE;
}

static float32 vehicle_control_target_yaw_rate_update(float32 dt_s)
{
    float32 raw_yaw_rate_rad_s;
    float32 yaw_rate_limit_rad_s;
    float32 available_yaw_rate_rad_s;
    float32 tau_s;
    float32 alpha;

    if ((vehicle_control_angle_2dof_runtime_active() == FALSE) || (dt_s <= 0.0f))
    {
        vehicle_control_target_yaw_rate_reset();
        return 0.0f;
    }

    if (vehicle_control_target_yaw_rate_ready == FALSE)
    {
        vehicle_control_target_theta_previous_rad = vehicle_control_state.target_theta_rad;
        vehicle_control_target_yaw_rate_ready = TRUE;
        return 0.0f;
    }

    raw_yaw_rate_rad_s =
        algorithm_attitude_wrap_pi(vehicle_control_state.target_theta_rad
                                   - vehicle_control_target_theta_previous_rad)
        / dt_s;
    vehicle_control_target_theta_previous_rad = vehicle_control_state.target_theta_rad;
    yaw_rate_limit_rad_s = vehicle_control_target_yaw_rate_limit_rad_s;
    available_yaw_rate_rad_s =
        vehicle_control_yaw_rate_available_get(vehicle_control_base_effective_target_mm_s);
    if (available_yaw_rate_rad_s < yaw_rate_limit_rad_s)
    {
        yaw_rate_limit_rad_s = available_yaw_rate_rad_s;
    }
    raw_yaw_rate_rad_s = algorithm_control_clamp_f32(raw_yaw_rate_rad_s,
                                                     -yaw_rate_limit_rad_s,
                                                     yaw_rate_limit_rad_s);

    if (vehicle_control_target_yaw_rate_lpf_cutoff_hz <= 0.0f)
    {
        vehicle_control_target_yaw_rate_rad_s = raw_yaw_rate_rad_s;
        return vehicle_control_target_yaw_rate_rad_s;
    }

    tau_s = 1.0f / (VEHICLE_CONTROL_TWO_PI * vehicle_control_target_yaw_rate_lpf_cutoff_hz);
    alpha = algorithm_control_clamp_f32(dt_s / (tau_s + dt_s), 0.0f, 1.0f);
    vehicle_control_target_yaw_rate_rad_s +=
        alpha * (raw_yaw_rate_rad_s - vehicle_control_target_yaw_rate_rad_s);
    vehicle_control_target_yaw_rate_rad_s = algorithm_control_clamp_f32(
        vehicle_control_target_yaw_rate_rad_s,
        -yaw_rate_limit_rad_s,
        yaw_rate_limit_rad_s);
    return vehicle_control_target_yaw_rate_rad_s;
}

static float32 vehicle_control_angle_correction_update(float32 dt_s)
{
    const vehicle_control_cfg_t* cfg = vehicle_control_cfg_get();
    const module_vehicle_pose_fusion_observation_t* pose_observation =
        module_vehicle_pose_fusion_observation_get();
    float32 current_theta_rad = pose_observation->theta_rad;
    float32 yaw_rate_rad_s;
    float32 target_yaw_rate_rad_s;
    float32 angle_error_rad;
    float32 feedback_output_mm_s;
    float32 feedback_correction_mm_s;
    float32 correction_mm_s;
    float32 nonlinear_output_mm_s;
    float32 feedforward_output_mm_s;
    float32 available_correction_mm_s;
    boolean extended_active;
    float32 correction_slew_limit_mm_s2 = VEHICLE_CONTROL_YAW_CORRECTION_SLEW_LIMIT_MM_S2;
    const algorithm_control_pid_cfg_t* angle_pid_cfg = &vehicle_control_angle_pid_cfg;
    algorithm_control_pid_state_t* pid = &vehicle_control_state.angle_pid;

    if (dt_s <= 0.0f)
    {
        return vehicle_control_state.yaw_speed_correction_mm_s;
    }

    if ((vehicle_control_replay_launch_drag_active() != FALSE)
        && (vehicle_control_launch_drag_correction_enabled == FALSE))
    {
        (void)vehicle_control_replay_launch_gate_update(dt_s);
        vehicle_control_state.yaw_speed_correction_mm_s = 0.0f;
        vehicle_control_yaw_feedback_correction_mm_s = 0.0f;
        vehicle_control_effective_yaw_speed_correction_mm_s = 0.0f;
        algorithm_control_pid_reset(&vehicle_control_launch_angle_pid);
        vehicle_control_d_yaw_rate_lpf_reset();
        return vehicle_control_state.yaw_speed_correction_mm_s;
    }

    if ((vehicle_control_state.mode == VEHICLE_CONTROL_MODE_REPLAY)
        && (vehicle_control_state.target_speed_mm_s <= 0.5f)
        && (vehicle_control_state.replay_target_valid == FALSE))
    {
        vehicle_control_state.yaw_speed_correction_mm_s = 0.0f;
        vehicle_control_yaw_feedback_correction_mm_s = 0.0f;
        vehicle_control_effective_yaw_speed_correction_mm_s = 0.0f;
        algorithm_control_pid_reset(&vehicle_control_state.angle_pid);
        algorithm_control_pid_reset(&vehicle_control_launch_angle_pid);
        vehicle_control_d_yaw_rate_lpf_reset();
        vehicle_control_target_yaw_rate_reset();
        return vehicle_control_state.yaw_speed_correction_mm_s;
    }

    if (vehicle_control_state.mode == VEHICLE_CONTROL_MODE_REPLAY)
    {
        current_theta_rad = pose_observation->theta_accum_rad;
        if (vehicle_control_replay_launch_angle_hold_active() != FALSE)
        {
            vehicle_control_state.target_theta_rad =
                vehicle_control_replay_launch_reference_theta_rad;
        }
        angle_error_rad = algorithm_attitude_wrap_pi(vehicle_control_state.target_theta_rad - current_theta_rad);
        (void)vehicle_control_replay_launch_gate_update(dt_s);
    }
    else
    {
        angle_error_rad = algorithm_attitude_wrap_pi(vehicle_control_state.target_theta_rad - current_theta_rad);
    }

    if (vehicle_control_launch_correction_active() != FALSE)
    {
        angle_pid_cfg = &vehicle_control_launch_angle_pid_cfg;
        pid = &vehicle_control_launch_angle_pid;
    }

    yaw_rate_rad_s = vehicle_control_d_yaw_rate_lpf_update(module_vehicle_gyro_yaw_rate_get(), dt_s);
    extended_active = vehicle_control_angle_2dof_runtime_active();
    target_yaw_rate_rad_s = vehicle_control_target_yaw_rate_update(dt_s);

    pid->target = vehicle_control_state.target_theta_rad;
    pid->feedback = current_theta_rad;
    pid->previous_error = pid->last_error;
    pid->last_error = pid->error;
    pid->error = angle_error_rad;
    pid->integral += angle_error_rad * dt_s;
    pid->integral = algorithm_control_clamp_f32(pid->integral,
                                                angle_pid_cfg->integral_min,
                                                angle_pid_cfg->integral_max);

    nonlinear_output_mm_s = (extended_active != FALSE)
        ? vehicle_control_angle_kp2 * angle_error_rad
          * ((angle_error_rad >= 0.0f) ? angle_error_rad : -angle_error_rad)
        : 0.0f;
    feedforward_output_mm_s = (extended_active != FALSE)
        ? vehicle_control_target_yaw_rate_feedforward_gain_mm * target_yaw_rate_rad_s
        : 0.0f;
    vehicle_control_angle_linear_output_mm_s = angle_pid_cfg->kp * angle_error_rad;
    vehicle_control_angle_nonlinear_output_mm_s = nonlinear_output_mm_s;
    vehicle_control_angle_rate_output_mm_s = -angle_pid_cfg->kd * yaw_rate_rad_s;
    vehicle_control_angle_feedforward_output_mm_s = feedforward_output_mm_s;
    feedback_output_mm_s = vehicle_control_angle_linear_output_mm_s
                         + nonlinear_output_mm_s
                         + (angle_pid_cfg->ki * pid->integral)
                         + vehicle_control_angle_rate_output_mm_s
                         + feedforward_output_mm_s;
    pid->output = algorithm_control_clamp_f32(feedback_output_mm_s,
                                              angle_pid_cfg->output_min,
                                              angle_pid_cfg->output_max);
    feedback_correction_mm_s = algorithm_control_clamp_f32(pid->output,
                                                           cfg->yaw_speed_correction_min_mm_s,
                                                           cfg->yaw_speed_correction_max_mm_s);
    if (vehicle_control_launch_correction_active() != FALSE)
    {
        feedback_correction_mm_s = algorithm_control_clamp_f32(
            feedback_correction_mm_s,
            -VEHICLE_CONTROL_REPLAY_LAUNCH_CORRECTION_LIMIT_MM_S,
            VEHICLE_CONTROL_REPLAY_LAUNCH_CORRECTION_LIMIT_MM_S);
        correction_slew_limit_mm_s2 =
            VEHICLE_CONTROL_REPLAY_LAUNCH_YAW_CORRECTION_SLEW_LIMIT_MM_S2;
    }
    if (vehicle_control_speed_protection_enabled != FALSE)
    {
        feedback_correction_mm_s = algorithm_control_clamp_f32(
            feedback_correction_mm_s,
            -(vehicle_control_angle_diff_limit_mm_s * 0.5f),
            vehicle_control_angle_diff_limit_mm_s * 0.5f);
        if ((vehicle_control_state.mode == VEHICLE_CONTROL_MODE_REPLAY)
            || (vehicle_control_launch_correction_active() != FALSE))
        {
            feedback_correction_mm_s = vehicle_control_yaw_correction_slew_step(
                vehicle_control_yaw_feedback_correction_mm_s,
                feedback_correction_mm_s,
                dt_s,
                correction_slew_limit_mm_s2);
        }
    }
    if (vehicle_control_launch_low_speed_active() != FALSE)
    {
        available_correction_mm_s = VEHICLE_CONTROL_REPLAY_LAUNCH_CORRECTION_LIMIT_MM_S;
        feedback_correction_mm_s = algorithm_control_clamp_f32(
            feedback_correction_mm_s,
            -available_correction_mm_s,
            available_correction_mm_s);
    }
    else if ((vehicle_control_state.mode == VEHICLE_CONTROL_MODE_REPLAY)
             && (vehicle_control_replay_launch_drag_active() == FALSE))
    {
        available_correction_mm_s = vehicle_control_yaw_correction_available_get(
            vehicle_control_base_effective_target_mm_s);
        feedback_correction_mm_s = algorithm_control_clamp_f32(
            feedback_correction_mm_s,
            -available_correction_mm_s,
            available_correction_mm_s);
    }
    vehicle_control_yaw_feedback_correction_mm_s = feedback_correction_mm_s;
    correction_mm_s = feedback_correction_mm_s;
    correction_mm_s = algorithm_control_clamp_f32(correction_mm_s,
                                                  cfg->yaw_speed_correction_min_mm_s,
                                                  cfg->yaw_speed_correction_max_mm_s);
    if (vehicle_control_speed_protection_enabled != FALSE)
    {
        correction_mm_s = algorithm_control_clamp_f32(correction_mm_s,
                                                      -(vehicle_control_angle_diff_limit_mm_s * 0.5f),
                                                      vehicle_control_angle_diff_limit_mm_s * 0.5f);
    }
    if (vehicle_control_launch_low_speed_active() != FALSE)
    {
        available_correction_mm_s = VEHICLE_CONTROL_REPLAY_LAUNCH_CORRECTION_LIMIT_MM_S;
        correction_mm_s = algorithm_control_clamp_f32(correction_mm_s,
                                                       -available_correction_mm_s,
                                                       available_correction_mm_s);
    }
    else if ((vehicle_control_state.mode == VEHICLE_CONTROL_MODE_REPLAY)
             && (vehicle_control_replay_launch_drag_active() == FALSE))
    {
        available_correction_mm_s = vehicle_control_yaw_correction_available_get(
            vehicle_control_base_effective_target_mm_s);
        correction_mm_s = algorithm_control_clamp_f32(correction_mm_s,
                                                      -available_correction_mm_s,
                                                      available_correction_mm_s);
    }
    vehicle_control_state.yaw_speed_correction_mm_s = correction_mm_s;
    return vehicle_control_state.yaw_speed_correction_mm_s;
}

/**
 * @brief 根据路径回放模块更新当前回放目标。
 * @param[in] void 无参数。
 * @return void
 */
static void vehicle_control_replay_update_target(void)
{
    const vehicle_path_state_t* path_state;
    vehicle_path_replay_target_t replay_target;

    vehicle_control_replay_path_lookahead_speed_cap_update();
    if (module_vehicle_path_replay_update() == FALSE)
    {
        path_state = module_vehicle_path_state_get();
        if ((path_state->status == VEHICLE_PATH_STATUS_FINISHED)
            && (vehicle_control_state.replay_target_valid != FALSE))
        {
            vehicle_control_state.replay_target.valid = TRUE;
            vehicle_control_state.replay_target.target_theta_rad = vehicle_control_state.target_theta_rad;
            vehicle_control_state.replay_target.feedforward_theta_rad = vehicle_control_state.target_theta_rad;
            vehicle_control_state.replay_target.target_point.theta_rad = vehicle_control_state.target_theta_rad;
            vehicle_control_state.replay_target.target_point.speed_mm_s = 0.0f;
            vehicle_control_state.replay_target.target_speed_mm_s = 0.0f;
            vehicle_control_replay_launch_gate_stop();
            module_vehicle_path_replay_lookahead_speed_cap_set(0.0f);
            vehicle_control_set_speed(0.0f);
            return;
        }

        vehicle_control_state.replay_target_valid = FALSE;
        vehicle_control_replay_launch_gate_stop();
        module_vehicle_path_replay_lookahead_speed_cap_set(0.0f);
        vehicle_control_state.mode = (vehicle_control_state.enabled != FALSE) ? VEHICLE_CONTROL_MODE_MANUAL
                                                                              : VEHICLE_CONTROL_MODE_IDLE;
        return;
    }

    if (module_vehicle_path_replay_target_get(&replay_target) == FALSE)
    {
        vehicle_control_state.replay_target_valid = FALSE;
        return;
    }

    vehicle_control_state.replay_target = replay_target;
    vehicle_control_replay_launch_target_limit(&vehicle_control_state.replay_target);
    vehicle_control_state.replay_target_valid = TRUE;
    vehicle_control_state.target_theta_rad = vehicle_control_state.replay_target.target_theta_rad;
    vehicle_control_set_speed(vehicle_control_state.replay_target.target_speed_mm_s);

    if (module_vehicle_path_state_get()->status == VEHICLE_PATH_STATUS_FINISHED)
    {
        vehicle_control_state.replay_target.target_speed_mm_s = 0.0f;
        vehicle_control_state.replay_target.target_point.speed_mm_s = 0.0f;
        module_vehicle_path_replay_lookahead_speed_cap_set(0.0f);
        vehicle_control_set_speed(0.0f);
    }
}

/**
 * @brief 将当前控制状态中的三路电调占空命令写入电调模块。
 * @param[in] void 无参数。
 * @return void
 */
void vehicle_control_apply_outputs(void)
{
    if (vehicle_control_launch_correction_active() != FALSE)
    {
        vehicle_control_drive_launch_duty_slew_pair_step(
            vehicle_control_left_output_duty,
            vehicle_control_right_output_duty,
            vehicle_control_state.left_duty,
            vehicle_control_state.right_duty,
            &vehicle_control_left_output_duty,
            &vehicle_control_right_output_duty);
        vehicle_control_left_output_duty = vehicle_control_drive_clamp_duty(
            VEHICLE_ESC_ROLE_LEFT_DRIVE,
            vehicle_control_left_output_duty);
        vehicle_control_right_output_duty = vehicle_control_drive_clamp_duty(
            VEHICLE_ESC_ROLE_RIGHT_DRIVE,
            vehicle_control_right_output_duty);
    }
    else
    {
        vehicle_control_left_output_duty =
            vehicle_control_drive_clamp_duty(
                VEHICLE_ESC_ROLE_LEFT_DRIVE,
                vehicle_control_drive_duty_slew_step(vehicle_control_left_output_duty,
                                                     vehicle_control_state.left_duty));
        vehicle_control_right_output_duty =
            vehicle_control_drive_clamp_duty(
                VEHICLE_ESC_ROLE_RIGHT_DRIVE,
                vehicle_control_drive_duty_slew_step(vehicle_control_right_output_duty,
                                                     vehicle_control_state.right_duty));
    }

    (void)module_vehicle_esc_set_duty(VEHICLE_ESC_ROLE_LEFT_DRIVE, vehicle_control_left_output_duty);
    (void)module_vehicle_esc_set_duty(VEHICLE_ESC_ROLE_RIGHT_DRIVE, vehicle_control_right_output_duty);
    vehicle_control_suction_process();
    module_vehicle_esc_flush(FALSE);
}

/**
 * @brief 将当前缓存的空闲占空命令写入电调模块。
 * @param[in] void 无参数。
 * @return void
 */
static void vehicle_control_apply_idle_outputs(void)
{
    vehicle_control_left_output_duty = vehicle_control_state.left_duty;
    vehicle_control_right_output_duty = vehicle_control_state.right_duty;
    vehicle_control_apply_outputs();
}

static sint32 vehicle_control_drive_duty_slew_step(sint32 current, sint32 target)
{
    sint32 delta = target - current;
    sint32 rise_slew_limit = VEHICLE_CONTROL_DRIVE_DUTY_SLEW_LIMIT_TICK_PER_UPDATE;
    sint32 fall_slew_limit = VEHICLE_CONTROL_DRIVE_DUTY_FALL_SLEW_LIMIT_TICK_PER_UPDATE;

    if (vehicle_control_launch_drag_active() != FALSE)
    {
        rise_slew_limit = VEHICLE_CONTROL_DRIVE_LAUNCH_DUTY_SLEW_LIMIT_TICK_PER_UPDATE;
        fall_slew_limit = VEHICLE_CONTROL_DRIVE_LAUNCH_DUTY_SLEW_LIMIT_TICK_PER_UPDATE;
    }
    else if (vehicle_control_launch_correction_active() != FALSE)
    {
        rise_slew_limit = VEHICLE_CONTROL_DRIVE_LAUNCH_CORRECTION_DUTY_SLEW_LIMIT_TICK_PER_UPDATE;
        fall_slew_limit = VEHICLE_CONTROL_DRIVE_LAUNCH_CORRECTION_DUTY_SLEW_LIMIT_TICK_PER_UPDATE;
    }
    if ((rise_slew_limit <= 0) || (fall_slew_limit <= 0))
    {
        return target;
    }

    if (delta > rise_slew_limit)
    {
        return current + rise_slew_limit;
    }

    if (delta < -fall_slew_limit)
    {
        return current - fall_slew_limit;
    }

    return target;
}

static void vehicle_control_drive_launch_duty_slew_pair_step(sint32 current_left,
                                                             sint32 current_right,
                                                             sint32 target_left,
                                                             sint32 target_right,
                                                             sint32* next_left,
                                                             sint32* next_right)
{
    sint32 slew_limit = VEHICLE_CONTROL_DRIVE_LAUNCH_CORRECTION_DUTY_SLEW_LIMIT_TICK_PER_UPDATE;
    sint32 current_common;
    sint32 target_common;
    sint32 current_diff;
    sint32 target_diff;
    sint32 diff_delta;
    sint32 common_delta;
    sint32 common_budget;

    if ((next_left == NULL_PTR) || (next_right == NULL_PTR))
    {
        return;
    }

    if (vehicle_control_launch_drag_active() != FALSE)
    {
        slew_limit = VEHICLE_CONTROL_DRIVE_LAUNCH_DUTY_SLEW_LIMIT_TICK_PER_UPDATE;
    }
    if (slew_limit <= 0)
    {
        *next_left = target_left;
        *next_right = target_right;
        return;
    }

    current_common = (current_left + current_right) / 2;
    target_common = (target_left + target_right) / 2;
    current_diff = (current_right - current_left) / 2;
    target_diff = (target_right - target_left) / 2;
    diff_delta = target_diff - current_diff;
    if (diff_delta > slew_limit)
    {
        diff_delta = slew_limit;
    }
    else if (diff_delta < -slew_limit)
    {
        diff_delta = -slew_limit;
    }

    common_budget = slew_limit - ((diff_delta >= 0) ? diff_delta : -diff_delta);
    common_delta = target_common - current_common;
    if (common_delta > common_budget)
    {
        common_delta = common_budget;
    }
    else if (common_delta < -common_budget)
    {
        common_delta = -common_budget;
    }

    current_common += common_delta;
    current_diff += diff_delta;
    *next_left = current_common - current_diff;
    *next_right = current_common + current_diff;

    if (target_left >= current_left)
    {
        if (*next_left < current_left)
        {
            *next_left = current_left;
        }
        else if (*next_left > target_left)
        {
            *next_left = target_left;
        }
    }
    else
    {
        if (*next_left > current_left)
        {
            *next_left = current_left;
        }
        else if (*next_left < target_left)
        {
            *next_left = target_left;
        }
    }

    if (target_right >= current_right)
    {
        if (*next_right < current_right)
        {
            *next_right = current_right;
        }
        else if (*next_right > target_right)
        {
            *next_right = target_right;
        }
    }
    else
    {
        if (*next_right > current_right)
        {
            *next_right = current_right;
        }
        else if (*next_right < target_right)
        {
            *next_right = target_right;
        }
    }
}

static sint32 vehicle_control_drive_keepalive_duty(sint32 duty, float32 target_speed_mm_s)
{
    if ((target_speed_mm_s <= VEHICLE_CONTROL_DRIVE_STOP_TARGET_EPSILON_MM_S)
        || (vehicle_control_launch_drag_active() != FALSE)
        || (vehicle_control_launch_correction_active() != FALSE))
    {
        return duty;
    }

    return (duty < VEHICLE_CONTROL_DRIVE_CLOSED_LOOP_KEEPALIVE_DUTY_TICK)
               ? VEHICLE_CONTROL_DRIVE_CLOSED_LOOP_KEEPALIVE_DUTY_TICK
               : duty;
}

static float32 vehicle_control_drive_start_assist(float32 target_speed_mm_s, float32 feedback_speed_mm_s)
{
    float32 feedback_abs = feedback_speed_mm_s;

    if ((target_speed_mm_s > -0.5f) && (target_speed_mm_s < 0.5f))
    {
        return 0.0f;
    }

    if (feedback_abs < 0.0f)
    {
        feedback_abs = -feedback_abs;
    }

    if (feedback_abs > VEHICLE_CONTROL_DRIVE_START_ASSIST_SPEED_MM_S)
    {
        return 0.0f;
    }

    return (target_speed_mm_s > 0.0f) ? VEHICLE_CONTROL_DRIVE_START_ASSIST_DUTY_TICK
                                      : -VEHICLE_CONTROL_DRIVE_START_ASSIST_DUTY_TICK;
}

static void vehicle_control_speed_integral_limit_clamp_state(algorithm_control_pid_state_t* pid)
{
    if ((vehicle_control_speed_integral_limit_enabled == FALSE) || (pid == NULL_PTR))
    {
        return;
    }

    pid->integral = algorithm_control_clamp_f32(pid->integral,
                                                -vehicle_control_speed_integral_limit_abs,
                                                vehicle_control_speed_integral_limit_abs);
}

/**
 * @brief 将 PID 输出增量叠加到基准占空命令并转换为非负 PWM ticks。
 * @param[in] idle_duty 基准占空命令，单位：GTM PWM ticks。
 * @param[in] duty_delta PID 输出增量，单位：GTM PWM ticks。
 * @return 非负 PWM 占空命令，单位：GTM PWM ticks。
 */
static void vehicle_control_speed_target_runtime_reset(void)
{
    vehicle_control_speed_accel_feedforward_reset();
    vehicle_control_base_effective_target_mm_s = 0.0f;
    vehicle_control_base_start_accel_active = FALSE;
    vehicle_control_base_accel_limit_mm_s2 = VEHICLE_CONTROL_START_ACCEL_LIMIT_DEFAULT_MM_S2;
    vehicle_control_left_effective_target_mm_s = 0.0f;
    vehicle_control_right_effective_target_mm_s = 0.0f;
    vehicle_control_left_start_accel_active = FALSE;
    vehicle_control_right_start_accel_active = FALSE;
    vehicle_control_left_accel_limit_mm_s2 = VEHICLE_CONTROL_START_ACCEL_LIMIT_DEFAULT_MM_S2;
    vehicle_control_right_accel_limit_mm_s2 = VEHICLE_CONTROL_START_ACCEL_LIMIT_DEFAULT_MM_S2;
    vehicle_control_start_integral_hold_timer_s = 0.0f;
    vehicle_control_start_integral_hold_moving = FALSE;
    vehicle_control_reverse_feedback_guard_reset();
    vehicle_control_reverse_feedback_event_count = 0u;
    vehicle_control_tracking_limit_left_before_mm_s = 0.0f;
    vehicle_control_tracking_limit_right_before_mm_s = 0.0f;
    vehicle_control_tracking_limit_left_after_mm_s = 0.0f;
    vehicle_control_tracking_limit_right_after_mm_s = 0.0f;
    vehicle_control_tracking_limit_active = FALSE;
    vehicle_control_tracking_limit_event_count = 0u;
    vehicle_control_replay_launch_wheel_ready_timer_s = 0.0f;
    vehicle_control_replay_launch_wheel_ready = FALSE;
}

static boolean vehicle_control_speed_integral_ready(float32 target_mm_s,
                                                    float32 feedback_mm_s,
                                                    float32 previous_target_mm_s,
                                                    float32 dt_s)
{
    float32 target_abs_mm_s = vehicle_control_abs_f32(target_mm_s);
    float32 error_abs_mm_s = vehicle_control_abs_f32(target_mm_s - feedback_mm_s);
    float32 error_limit_mm_s = VEHICLE_CONTROL_SPEED_INTEGRAL_ERROR_LIMIT_MM_S;
    float32 feedback_along_target_mm_s;
    float32 target_delta_abs_mm_s;
    float32 target_delta_limit_mm_s;

    if ((dt_s <= 0.0f)
        || (target_abs_mm_s <= VEHICLE_CONTROL_START_INTEGRAL_TARGET_THRESHOLD_MM_S))
    {
        return FALSE;
    }

    target_delta_abs_mm_s = vehicle_control_abs_f32(target_mm_s - previous_target_mm_s);
    target_delta_limit_mm_s =
        (VEHICLE_CONTROL_SPEED_INTEGRAL_TARGET_SLEW_LIMIT_MM_S2 * dt_s) + 0.5f;
    if (target_delta_abs_mm_s > target_delta_limit_mm_s)
    {
        return FALSE;
    }

    if ((target_abs_mm_s * VEHICLE_CONTROL_SPEED_INTEGRAL_ERROR_RATIO) > error_limit_mm_s)
    {
        error_limit_mm_s = target_abs_mm_s * VEHICLE_CONTROL_SPEED_INTEGRAL_ERROR_RATIO;
    }

    if (error_abs_mm_s <= error_limit_mm_s)
    {
        return TRUE;
    }

    feedback_along_target_mm_s = (target_mm_s >= 0.0f) ? feedback_mm_s : -feedback_mm_s;
    if ((feedback_along_target_mm_s
         < (target_abs_mm_s * VEHICLE_CONTROL_SPEED_INTEGRAL_RECOVERY_MIN_RATIO))
        || (feedback_along_target_mm_s
            > (target_abs_mm_s * VEHICLE_CONTROL_SPEED_INTEGRAL_RECOVERY_MAX_RATIO)))
    {
        return FALSE;
    }

    return TRUE;
}

static void vehicle_control_speed_integral_clear(void)
{
    vehicle_control_state.left_speed_pid.integral = 0.0f;
    vehicle_control_state.right_speed_pid.integral = 0.0f;
}

static void vehicle_control_start_integral_hold_update(float32 left_target_mm_s,
                                                       float32 right_target_mm_s,
                                                       float32 dt_s)
{
    boolean moving_target =
        ((left_target_mm_s > VEHICLE_CONTROL_START_INTEGRAL_TARGET_THRESHOLD_MM_S)
         || (left_target_mm_s < -VEHICLE_CONTROL_START_INTEGRAL_TARGET_THRESHOLD_MM_S)
         || (right_target_mm_s > VEHICLE_CONTROL_START_INTEGRAL_TARGET_THRESHOLD_MM_S)
         || (right_target_mm_s < -VEHICLE_CONTROL_START_INTEGRAL_TARGET_THRESHOLD_MM_S));

    if (moving_target == FALSE)
    {
        vehicle_control_start_integral_hold_timer_s = 0.0f;
        vehicle_control_start_integral_hold_moving = FALSE;
        return;
    }

    if (vehicle_control_start_integral_hold_moving == FALSE)
    {
        vehicle_control_start_integral_hold_timer_s = VEHICLE_CONTROL_START_INTEGRAL_HOLD_S;
        vehicle_control_start_integral_hold_moving = TRUE;
        return;
    }

    if ((vehicle_control_start_integral_hold_timer_s > 0.0f) && (dt_s > 0.0f))
    {
        vehicle_control_start_integral_hold_timer_s -= dt_s;
        if (vehicle_control_start_integral_hold_timer_s < 0.0f)
        {
            vehicle_control_start_integral_hold_timer_s = 0.0f;
        }
    }
}

static boolean vehicle_control_start_integral_hold_active(void)
{
    return (vehicle_control_start_integral_hold_timer_s > 0.0f) ? TRUE : FALSE;
}

static void vehicle_control_replay_launch_gate_start(void)
{
    if ((vehicle_control_launch_drag_enabled == FALSE)
        && (vehicle_control_launch_correction_enabled == FALSE))
    {
        vehicle_control_replay_launch_gate_stop();
        return;
    }

    vehicle_control_replay_launch_gate_active = TRUE;
    vehicle_control_replay_launch_phase = (vehicle_control_launch_drag_enabled != FALSE)
                                              ? VEHICLE_CONTROL_REPLAY_LAUNCH_PHASE_DRAG_TIME
                                              : VEHICLE_CONTROL_REPLAY_LAUNCH_PHASE_CROSS_TRACK_STABILIZE;
    vehicle_control_replay_launch_gate_timer_s = 0.0f;
    vehicle_control_replay_launch_gate_stable_timer_s = 0.0f;
    vehicle_control_replay_launch_wheel_ready_timer_s = 0.0f;
    vehicle_control_replay_launch_wheel_ready = FALSE;
    vehicle_control_replay_launch_reference_theta_rad =
        module_vehicle_pose_fusion_observation_get()->theta_accum_rad;
    if (vehicle_control_replay_launch_phase
        == VEHICLE_CONTROL_REPLAY_LAUNCH_PHASE_CROSS_TRACK_STABILIZE)
    {
        vehicle_control_replay_launch_wheel_ready = TRUE;
        vehicle_control_base_effective_target_mm_s =
            VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S;
        vehicle_control_left_effective_target_mm_s =
            VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S;
        vehicle_control_right_effective_target_mm_s =
            VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S;
        vehicle_control_set_speed(VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S);
    }
}

static void vehicle_control_replay_launch_gate_stop(void)
{
    vehicle_control_replay_launch_gate_active = FALSE;
    vehicle_control_replay_launch_phase = VEHICLE_CONTROL_REPLAY_LAUNCH_PHASE_OFF;
    vehicle_control_replay_launch_gate_timer_s = 0.0f;
    vehicle_control_replay_launch_gate_stable_timer_s = 0.0f;
    vehicle_control_replay_launch_wheel_ready_timer_s = 0.0f;
    vehicle_control_replay_launch_wheel_ready = FALSE;
}

static void vehicle_control_replay_launch_complete(void)
{
    vehicle_control_replay_launch_gate_stop();
    vehicle_control_speed_integral_clear();
    vehicle_control_replay_update_target();
    if ((vehicle_control_state.mode == VEHICLE_CONTROL_MODE_REPLAY)
        && (vehicle_control_state.target_speed_mm_s > VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S))
    {
        vehicle_control_base_start_accel_active = TRUE;
        vehicle_control_base_accel_limit_mm_s2 = vehicle_control_start_accel_limit_mm_s2;
    }
}

static boolean vehicle_control_replay_launch_gate_update(float32 dt_s)
{
    if (vehicle_control_replay_launch_gate_active == FALSE)
    {
        return FALSE;
    }

    if ((vehicle_control_state.mode != VEHICLE_CONTROL_MODE_REPLAY)
        || (dt_s <= 0.0f)
        || (vehicle_control_state.replay_target_valid == FALSE))
    {
        vehicle_control_replay_launch_gate_stop();
        return FALSE;
    }

    if (vehicle_control_replay_launch_phase == VEHICLE_CONTROL_REPLAY_LAUNCH_PHASE_DRAG_TIME)
    {
        vehicle_control_replay_launch_gate_timer_s += dt_s;
        vehicle_control_replay_launch_gate_stable_timer_s = 0.0f;
        if (vehicle_control_replay_launch_gate_timer_s >= VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_TIME_S)
        {
            vehicle_control_replay_launch_phase =
                VEHICLE_CONTROL_REPLAY_LAUNCH_PHASE_CROSS_TRACK_STABILIZE;
            vehicle_control_replay_launch_gate_timer_s = 0.0f;
            vehicle_control_replay_launch_wheel_ready = TRUE;
            vehicle_control_base_effective_target_mm_s =
                VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S;
            vehicle_control_left_effective_target_mm_s =
                VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S;
            vehicle_control_right_effective_target_mm_s =
                VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S;
            vehicle_control_set_speed(VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S);
            vehicle_control_speed_integral_clear();
            vehicle_control_state.yaw_speed_correction_mm_s = 0.0f;
            vehicle_control_yaw_feedback_correction_mm_s = 0.0f;
            vehicle_control_effective_yaw_speed_correction_mm_s = 0.0f;
        }
        return TRUE;
    }

    if (vehicle_control_replay_launch_phase == VEHICLE_CONTROL_REPLAY_LAUNCH_PHASE_CROSS_TRACK_STABILIZE)
    {
        vehicle_control_replay_launch_gate_timer_s += dt_s;
        if (vehicle_control_replay_launch_gate_timer_s
            >= VEHICLE_CONTROL_REPLAY_LAUNCH_LOW_SPEED_HOLD_TIME_S)
        {
            vehicle_control_replay_launch_complete();
        }
        return TRUE;
    }

    vehicle_control_replay_launch_gate_stop();
    return FALSE;
}

static boolean vehicle_control_replay_launch_drag_active(void)
{
    return ((vehicle_control_replay_launch_gate_active != FALSE)
            && (vehicle_control_replay_launch_phase == VEHICLE_CONTROL_REPLAY_LAUNCH_PHASE_DRAG_TIME))
               ? TRUE
               : FALSE;
}

static boolean vehicle_control_replay_launch_fixed_feedforward_active(void)
{
    return ((vehicle_control_replay_launch_drag_active() != FALSE)
            && (vehicle_control_replay_launch_gate_timer_s
                < VEHICLE_CONTROL_REPLAY_LAUNCH_FIXED_FEEDFORWARD_TIME_S))
               ? TRUE
               : FALSE;
}

static boolean vehicle_control_replay_launch_wheel_wait_active(void)
{
    return ((vehicle_control_replay_launch_gate_active != FALSE)
            && (vehicle_control_replay_launch_phase
                == VEHICLE_CONTROL_REPLAY_LAUNCH_PHASE_CROSS_TRACK_STABILIZE)
            && (vehicle_control_replay_launch_wheel_ready == FALSE))
               ? TRUE
               : FALSE;
}

static boolean vehicle_control_replay_launch_angle_hold_active(void)
{
    return vehicle_control_replay_launch_drag_active();
}

static void vehicle_control_replay_path_lookahead_speed_cap_update(void)
{
    float32 cap_mm_s = 0.0f;

    if ((vehicle_control_state.mode == VEHICLE_CONTROL_MODE_REPLAY)
        && (vehicle_control_replay_launch_gate_active != FALSE))
    {
        if (vehicle_control_replay_launch_drag_active() != FALSE)
        {
            cap_mm_s = (vehicle_control_launch_drag_correction_enabled != FALSE)
                           ? VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_SPEED_MM_S
                           : 0.0f;
        }
        else
        {
            cap_mm_s = VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S;
        }
    }

    module_vehicle_path_replay_lookahead_speed_cap_set(cap_mm_s);
}

static void vehicle_control_replay_launch_target_limit(vehicle_path_replay_target_t* replay_target)
{
    float32 launch_speed_mm_s;
    float32 path_speed_mm_s;

    if ((vehicle_control_replay_launch_gate_active == FALSE) || (replay_target == NULL_PTR))
    {
        return;
    }

    path_speed_mm_s = replay_target->target_speed_mm_s;
    if (path_speed_mm_s <= VEHICLE_CONTROL_START_INTEGRAL_TARGET_THRESHOLD_MM_S)
    {
        launch_speed_mm_s = 0.0f;
    }
    else
    {
        if (vehicle_control_replay_launch_drag_active() != FALSE)
        {
            launch_speed_mm_s = (vehicle_control_launch_drag_correction_enabled != FALSE)
                                    ? VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_SPEED_MM_S
                                    : 0.0f;
        }
        else
        {
            launch_speed_mm_s = VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S;
        }
    }

    replay_target->target_speed_mm_s = launch_speed_mm_s;
    replay_target->target_point.speed_mm_s = launch_speed_mm_s;
}

static void vehicle_control_test_launch_gate_start(float32 reference_theta_rad)
{
    if ((vehicle_control_launch_drag_enabled == FALSE)
        && (vehicle_control_launch_correction_enabled == FALSE))
    {
        vehicle_control_test_launch_gate_stop();
        return;
    }

    vehicle_control_test_launch_gate_active = TRUE;
    vehicle_control_test_launch_phase = (vehicle_control_launch_drag_enabled != FALSE)
                                            ? VEHICLE_CONTROL_REPLAY_LAUNCH_PHASE_DRAG_TIME
                                            : VEHICLE_CONTROL_REPLAY_LAUNCH_PHASE_CROSS_TRACK_STABILIZE;
    vehicle_control_test_launch_gate_timer_s = 0.0f;
    vehicle_control_test_launch_gate_stable_timer_s = 0.0f;
    vehicle_control_test_launch_wheel_ready_timer_s = 0.0f;
    vehicle_control_test_launch_wheel_ready = FALSE;
    if (vehicle_control_test_launch_phase
        == VEHICLE_CONTROL_REPLAY_LAUNCH_PHASE_CROSS_TRACK_STABILIZE)
    {
        vehicle_control_test_launch_wheel_ready = TRUE;
        vehicle_control_left_effective_target_mm_s =
            VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S;
        vehicle_control_right_effective_target_mm_s =
            VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S;
    }
    vehicle_control_test_launch_reference_theta_rad =
        algorithm_attitude_wrap_pi(reference_theta_rad);
    vehicle_control_state.target_theta_rad = vehicle_control_test_launch_reference_theta_rad;
    vehicle_control_state.yaw_speed_correction_mm_s = 0.0f;
    vehicle_control_yaw_feedback_correction_mm_s = 0.0f;
    vehicle_control_effective_yaw_speed_correction_mm_s = 0.0f;
}

static void vehicle_control_test_launch_gate_stop(void)
{
    vehicle_control_test_launch_gate_active = FALSE;
    vehicle_control_test_launch_phase = VEHICLE_CONTROL_REPLAY_LAUNCH_PHASE_OFF;
    vehicle_control_test_launch_gate_timer_s = 0.0f;
    vehicle_control_test_launch_gate_stable_timer_s = 0.0f;
    vehicle_control_test_launch_wheel_ready_timer_s = 0.0f;
    vehicle_control_test_launch_wheel_ready = FALSE;
}

static void vehicle_control_test_launch_complete(void)
{
    vehicle_control_test_launch_gate_stop();
    vehicle_control_speed_integral_clear();
    vehicle_control_base_start_accel_active = TRUE;
    vehicle_control_base_accel_limit_mm_s2 = vehicle_control_start_accel_limit_mm_s2;
    vehicle_control_left_start_accel_active = TRUE;
    vehicle_control_left_accel_limit_mm_s2 = vehicle_control_start_accel_limit_mm_s2;
    vehicle_control_right_start_accel_active = TRUE;
    vehicle_control_right_accel_limit_mm_s2 = vehicle_control_start_accel_limit_mm_s2;
}

static boolean vehicle_control_test_launch_gate_update(float32 dt_s)
{
    if (vehicle_control_test_launch_gate_active == FALSE)
    {
        return FALSE;
    }

    if (dt_s <= 0.0f)
    {
        return TRUE;
    }

    if (vehicle_control_test_launch_phase == VEHICLE_CONTROL_REPLAY_LAUNCH_PHASE_DRAG_TIME)
    {
        vehicle_control_test_launch_gate_timer_s += dt_s;
        vehicle_control_test_launch_gate_stable_timer_s = 0.0f;
        vehicle_control_state.target_theta_rad = vehicle_control_test_launch_reference_theta_rad;
        if (vehicle_control_test_launch_gate_timer_s >= VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_TIME_S)
        {
            vehicle_control_test_launch_phase =
                VEHICLE_CONTROL_REPLAY_LAUNCH_PHASE_CROSS_TRACK_STABILIZE;
            vehicle_control_test_launch_gate_timer_s = 0.0f;
            vehicle_control_test_launch_wheel_ready = TRUE;
            vehicle_control_left_effective_target_mm_s =
                VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S;
            vehicle_control_right_effective_target_mm_s =
                VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S;
            vehicle_control_speed_integral_clear();
            vehicle_control_state.yaw_speed_correction_mm_s = 0.0f;
            vehicle_control_yaw_feedback_correction_mm_s = 0.0f;
            vehicle_control_effective_yaw_speed_correction_mm_s = 0.0f;
        }
        return TRUE;
    }

    if (vehicle_control_test_launch_phase == VEHICLE_CONTROL_REPLAY_LAUNCH_PHASE_CROSS_TRACK_STABILIZE)
    {
        vehicle_control_state.target_theta_rad = vehicle_control_test_launch_reference_theta_rad;
        vehicle_control_test_launch_gate_timer_s += dt_s;
        if (vehicle_control_test_launch_gate_timer_s
            >= VEHICLE_CONTROL_REPLAY_LAUNCH_LOW_SPEED_HOLD_TIME_S)
        {
            vehicle_control_test_launch_complete();
        }
        return TRUE;
    }

    vehicle_control_test_launch_gate_stop();
    return FALSE;
}

static boolean vehicle_control_test_launch_drag_active(void)
{
    return ((vehicle_control_test_launch_gate_active != FALSE)
            && (vehicle_control_test_launch_phase == VEHICLE_CONTROL_REPLAY_LAUNCH_PHASE_DRAG_TIME))
               ? TRUE
               : FALSE;
}

static boolean vehicle_control_launch_correction_active(void)
{
    return (((vehicle_control_replay_launch_gate_active != FALSE)
             && (((vehicle_control_replay_launch_phase
                   == VEHICLE_CONTROL_REPLAY_LAUNCH_PHASE_DRAG_TIME)
                  && (vehicle_control_launch_drag_correction_enabled != FALSE))
                 || (vehicle_control_replay_launch_phase
                     == VEHICLE_CONTROL_REPLAY_LAUNCH_PHASE_CROSS_TRACK_STABILIZE)))
            || ((vehicle_control_test_launch_gate_active != FALSE)
                && (((vehicle_control_test_launch_phase
                      == VEHICLE_CONTROL_REPLAY_LAUNCH_PHASE_DRAG_TIME)
                     && (vehicle_control_launch_drag_correction_enabled != FALSE))
                    || (vehicle_control_test_launch_phase
                        == VEHICLE_CONTROL_REPLAY_LAUNCH_PHASE_CROSS_TRACK_STABILIZE)))
            || ((vehicle_control_load_test_state.step == VEHICLE_CONTROL_LOAD_TEST_STEP_DRAG)
                && (vehicle_control_launch_drag_correction_enabled != FALSE))
            || (vehicle_control_load_test_state.step
                == VEHICLE_CONTROL_LOAD_TEST_STEP_LOW_SPEED_HOLD))
               ? TRUE
               : FALSE;
}

static boolean vehicle_control_launch_drag_active(void)
{
    return ((vehicle_control_replay_launch_drag_active() != FALSE)
            || (vehicle_control_test_launch_drag_active() != FALSE)
            || (vehicle_control_load_test_state.step == VEHICLE_CONTROL_LOAD_TEST_STEP_DRAG))
               ? TRUE
               : FALSE;
}

static boolean vehicle_control_launch_low_speed_active(void)
{
    return vehicle_control_launch_correction_active();
}

static boolean vehicle_control_test_launch_moving_requested(void)
{
    return ((vehicle_control_speed_test_left_target_mm_s > VEHICLE_CONTROL_START_INTEGRAL_TARGET_THRESHOLD_MM_S)
            || (vehicle_control_speed_test_right_target_mm_s
                > VEHICLE_CONTROL_START_INTEGRAL_TARGET_THRESHOLD_MM_S))
               ? TRUE
               : FALSE;
}

static boolean vehicle_control_test_launch_pair_requested(void)
{
    return ((vehicle_control_speed_test_left_target_mm_s
             > VEHICLE_CONTROL_START_INTEGRAL_TARGET_THRESHOLD_MM_S)
            && (vehicle_control_speed_test_right_target_mm_s
                > VEHICLE_CONTROL_START_INTEGRAL_TARGET_THRESHOLD_MM_S))
               ? TRUE
               : FALSE;
}

static float32 vehicle_control_speed_target_slew_step(float32 current_target_mm_s,
                                                      float32 desired_target_mm_s,
                                                      float32 dt_s,
                                                      boolean* start_accel_active,
                                                      float32* accel_limit_runtime_mm_s2)
{
    float32 max_delta_mm_s;
    float32 delta_mm_s;
    float32 accel_limit_mm_s2 = vehicle_control_target_accel_limit_mm_s2;
    boolean launch_drag_active;

    if ((vehicle_control_speed_protection_enabled == FALSE) || (dt_s <= 0.0f))
    {
        return desired_target_mm_s;
    }

    delta_mm_s = desired_target_mm_s - current_target_mm_s;
    if (delta_mm_s < 0.0f)
    {
        if (vehicle_control_target_decel_limit_mm_s2 <= 0.0f)
        {
            return desired_target_mm_s;
        }
        accel_limit_mm_s2 = vehicle_control_target_decel_limit_mm_s2;
    }
    else if (vehicle_control_target_accel_limit_mm_s2 <= 0.0f)
    {
        return desired_target_mm_s;
    }
    launch_drag_active = vehicle_control_launch_drag_active();

    if ((launch_drag_active != FALSE)
        && (delta_mm_s > 0.0f)
        && (VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_ACCEL_LIMIT_MM_S2 > 0.0f))
    {
        accel_limit_mm_s2 = VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_ACCEL_LIMIT_MM_S2;
        if (start_accel_active != NULL_PTR)
        {
            *start_accel_active = TRUE;
        }
        if (accel_limit_runtime_mm_s2 != NULL_PTR)
        {
            *accel_limit_runtime_mm_s2 = VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_ACCEL_LIMIT_MM_S2;
        }
    }
    else if (start_accel_active != NULL_PTR)
    {
        if ((desired_target_mm_s > VEHICLE_CONTROL_START_INTEGRAL_TARGET_THRESHOLD_MM_S)
            && (current_target_mm_s <= VEHICLE_CONTROL_START_INTEGRAL_TARGET_THRESHOLD_MM_S)
            && (delta_mm_s > 0.0f))
        {
            *start_accel_active = TRUE;
            if (accel_limit_runtime_mm_s2 != NULL_PTR)
            {
                *accel_limit_runtime_mm_s2 = vehicle_control_start_accel_limit_mm_s2;
            }
        }
        if ((desired_target_mm_s <= VEHICLE_CONTROL_START_INTEGRAL_TARGET_THRESHOLD_MM_S)
            || (delta_mm_s <= 0.5f))
        {
            *start_accel_active = FALSE;
            if (accel_limit_runtime_mm_s2 != NULL_PTR)
            {
                *accel_limit_runtime_mm_s2 = vehicle_control_start_accel_limit_mm_s2;
            }
        }
        if ((*start_accel_active != FALSE)
            && (delta_mm_s > 0.0f)
            && (vehicle_control_start_accel_limit_mm_s2 > 0.0f))
        {
            if (vehicle_control_target_accel_limit_mm_s2 <= vehicle_control_start_accel_limit_mm_s2)
            {
                accel_limit_mm_s2 = vehicle_control_target_accel_limit_mm_s2;
            }
            else if (accel_limit_runtime_mm_s2 != NULL_PTR)
            {
                if (*accel_limit_runtime_mm_s2 < vehicle_control_start_accel_limit_mm_s2)
                {
                    *accel_limit_runtime_mm_s2 = vehicle_control_start_accel_limit_mm_s2;
                }
                if (*accel_limit_runtime_mm_s2 < vehicle_control_target_accel_limit_mm_s2)
                {
                    *accel_limit_runtime_mm_s2 += VEHICLE_CONTROL_ACCEL_LIMIT_RAMP_DEFAULT_MM_S3 * dt_s;
                    if (*accel_limit_runtime_mm_s2 > vehicle_control_target_accel_limit_mm_s2)
                    {
                        *accel_limit_runtime_mm_s2 = vehicle_control_target_accel_limit_mm_s2;
                    }
                }
                accel_limit_mm_s2 = *accel_limit_runtime_mm_s2;
            }
            else
            {
                accel_limit_mm_s2 = vehicle_control_start_accel_limit_mm_s2;
            }
        }
    }

    max_delta_mm_s = accel_limit_mm_s2 * dt_s;

    if (delta_mm_s > max_delta_mm_s)
    {
        return current_target_mm_s + max_delta_mm_s;
    }

    if (delta_mm_s < -max_delta_mm_s)
    {
        return current_target_mm_s - max_delta_mm_s;
    }

    return desired_target_mm_s;
}

static float32 vehicle_control_base_target_update(float32 desired_base_mm_s, float32 dt_s)
{
    desired_base_mm_s = vehicle_control_speed_no_reverse_target(desired_base_mm_s);
    vehicle_control_base_effective_target_mm_s =
        vehicle_control_speed_no_reverse_target(vehicle_control_speed_target_slew_step(
            vehicle_control_base_effective_target_mm_s,
            desired_base_mm_s,
            dt_s,
            &vehicle_control_base_start_accel_active,
            &vehicle_control_base_accel_limit_mm_s2));
    return vehicle_control_base_effective_target_mm_s;
}

static float32 vehicle_control_yaw_correction_available_get(float32 base_target_mm_s)
{
    if (base_target_mm_s <= VEHICLE_CONTROL_ANGLE_CORRECTION_MIN_WHEEL_SPEED_MM_S)
    {
        return 0.0f;
    }

    return base_target_mm_s - VEHICLE_CONTROL_ANGLE_CORRECTION_MIN_WHEEL_SPEED_MM_S;
}

static float32 vehicle_control_yaw_rate_available_get(float32 base_target_mm_s)
{
    return (2.0f * vehicle_control_yaw_correction_available_get(base_target_mm_s))
           / VEHICLE_CONTROL_WHEEL_BASE_MM;
}

static float32 vehicle_control_yaw_correction_slew_step(float32 current_mm_s,
                                                        float32 target_mm_s,
                                                        float32 dt_s,
                                                        float32 slew_limit_mm_s2)
{
    float32 max_delta_mm_s;
    float32 delta_mm_s;

    if ((slew_limit_mm_s2 <= 0.0f) || (dt_s <= 0.0f))
    {
        return target_mm_s;
    }

    max_delta_mm_s = slew_limit_mm_s2 * dt_s;
    delta_mm_s = target_mm_s - current_mm_s;

    if (delta_mm_s > max_delta_mm_s)
    {
        return current_mm_s + max_delta_mm_s;
    }

    if (delta_mm_s < -max_delta_mm_s)
    {
        return current_mm_s - max_delta_mm_s;
    }

    return target_mm_s;
}

static void vehicle_control_wheel_target_accel_limit_apply(float32* left_target_mm_s,
                                                          float32* right_target_mm_s,
                                                          float32 left_feedback_mm_s,
                                                          float32 right_feedback_mm_s,
                                                          float32 dt_s)
{
    const vehicle_control_cfg_t* cfg = vehicle_control_cfg_get();
    float32 desired_left_mm_s;
    float32 desired_right_mm_s;
    float32 limited_left_mm_s;
    float32 limited_right_mm_s;

    if ((left_target_mm_s == NULL_PTR) || (right_target_mm_s == NULL_PTR))
    {
        return;
    }

    desired_left_mm_s =
        vehicle_control_speed_no_reverse_target(algorithm_control_clamp_f32(*left_target_mm_s,
                                                                            cfg->target_speed_min_mm_s,
                                                                            cfg->target_speed_max_mm_s));
    desired_right_mm_s =
        vehicle_control_speed_no_reverse_target(algorithm_control_clamp_f32(*right_target_mm_s,
                                                                            cfg->target_speed_min_mm_s,
                                                                            cfg->target_speed_max_mm_s));
    vehicle_control_reverse_guard_left_before_mm_s = desired_left_mm_s;
    vehicle_control_reverse_guard_right_before_mm_s = desired_right_mm_s;

    vehicle_control_speed_target_pair_update(desired_left_mm_s,
                                             desired_right_mm_s,
                                             left_feedback_mm_s,
                                             right_feedback_mm_s,
                                             dt_s,
                                             &limited_left_mm_s,
                                             &limited_right_mm_s);

    limited_left_mm_s =
        vehicle_control_speed_no_reverse_target(algorithm_control_clamp_f32(limited_left_mm_s,
                                                                            cfg->target_speed_min_mm_s,
                                                                            cfg->target_speed_max_mm_s));
    limited_right_mm_s =
        vehicle_control_speed_no_reverse_target(algorithm_control_clamp_f32(limited_right_mm_s,
                                                                            cfg->target_speed_min_mm_s,
                                                                            cfg->target_speed_max_mm_s));
    vehicle_control_reverse_guard_left_after_mm_s = limited_left_mm_s;
    vehicle_control_reverse_guard_right_after_mm_s = limited_right_mm_s;
    *left_target_mm_s = limited_left_mm_s;
    *right_target_mm_s = limited_right_mm_s;
}

static void vehicle_control_wheel_target_tracking_limit_apply(float32* left_target_mm_s,
                                                              float32* right_target_mm_s,
                                                              float32 left_feedback_mm_s,
                                                              float32 right_feedback_mm_s)
{
    float32 left_max_target_mm_s;
    float32 right_max_target_mm_s;
    boolean limited = FALSE;

    if ((left_target_mm_s == NULL_PTR) || (right_target_mm_s == NULL_PTR))
    {
        return;
    }

    vehicle_control_tracking_limit_left_before_mm_s = *left_target_mm_s;
    vehicle_control_tracking_limit_right_before_mm_s = *right_target_mm_s;
    vehicle_control_tracking_limit_left_after_mm_s = *left_target_mm_s;
    vehicle_control_tracking_limit_right_after_mm_s = *right_target_mm_s;
    vehicle_control_tracking_limit_active = FALSE;

    if ((vehicle_control_speed_protection_enabled == FALSE)
        || ((vehicle_control_state.mode != VEHICLE_CONTROL_MODE_REPLAY)
            && (vehicle_control_speed_test_enabled == FALSE)))
    {
        return;
    }

    left_max_target_mm_s =
        left_feedback_mm_s + VEHICLE_CONTROL_WHEEL_TARGET_ACCEL_LEAD_LIMIT_MM_S;
    right_max_target_mm_s =
        right_feedback_mm_s + VEHICLE_CONTROL_WHEEL_TARGET_ACCEL_LEAD_LIMIT_MM_S;
    if (*left_target_mm_s > left_max_target_mm_s)
    {
        *left_target_mm_s = left_max_target_mm_s;
        limited = TRUE;
    }
    if (*right_target_mm_s > right_max_target_mm_s)
    {
        *right_target_mm_s = right_max_target_mm_s;
        limited = TRUE;
    }

    vehicle_control_tracking_limit_left_after_mm_s = *left_target_mm_s;
    vehicle_control_tracking_limit_right_after_mm_s = *right_target_mm_s;
    vehicle_control_tracking_limit_active = limited;
    if (limited != FALSE)
    {
        vehicle_control_tracking_limit_event_count++;
    }
}

static float32 vehicle_control_speed_no_reverse_target(float32 target_mm_s)
{
    if (target_mm_s < 0.0f)
    {
        return 0.0f;
    }

    return target_mm_s;
}

static float32 vehicle_control_reverse_guard_target(float32 desired_target_mm_s,
                                                    float32 feedback_speed_mm_s)
{
    (void)feedback_speed_mm_s;

    if (desired_target_mm_s < 0.0f)
    {
        return 0.0f;
    }

    return desired_target_mm_s;
}

static void vehicle_control_reverse_feedback_guard_reset(void)
{
    vehicle_control_left_reverse_feedback_timer_s = 0.0f;
    vehicle_control_right_reverse_feedback_timer_s = 0.0f;
    vehicle_control_reverse_feedback_cap_hold_timer_s = 0.0f;
    vehicle_control_reverse_feedback_cap_active = FALSE;
    vehicle_control_reverse_feedback_fault_latched = FALSE;
    vehicle_control_reverse_guard_left_before_mm_s = 0.0f;
    vehicle_control_reverse_guard_right_before_mm_s = 0.0f;
    vehicle_control_reverse_guard_left_after_mm_s = 0.0f;
    vehicle_control_reverse_guard_right_after_mm_s = 0.0f;
    vehicle_control_reverse_guard_base_cap_mm_s = 0.0f;
}

static void vehicle_control_reverse_feedback_guard_update(float32 left_target_mm_s,
                                                         float32 right_target_mm_s,
                                                         float32 left_feedback_mm_s,
                                                         float32 right_feedback_mm_s,
                                                         float32 dt_s)
{
    boolean left_reverse;
    boolean right_reverse;
    boolean was_cap_active;

    if ((vehicle_control_state.mode != VEHICLE_CONTROL_MODE_REPLAY)
        || (vehicle_control_speed_protection_enabled == FALSE)
        || (vehicle_control_reverse_guard_speed_mm_s <= 0.0f)
        || (dt_s <= 0.0f))
    {
        vehicle_control_reverse_feedback_guard_reset();
        return;
    }

    left_reverse =
        ((left_target_mm_s > VEHICLE_CONTROL_REVERSE_GUARD_TARGET_THRESHOLD_MM_S)
         && (left_feedback_mm_s < -vehicle_control_reverse_guard_speed_mm_s))
            ? TRUE
            : FALSE;
    right_reverse =
        ((right_target_mm_s > VEHICLE_CONTROL_REVERSE_GUARD_TARGET_THRESHOLD_MM_S)
         && (right_feedback_mm_s < -vehicle_control_reverse_guard_speed_mm_s))
            ? TRUE
            : FALSE;

    if (left_reverse != FALSE)
    {
        vehicle_control_left_reverse_feedback_timer_s += dt_s;
    }
    else
    {
        vehicle_control_left_reverse_feedback_timer_s = 0.0f;
    }

    if (right_reverse != FALSE)
    {
        vehicle_control_right_reverse_feedback_timer_s += dt_s;
    }
    else
    {
        vehicle_control_right_reverse_feedback_timer_s = 0.0f;
    }

    if ((vehicle_control_left_reverse_feedback_timer_s >= VEHICLE_CONTROL_REVERSE_GUARD_CAP_TIME_S)
        || (vehicle_control_right_reverse_feedback_timer_s >= VEHICLE_CONTROL_REVERSE_GUARD_CAP_TIME_S))
    {
        vehicle_control_reverse_feedback_cap_hold_timer_s = VEHICLE_CONTROL_REVERSE_GUARD_CAP_HOLD_S;
    }
    else if (vehicle_control_reverse_feedback_cap_hold_timer_s > 0.0f)
    {
        vehicle_control_reverse_feedback_cap_hold_timer_s -= dt_s;
        if (vehicle_control_reverse_feedback_cap_hold_timer_s < 0.0f)
        {
            vehicle_control_reverse_feedback_cap_hold_timer_s = 0.0f;
        }
    }

    was_cap_active = vehicle_control_reverse_feedback_cap_active;
    vehicle_control_reverse_feedback_cap_active =
        ((vehicle_control_reverse_feedback_cap_hold_timer_s > 0.0f)
         || (vehicle_control_reverse_feedback_fault_latched != FALSE))
            ? TRUE
            : FALSE;

    if ((was_cap_active == FALSE) && (vehicle_control_reverse_feedback_cap_active != FALSE))
    {
        vehicle_control_reverse_feedback_event_count++;
    }

    if ((vehicle_control_left_reverse_feedback_timer_s >= VEHICLE_CONTROL_REVERSE_GUARD_FAULT_TIME_S)
        || (vehicle_control_right_reverse_feedback_timer_s >= VEHICLE_CONTROL_REVERSE_GUARD_FAULT_TIME_S))
    {
        vehicle_control_reverse_feedback_fault_latched = TRUE;
    }

    vehicle_control_reverse_guard_base_cap_mm_s =
        (vehicle_control_reverse_feedback_cap_active != FALSE)
            ? VEHICLE_CONTROL_REVERSE_GUARD_BASE_CAP_MM_S
            : 0.0f;
}

static void vehicle_control_speed_target_pair_update(float32 desired_left_mm_s,
                                                     float32 desired_right_mm_s,
                                                     float32 left_feedback_mm_s,
                                                     float32 right_feedback_mm_s,
                                                     float32 dt_s,
                                                     float32* left_target_mm_s,
                                                     float32* right_target_mm_s)
{
    float32 guarded_left_mm_s;
    float32 guarded_right_mm_s;

    guarded_left_mm_s = vehicle_control_reverse_guard_target(desired_left_mm_s, left_feedback_mm_s);
    guarded_right_mm_s = vehicle_control_reverse_guard_target(desired_right_mm_s, right_feedback_mm_s);
    guarded_left_mm_s = vehicle_control_speed_no_reverse_target(guarded_left_mm_s);
    guarded_right_mm_s = vehicle_control_speed_no_reverse_target(guarded_right_mm_s);
    vehicle_control_left_effective_target_mm_s =
        vehicle_control_speed_no_reverse_target(vehicle_control_left_effective_target_mm_s);
    vehicle_control_right_effective_target_mm_s =
        vehicle_control_speed_no_reverse_target(vehicle_control_right_effective_target_mm_s);

    vehicle_control_left_effective_target_mm_s =
        vehicle_control_speed_no_reverse_target(vehicle_control_speed_target_slew_step(
            vehicle_control_left_effective_target_mm_s,
            guarded_left_mm_s,
            dt_s,
            &vehicle_control_left_start_accel_active,
            &vehicle_control_left_accel_limit_mm_s2));
    vehicle_control_right_effective_target_mm_s =
        vehicle_control_speed_no_reverse_target(vehicle_control_speed_target_slew_step(
            vehicle_control_right_effective_target_mm_s,
            guarded_right_mm_s,
            dt_s,
            &vehicle_control_right_start_accel_active,
            &vehicle_control_right_accel_limit_mm_s2));

    if (left_target_mm_s != NULL_PTR)
    {
        *left_target_mm_s = vehicle_control_left_effective_target_mm_s;
    }

    if (right_target_mm_s != NULL_PTR)
    {
        *right_target_mm_s = vehicle_control_right_effective_target_mm_s;
    }
}

static sint32 vehicle_control_duty_from_delta(sint32 idle_duty, float32 duty_delta)
{
    float32 duty = (float32)idle_duty + duty_delta;

    if (duty >= 0.0f)
    {
        return (sint32)(duty + 0.5f);
    }

    return (sint32)(duty - 0.5f);
}

static sint32 vehicle_control_drive_clamp_duty(vehicle_esc_role_t role, sint32 duty_cycle)
{
    duty_cycle = module_vehicle_esc_clamp_duty(role, duty_cycle);

    if (duty_cycle > VEHICLE_CONTROL_DRIVE_DUTY_LIMIT_TICK)
    {
        duty_cycle = VEHICLE_CONTROL_DRIVE_DUTY_LIMIT_TICK;
    }

    return module_vehicle_esc_clamp_duty(role, duty_cycle);
}

static void vehicle_control_speed_feedforward_table_reset(void)
{
    vehicle_control_speed_feedforward_table_reset_one(vehicle_control_left_speed_feedforward_table);
    vehicle_control_speed_feedforward_table_reset_one(vehicle_control_right_speed_feedforward_table);
}

static void vehicle_control_speed_feedforward_table_reset_one(
    vehicle_control_speed_feedforward_point_t* table)
{
    static const vehicle_control_speed_feedforward_point_t left_default_table[VEHICLE_CONTROL_SPEED_FEEDFORWARD_POINT_COUNT] =
    {
        {0.0f, 0.0f, TRUE},
        {400.0f, 717.0f, TRUE},
        {500.0f, 806.0f, TRUE},
        {600.0f, 895.0f, TRUE},
        {700.0f, 984.0f, TRUE},
        {800.0f, 1074.0f, TRUE},
        {900.0f, 1163.0f, TRUE},
        {1000.0f, 1252.0f, TRUE},
        {1200.0f, 1430.0f, TRUE},
        {1500.0f, 2188.0f, TRUE},
        {2000.0f, 2787.0f, TRUE},
        {2500.0f, 3324.0f, TRUE},
        {3000.0f, 3890.0f, TRUE},
        {3500.0f, 0.0f, FALSE},
        {4000.0f, 4869.0f, TRUE},
        {4500.0f, 0.0f, FALSE},
        {4750.0f, 0.0f, FALSE},
        {5000.0f, 0.0f, FALSE},
        {5250.0f, 0.0f, FALSE},
        {5500.0f, 0.0f, FALSE},
        {5750.0f, 0.0f, FALSE},
        {6000.0f, 0.0f, FALSE},
        {6500.0f, 0.0f, FALSE},
        {7000.0f, 0.0f, FALSE},
        {9000.0f, 0.0f, FALSE},
    };
    static const vehicle_control_speed_feedforward_point_t right_default_table[VEHICLE_CONTROL_SPEED_FEEDFORWARD_POINT_COUNT] =
    {
        {0.0f, 0.0f, TRUE},
        {400.0f, 620.0f, TRUE},
        {500.0f, 714.0f, TRUE},
        {600.0f, 808.0f, TRUE},
        {700.0f, 901.0f, TRUE},
        {800.0f, 995.0f, TRUE},
        {900.0f, 1088.0f, TRUE},
        {1000.0f, 1182.0f, TRUE},
        {1200.0f, 1369.0f, TRUE},
        {1500.0f, 2217.0f, TRUE},
        {2000.0f, 2825.0f, TRUE},
        {2500.0f, 3370.0f, TRUE},
        {3000.0f, 3950.0f, TRUE},
        {3500.0f, 0.0f, FALSE},
        {4000.0f, 4486.0f, TRUE},
        {4500.0f, 0.0f, FALSE},
        {4750.0f, 0.0f, FALSE},
        {5000.0f, 0.0f, FALSE},
        {5250.0f, 0.0f, FALSE},
        {5500.0f, 0.0f, FALSE},
        {5750.0f, 0.0f, FALSE},
        {6000.0f, 0.0f, FALSE},
        {6500.0f, 0.0f, FALSE},
        {7000.0f, 0.0f, FALSE},
        {9000.0f, 0.0f, FALSE},
    };
    const vehicle_control_speed_feedforward_point_t* default_table = left_default_table;
    uint32 index;

    if (table == NULL_PTR)
    {
        return;
    }

    if (table == vehicle_control_right_speed_feedforward_table)
    {
        default_table = right_default_table;
    }

    for (index = 0u; index < VEHICLE_CONTROL_SPEED_FEEDFORWARD_POINT_COUNT; index++)
    {
        table[index] = default_table[index];
    }
}

static void vehicle_control_speed_feedforward_table_print(
    const char* side,
    const vehicle_control_speed_feedforward_point_t* table)
{
    uint32 index;

    if ((side == NULL_PTR) || (table == NULL_PTR))
    {
        return;
    }

    for (index = 0u; index < VEHICLE_CONTROL_SPEED_FEEDFORWARD_POINT_COUNT; index++)
    {
        if (table[index].valid != FALSE)
        {
            tools_printf("{vff}%s,%u,%.3f,%.3f\r\n",
                         side,
                         (unsigned int)index,
                         (double)table[index].speed_mm_s,
                         (double)table[index].duty_tick);
        }
    }
}

static boolean vehicle_control_speed_feedforward_point_set(
    vehicle_control_speed_feedforward_point_t* table,
    float32 speed_mm_s,
    float32 duty_tick)
{
    const vehicle_control_cfg_t* cfg = vehicle_control_cfg_get();
    uint32 index;
    uint32 empty_index = VEHICLE_CONTROL_SPEED_FEEDFORWARD_POINT_COUNT;
    float32 speed_diff;

    if (speed_mm_s < 0.0f)
    {
        speed_mm_s = -speed_mm_s;
    }

    speed_mm_s = algorithm_control_clamp_f32(speed_mm_s, 0.0f, cfg->target_speed_max_mm_s);
    if (duty_tick < 0.0f)
    {
        duty_tick = 0.0f;
    }

    if (table == NULL_PTR)
    {
        return FALSE;
    }

    for (index = 0u; index < VEHICLE_CONTROL_SPEED_FEEDFORWARD_POINT_COUNT; index++)
    {
        if (table[index].valid == FALSE)
        {
            if (empty_index == VEHICLE_CONTROL_SPEED_FEEDFORWARD_POINT_COUNT)
            {
                empty_index = index;
            }
            continue;
        }

        speed_diff = table[index].speed_mm_s - speed_mm_s;
        if ((speed_diff > -0.5f) && (speed_diff < 0.5f))
        {
            table[index].speed_mm_s = speed_mm_s;
            table[index].duty_tick = duty_tick;
            return TRUE;
        }
    }

    if (empty_index >= VEHICLE_CONTROL_SPEED_FEEDFORWARD_POINT_COUNT)
    {
        return FALSE;
    }

    table[empty_index].speed_mm_s = speed_mm_s;
    table[empty_index].duty_tick = duty_tick;
    table[empty_index].valid = TRUE;
    return TRUE;
}

static float32 vehicle_control_speed_feedforward_calculate(
    const vehicle_control_speed_feedforward_point_t* table,
    float32 target_speed_mm_s)
{
    uint32 index;
    boolean lower_valid = FALSE;
    boolean upper_valid = FALSE;
    float32 lower_speed = 0.0f;
    float32 upper_speed = 0.0f;
    float32 lower_duty = 0.0f;
    float32 upper_duty = 0.0f;
    boolean previous_valid = FALSE;
    float32 previous_speed = 0.0f;
    float32 previous_duty = 0.0f;
    float32 target_abs_mm_s = target_speed_mm_s;
    float32 sign = 1.0f;
    float32 ratio;

    if ((vehicle_control_speed_feedforward_enabled == FALSE) || (table == NULL_PTR))
    {
        return 0.0f;
    }

    if (target_abs_mm_s < 0.0f)
    {
        target_abs_mm_s = -target_abs_mm_s;
        sign = -1.0f;
    }

    if (target_abs_mm_s <= 0.5f)
    {
        return 0.0f;
    }

    for (index = 0u; index < VEHICLE_CONTROL_SPEED_FEEDFORWARD_POINT_COUNT; index++)
    {
        if (table[index].valid == FALSE)
        {
            continue;
        }

        if (table[index].speed_mm_s <= target_abs_mm_s)
        {
            if ((lower_valid == FALSE)
                || (table[index].speed_mm_s > lower_speed))
            {
                lower_speed = table[index].speed_mm_s;
                lower_duty = table[index].duty_tick;
                lower_valid = TRUE;
            }
        }

        if (table[index].speed_mm_s >= target_abs_mm_s)
        {
            if ((upper_valid == FALSE)
                || (table[index].speed_mm_s < upper_speed))
            {
                upper_speed = table[index].speed_mm_s;
                upper_duty = table[index].duty_tick;
                upper_valid = TRUE;
            }
        }
    }

    if ((lower_valid != FALSE) && (upper_valid != FALSE))
    {
        if ((upper_speed - lower_speed) < 0.5f)
        {
            return sign * lower_duty;
        }

        ratio = (target_abs_mm_s - lower_speed) / (upper_speed - lower_speed);
        return sign * (lower_duty + ((upper_duty - lower_duty) * ratio));
    }

    if (lower_valid != FALSE)
    {
        for (index = 0u; index < VEHICLE_CONTROL_SPEED_FEEDFORWARD_POINT_COUNT; index++)
        {
            if (table[index].valid == FALSE)
            {
                continue;
            }

            if (table[index].speed_mm_s < lower_speed)
            {
                if ((previous_valid == FALSE)
                    || (table[index].speed_mm_s > previous_speed))
                {
                    previous_speed = table[index].speed_mm_s;
                    previous_duty = table[index].duty_tick;
                    previous_valid = TRUE;
                }
            }
        }

        if ((previous_valid != FALSE) && ((lower_speed - previous_speed) > 0.5f))
        {
            ratio = (target_abs_mm_s - previous_speed) / (lower_speed - previous_speed);
            return sign * (previous_duty + ((lower_duty - previous_duty) * ratio));
        }

        return sign * lower_duty;
    }

    if (upper_valid != FALSE)
    {
        return sign * upper_duty;
    }

    return sign * (VEHICLE_CONTROL_SPEED_TEST_FEEDFORWARD_KS_TICK
                   + (VEHICLE_CONTROL_SPEED_TEST_FEEDFORWARD_KV_TICK_PER_MM_S * target_abs_mm_s));
}

static float32 vehicle_control_abs_f32(float32 value)
{
    return (value < 0.0f) ? -value : value;
}

static float32 vehicle_control_speed_position_pid_update(const algorithm_control_pid_cfg_t* cfg,
                                                         algorithm_control_pid_state_t* pid,
                                                         float32 target,
                                                        float32 feedback,
                                                        float32 dt_s,
                                                        boolean integral_enable)
{
    float32 error = target - feedback;
    float32 derivative = 0.0f;
    float32 integral_output;

    if (dt_s > 0.0f)
    {
        derivative = (error - pid->error) / dt_s;
        if (integral_enable != FALSE)
        {
            pid->integral += error * dt_s;
            vehicle_control_speed_integral_limit_clamp_state(pid);
        }
    }

    pid->target = target;
    pid->feedback = feedback;
    pid->previous_error = pid->last_error;
    pid->last_error = pid->error;
    pid->error = error;
    integral_output = cfg->ki * pid->integral;
    pid->output = (cfg->kp * error) + integral_output + (cfg->kd * derivative);
    pid->output = algorithm_control_clamp_f32(pid->output, cfg->output_min, cfg->output_max);

    return pid->output;
}

static uint32 vehicle_control_suction_duty_from_percent(float32 percent)
{
    float32 duty;
    uint32 max_duty = (uint32)module_vehicle_esc_clamp_duty(VEHICLE_ESC_ROLE_SUCTION, 10000);

    percent = algorithm_control_clamp_f32(percent, 0.0f, 100.0f);
    duty = ((float32)max_duty * percent) / 100.0f;
    return module_vehicle_esc_clamp_duty(VEHICLE_ESC_ROLE_SUCTION, (uint32)(duty + 0.5f));
}

static void vehicle_control_suction_apply_duty(uint32 duty_cycle)
{
    vehicle_control_set_suction_duty(duty_cycle);
    vehicle_control_suction_process();
    module_vehicle_esc_flush(TRUE);
}

void vehicle_control_suction_process(void)
{
    uint32 target_duty = module_vehicle_esc_clamp_duty(VEHICLE_ESC_ROLE_SUCTION, vehicle_control_state.suction_duty);
    uint32 last_output_duty = vehicle_control_suction_output_duty;

    if (vehicle_control_suction_output_duty < target_duty)
    {
        uint32 delta = target_duty - vehicle_control_suction_output_duty;

        if (delta > VEHICLE_CONTROL_SUCTION_DUTY_SLEW_LIMIT_TICK_PER_UPDATE)
        {
            vehicle_control_suction_output_duty += VEHICLE_CONTROL_SUCTION_DUTY_SLEW_LIMIT_TICK_PER_UPDATE;
        }
        else
        {
            vehicle_control_suction_output_duty = target_duty;
        }
    }
    else if (vehicle_control_suction_output_duty > target_duty)
    {
        uint32 delta = vehicle_control_suction_output_duty - target_duty;

        if (delta > VEHICLE_CONTROL_SUCTION_DUTY_SLEW_LIMIT_TICK_PER_UPDATE)
        {
            vehicle_control_suction_output_duty -= VEHICLE_CONTROL_SUCTION_DUTY_SLEW_LIMIT_TICK_PER_UPDATE;
        }
        else
        {
            vehicle_control_suction_output_duty = target_duty;
        }
    }

    vehicle_control_suction_output_duty =
        module_vehicle_esc_clamp_duty(VEHICLE_ESC_ROLE_SUCTION, vehicle_control_suction_output_duty);
    module_vehicle_gyro_suction_duty_notify(vehicle_control_suction_output_duty);
    (void)module_vehicle_esc_set_duty(VEHICLE_ESC_ROLE_SUCTION, vehicle_control_suction_output_duty);

    if (vehicle_control_suction_output_duty != last_output_duty)
    {
        module_vehicle_esc_flush(TRUE);
    }
}

static void vehicle_control_load_test_capture_reset(void)
{
    vehicle_control_load_test_state.left_speed_sum_mm_s = 0.0f;
    vehicle_control_load_test_state.right_speed_sum_mm_s = 0.0f;
    vehicle_control_load_test_state.left_duty_sum_tick = 0.0f;
    vehicle_control_load_test_state.right_duty_sum_tick = 0.0f;
    vehicle_control_load_test_state.capture_count = 0u;
}

static void vehicle_control_load_test_drag_begin(void)
{
    vehicle_control_test_launch_reference_theta_rad =
        module_vehicle_pose_fusion_heading_get();
    vehicle_control_load_test_state.step = VEHICLE_CONTROL_LOAD_TEST_STEP_DRAG;
    vehicle_control_load_test_state.timer_s = 0.0f;
    vehicle_control_load_test_capture_reset();
    vehicle_control_speed_test_set_target(0.0f, 0.0f);
    vehicle_control_test_launch_gate_stop();
    vehicle_control_speed_integral_clear();
    vehicle_control_speed_target_runtime_reset();
    vehicle_control_speed_accel_feedforward_reset();
    vehicle_control_state.target_theta_rad = vehicle_control_test_launch_reference_theta_rad;
    vehicle_control_state.yaw_speed_correction_mm_s = 0.0f;
    vehicle_control_yaw_feedback_correction_mm_s = 0.0f;
    vehicle_control_effective_yaw_speed_correction_mm_s = 0.0f;
    algorithm_control_pid_reset(&vehicle_control_launch_angle_pid);
    vehicle_control_d_yaw_rate_lpf_reset();
    vehicle_control_target_yaw_rate_reset();
    vehicle_control_steer_loop_divider = 0u;
    tools_printf("{vload}drag,pwm,750,time_ms,400,speed_pid,0,heading,1,limit,200\r\n");
}

static void vehicle_control_load_test_closed_loop_begin(void)
{
    vehicle_control_speed_integral_clear();
    vehicle_control_speed_accel_feedforward_reset();
    vehicle_control_speed_test_heading_enabled = FALSE;
    vehicle_control_state.yaw_speed_correction_mm_s = 0.0f;
    vehicle_control_yaw_feedback_correction_mm_s = 0.0f;
    vehicle_control_effective_yaw_speed_correction_mm_s = 0.0f;
    algorithm_control_pid_reset(&vehicle_control_launch_angle_pid);
    vehicle_control_d_yaw_rate_lpf_reset();
    vehicle_control_target_yaw_rate_reset();
    vehicle_control_load_test_state.step = VEHICLE_CONTROL_LOAD_TEST_STEP_RUN;
    vehicle_control_load_test_state.timer_s = 0.0f;
    vehicle_control_load_test_capture_reset();
    vehicle_control_speed_test_set_target(vehicle_control_load_test_state.left_speed_mm_s,
                                          vehicle_control_load_test_state.right_speed_mm_s);
    vehicle_control_test_launch_gate_stop();
    tools_printf("{vload}closed_loop,run_ms,750,capture_ms,250,ff,%u,accff,0,heading,0,lead,900\r\n",
                 (unsigned int)((vehicle_control_load_test_feedforward_enabled != FALSE) ? 1u : 0u));
}

static void vehicle_control_load_test_low_speed_hold_begin(void)
{
    vehicle_control_speed_integral_clear();
    vehicle_control_speed_target_runtime_reset();
    vehicle_control_left_effective_target_mm_s = VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S;
    vehicle_control_right_effective_target_mm_s = VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S;
    vehicle_control_base_effective_target_mm_s = VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S;
    vehicle_control_load_test_state.step = VEHICLE_CONTROL_LOAD_TEST_STEP_LOW_SPEED_HOLD;
    vehicle_control_load_test_state.timer_s = 0.0f;
    vehicle_control_speed_test_set_target(VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S,
                                          VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S);
    vehicle_control_test_launch_gate_stop();
    vehicle_control_speed_test_heading_enabled = FALSE;
    vehicle_control_state.yaw_speed_correction_mm_s = 0.0f;
    vehicle_control_yaw_feedback_correction_mm_s = 0.0f;
    vehicle_control_effective_yaw_speed_correction_mm_s = 0.0f;
    vehicle_control_test_launch_gate_stable_timer_s = 0.0f;
    vehicle_control_test_launch_wheel_ready_timer_s = 0.0f;
    vehicle_control_test_launch_wheel_ready = FALSE;
    algorithm_control_pid_reset(&vehicle_control_launch_angle_pid);
    vehicle_control_d_yaw_rate_lpf_reset();
    vehicle_control_target_yaw_rate_reset();
    tools_printf("{vload}low_speed_hold,speed,400,heading,0,ready_ms,100,stable_ms,300\r\n");
}

static void vehicle_control_load_test_cancel(void)
{
    vehicle_control_load_test_state.step = VEHICLE_CONTROL_LOAD_TEST_STEP_IDLE;
    vehicle_control_load_test_state.timer_s = 0.0f;
    vehicle_control_load_test_state.left_speed_mm_s = 0.0f;
    vehicle_control_load_test_state.right_speed_mm_s = 0.0f;
    vehicle_control_load_test_capture_reset();
}

static void vehicle_control_load_test_update(float32 dt_s)
{
    const module_vehicle_encoder_observation_t* encoder_observation;
    float32 left_speed_average_mm_s;
    float32 right_speed_average_mm_s;
    float32 left_duty_average_tick;
    float32 right_duty_average_tick;
    float32 left_error_limit_mm_s;
    float32 right_error_limit_mm_s;
    boolean result_valid;

    if (vehicle_control_load_test_state.step == VEHICLE_CONTROL_LOAD_TEST_STEP_IDLE)
    {
        return;
    }

    switch (vehicle_control_load_test_state.step)
    {
        case VEHICLE_CONTROL_LOAD_TEST_STEP_WAIT_START:
            vehicle_control_load_test_state.timer_s += dt_s;
            if (vehicle_control_load_test_state.timer_s >= VEHICLE_CONTROL_LOAD_TEST_START_DELAY_S)
            {
                if (vehicle_control_auto_suction_enabled != FALSE)
                {
                    vehicle_control_suction_apply_duty(VEHICLE_CONTROL_LOAD_TEST_SUCTION_DUTY);
                    vehicle_control_speed_test_set_target(0.0f, 0.0f);
                    vehicle_control_load_test_state.step = VEHICLE_CONTROL_LOAD_TEST_STEP_SUCTION_HOLD;
                    vehicle_control_load_test_state.timer_s = 0.0f;
                }
                else
                {
                    vehicle_control_load_test_drag_begin();
                }
            }
            break;

        case VEHICLE_CONTROL_LOAD_TEST_STEP_SUCTION_HOLD:
            vehicle_control_load_test_state.timer_s += dt_s;
            if (vehicle_control_load_test_state.timer_s >= VEHICLE_CONTROL_LOAD_TEST_SUCTION_HOLD_S)
            {
                vehicle_control_load_test_drag_begin();
            }
            break;

        case VEHICLE_CONTROL_LOAD_TEST_STEP_DRAG:
            vehicle_control_load_test_state.timer_s += dt_s;
            if (vehicle_control_load_test_state.timer_s >= VEHICLE_CONTROL_LOAD_TEST_DRAG_TIME_S)
            {
                vehicle_control_load_test_low_speed_hold_begin();
            }
            break;

        case VEHICLE_CONTROL_LOAD_TEST_STEP_LOW_SPEED_HOLD:
            vehicle_control_load_test_state.timer_s += dt_s;
            encoder_observation = module_vehicle_encoder_observation_get();
            if (vehicle_control_test_launch_wheel_ready == FALSE)
            {
                if ((encoder_observation != NULL_PTR)
                    && (encoder_observation->left_speed_mm_s
                        >= VEHICLE_CONTROL_REPLAY_LAUNCH_WHEEL_READY_SPEED_MM_S)
                    && (encoder_observation->right_speed_mm_s
                        >= VEHICLE_CONTROL_REPLAY_LAUNCH_WHEEL_READY_SPEED_MM_S))
                {
                    vehicle_control_test_launch_wheel_ready_timer_s += dt_s;
                    if (vehicle_control_test_launch_wheel_ready_timer_s
                        >= VEHICLE_CONTROL_REPLAY_LAUNCH_WHEEL_READY_TIME_S)
                    {
                        vehicle_control_test_launch_wheel_ready = TRUE;
                        vehicle_control_test_launch_gate_stable_timer_s = 0.0f;
                    }
                }
                else
                {
                    vehicle_control_test_launch_wheel_ready_timer_s = 0.0f;
                }
            }
            else if ((encoder_observation != NULL_PTR)
                     && (vehicle_control_abs_f32(encoder_observation->left_speed_mm_s
                                                - VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S)
                         <= VEHICLE_CONTROL_REPLAY_LAUNCH_CORRECTION_STABLE_LIMIT_MM_S)
                     && (vehicle_control_abs_f32(encoder_observation->right_speed_mm_s
                                                - VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S)
                         <= VEHICLE_CONTROL_REPLAY_LAUNCH_CORRECTION_STABLE_LIMIT_MM_S))
            {
                vehicle_control_test_launch_gate_stable_timer_s += dt_s;
                if (vehicle_control_test_launch_gate_stable_timer_s
                    >= VEHICLE_CONTROL_REPLAY_LAUNCH_LOW_SPEED_HOLD_TIME_S)
                {
                    vehicle_control_load_test_closed_loop_begin();
                }
            }
            else
            {
                vehicle_control_test_launch_gate_stable_timer_s = 0.0f;
            }
            break;

        case VEHICLE_CONTROL_LOAD_TEST_STEP_RUN:
            vehicle_control_load_test_state.timer_s += dt_s;
            if (vehicle_control_load_test_state.timer_s >= VEHICLE_CONTROL_LOAD_TEST_CAPTURE_START_S)
            {
                encoder_observation = module_vehicle_encoder_observation_get();
                if (encoder_observation != NULL_PTR)
                {
                    vehicle_control_load_test_state.left_speed_sum_mm_s +=
                        encoder_observation->left_speed_mm_s;
                    vehicle_control_load_test_state.right_speed_sum_mm_s +=
                        encoder_observation->right_speed_mm_s;
                    vehicle_control_load_test_state.left_duty_sum_tick +=
                        (float32)vehicle_control_left_output_duty;
                    vehicle_control_load_test_state.right_duty_sum_tick +=
                        (float32)vehicle_control_right_output_duty;
                    vehicle_control_load_test_state.capture_count++;
                }
            }
            if (vehicle_control_load_test_state.timer_s >= VEHICLE_CONTROL_LOAD_TEST_RUN_S)
            {
                if (vehicle_control_load_test_state.capture_count > 0u)
                {
                    left_speed_average_mm_s =
                        vehicle_control_load_test_state.left_speed_sum_mm_s
                        / (float32)vehicle_control_load_test_state.capture_count;
                    right_speed_average_mm_s =
                        vehicle_control_load_test_state.right_speed_sum_mm_s
                        / (float32)vehicle_control_load_test_state.capture_count;
                    left_duty_average_tick =
                        vehicle_control_load_test_state.left_duty_sum_tick
                        / (float32)vehicle_control_load_test_state.capture_count;
                    right_duty_average_tick =
                        vehicle_control_load_test_state.right_duty_sum_tick
                        / (float32)vehicle_control_load_test_state.capture_count;
                    left_error_limit_mm_s = VEHICLE_CONTROL_LOAD_TEST_RESULT_SPEED_ERROR_MIN_MM_S;
                    right_error_limit_mm_s = VEHICLE_CONTROL_LOAD_TEST_RESULT_SPEED_ERROR_MIN_MM_S;
                    if ((vehicle_control_load_test_state.left_speed_mm_s
                         * VEHICLE_CONTROL_LOAD_TEST_RESULT_SPEED_ERROR_RATIO)
                        > left_error_limit_mm_s)
                    {
                        left_error_limit_mm_s =
                            vehicle_control_load_test_state.left_speed_mm_s
                            * VEHICLE_CONTROL_LOAD_TEST_RESULT_SPEED_ERROR_RATIO;
                    }
                    if ((vehicle_control_load_test_state.right_speed_mm_s
                         * VEHICLE_CONTROL_LOAD_TEST_RESULT_SPEED_ERROR_RATIO)
                        > right_error_limit_mm_s)
                    {
                        right_error_limit_mm_s =
                            vehicle_control_load_test_state.right_speed_mm_s
                            * VEHICLE_CONTROL_LOAD_TEST_RESULT_SPEED_ERROR_RATIO;
                    }
                    result_valid =
                        (((vehicle_control_load_test_state.left_speed_mm_s <= 0.5f)
                          || (vehicle_control_abs_f32(left_speed_average_mm_s
                                                     - vehicle_control_load_test_state.left_speed_mm_s)
                              <= left_error_limit_mm_s))
                         && ((vehicle_control_load_test_state.right_speed_mm_s <= 0.5f)
                             || (vehicle_control_abs_f32(right_speed_average_mm_s
                                                        - vehicle_control_load_test_state.right_speed_mm_s)
                                 <= right_error_limit_mm_s)))
                            ? TRUE
                            : FALSE;
                    tools_printf("{vload}result,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%lu\r\n",
                                 (unsigned int)((result_valid != FALSE) ? 1u : 0u),
                                 (double)vehicle_control_load_test_state.left_speed_mm_s,
                                 (double)vehicle_control_load_test_state.right_speed_mm_s,
                                 (double)left_speed_average_mm_s,
                                 (double)right_speed_average_mm_s,
                                 (double)left_duty_average_tick,
                                 (double)right_duty_average_tick,
                                 (unsigned long)vehicle_control_load_test_state.capture_count);
                }
                else
                {
                    tools_printf("{vload}result,0,%.1f,%.1f,0.0,0.0,0.0,0.0,0\r\n",
                                 (double)vehicle_control_load_test_state.left_speed_mm_s,
                                 (double)vehicle_control_load_test_state.right_speed_mm_s);
                }
                vehicle_control_speed_integral_clear();
                vehicle_control_speed_test_set_target(
                    (vehicle_control_load_test_state.left_speed_mm_s
                     > VEHICLE_CONTROL_LOAD_TEST_FINISH_SPEED_MM_S)
                        ? VEHICLE_CONTROL_LOAD_TEST_FINISH_SPEED_MM_S
                        : vehicle_control_load_test_state.left_speed_mm_s,
                    (vehicle_control_load_test_state.right_speed_mm_s
                     > VEHICLE_CONTROL_LOAD_TEST_FINISH_SPEED_MM_S)
                        ? VEHICLE_CONTROL_LOAD_TEST_FINISH_SPEED_MM_S
                        : vehicle_control_load_test_state.right_speed_mm_s);
                vehicle_control_load_test_state.step = VEHICLE_CONTROL_LOAD_TEST_STEP_FINISH_HOLD;
                vehicle_control_load_test_state.timer_s = 0.0f;
            }
            break;

        case VEHICLE_CONTROL_LOAD_TEST_STEP_FINISH_HOLD:
            vehicle_control_load_test_state.timer_s += dt_s;
            if (vehicle_control_load_test_state.timer_s >= VEHICLE_CONTROL_LOAD_TEST_FINISH_HOLD_S)
            {
                vehicle_control_speed_test_set_target(0.0f, 0.0f);
                vehicle_control_load_test_state.step = VEHICLE_CONTROL_LOAD_TEST_STEP_STOP_HOLD;
                vehicle_control_load_test_state.timer_s = 0.0f;
            }
            break;

        case VEHICLE_CONTROL_LOAD_TEST_STEP_STOP_HOLD:
            vehicle_control_load_test_state.timer_s += dt_s;
            if (vehicle_control_load_test_state.timer_s >= VEHICLE_CONTROL_LOAD_TEST_STOP_HOLD_S)
            {
                vehicle_control_suction_apply_duty(0u);
                vehicle_control_load_test_cancel();
                vehicle_control_speed_test_enable(FALSE);
            }
            break;

        default:
            vehicle_control_load_test_cancel();
            break;
    }
}

static void vehicle_control_angle_test_cancel(void)
{
    vehicle_control_angle_test_state.step = VEHICLE_CONTROL_ANGLE_TEST_STEP_IDLE;
    vehicle_control_angle_test_state.timer_s = 0.0f;
    vehicle_control_angle_test_state.step_deg = 0.0f;
    vehicle_control_angle_test_state.speed_mm_s = 0.0f;
    vehicle_control_angle_test_state.reference_theta_rad = 0.0f;
    vehicle_control_angle_test_state.target_theta_rad = 0.0f;
    vehicle_control_angle_test_state.suction_off_done = FALSE;
    vehicle_control_test_launch_gate_stop();
    vehicle_control_angle_test_print_reset();
}

static void vehicle_control_angle_test_update(float32 dt_s)
{
    if (vehicle_control_angle_test_state.step == VEHICLE_CONTROL_ANGLE_TEST_STEP_IDLE)
    {
        return;
    }

    if (vehicle_control_test_launch_gate_active != FALSE)
    {
        (void)vehicle_control_test_launch_gate_update(dt_s);
        if (vehicle_control_test_launch_drag_active() != FALSE)
        {
            vehicle_control_set_speed(VEHICLE_CONTROL_REPLAY_LAUNCH_DRAG_SPEED_MM_S);
        }
        else
        {
            vehicle_control_set_speed(VEHICLE_CONTROL_REPLAY_LAUNCH_SPEED_MM_S);
        }
        vehicle_control_set_theta(vehicle_control_angle_test_state.reference_theta_rad);
        return;
    }

    vehicle_control_angle_test_state.timer_s += dt_s;
    vehicle_control_set_speed(vehicle_control_angle_test_state.speed_mm_s);

    switch (vehicle_control_angle_test_state.step)
    {
        case VEHICLE_CONTROL_ANGLE_TEST_STEP_WAIT_SUCTION:
            vehicle_control_angle_test_state.step = VEHICLE_CONTROL_ANGLE_TEST_STEP_WAIT_ANGLE;
            vehicle_control_angle_test_state.timer_s = 0.0f;
            break;

        case VEHICLE_CONTROL_ANGLE_TEST_STEP_WAIT_ANGLE:
            vehicle_control_set_theta(vehicle_control_angle_test_state.reference_theta_rad);
            if (vehicle_control_angle_test_state.timer_s >= VEHICLE_CONTROL_ANGLE_TEST_STRAIGHT_TIME_S)
            {
                vehicle_control_angle_test_state.target_theta_rad =
                    algorithm_attitude_wrap_pi(vehicle_control_angle_test_state.reference_theta_rad
                                               + (vehicle_control_angle_test_state.step_deg
                                                  * VEHICLE_CONTROL_DEG_TO_RAD));
                vehicle_control_set_theta(vehicle_control_angle_test_state.reference_theta_rad);
                algorithm_control_pid_reset(&vehicle_control_state.angle_pid);
                vehicle_control_angle_test_state.step = VEHICLE_CONTROL_ANGLE_TEST_STEP_RUN;
                vehicle_control_angle_test_state.timer_s = 0.0f;
            }
            break;

        case VEHICLE_CONTROL_ANGLE_TEST_STEP_RUN:
        {
            float32 ratio = vehicle_control_angle_test_state.timer_s / VEHICLE_CONTROL_ANGLE_TEST_TURN_TIME_S;
            float32 smooth_ratio;
            float32 target_delta_rad;
            float32 command_theta_rad;

            if (ratio < 0.0f)
            {
                ratio = 0.0f;
            }
            else if (ratio > 1.0f)
            {
                ratio = 1.0f;
            }

            smooth_ratio = ratio * ratio * (3.0f - (2.0f * ratio));
            target_delta_rad = vehicle_control_angle_test_state.step_deg * VEHICLE_CONTROL_DEG_TO_RAD;
            command_theta_rad = algorithm_attitude_wrap_pi(vehicle_control_angle_test_state.reference_theta_rad
                                                           + (target_delta_rad * smooth_ratio));
            vehicle_control_set_theta(command_theta_rad);

            if (vehicle_control_angle_test_state.timer_s >= VEHICLE_CONTROL_ANGLE_TEST_TURN_TIME_S)
            {
                if (vehicle_control_auto_suction_enabled != FALSE)
                {
                    vehicle_control_suction_apply_duty(0u);
                }
                vehicle_control_angle_test_state.suction_off_done = TRUE;
                vehicle_control_set_speed(0.0f);
                vehicle_control_enable(FALSE);
            }
            break;
        }

        default:
            vehicle_control_angle_test_cancel();
            break;
    }
}

static void vehicle_control_angle_test_print_capture(void)
{
    vehicle_control_angle_test_print_frame_t frame;
    const module_vehicle_esc_role_status_t* left_esc_status;
    const module_vehicle_esc_role_status_t* right_esc_status;
    uint64 current_tick;
    uint64 elapsed_tick;
    uint64 elapsed_us;
    float32 sample_dt_ms = 0.0f;

    if (vehicle_control_angle_test_state.step == VEHICLE_CONTROL_ANGLE_TEST_STEP_IDLE)
    {
        return;
    }

    vehicle_control_angle_test_print_divider++;
    if (vehicle_control_angle_test_print_divider < VEHICLE_CONTROL_ANGLE_TEST_PRINT_DIVIDER)
    {
        return;
    }

    vehicle_control_angle_test_print_divider = 0u;
    current_tick = sysTick_getTick(SYSTICK1);
    if (vehicle_control_angle_test_capture_last_tick_valid != FALSE)
    {
        elapsed_tick = current_tick - vehicle_control_angle_test_capture_last_tick;
        elapsed_us = sysTick_ticksToMicroseconds(SYSTICK1, elapsed_tick);
        sample_dt_ms = (float32)elapsed_us * 0.001f;
    }
    vehicle_control_angle_test_capture_last_tick = current_tick;
    vehicle_control_angle_test_capture_last_tick_valid = TRUE;
    left_esc_status = module_vehicle_esc_role_status_get(VEHICLE_ESC_ROLE_LEFT_DRIVE);
    right_esc_status = module_vehicle_esc_role_status_get(VEHICLE_ESC_ROLE_RIGHT_DRIVE);

    frame.target_theta_deg =
        algorithm_attitude_wrap_pi(vehicle_control_state.target_theta_rad
                                   - vehicle_control_angle_test_state.reference_theta_rad)
        / VEHICLE_CONTROL_DEG_TO_RAD;
    frame.current_theta_deg =
        algorithm_attitude_wrap_pi(module_vehicle_pose_fusion_heading_get()
                                   - vehicle_control_angle_test_state.reference_theta_rad)
        / VEHICLE_CONTROL_DEG_TO_RAD;
    frame.yaw_speed_correction_mm_s =
        vehicle_control_state.yaw_speed_correction_mm_s;
    frame.left_target_mm_s =
        vehicle_control_state.target_left_speed_mm_s;
    frame.right_target_mm_s =
        vehicle_control_state.target_right_speed_mm_s;
    frame.left_duty = vehicle_control_state.left_duty;
    frame.right_duty = vehicle_control_state.right_duty;
    frame.suction_duty = vehicle_control_state.suction_duty;
    frame.step = (uint32)vehicle_control_angle_test_state.step;
    frame.sample_dt_ms = sample_dt_ms;
    frame.left_esc_state = left_esc_status->state;
    frame.left_esc_fault = left_esc_status->fault;
    frame.right_esc_state = right_esc_status->state;
    frame.right_esc_fault = right_esc_status->fault;
    vehicle_control_angle_test_print_live_frame = frame;
    vehicle_control_angle_test_print_live_pending = TRUE;
}

/**
 * @brief 将串口命令参数字符串解析为浮点数。
 * @param[in] text 命令参数字符串指针。
 * @return 解析得到的浮点数。
 */
static float32 vehicle_control_command_float_get(const uint8* text)
{
    float32 value = 0.0f;
    float32 fraction = 0.1f;
    sint32 sign = 1;
    uint32 index = 0u;
    boolean fraction_part = FALSE;

    if (text == NULL_PTR)
    {
        return 0.0f;
    }

    if (text[index] == (uint8)'-')
    {
        sign = -1;
        index++;
    }
    else if (text[index] == (uint8)'+')
    {
        index++;
    }

    while (text[index] != (uint8)'\0')
    {
        if (text[index] == (uint8)'.')
        {
            fraction_part = TRUE;
            index++;
            continue;
        }

        if ((text[index] < (uint8)'0') || (text[index] > (uint8)'9'))
        {
            break;
        }

        if (fraction_part == FALSE)
        {
            value = (value * 10.0f) + (float32)(text[index] - (uint8)'0');
        }
        else
        {
            value += (float32)(text[index] - (uint8)'0') * fraction;
            fraction *= 0.1f;
        }

        index++;
    }

    return (sign < 0) ? -value : value;
}

/**
 * @brief 判断串口命令参数是否表示零速度。
 * @param[in] text 命令参数字符串指针。
 * @return 近似为零返回 TRUE，否则返回 FALSE。
 */
static boolean vehicle_control_command_is_zero(const uint8* text)
{
    float32 value;

    if (text == NULL_PTR)
    {
        return FALSE;
    }

    value = vehicle_control_command_float_get(text);
    return ((value > -0.5f) && (value < 0.5f)) ? TRUE : FALSE;
}

static boolean vehicle_control_command_word_is(const uint8* text, const char* word)
{
    uint32 index = 0u;
    uint8 text_ch;
    uint8 word_ch;

    if ((text == NULL_PTR) || (word == NULL_PTR))
    {
        return FALSE;
    }

    while ((text[index] != (uint8)'\0') && (word[index] != '\0'))
    {
        text_ch = text[index];
        word_ch = (uint8)word[index];

        if ((text_ch >= (uint8)'A') && (text_ch <= (uint8)'Z'))
        {
            text_ch = (uint8)(text_ch + ((uint8)'a' - (uint8)'A'));
        }

        if ((word_ch >= (uint8)'A') && (word_ch <= (uint8)'Z'))
        {
            word_ch = (uint8)(word_ch + ((uint8)'a' - (uint8)'A'));
        }

        if (text_ch != word_ch)
        {
            return FALSE;
        }

        index++;
    }

    return ((text[index] == (uint8)'\0') && (word[index] == '\0')) ? TRUE : FALSE;
}

/**
 * @brief 按打印分频捕获一次速度环测试目标、反馈和输出占空命令。
 * @param[in] encoder_observation 编码器速度观测指针，速度单位：毫米/秒。
 * @return void
 */
static void vehicle_control_speed_test_print_capture(const module_vehicle_encoder_observation_t* encoder_observation)
{
    vehicle_control_speed_test_print_frame_t frame;
    const module_vehicle_esc_role_status_t* left_esc_status;
    const module_vehicle_esc_role_status_t* right_esc_status;
    uint64 current_tick;
    uint64 elapsed_tick;
    uint64 elapsed_us;
    float32 sample_dt_ms = vehicle_control_speed_test_print_live_frame.sample_dt_ms;
    boolean core_print_due = FALSE;
    boolean encoder_print_due = FALSE;
    boolean diag_print_due = FALSE;

    vehicle_control_speed_test_core_print_divider++;
    if (vehicle_control_speed_test_core_print_divider >= VEHICLE_CONTROL_SPEED_TEST_CORE_PRINT_DIVIDER)
    {
        vehicle_control_speed_test_core_print_divider = 0u;
        core_print_due = TRUE;
    }

    vehicle_control_speed_test_encoder_print_divider++;
    if (vehicle_control_speed_test_encoder_print_divider >= VEHICLE_CONTROL_SPEED_TEST_ENCODER_PRINT_DIVIDER)
    {
        vehicle_control_speed_test_encoder_print_divider = 0u;
        encoder_print_due = TRUE;
    }

    vehicle_control_speed_test_diag_print_divider++;
    if (vehicle_control_speed_test_diag_print_divider >= VEHICLE_CONTROL_SPEED_TEST_DIAG_PRINT_DIVIDER)
    {
        vehicle_control_speed_test_diag_print_divider = 0u;
        diag_print_due = TRUE;
    }

    if ((core_print_due == FALSE) && (encoder_print_due == FALSE) && (diag_print_due == FALSE))
    {
        return;
    }

    if (core_print_due != FALSE)
    {
        current_tick = sysTick_getTick(SYSTICK1);
        if (vehicle_control_speed_test_capture_last_tick_valid != FALSE)
        {
            elapsed_tick = current_tick - vehicle_control_speed_test_capture_last_tick;
            elapsed_us = sysTick_ticksToMicroseconds(SYSTICK1, elapsed_tick);
            sample_dt_ms = (float32)elapsed_us * 0.001f;
        }
        vehicle_control_speed_test_capture_last_tick = current_tick;
        vehicle_control_speed_test_capture_last_tick_valid = TRUE;
    }
    left_esc_status = module_vehicle_esc_role_status_get(VEHICLE_ESC_ROLE_LEFT_DRIVE);
    right_esc_status = module_vehicle_esc_role_status_get(VEHICLE_ESC_ROLE_RIGHT_DRIVE);

    frame.left_target_mm_s = vehicle_control_state.target_left_speed_mm_s;
    frame.right_target_mm_s = vehicle_control_state.target_right_speed_mm_s;
    frame.left_speed_mm_s = encoder_observation->left_speed_mm_s;
    frame.right_speed_mm_s = encoder_observation->right_speed_mm_s;
    frame.left_integral = vehicle_control_state.left_speed_pid.integral;
    frame.right_integral = vehicle_control_state.right_speed_pid.integral;
    frame.left_duty = vehicle_control_left_output_duty;
    frame.right_duty = vehicle_control_right_output_duty;
    frame.sample_dt_ms = sample_dt_ms;
    frame.target_theta_deg = vehicle_control_state.target_theta_rad / VEHICLE_CONTROL_DEG_TO_RAD;
    frame.current_theta_deg = module_vehicle_pose_fusion_heading_get() / VEHICLE_CONTROL_DEG_TO_RAD;
    frame.yaw_speed_correction_mm_s = vehicle_control_state.yaw_speed_correction_mm_s;
    frame.left_esc_state = left_esc_status->state;
    frame.left_esc_fault = left_esc_status->fault;
    frame.right_esc_state = right_esc_status->state;
    frame.right_esc_fault = right_esc_status->fault;
    frame.left_raw_angle = encoder_observation->left_raw_angle;
    frame.right_raw_angle = encoder_observation->right_raw_angle;
    frame.left_raw_delta_count = encoder_observation->left_raw_delta_count;
    frame.right_raw_delta_count = encoder_observation->right_raw_delta_count;
    frame.left_delta_count = encoder_observation->left_delta_count;
    frame.right_delta_count = encoder_observation->right_delta_count;
    frame.left_raw_speed_mm_s = encoder_observation->left_raw_speed_mm_s;
    frame.right_raw_speed_mm_s = encoder_observation->right_raw_speed_mm_s;
    frame.left_sample_dt_ms = encoder_observation->left_sample_dt_s * 1000.0f;
    frame.right_sample_dt_ms = encoder_observation->right_sample_dt_s * 1000.0f;
    frame.left_sample_count = encoder_observation->left_sample_count;
    frame.right_sample_count = encoder_observation->right_sample_count;
    frame.encoder_drop_count = encoder_observation->encoder_drop_count;
    frame.left_invalid_sample_count = encoder_observation->left_invalid_sample_count;
    frame.right_invalid_sample_count = encoder_observation->right_invalid_sample_count;
    frame.left_invalid_streak_count = encoder_observation->left_invalid_streak_count;
    frame.right_invalid_streak_count = encoder_observation->right_invalid_streak_count;
    frame.left_rejected_raw_angle = encoder_observation->left_rejected_raw_angle;
    frame.right_rejected_raw_angle = encoder_observation->right_rejected_raw_angle;
    vehicle_control_speed_test_print_live_frame = frame;
    if (core_print_due != FALSE)
    {
        vehicle_control_speed_test_core_print_live_pending = TRUE;
    }
    if (encoder_print_due != FALSE)
    {
        vehicle_control_speed_test_encoder_print_live_pending = TRUE;
    }
    if (diag_print_due != FALSE)
    {
        vehicle_control_speed_test_diag_print_live_pending = TRUE;
    }
}
