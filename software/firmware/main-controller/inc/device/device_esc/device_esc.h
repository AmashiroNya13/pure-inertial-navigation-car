/**
 * @file device_esc.h
 * @brief 电调 PWM 输出对外调用接口。
 */

#ifndef MAD_CIRCUITS_DEVICE_ESC_H
#define MAD_CIRCUITS_DEVICE_ESC_H

#include "../../../config/device/device_esc/device_esc_cfg.h"

/**
 * @brief 初始化指定电调 PWM 输出。
 * @param[in] esc_id 电调设备编号。
 * @return void
 */
void device_esc_init(device_esc_id_t esc_id);

/**
 * @brief 初始化全部已配置的电调 PWM 输出。
 * @param[in] void 无参数。
 * @return void
 */
void device_esc_init_all(void);

/**
 * @brief 注册指定电调 PWM 输出事件回调函数。
 * @param[in] esc_id 电调设备编号。
 * @param[in] device_esc_callback 回调函数指针。
 * @return void
 */
void device_esc_register_callback(device_esc_id_t esc_id, void (*device_esc_callback) (void));

/**
 * @brief 设置电调 PWM 占空比。
 * @param[in] esc_id 电调设备编号。
 * @param[in] duty_cycle PWM 占空比命令，单位跟随 GTM PWM 配置。
 * @return void
 */
void device_esc_set_duty_cycle(device_esc_id_t esc_id, uint32 duty_cycle);

typedef struct
{
    uint8 vacuum_state;
    uint8 vacuum_fault;
    uint8 right_state;
    uint8 right_fault;
    uint8 left_state;
    uint8 left_fault;
    uint32 sequence;
    boolean valid;
} device_esc_uart_status_t;

/**
 * @brief Send left/right drive duty and suction duty to the UART ESC board.
 * @param[in] left_duty Signed left drive duty, unit: protocol tick, range -3000..14500.
 * @param[in] right_duty Signed right drive duty, unit: protocol tick, range -3000..14500.
 * @param[in] suction_duty Suction duty, unit: protocol tick, range 0..12000.
 * @return void
 */
void device_esc_uart_send_all(sint16 left_duty, sint16 right_duty, uint16 suction_duty);

/**
 * @brief Send only left/right drive duty to the UART ESC board.
 * @param[in] left_duty Signed left drive duty, unit: protocol tick, range -3000..14500.
 * @param[in] right_duty Signed right drive duty, unit: protocol tick, range -3000..14500.
 * @return void
 */
void device_esc_uart_send_drive(sint16 left_duty, sint16 right_duty);

/**
 * @brief Ask the UART ESC board to play a motor music score.
 * @param[in] song_id Song index in the ESC firmware music table.
 * @return void
 */
void device_esc_uart_send_music(uint8 song_id);

/**
 * @brief Ask the UART ESC board to stop motor music playback.
 * @param[in] void No parameter.
 * @return void
 */
void device_esc_uart_stop_music(void);

void device_esc_uart_request_status(void);

void device_esc_uart_poll_status(void);

const device_esc_uart_status_t* device_esc_uart_status_get(void);

#endif
