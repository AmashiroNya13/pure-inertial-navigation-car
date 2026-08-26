/**
 * @file device_esc.c
 * @brief 电调 PWM 输出对外调用接口实现。
 */

#include "../../../inc/device/device_esc/device_esc.h"

#define DEVICE_ESC_UART_STATUS_REQUEST_CMD (0x20u)
#define DEVICE_ESC_UART_STATUS_RESPONSE_CMD (0xA0u)
#define DEVICE_ESC_UART_STATUS_RESPONSE_LENGTH (9u)

static uint8 device_esc_uart_checksum(const uint8* packet, uint32 length);
static void device_esc_uart_status_parser_push(uint8 data);

static device_esc_uart_status_t device_esc_uart_status;
static uint8 device_esc_uart_status_packet[DEVICE_ESC_UART_STATUS_RESPONSE_LENGTH];
static uint32 device_esc_uart_status_packet_index;

/**
 * @brief 初始化指定电调 PWM 输出。
 * @param[in] esc_id 电调设备编号。
 * @return void
 */
void device_esc_init(device_esc_id_t esc_id)
{
    device_esc_cfg_t* esc_cfg = device_esc_cfg_table_get();
    device_esc_runtime_t* esc_runtime = device_esc_runtime_table_get();

    driver_gtm_atom_pwm_init(&esc_cfg[esc_id].atom_pwm_cfg, &esc_runtime[esc_id].atom_pwm_runtime);
    driver_gtm_atom_pwm_start(&esc_runtime[esc_id].atom_pwm_runtime);
    driver_gtm_atom_pwm_setDutyCycle(&esc_runtime[esc_id].atom_pwm_runtime, 0u);
}

/**
 * @brief 初始化全部已配置的电调 PWM 输出。
 * @param[in] void 无参数。
 * @return void
 */
void device_esc_init_all(void)
{
    uint32 index;

    for (index = 0u; index < (uint32)DEVICE_ESC_COUNT; index++)
    {
        device_esc_init((device_esc_id_t)index);
    }
}

/**
 * @brief 注册指定电调 PWM 输出事件回调函数。
 * @param[in] esc_id 电调设备编号。
 * @param[in] device_esc_callback 回调函数指针。
 * @return void
 */
void device_esc_register_callback(device_esc_id_t esc_id, void (*device_esc_callback) (void))
{
    device_esc_runtime_t* esc_runtime = device_esc_runtime_table_get();
    esc_runtime[esc_id].device_esc_callback = device_esc_callback;
}

/**
 * @brief 设置电调 PWM 占空比。
 * @param[in] esc_id 电调设备编号。
 * @param[in] duty_cycle PWM 占空比命令，单位跟随 GTM PWM 配置。
 * @return void
 */
void device_esc_set_duty_cycle(device_esc_id_t esc_id, uint32 duty_cycle)
{
    device_esc_cfg_t* esc_cfg = device_esc_cfg_table_get();
    device_esc_runtime_t* esc_runtime = device_esc_runtime_table_get();

    if (duty_cycle < esc_cfg[esc_id].min_duty_cycle)
    {
        duty_cycle = esc_cfg[esc_id].min_duty_cycle;
    }

    if (duty_cycle > esc_cfg[esc_id].max_duty_cycle)
    {
        duty_cycle = esc_cfg[esc_id].max_duty_cycle;
    }

    driver_gtm_atom_pwm_setDutyCycle(&esc_runtime[esc_id].atom_pwm_runtime, duty_cycle);
}

void device_esc_uart_send_all(sint16 left_duty, sint16 right_duty, uint16 suction_duty)
{
    device_esc_uart_runtime_t* uart_runtime = device_esc_uart_runtime_get();
    uint8 packet[9];

    packet[0] = 0xA5u;
    packet[1] = 0x14u;
    packet[2] = (uint8)(((uint16)left_duty >> 8u) & 0xFFu);
    packet[3] = (uint8)((uint16)left_duty & 0xFFu);
    packet[4] = (uint8)(((uint16)right_duty >> 8u) & 0xFFu);
    packet[5] = (uint8)((uint16)right_duty & 0xFFu);
    packet[6] = (uint8)((suction_duty >> 8u) & 0xFFu);
    packet[7] = (uint8)(suction_duty & 0xFFu);
    packet[8] = device_esc_uart_checksum(packet, 8u);

    driver_asclin_asc_write_buffer(&uart_runtime->asclin_asc_runtime, packet, 9u);
}

void device_esc_uart_send_drive(sint16 left_duty, sint16 right_duty)
{
    device_esc_uart_runtime_t* uart_runtime = device_esc_uart_runtime_get();
    uint8 packet[7];

    packet[0] = 0xA5u;
    packet[1] = 0x11u;
    packet[2] = (uint8)(((uint16)left_duty >> 8u) & 0xFFu);
    packet[3] = (uint8)((uint16)left_duty & 0xFFu);
    packet[4] = (uint8)(((uint16)right_duty >> 8u) & 0xFFu);
    packet[5] = (uint8)((uint16)right_duty & 0xFFu);
    packet[6] = device_esc_uart_checksum(packet, 6u);

    driver_asclin_asc_write_buffer(&uart_runtime->asclin_asc_runtime, packet, 7u);
}

void device_esc_uart_send_music(uint8 song_id)
{
    device_esc_uart_runtime_t* uart_runtime = device_esc_uart_runtime_get();
    uint8 packet[4];

    packet[0] = 0xA5u;
    packet[1] = 0x30u;
    packet[2] = song_id;
    packet[3] = device_esc_uart_checksum(packet, 3u);

    driver_asclin_asc_write_buffer(&uart_runtime->asclin_asc_runtime, packet, 4u);
}

void device_esc_uart_stop_music(void)
{
    device_esc_uart_runtime_t* uart_runtime = device_esc_uart_runtime_get();
    uint8 packet[3];

    packet[0] = 0xA5u;
    packet[1] = 0x31u;
    packet[2] = device_esc_uart_checksum(packet, 2u);

    driver_asclin_asc_write_buffer(&uart_runtime->asclin_asc_runtime, packet, 3u);
}

void device_esc_uart_request_status(void)
{
    device_esc_uart_runtime_t* uart_runtime = device_esc_uart_runtime_get();
    uint8 packet[3];

    packet[0] = 0xA5u;
    packet[1] = DEVICE_ESC_UART_STATUS_REQUEST_CMD;
    packet[2] = device_esc_uart_checksum(packet, 2u);

    driver_asclin_asc_write_buffer(&uart_runtime->asclin_asc_runtime, packet, 3u);
}

void device_esc_uart_poll_status(void)
{
    device_esc_uart_runtime_t* uart_runtime = device_esc_uart_runtime_get();
    uint8 data;

    while (IfxAsclin_getRxFifoFillLevel(uart_runtime->asclin_asc_runtime.asclin_asc_modulehn.asclin) > 0u)
    {
        (void)IfxAsclin_read8(uart_runtime->asclin_asc_runtime.asclin_asc_modulehn.asclin, &data, 1u);
        device_esc_uart_status_parser_push(data);
    }
}

const device_esc_uart_status_t* device_esc_uart_status_get(void)
{
    return &device_esc_uart_status;
}

static uint8 device_esc_uart_checksum(const uint8* packet, uint32 length)
{
    uint32 index;
    uint8 checksum = 0u;

    for (index = 0u; index < length; index++)
    {
        checksum = (uint8)(checksum + packet[index]);
    }

    return checksum;
}

static void device_esc_uart_status_parser_push(uint8 data)
{
    if (device_esc_uart_status_packet_index == 0u)
    {
        if (data != 0xA5u)
        {
            return;
        }

        device_esc_uart_status_packet[0] = data;
        device_esc_uart_status_packet_index = 1u;
        return;
    }

    device_esc_uart_status_packet[device_esc_uart_status_packet_index] = data;
    device_esc_uart_status_packet_index++;

    if ((device_esc_uart_status_packet_index == 2u)
        && (device_esc_uart_status_packet[1] != DEVICE_ESC_UART_STATUS_RESPONSE_CMD))
    {
        device_esc_uart_status_packet_index = (data == 0xA5u) ? 1u : 0u;
        if (device_esc_uart_status_packet_index == 1u)
        {
            device_esc_uart_status_packet[0] = data;
        }
        return;
    }

    if (device_esc_uart_status_packet_index < DEVICE_ESC_UART_STATUS_RESPONSE_LENGTH)
    {
        return;
    }

    if (device_esc_uart_checksum(device_esc_uart_status_packet, DEVICE_ESC_UART_STATUS_RESPONSE_LENGTH - 1u)
        == device_esc_uart_status_packet[DEVICE_ESC_UART_STATUS_RESPONSE_LENGTH - 1u])
    {
        device_esc_uart_status.vacuum_state = device_esc_uart_status_packet[2];
        device_esc_uart_status.vacuum_fault = device_esc_uart_status_packet[3];
        device_esc_uart_status.right_state = device_esc_uart_status_packet[4];
        device_esc_uart_status.right_fault = device_esc_uart_status_packet[5];
        device_esc_uart_status.left_state = device_esc_uart_status_packet[6];
        device_esc_uart_status.left_fault = device_esc_uart_status_packet[7];
        device_esc_uart_status.sequence++;
        device_esc_uart_status.valid = TRUE;
    }

    device_esc_uart_status_packet_index = 0u;
}
