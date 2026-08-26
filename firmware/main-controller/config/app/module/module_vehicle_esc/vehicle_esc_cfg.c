/**
 * @file vehicle_esc_cfg.c
 * @brief 车体三路无刷电调 PWM 角色默认配置。
 */

#include "./vehicle_esc_cfg.h"

static const vehicle_esc_cfg_t vehicle_esc_cfg =
{
    .role_cfg =
    {
        {
            .device_esc_id = DEVICE_ESC_1,
            .limit =
            {
                .min_duty = -3000,
                .max_duty = 14500,
                .stop_duty = 0,
            },
        },
        {
            .device_esc_id = DEVICE_ESC_2,
            .limit =
            {
                .min_duty = -3000,
                .max_duty = 14500,
                .stop_duty = 0,
            },
        },
        {
            .device_esc_id = DEVICE_ESC_3,
            .limit =
            {
                .min_duty = 0,
                .max_duty = 15000,
                .stop_duty = 0,
            },
        },
    },
};

/**
 * @brief 获取只读的车体三路无刷电调 PWM 角色配置。
 * @param[in] void 无参数。
 * @return 车体三路无刷电调 PWM 角色配置指针。
 */
const vehicle_esc_cfg_t* vehicle_esc_cfg_get(void)
{
    return &vehicle_esc_cfg;
}
