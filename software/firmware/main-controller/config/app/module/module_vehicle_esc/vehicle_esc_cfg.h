/**
 * @file vehicle_esc_cfg.h
 * @brief 车体三路无刷电调 PWM 角色配置类型。
 */

#ifndef MAD_CIRCUITS_VEHICLE_ESC_CFG_H
#define MAD_CIRCUITS_VEHICLE_ESC_CFG_H

#include "Ifx_Types.h"
#include "../../../../config/device/device_esc/device_esc_cfg.h"

typedef enum
{
    VEHICLE_ESC_ROLE_LEFT_DRIVE = 0,  /**< 左侧行进无刷电调，退出条件由上层业务决定。 */
    VEHICLE_ESC_ROLE_RIGHT_DRIVE = 1, /**< 右侧行进无刷电调，退出条件由上层业务决定。 */
    VEHICLE_ESC_ROLE_SUCTION = 2,     /**< 负压无刷电调，退出条件由上层业务决定。 */
    VEHICLE_ESC_ROLE_COUNT = 3,       /**< 车体无刷电调角色数量，单位：个。 */
} vehicle_esc_role_t;

typedef struct
{
    sint32 min_duty;  /**< 允许输出的最小 PWM 占空命令，单位跟随 GTM PWM 配置。 */
    sint32 max_duty;  /**< 允许输出的最大 PWM 占空命令，单位跟随 GTM PWM 配置。 */
    sint32 stop_duty; /**< 停止输出使用的 PWM 占空命令，单位跟随 GTM PWM 配置。 */
} vehicle_esc_limit_t;

typedef struct
{
    device_esc_id_t device_esc_id; /**< 角色绑定的底层电调设备编号。 */
    vehicle_esc_limit_t limit;     /**< 角色对应的 PWM 占空命令限制。 */
} vehicle_esc_role_cfg_t;

typedef struct
{
    vehicle_esc_role_cfg_t role_cfg[VEHICLE_ESC_ROLE_COUNT]; /**< 三路无刷电调角色配置表。 */
} vehicle_esc_cfg_t;

/**
 * @brief 获取只读的车体三路无刷电调 PWM 角色配置。
 * @param[in] void 无参数。
 * @return 车体三路无刷电调 PWM 角色配置指针。
 */
const vehicle_esc_cfg_t* vehicle_esc_cfg_get(void);

#endif
