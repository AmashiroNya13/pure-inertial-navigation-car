/**
 * @file driver_gtm_atom_pwm.h
 * @brief GTM ATOM PWM 底层驱动接口。
 */

#ifndef MAD_CIRCUITS_DRIVER_GTM_ATOM_PWM_H
#define MAD_CIRCUITS_DRIVER_GTM_ATOM_PWM_H

#include "IfxGtm_Atom_Pwm.h"

typedef struct
{
    Ifx_GTM* gtm_module;                         /**< GTM 模块寄存器基地址指针。 */
    IfxGtm_Atom_Pwm_Config atom_pwm_modulecfg;   /**< GTM ATOM PWM 初始化配置。 */
} driver_gtm_atom_pwm_cfg_t;

typedef struct
{
    IfxGtm_Atom_Pwm_Driver atom_pwm_modulehn;    /**< GTM ATOM PWM iLLD 运行句柄。 */
} driver_gtm_atom_pwm_runtime_t;

/**
 * @brief 初始化 GTM ATOM PWM 驱动实例。
 * @param[in] gtm_atom_pwm_cfg GTM ATOM PWM 配置指针。
 * @param[in,out] gtm_atom_pwm_runtime GTM ATOM PWM 运行句柄指针。
 * @return void
 */
void driver_gtm_atom_pwm_init(driver_gtm_atom_pwm_cfg_t* gtm_atom_pwm_cfg, driver_gtm_atom_pwm_runtime_t* gtm_atom_pwm_runtime);

/**
 * @brief 启动 GTM ATOM PWM 输出。
 * @param[in,out] gtm_atom_pwm_runtime GTM ATOM PWM 运行句柄指针。
 * @return void
 */
void driver_gtm_atom_pwm_start(driver_gtm_atom_pwm_runtime_t* gtm_atom_pwm_runtime);

void driver_gtm_atom_pwm_stop(driver_gtm_atom_pwm_runtime_t* gtm_atom_pwm_runtime, boolean immediate);

/**
 * @brief 设置 GTM ATOM PWM 比较值并触发影子寄存器更新。
 * @param[in,out] gtm_atom_pwm_runtime GTM ATOM PWM 运行句柄指针。
 * @param[in] dutyCycle PWM 占空命令，单位：GTM ATOM 比较计数。
 * @return void
 */
void driver_gtm_atom_pwm_setDutyCycle(driver_gtm_atom_pwm_runtime_t* gtm_atom_pwm_runtime, uint32 dutyCycle);

#endif
