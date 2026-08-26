/**
 * @file driver_gtm_atom_pwm.c
 * @brief GTM ATOM PWM 底层驱动实现。
 */

#include "../../../inc/driver/driver_gtm/driver_gtm_atom_pwm.h"

/**
 * @brief 初始化 GTM ATOM PWM 驱动实例。
 * @param[in] gtm_atom_pwm_cfg GTM ATOM PWM 配置指针。
 * @param[in,out] gtm_atom_pwm_runtime GTM ATOM PWM 运行句柄指针。
 * @return void
 */
void driver_gtm_atom_pwm_init(driver_gtm_atom_pwm_cfg_t* gtm_atom_pwm_cfg, driver_gtm_atom_pwm_runtime_t* gtm_atom_pwm_runtime)
{
    IfxGtm_Atom_Pwm_init(&gtm_atom_pwm_runtime->atom_pwm_modulehn, &gtm_atom_pwm_cfg->atom_pwm_modulecfg);
}

/**
 * @brief 启动 GTM ATOM PWM 输出。
 * @param[in,out] gtm_atom_pwm_runtime GTM ATOM PWM 运行句柄指针。
 * @return void
 */
void driver_gtm_atom_pwm_start(driver_gtm_atom_pwm_runtime_t* gtm_atom_pwm_runtime)
{
    IfxGtm_Atom_Pwm_start(&gtm_atom_pwm_runtime->atom_pwm_modulehn, TRUE);
}

void driver_gtm_atom_pwm_stop(driver_gtm_atom_pwm_runtime_t* gtm_atom_pwm_runtime, boolean immediate)
{
    IfxGtm_Atom_Pwm_stop(&gtm_atom_pwm_runtime->atom_pwm_modulehn, immediate);
}

/**
 * @brief 设置 GTM ATOM PWM 比较值并触发影子寄存器更新。
 * @param[in,out] gtm_atom_pwm_runtime GTM ATOM PWM 运行句柄指针。
 * @param[in] dutyCycle PWM 占空命令，单位：GTM ATOM 比较计数。
 * @return void
 */
void driver_gtm_atom_pwm_setDutyCycle(driver_gtm_atom_pwm_runtime_t* gtm_atom_pwm_runtime, uint32 dutyCycle)
{
    IfxGtm_Atom_Ch_setCompareOneShadow(gtm_atom_pwm_runtime->atom_pwm_modulehn.atom, gtm_atom_pwm_runtime->atom_pwm_modulehn.atomChannel, dutyCycle);
    IfxGtm_Atom_Agc_trigger(gtm_atom_pwm_runtime->atom_pwm_modulehn.agc);
}
