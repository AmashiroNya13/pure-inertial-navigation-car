/**
 * @file startup.h
 * @brief 系统启动初始化与运行入口。
 */

#ifndef MAD_CIRCUITS_STARTUP_H
#define MAD_CIRCUITS_STARTUP_H

/**
 * @brief 初始化系统所需底层设备、中间件、服务与应用模块。
 * @param[in] void 无参数。
 * @return void
 */
void startup_init_all(void);

/**
 * @brief 执行一次系统运行循环。
 * @param[in] void 无参数。
 * @return void
 */
void startup_run(void);

#endif
