/**
 * @file tools_print.h
 * @brief 调试串口打印工具接口。
 */

#ifndef MAD_CIRCUITS_TOOLS_PRINT_H
#define MAD_CIRCUITS_TOOLS_PRINT_H

#include "../../../../config/middleware/tools/tools_print/tools_print_cfg.h"
#include "../../../../inc/device/device_debug/device_debug.h"

/**
 * @brief 通过调试串口输出单个字符。
 * @param[in] ch 待输出字符。
 * @return void
 */
void tools_print_putc(uint8 ch);

/**
 * @brief 通过调试串口输出字符串。
 * @param[in] string 待输出字符串指针。
 * @return void
 */
void tools_print(const uint8* string);

/**
 * @brief 通过调试串口输出字符串并追加换行。
 * @param[in] string 待输出字符串指针。
 * @return void
 */
void tools_println(const uint8* string);

/**
 * @brief 按 printf 格式通过调试串口输出字符串。
 * @param[in] format printf 格式字符串指针。
 * @return void
 */
void tools_printf(const char* format, ...);

void tools_print_write_buffer(const uint8* buffer, uint32 length);

/**
 * @brief 输出断言失败信息并停机。
 * @param[in] expression 断言表达式字符串指针。
 * @param[in] file 断言所在文件字符串指针。
 * @param[in] line 断言所在行号，单位：行。
 * @param[in] function 断言所在函数字符串指针。
 * @return void
 */
void tools_print_assert_failed(const char* expression,
                               const char* file,
                               uint32 line,
                               const char* function);

/**
 * @brief 输出严重错误信息并停机。
 * @param[in] message 严重错误信息字符串指针。
 * @return void
 */
void tools_panic(const uint8* message);

#define TOOLS_PRINT_ASSERT(expr) \
    (((expr) != FALSE) ? (void)0 : tools_print_assert_failed(#expr, __FILE__, __LINE__, __func__))

#endif
