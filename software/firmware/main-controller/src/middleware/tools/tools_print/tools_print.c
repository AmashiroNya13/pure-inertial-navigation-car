/**
 * @file tools_print.c
 * @brief 调试串口打印工具实现。
 */

#include "../../../../inc/middleware/tools/tools_print/tools_print.h"

#include <stdarg.h>
#include <stdio.h>

#include "../../../../Libraries/iLLD/TC26B/Tricore/Cpu/Std/IfxCpu.h"

BEGIN_DATA_SECTION(.bss_lmu)
static IfxCpu_mutexLock tools_print_lock;
static char tools_print_buffer[TOOLS_PRINTF_BUFFER_SIZE];
END_DATA_SECTION

IFX_INLINE boolean tools_print_lock_acquire(void);
IFX_INLINE void tools_print_lock_release(void);

/**
 * @brief 获取打印工具绑定的调试串口编号。
 * @param[in] void 无参数。
 * @return 调试串口设备编号。
 */
IFX_INLINE device_debug_id_t tools_print_debug_id_get(void);

/**
 * @brief 获取打印工具绑定的调试串口编号。
 * @param[in] void 无参数。
 * @return 调试串口设备编号。
 */
IFX_INLINE device_debug_id_t tools_print_debug_id_get(void)
{
    tools_print_cfg_t* tools_print_cfg = tools_print_cfg_table_get();

    return tools_print_cfg[TOOLS_PRINT_1].debug_id;
}

IFX_INLINE boolean tools_print_lock_acquire(void)
{
    return IfxCpu_acquireMutex(&tools_print_lock);
}

IFX_INLINE void tools_print_lock_release(void)
{
    IfxCpu_releaseMutex(&tools_print_lock);
}

/**
 * @brief 通过调试串口输出单个字符。
 * @param[in] ch 待输出字符。
 * @return void
 */
void tools_print_putc(uint8 ch)
{
    if (tools_print_lock_acquire() == FALSE)
    {
        return;
    }

    device_debug_write_byte(tools_print_debug_id_get(), ch);
    tools_print_lock_release();
}

/**
 * @brief 通过调试串口输出字符串。
 * @param[in] string 待输出字符串指针。
 * @return void
 */
void tools_print(const uint8* string)
{
    if (string == NULL_PTR)
    {
        return;
    }

    if (tools_print_lock_acquire() == FALSE)
    {
        return;
    }

    device_debug_write_string(tools_print_debug_id_get(), string);
    tools_print_lock_release();
}

/**
 * @brief 通过调试串口输出字符串并追加换行。
 * @param[in] string 待输出字符串指针。
 * @return void
 */
void tools_println(const uint8* string)
{
    if ((string == NULL_PTR) || (tools_print_lock_acquire() == FALSE))
    {
        return;
    }

    device_debug_write_string(tools_print_debug_id_get(), string);
    device_debug_write_byte(tools_print_debug_id_get(), '\r');
    device_debug_write_byte(tools_print_debug_id_get(), '\n');
    tools_print_lock_release();
}

/**
 * @brief 按 printf 格式通过调试串口输出字符串。
 * @param[in] format printf 格式字符串指针。
 * @return void
 */
void tools_printf(const char* format, ...)
{
    va_list args;
    sint32 length;
    uint32 write_length;

    if (format == NULL_PTR)
    {
        return;
    }

    if (tools_print_lock_acquire() == FALSE)
    {
        return;
    }

    va_start(args, format);
    length = vsnprintf(tools_print_buffer, sizeof(tools_print_buffer), format, args);
    va_end(args);

    if (length <= 0)
    {
        tools_print_lock_release();
        return;
    }

    if (length >= (sint32)TOOLS_PRINTF_BUFFER_SIZE)
    {
        tools_print_buffer[TOOLS_PRINTF_BUFFER_SIZE - 3u] = '\r';
        tools_print_buffer[TOOLS_PRINTF_BUFFER_SIZE - 2u] = '\n';
        tools_print_buffer[TOOLS_PRINTF_BUFFER_SIZE - 1u] = '\0';
        write_length = (uint32)(TOOLS_PRINTF_BUFFER_SIZE - 1u);
    }
    else
    {
        write_length = (uint32)length;
    }
    device_debug_write_buffer(tools_print_debug_id_get(),
                              (const uint8*)tools_print_buffer,
                              write_length);
    tools_print_lock_release();
}

void tools_print_write_buffer(const uint8* buffer, uint32 length)
{
    if ((buffer == NULL_PTR) || (length == 0u))
    {
        return;
    }

    if (tools_print_lock_acquire() == FALSE)
    {
        return;
    }

    device_debug_write_buffer(tools_print_debug_id_get(), buffer, length);
    tools_print_lock_release();
}

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
                               const char* function)
{
    tools_printf("ASSERT FAILED expr=%s file=%s line=%lu func=%s\r\n",
                 expression,
                 file,
                 (unsigned long)line,
                 function);

    while (1)
    {
    }
}

/**
 * @brief 输出严重错误信息并停机。
 * @param[in] message 严重错误信息字符串指针。
 * @return void
 */
void tools_panic(const uint8* message)
{
    tools_print((const uint8*)"PANIC: ");
    tools_println(message);

    while (1)
    {
    }
}
