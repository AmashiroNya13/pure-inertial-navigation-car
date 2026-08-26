/*********************************************************************************************************************
* AI8051U Opensourec Library 即（AI8051U 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是STC 开源库的一部分
*
* AI8051U 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK FOR C251
* 适用平台          AI8051U
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2024-08-01        大W            first version
********************************************************************************************************************/

#include "zf_common_debug.h"
#include "zf_common_function.h"
#include <stdarg.h>  // 包含标准可变参数宏的头文件
#include <stdio.h>   // 包含标准输入输出的头文件

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取整型数的最大公约数 九章算术之更相减损术
// 参数说明     num1            数字1
// 参数说明     num2            数字2
// 返回参数     uint32          最大公约数
// 使用示例     return func_get_greatest_common_divisor(144, 36);               // 获取 144 与 36 的最大公约数
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint32 func_get_greatest_common_divisor (uint32 num1, uint32 num2)
{
    while(num1 != num2)
    {
        if(num1 > num2)
        {
            num1 = num1 - num2;
        }
        if(num1 < num2)
        {
            num2 = num2 - num1;
        }
    }
    return num1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件延时
// 参数说明     t               延时时间
// 返回参数     void
// 使用示例     func_soft_delay(100);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void func_soft_delay (volatile long t)
{
    while(t --);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     字符串转整形数字 数据范围是 [-32768,32767]
// 参数说明     *str            传入字符串 可带符号
// 返回参数     int32           转换后的数据          
// 使用示例     int32 dat = func_str_to_int("-100");
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
int32 func_str_to_int (char *str)
{
    uint8 sign = 0;                                                             // 标记符号 0-正数 1-负数
    int32 temp = 0;                                                             // 临时计算变量
	zf_assert(str != NULL);
	do
    {
        if(NULL == str)
        {
            break;
        }

        if('-' == *str)                                                         // 如果第一个字符是负号
        {
            sign = 1;                                                           // 标记负数
            str ++;
        }
        else if('+' == *str)                                                    // 如果第一个字符是正号
        {
            str ++;
        }

        while(('0' <= *str) && ('9' >= *str))                                   // 确定这是个数字
        {
            temp = temp * 10 + ((uint8)(*str) - 0x30);                          // 计算数值
            str ++;
        }

        if(sign)
        {
            temp = -temp;
        }
    }while(0);
    return temp;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     整形数字转字符串 数据范围是 [-32768,32767]
// 参数说明     *str            字符串指针
// 参数说明     number          传入的数据
// 返回参数     void
// 使用示例     func_int_to_str(data_buffer, -300);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void func_int_to_str (char *str, int32 number)
{
    uint8 data_temp[16];                                                        // 缓冲区
    uint8 temp_bit = 0;                                                              // 数字位数
    int32 number_temp = 0;
    zf_assert(str != NULL);
    do
    {
        if(NULL == str)
        {
            break;
        }

        if(0 > number)                                                          // 负数
        {
            *str ++ = '-';
            number = -number;
        }
        else if(0 == number)                                                    // 或者这是个 0
        {
            *str = '0';
            break;
        }

        while(0 != number)                                                      // 循环直到数值归零
        {
            number_temp = number % 10;
            data_temp[temp_bit ++] = func_abs(number_temp);                          // 倒序将数值提取出来
            number /= 10;                                                       // 削减被提取的个位数
        }
        while(0 != temp_bit)                                                         // 提取的数字个数递减处理
        {
            *str ++ = (data_temp[temp_bit - 1] + 0x30);                              // 将数字从倒序数组中倒序取出 变成正序放入字符串
            temp_bit --;
        }
    }while(0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     字符串转整形数字 数据范围是 [0,65535]
// 参数说明     *str            传入字符串 无符号
// 返回参数     uint32          转换后的数据          
// 使用示例     uint32 dat = func_str_to_uint("100");
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint32 func_str_to_uint (char *str)
{
	uint32 temp = 0;                                                            // 临时计算变量
    zf_assert(str != NULL);

    do
    {
        if(NULL == str)
        {
            break;
        }

        while(('0' <= *str) && ('9' >= *str))                                  // 确定这是个数字
        {
            temp = temp * 10 + ((uint8)(*str) - 0x30);                         // 计算数值
            str ++;
        }
    }while(0);

    return temp;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     整形数字转字符串 数据范围是 [0,65535]
// 参数说明     *str            字符串指针
// 参数说明     number          传入的数据
// 返回参数     void
// 使用示例     func_uint_to_str(data_buffer, 300);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void func_uint_to_str (char *str, uint32 number)
{
    int8 data_temp[16];                                                         // 缓冲区
    uint8 temp_bit = 0;                                                              // 数字位数
    
	zf_assert(str != NULL);
    
	do
    {
        if(NULL == str)
        {
            break;
        }

        if(0 == number)                                                         // 这是个 0
        {
            *str = '0';
            break;
        }

        while(0 != number)                                                      // 循环直到数值归零
        {
            data_temp[temp_bit ++] = (number % 10);                                  // 倒序将数值提取出来
            number /= 10;                                                       // 削减被提取的个位数
        }
        while(0 != temp_bit)                                                         // 提取的数字个数递减处理
        {
            *str ++ = (data_temp[temp_bit - 1] + 0x30);                              // 将数字从倒序数组中倒序取出 变成正序放入字符串
            temp_bit --;
        }
    }while(0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     字符串转浮点数 有效累计精度为小数点后六位
// 参数说明     *str            传入字符串 可带符号
// 返回参数     float           转换后的数据          
// 使用示例     float dat = func_str_to_float("-100.2");
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
float func_str_to_float (char *str)
{
    uint8 sign = 0;                                                             // 标记符号 0-正数 1-负数
    float temp = 0.0;                                                           // 临时计算变量 整数部分
    float temp_point = 0.0;                                                     // 临时计算变量 小数部分
    float point_bit = 1;                                                        // 小数累计除数
    
	zf_assert(str != NULL);
    
	do
    {
        if(NULL == str)
        {
            break;
        }

        if('-' == *str)                                                         // 负数
        {
            sign = 1;                                                           // 标记负数
            str ++;
        }
        else if('+' == *str)                                                    // 如果第一个字符是正号
        {
            str ++;
        }

        // 提取整数部分
        while(('0' <= *str) && ('9' >= *str))                                   // 确定这是个数字
        {
            temp = temp * 10 + ((uint8)(*str) - 0x30);                          // 将数值提取出来
            str ++;
        }
        if('.' == *str)
        {
            str ++;
            while(('0' <= *str) && ('9' >= *str) && point_bit < 1000000.0)      // 确认这是个数字 并且精度控制还没到六位
            {
                temp_point = temp_point * 10 + ((uint8)(*str) - 0x30);          // 提取小数部分数值
                point_bit *= 10;                                                // 计算这部分小数的除数
                str ++;
            }
            temp_point /= point_bit;                                            // 计算小数
        }
        temp += temp_point;                                                     // 将数值拼合

        if(sign)
        {
            temp = -temp;
        }
    }while(0);
    return temp;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     浮点数字转字符串
// 参数说明     *str            字符串指针
// 参数说明     number          传入的数据
// 参数说明     point_bit       小数点精度
// 返回参数     void
// 使用示例     func_float_to_str(data_buffer, 3.1415, 2);                      // 结果输出 data_buffer = "3.14"
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void func_float_to_str (char *str, float number, uint8 point_bit)
{
    int data_int = 0;                                                           // 整数部分
    int data_float = 0.0;                                                       // 小数部分
    int data_temp[8];                                                           // 整数字符缓冲
    int data_temp_point[6];                                                     // 小数字符缓冲
    uint8 temp_bit = point_bit;                                                      // 转换精度位数
    
	zf_assert(str != NULL);
    
	do
    {
        if(NULL == str)
        {
            break;
        }

        // 提取整数部分
        data_int = (int)number;                                                 // 直接强制转换为 int
        if(0 > number)                                                          // 判断源数据是正数还是负数
        {
            *str ++ = '-';
        }
        else if(0.0 == number)                                                  // 如果是个 0
        {
            *str ++ = '0';
            *str ++ = '.';
            *str = '0';
            break;
        }

        // 提取小数部分
        number = number - data_int;                                             // 减去整数部分即可
        while(temp_bit --)
        {
            number = number * 10;                                               // 将需要的小数位数提取到整数部分
        }
        data_float = (int)number;                                               // 获取这部分数值

        // 整数部分转为字符串
        temp_bit = 0;
        do
        {
            data_temp[temp_bit ++] = data_int % 10;                                  // 将整数部分倒序写入字符缓冲区
            data_int /= 10;
        }while(0 != data_int);
        while(0 != temp_bit)
        {
            *str ++ = (func_abs(data_temp[temp_bit - 1]) + 0x30);                    // 再倒序将倒序的数值写入字符串 得到正序数值
            temp_bit --;
        }

        // 小数部分转为字符串
        if(point_bit != 0)
        {
            temp_bit = 0;
            *str ++ = '.';
            if(0 == data_float)
            {
                *str = '0';
            }
            else
            {
                while(0 != point_bit)                                           // 判断有效位数
                {
                    data_temp_point[temp_bit ++] = data_float % 10;                  // 倒序写入字符缓冲区
                    data_float /= 10;
                    point_bit --;                                                
                }
                while(0 != temp_bit)
                {
                    *str ++ = (func_abs(data_temp_point[temp_bit - 1]) + 0x30);      // 再倒序将倒序的数值写入字符串 得到正序数值
                    temp_bit --;
                }
            }
        }
    }while(0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     字符串转浮点数 有效累计精度为小数点后九位
// 参数说明     str             传入字符串 可带符号
// 返回参数     double          转换后的数据          
// 使用示例     double dat = func_str_to_double("-100.2");
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
double func_str_to_double (char *str)
{
    uint8 sign = 0;                                                             // 标记符号 0-正数 1-负数
    double temp = 0.0;                                                          // 临时计算变量 整数部分
    double temp_point = 0.0;                                                    // 临时计算变量 小数部分
    double point_bit = 1;                                                       // 小数累计除数
    
	zf_assert(str != NULL);
    
	do
    {
        if(NULL == str)
        {
            break;
        }

        if('-' == *str)                                                         // 负数
        {
            sign = 1;                                                           // 标记负数
            str ++;
        }
        else if('+' == *str)                                                    // 如果第一个字符是正号
        {
            str ++;
        }

        // 提取整数部分
        while(('0' <= *str) && ('9' >= *str))                                   // 确定这是个数字
        {
            temp = temp * 10 + ((uint8)(*str) - 0x30);                          // 将数值提取出来
            str ++;
        }
        if('.' == *str)
        {
            str ++;
            while(('0' <= *str) && ('9' >= *str) && point_bit < 1000000000.0)   // 确认这是个数字 并且精度控制还没到九位
            {
                temp_point = temp_point * 10 + ((uint8)(*str) - 0x30);          // 提取小数部分数值
                point_bit *= 10;                                                // 计算这部分小数的除数
                str ++;
            }
            temp_point /= point_bit;                                            // 计算小数
        }
        temp += temp_point;                                                     // 将数值拼合

        if(sign)
        {
            temp = -temp;
        }
    }while(0);
    return temp;

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     浮点数字转字符串
// 参数说明     *str            字符串指针
// 参数说明     number          传入的数据
// 参数说明     point_bit       小数点精度
// 返回参数     void
// 使用示例     func_double_to_str(data_buffer, 3.1415, 2);                     // 结果输出 data_buffer = "3.14"
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void func_double_to_str (char *str, double number, uint8 point_bit)
{
    int data_int = 0;                                                           // 整数部分
    int data_float = 0.0;                                                       // 小数部分
    int data_temp[12];                                                          // 整数字符缓冲
    int data_temp_point[9];                                                     // 小数字符缓冲
    uint8 temp_bit = point_bit;                                                      // 转换精度位数
    
	zf_assert(str != NULL);
    
	do
    {
        if(NULL == str)
        {
            break;
        }

        // 提取整数部分
        data_int = (int)number;                                                 // 直接强制转换为 int
        if(0 > number)                                                          // 判断源数据是正数还是负数
        {
            *str ++ = '-';
        }
        else if(0.0 == number)                                                  // 如果是个 0
        {
            *str ++ = '0';
            *str ++ = '.';
            *str = '0';
            break;
        }

        // 提取小数部分
        number = number - data_int;                                             // 减去整数部分即可
        while(temp_bit --)
        {
            number = number * 10;                                               // 将需要的小数位数提取到整数部分
        }
        data_float = (int)number;                                               // 获取这部分数值

        // 整数部分转为字符串
        temp_bit = 0;
        do
        {
            data_temp[temp_bit ++] = data_int % 10;                                  // 将整数部分倒序写入字符缓冲区
            data_int /= 10;
        }while(0 != data_int);
        while(0 != temp_bit)
        {
            *str ++ = (func_abs(data_temp[temp_bit - 1]) + 0x30);                    // 再倒序将倒序的数值写入字符串 得到正序数值
            temp_bit --;
        }

        // 小数部分转为字符串
        if(point_bit != 0)
        {
            temp_bit = 0;
            *str ++ = '.';
            if(0 == data_float)
                *str = '0';
            else
            {
                while(0 != point_bit)                                           // 判断有效位数
                {
                    data_temp_point[temp_bit ++] = data_float % 10;                  // 倒序写入字符缓冲区
                    data_float /= 10;
                    point_bit --;                                                
                }
                while(0 != temp_bit)
                {
                    *str ++ = (func_abs(data_temp_point[temp_bit - 1]) + 0x30);      // 再倒序将倒序的数值写入字符串 得到正序数值
                    temp_bit --;
                }
            }
        }
    }while(0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     字符串转 Hex
// 参数说明     str             传入字符串 无符号
// 返回参数     uint32          转换后的数据
// 使用示例     uint32 dat = func_str_to_hex("0x11");
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint32 func_str_to_hex (char *str)
{
    uint32 str_len = strlen(str);                                               // 字符串长
    uint32 result_data = 0;                                                     // 结果缓存
    uint8 temp = 0;                                                             // 计算变量
    uint8 flag = 0;                                                             // 标志位

	zf_assert(str != NULL);
	
    do
    {
        if(NULL == str)
        {
            break;
        }

        if(flag)
        {
            if(('a' <= *str) && ('f' >= *str))
            {
                temp = (*str - 87);
            }
            else if(('A' <= *str) && ('F' >= *str))
            {
                temp = (*str - 55);
            }
            else if(('0' <= *str) && ('9' >= *str))
            {
                temp = (*str - 48);
            }
            else
            {
                break;
            }
            result_data = ((result_data << 4) | (temp & 0x0F));
        }
        else
        {
//            if(strncmp("0x", str, 2))
            if((*str == '0') && (*(str + 1) == 'x'))
            {
                str ++;
                flag = 1;
            }
        }
        str ++;
    }while(str_len --);

    return result_data;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Hex 转字符串
// 参数说明     *str            字符串指针
// 参数说明     number          传入的数据
// 返回参数     void
// 使用示例     func_hex_to_str(data_buffer, 0x11);                             // 结果输出 data_buffer = "0x11"
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void func_hex_to_str (char *str, uint32 number)
{
    const char hex_index[16] = {
        '0', '1', '2', '3',
        '4', '5', '6', '7',
        '8', '9', 'A', 'B',
        'C', 'D', 'E', 'F'};
    int8 data_temp[12];                                                         // 缓冲区
    uint8 temp_bit = 0;                                                              // 数字位数

    zf_assert(str != NULL);
		
    *str++ = '0';
    *str++ = 'x';
    do
    {
        if(NULL == str)
        {
            break;
        }

        if(0 == number)                                                         // 这是个 0
        {
            *str = '0';
            break;
        }

        while(0 != number)                                                      // 循环直到数值归零
        {
            data_temp[temp_bit ++] = (number & 0xF);                                 // 倒序将数值提取出来
            number >>= 4;                                                       // 削减被提取的个位数
        }
        while(0 != temp_bit)                                                         // 提取的数字个数递减处理
        {
            *str ++ = hex_index[data_temp[temp_bit - 1]];                            // 将数字从倒序数组中倒序取出 变成正序放入字符串
            temp_bit --;
        }
    }while(0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     数字转换为 ASCII 值
// 参数说明     dat             传入的数据
// 参数说明     *p              数据缓冲
// 参数说明     neg_type        数据类型
// 参数说明     radix           进制
// 返回参数     uint8           数据
// 使用示例     number_conversion_ascii((uint32)ival, vstr, 1, 10);
// 备注信息     本函数在文件内部调用 用户不用关注 也不可修改
//-------------------------------------------------------------------------------------------------------------------
static uint8 number_conversion_ascii (uint32 dat, int8 *p, uint8 neg_type, uint8 radix)
{
    int32   neg_dat;
    uint32  pos_dat;
    uint8   temp_data = 0;
    uint8   valid_num = 0;

    if(neg_type)
    {
        neg_dat = (int32)dat;
        if(0 > neg_dat)
        {
            neg_dat = -neg_dat;
        }
        while(1)
        {
            *p = neg_dat%radix + '0';
            neg_dat = neg_dat/radix;
            valid_num ++;

            if(!neg_dat)
            {
                break;
            }
            p ++;
        }
    }
    else
    {
        pos_dat = dat;
        while(1)
        {
            temp_data = pos_dat%radix;
            if(10 <= temp_data)
            {
                temp_data += 'A'-10;
            }
            else
            {
                temp_data += '0';
            }

            *p = temp_data;

            pos_dat = pos_dat/radix;
            valid_num ++;

            if(!pos_dat)
            {
                break;
            }
            p ++;
        }
    }
    return valid_num;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     printf 显示转换
// 参数说明     *d_buff         缓冲区
// 参数说明     len             长度
// 返回参数     void
// 使用示例     printf_reverse_order(vstr, vlen);
// 备注信息     本函数在文件内部调用 用户不用关注 也不可修改
//-------------------------------------------------------------------------------------------------------------------
static void printf_reverse_order (int8 *d_buff, uint32 len)
{
    uint32 i;
    int8  temp_data;
    for(i = 0; len / 2 > i; i ++)
    {
        temp_data = d_buff[len - 1 - i];
        d_buff[len - 1 -i ] = d_buff[i];
        d_buff[i] = temp_data; 
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     sprintf 函数实现
// 参数说明     *buffer         缓冲区
// 参数说明		siz				大小
// 参数说明     *format         源字符串
// 参数说明     args            可变参数列表
// 返回参数     int32          处理后数据长
// 使用示例     zf_sprintf(buff, "Data : %d", 100);
// 备注信息     本函数在文件内部调用 用户不用关注 也不可修改
//-------------------------------------------------------------------------------------------------------------------
int32 vsnprintf(char *buffer, uint32 siz, const char *format, va_list args) 
{
    uint32 i;
    char *ptr;
    int j;
    int num;
    char temp[16];
    int len;
    int is_negative;
    char *str;
    uint32 num_x;
    char temp_x[9];
    const char hex_digits[] = "0123456789abcdef";

    i = 0;
    ptr = buffer;
    
    if (buffer == NULL || format == NULL || siz == 0)
	{
        return 0;
    }
		
    while (*format != '\0' && i < siz - 1) 
	{
        if (*format != '%') 
		{
            *ptr++ = *format++;
            i++;
            continue;
        }

        /* 处理格式符 */
        format++;
        switch (*format) 
		{
            case 'd':
                num = va_arg(args, int);
                len = 0;
                is_negative = 0;

                if (num < 0) 
				{
                    is_negative = 1;
                    num = -num;
                }

                /* 数字转字符串 */
                do {
                    temp[len++] = '0' + (num % 10);
                    num /= 10;
                } while (num > 0 && len < (int)sizeof(temp) - 1);

                /* 添加负号 */
                if (is_negative) 
				{
                    temp[len++] = '-';
                }

                /* 反转字符串 */
                for (j = 0; j < len / 2; j++) 
				{
                    char t = temp[j];
                    temp[j] = temp[len - j - 1];
                    temp[len - j - 1] = t;
                }

                /* 复制到缓冲区 */
                for (j = 0; j < len && i < siz - 1; j++) 
				{
                    *ptr++ = temp[j];
                    i++;
                }
                break;

            case 's':
                str = va_arg(args, char*);
                if (str == NULL) 
				{
                    str = "(null)";
                }
                
                while (*str != '\0' && i < siz - 1) 
				{
                    *ptr++ = *str++;
                    i++;
                }
                break;

            case 'c':
                *ptr++ = (char)va_arg(args, int);
                i++;
                break;

            case 'x':
                num_x = va_arg(args, uint32);
                len = 0;

                do {
                    temp_x[len++] = hex_digits[num_x % 16];
                    num_x /= 16;
                } while (num_x > 0 && len < (int)sizeof(temp_x) - 1);

                /* 反转字符串 */
                for (j = 0; j < len / 2; j++) 
				{
                    char t = temp_x[j];
                    temp_x[j] = temp_x[len - j - 1];
                    temp_x[len - j - 1] = t;
                }

                /* 复制到缓冲区 */
                for (j = 0; j < len && i < siz - 1; j++) 
				{
                    *ptr++ = temp_x[j];
                    i++;
                }
                break;

            default:
                *ptr++ = *format;
                i++;
                break;
        }
        format++;
    }

    *ptr = '\0';  /* 确保字符串结束 */
    return (int32)i;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     sprintf 函数实现
// 参数说明     *buff           缓冲区
// 参数说明     *format         源字符串
// 参数说明     ...             可变参数列表
// 返回参数     uint32          处理后数据长
// 使用示例     zf_sprintf(buff, "Data : %d", 100);
// 备注信息     本函数在文件内部调用 用户不用关注 也不可修改
//-------------------------------------------------------------------------------------------------------------------
uint32 zf_sprintf (int8 *buff, const int8 *format, ...)
{
    uint32 buff_len = 0;
	va_list arg;
	va_start(arg, format);

    while (*format)
    {
        int8 ret = *format;
        if ('%' == ret)
        {
            switch (*++ format)
            {
                case 'a':// 十六进制p计数法输出浮点数 暂未实现
                    {
                    }
                    break;

                case 'c':// 一个字符
                    {
                        int8 ch = (int8)va_arg(arg, uint32);
                        *buff = ch;
                        buff ++;
                        buff_len ++;
                    }
                    break;

                case 'd':
                case 'i':// 有符号十进制整数
                    {
                        int8 vstr[33];
                        int32 ival = (int32)va_arg(arg, int32);
                        uint8 vlen = number_conversion_ascii((uint32)ival, vstr, 1, 10);

                        if(0 > ival)  
                        {
                            vstr[vlen] = '-';
                            vlen ++;
                        }
                        printf_reverse_order(vstr, vlen);
                        memcpy(buff, vstr, vlen);
                        buff += vlen;
                        buff_len += vlen;
                    }
                    break;

                case 'f':// 浮点数，输出小数点后六位  不能指定输出精度
                case 'F':// 浮点数，输出小数点后六位  不能指定输出精度
                {
                    int8 vstr[33];
                    double ival = (double)va_arg(arg, double);
                    uint8 vlen = number_conversion_ascii((uint32)(int32)ival, vstr, 1, 10);

                    if(0 > ival)  
                    {
                        vstr[vlen] = '-';
                        vlen ++;
                    }
                    printf_reverse_order(vstr, vlen);
                    memcpy(buff, vstr, vlen);
                    buff += vlen;
                    buff_len += vlen;

                    ival = ((double)ival - (int32)ival)*1000000;
                    if(ival)
                    {
                        vlen = number_conversion_ascii((uint32)(int32)ival, vstr, 1, 10);
                    }
                    else
                    {
                        vstr[0] = vstr[1] = vstr[2] = vstr[3] = vstr[4] = vstr[5] = '0';
                        vlen = 6;
                    }

                    while(6 > vlen)
                    {
                        vstr[vlen] = '0';
                        vlen ++;
                    }

                    vstr[vlen] = '.';
                    vlen ++;

                    printf_reverse_order(vstr, vlen);
                    memcpy(buff, vstr, vlen);
                    buff += vlen;
                    buff_len += vlen;
                }
                break;

                case 'u':// 无符号十进制整数
                    {
                        int8 vstr[33];
                        uint32 ival = (uint32)va_arg(arg, uint32);
                        uint8 vlen = number_conversion_ascii(ival, vstr, 0, 10);

                        printf_reverse_order(vstr, vlen);
                        memcpy(buff, vstr, vlen);
                        buff += vlen;
                        buff_len += vlen;
                    }
                    break;

                case 'o':// 无符号八进制整数 
                    {
                        int8 vstr[33];
                        uint32 ival = (uint32)va_arg(arg, uint32);
                        uint8 vlen = number_conversion_ascii(ival, vstr, 0, 8);

                        printf_reverse_order(vstr, vlen);
                        memcpy(buff, vstr, vlen);
                        buff += vlen;
                        buff_len += vlen;

                    }
                    break;

                case 'x':// 无符号十六进制整数
                case 'X':// 无符号十六进制整数
                    {
                        int8 vstr[33];
                        uint32 ival = (uint32)va_arg(arg, uint32);
                        uint8 vlen = number_conversion_ascii(ival, vstr, 0, 16);

                        printf_reverse_order(vstr, vlen);
                        memcpy(buff, vstr, vlen);
                        buff += vlen;
                        buff_len += vlen;
                    }
                    break;

                case 's':// 字符串
                    {
                        int8 *pc = va_arg(arg, int8 *);
                        while (*pc)
                        {
                            *buff = *pc;
                            buff ++;
                            buff_len ++;
                            pc ++;
                        }
                    }
                    break;

                case 'p':// 以16进制形式输出指针
                    {
                        int8 vstr[33];
                        uint32 ival = (uint32)va_arg(arg, uint32);
                        //uint8 vlen = number_conversion_ascii(ival, vstr, 0, 16);
                        number_conversion_ascii(ival, vstr, 0, 16);
                        printf_reverse_order(vstr, 8);
                        memcpy(buff, vstr, 8);
                        buff += 8;
                        buff_len += 8;
                    }
                    break;

                case '%':// 输出字符% 
                    {
                        *buff = '%';
                        buff ++;
                        buff_len ++;
                    }
                    break;

                default:
                    break;
            }
        }
        else
        {
            *buff = (int8)(*format);
            buff ++;
            buff_len ++;
        }
        format ++;
    }
    va_end(arg);

    return buff_len;
}


