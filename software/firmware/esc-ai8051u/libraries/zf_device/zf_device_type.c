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

#include "zf_device_type.h"

wireless_type_enum  wireless_type	= NO_WIRELESS;
tof_type_enum       tof_type 		= NO_TOF;
gps_type_enum       gps_type 		= NO_GPS;

void (*tof_module_exti_handler)(uint8 dat) = NULL;			// ToF 模块 INT 更新中断

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置无线模块类型
// 参数说明     type_set        选定的无线模块类型
// 参数说明     uart_callback   设备的串口回调函数
// 返回参数     void
// 使用示例     set_wireless_type(WIRELESS_UART, uart_callback);
// 备注信息     一般由各摄像头初始化内部调用
//-------------------------------------------------------------------------------------------------------------------
void set_wireless_type (wireless_type_enum type_set, uart_index_enum uartx, void *uart_callback)
{
    wireless_type = type_set;
    
    if(uartx == UART_1)
    {
        uart1_irq_handler = uart_callback;
    }
    else if(uartx == UART_2)
    {
        uart2_irq_handler = uart_callback;
    }
    else if(uartx == UART_3)
    {
        uart3_irq_handler = uart_callback;
    }
    else if(uartx == UART_4)
    {
        uart4_irq_handler = uart_callback;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置 ToF 模块类型
// 参数说明     type_set        选定的 ToF 模块类型
// 参数说明     exti_callback   设备的外部中断回调函数
// 返回参数     void
// 使用示例     set_tof_type(TOF_DL1A, dl1a_int_handler);
// 备注信息     一般由各摄像头初始化内部调用
//-------------------------------------------------------------------------------------------------------------------
void set_tof_type (tof_type_enum type_set, void *exti_callback)
{
    tof_type = type_set;
    tof_module_exti_handler = exti_callback;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置 ToF 模块类型
// 参数说明     type_set        选定的 ToF 模块类型
// 参数说明     exti_callback   设备的外部中断回调函数
// 返回参数     void
// 使用示例     set_tof_type(TOF_DL1A, dl1a_int_handler);
// 备注信息     一般由各摄像头初始化内部调用
//-------------------------------------------------------------------------------------------------------------------
void set_gps_type (gps_type_enum type_set, uart_index_enum uartx, void *uart_callback)
{
    gps_type = type_set;
    if(uartx == UART_1)
    {
        uart1_irq_handler = uart_callback;
    }
    else if(uartx == UART_2)
    {
        uart2_irq_handler = uart_callback;
    }
    else if(uartx == UART_3)
    {
        uart3_irq_handler = uart_callback;
    }
    else if(uartx == UART_4)
    {
        uart4_irq_handler = uart_callback;
    }
}

