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
#ifndef _zf_device_type_h_
#define _zf_device_type_h_

#include "zf_common_typedef.h"
#include "zf_driver_uart.h"

typedef enum
{
    NO_WIRELESS = 0,                                                            // 无设备
    WIRELESS_UART,                                                              // 无线串口
    BLE6A20,																	// 蓝牙BLE6A20
	LORA3A22_UART																// LORA
} wireless_type_enum;

typedef enum
{
    NO_TOF = 0,                                                                 // 无设备
    TOF_DL1A,                                                                   // DL1A
    TOF_DL1B,                                                                   // DL1B
} tof_type_enum;

typedef enum
{
    NO_GPS = 0,                                                                 // 无设备
    GPS_TAU1201,                                                                   // DL1A

} gps_type_enum;


#define	SOFT_IIC 		(0)
#define SOFT_SPI 		(1)                                                              // 无设备
#define	HARDWARE_IIC 	(2)
#define	HARDWARE_SPI 	(3)


extern gps_type_enum 		gps_type;
extern tof_type_enum 		tof_type;
extern wireless_type_enum  	wireless_type;

extern void (*wireless_module_uart_handler)	(uint8 dat);
extern void (*tof_module_exti_handler)		(uint8 dat);


extern void set_wireless_type (wireless_type_enum type_set, uart_index_enum uartx, void *uart_callback);
extern void set_tof_type (tof_type_enum type_set, void *exti_callback);
extern void set_gps_type (gps_type_enum type_set, uart_index_enum uartx, void *uart_callback);

#endif
