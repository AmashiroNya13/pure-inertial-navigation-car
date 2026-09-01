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


#ifndef __ZF_DRIVER_GPIO_H
#define __ZF_DRIVER_GPIO_H

#include "zf_common_typedef.h"


typedef enum
{
    IO_P00 = 0x10, IO_P01, IO_P02, IO_P03, IO_P04, IO_P05, IO_P06, IO_P07,
    IO_P10 = 0x20, IO_P11, IO_P12, IO_P13, IO_P14, IO_P15, IO_P16, IO_P17,
    IO_P20 = 0x30, IO_P21, IO_P22, IO_P23, IO_P24, IO_P25, IO_P26, IO_P27,
    IO_P30 = 0x40, IO_P31, IO_P32, IO_P33, IO_P34, IO_P35, IO_P36, IO_P37,
    IO_P40 = 0x50, IO_P41, IO_P42, IO_P43, IO_P44, IO_P45, IO_P46, IO_P47,
    IO_P50 = 0x60, IO_P51, IO_P52, IO_P53, IO_P54, IO_P55, IO_P56, IO_P57,
//    IO_P60 = 0x70, IO_P61, IO_P62, IO_P63, IO_P64, IO_P65, IO_P66, IO_P67,
//    IO_P70 = 0x80, IO_P71, IO_P72, IO_P73, IO_P74, IO_P75, IO_P76, IO_P77,
	IO_NULL = 0xFF,
} gpio_pin_enum;


typedef enum
{
    // 输入
	GPI_IMPEDANCE       = 0x00,			// 高阻输入
    GPI_FLOATING_IN     = 0x01,         // 浮空输入模式
    GPI_PULL_DOWN       = 0x02,         // 下拉输入
    GPI_PULL_UP         = 0x03,         // 上拉输入
	
	
    // 输出
    GPO_PUSH_PULL       = 0x10,         // 通用推挽输出模式
    GPO_OPEN_DTAIN      = 0x11,         // 通用开漏输出模式
	
	// 双向GPIO，既可以输入也可以输出
	GPIO_NO_PULL   		= 0x20,			// GPIO无上下拉。
}gpio_mode_enum;

typedef enum
{
    //pnm1 pnm0

	GPI  = 0,				// 输入，仅高阻输入
	GPO  = 1,				// 输出，推挽输出，或者开漏
	GPIO = 2,				// 准双向口(弱上拉)
}gpio_dir_enum;


typedef enum
{
    GPIO_LOW  = 0,  		//  低电平
    GPIO_HIGH = 1,  		//  高电平
}gpio_level_enum;


typedef enum
{
    GPIO_SPEED_LOW  = 1,  	// 电平转换速度快，相应的上下冲比较大
    GPIO_SPEED_FAST = 0,  	// 电平转换速度慢，相应的上下冲比较小
}gpio_speed_enum;


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     对应 IO 复位为低电平
// 参数说明     x           选择的引脚 (可选择范围由 zf_driver_gpio.h 内 gpio_pin_enum 枚举值确定)
// 返回参数     void
// 使用示例     gpio_low(D5);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
#define gpio_low(x)            	gpio_set_level(x, 0)

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     对应 IO 置位为高电平
// 参数说明     x           选择的引脚 (可选择范围由 zf_driver_gpio.h 内 gpio_pin_enum 枚举值确定)
// 返回参数     void
// 使用示例     gpio_high(D5);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
#define gpio_high(x)            gpio_set_level(x, 1)

void 	gpio_set_level_speed	(gpio_pin_enum pin, gpio_speed_enum speed);

void    gpio_set_level          (gpio_pin_enum pin, uint8 dat);
uint8   gpio_get_level          (gpio_pin_enum pin);
void    gpio_toggle_level       (gpio_pin_enum pin);
void    gpio_init               (gpio_pin_enum pin, gpio_dir_enum dir, const uint8 dat, gpio_mode_enum mode);


#endif
