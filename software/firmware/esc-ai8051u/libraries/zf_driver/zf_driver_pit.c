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

#include "zf_common_clock.h"
#include "zf_driver_pit.h"

#pragma warning disable = 183
#pragma warning disable = 177

//-------------------------------------------------------------------------------------------------------------------
//  @brief      定时器周期中断
//  @param      pit_n      定时器通道号
//  @param      time_ms    时间(ms)
//  @return     void
//  Sample usage:          pit_timer_ms(TIM_0, 10)
//						   使用定时器0做周期中断，时间10ms一次。
//-------------------------------------------------------------------------------------------------------------------
void pit_init(pit_index_enum pit_n, uint32 period)
{
	uint8 freq_div = 0;                
    uint16 period_temp = 0;               
    uint16 temp = 0;


	if(period > (255*65535))
	{
		// 设置定时器为12T模式防止溢出
		period = period / 12;
		
		if(TIM0_PIT == pit_n)
		{
			AUXR &= ~0x80;		
		}
		else if(TIM1_PIT == pit_n)
		{
			AUXR &= ~0x40;		
		}
		else if(TIM2_PIT == pit_n)
		{
			AUXR &= ~0x04;		
		}
		else if(TIM3_PIT == pit_n)
		{
			T4T3M &= ~0x02;	
		}
		else if(TIM4_PIT == pit_n)
		{
			T4T3M &= ~0x20;		
		}
		else if(TIM11_PIT == pit_n)
		{
			T11CR &= ~0x10;		
		}

	}
	else
	{
		// 设置为1T模式
		if(TIM0_PIT == pit_n)
		{
			AUXR |= 0x80;		
		}
		else if(TIM1_PIT == pit_n)
		{
			AUXR |= 0x40;		
		}
		else if(TIM2_PIT == pit_n)
		{
			AUXR |= 0x04;		
		}
		else if(TIM3_PIT == pit_n)
		{
			T4T3M |= 0x02;	
		}
		else if(TIM4_PIT == pit_n)
		{
			T4T3M |= 0x20;		
		}
		else if(TIM11_PIT == pit_n)
		{
			T11CR |= 0x10;		
		}
	}
		
	
	freq_div = ((period) >> 15);                   // 计算预分频
	period_temp = ((period) / (freq_div+1));       // 计算自动重装载值

	temp = (uint16)65536 - period_temp;

    if(TIM0_PIT == pit_n)
    {
		TM0PS = freq_div;	// 设置分频值
        TMOD |= 0x00; 		// 模式 0
        TL0 = temp;
        TH0 = temp >> 8;
        TR0 = 1; 			// 启动定时器
        ET0 = 1; 			// 使能定时器中断
    }
    else if(TIM1_PIT == pit_n)
    {
		TM1PS = freq_div;	// 设置分频值
        TMOD |= 0x00; // 模式 0
        TL1 = temp;
        TH1 = temp >> 8;
        TR1 = 1; // 启动定时器
        ET1 = 1; // 使能定时器中断
    }
    else if(TIM2_PIT == pit_n)
    {
		TM2PS = freq_div;	// 设置分频值
        T2L = temp;
        T2H = temp >> 8;
        AUXR |= 0x10; // 启动定时器
        IE2 |= 0x04; // 使能定时器中断
    }
    else if(TIM3_PIT == pit_n)
    {
		TM3PS = freq_div;	// 设置分频值
        T3L = temp;
        T3H = temp >> 8;
        T4T3M |= 0x08; // 启动定时器
        IE2 |= 0x20; // 使能定时器中断
    }
    else if(TIM4_PIT == pit_n)
    {
		TM4PS = freq_div;	// 设置分频值
        T4L = temp;
        T4H = temp >> 8;
        T4T3M |= 0x80; // 启动定时器
        IE2 |= 0x40; // 使能定时器中断
    }
	else if(TIM11_PIT == pit_n)
	{
		T11PS = freq_div;	// 设置分频值
		T11L = temp;
        T11H = temp >> 8;
		T11CR = 0x82;	// 启动定时器，使能定时器中断
		
	}
}

