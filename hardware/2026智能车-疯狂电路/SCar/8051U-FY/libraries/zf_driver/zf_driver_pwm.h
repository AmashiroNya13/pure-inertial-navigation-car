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

#ifndef __ZF_DRIVER_PWM_H
#define __ZF_DRIVER_PWM_H
#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"



// 16位
// BIT[15] 		: 0->PWMA 1->PWMB
// BIT[14:12]  	: 0->CH1 1->CH2 2->CH3 3->CH4 4->CH5 5->CH6
// BIT[11]		: 0->P 1->N
// BIT[10:9]	: 0->PS0 1->PS1 2->PS2 3->PS3
// BIT[7:0]		: IO_PIN
	
typedef enum
{
    // PWMA和PWMB是两组不同的PWM
    
    // 以下是PWMA通道，初始化只能有一个频率
    // 同一组PWM，同一时刻，只能有同一个PWM输出。
    // 例如:PWMA_CH1P_P10 和 PWMA_CH1N_P11不能一起输出。
	// 但是不同的通道可以同一时刻输出。
    // 例如:PWMA_CH1P_P10 和 PWMA_CH2N_P03 可以同时输出
    PWMA_CH1P_P10 = 0<<15|0<<12|0<<11|0<<9|IO_P10, PWMA_CH1N_P11 = 0<<15|0<<12|1<<11|0<<9|IO_P11,
    PWMA_CH1P_P00 = 0<<15|0<<12|0<<11|1<<9|IO_P00, PWMA_CH1N_P01 = 0<<15|0<<12|1<<11|1<<9|IO_P01,
    PWMA_CH1P_P20 = 0<<15|0<<12|0<<11|2<<9|IO_P20, PWMA_CH1N_P21 = 0<<15|0<<12|1<<11|2<<9|IO_P21,
																	
    PWMA_CH2P_P12 = 0<<15|1<<12|0<<11|0<<9|IO_P12, PWMA_CH2N_P13 = 0<<15|1<<12|1<<11|0<<9|IO_P13,
    PWMA_CH2P_P02 = 0<<15|1<<12|0<<11|1<<9|IO_P02, PWMA_CH2N_P03 = 0<<15|1<<12|1<<11|1<<9|IO_P03,
    PWMA_CH2P_P22 = 0<<15|1<<12|0<<11|2<<9|IO_P22, PWMA_CH2N_P23 = 0<<15|1<<12|1<<11|2<<9|IO_P23,
																		 
    PWMA_CH3P_P14 = 0<<15|2<<12|0<<11|0<<9|IO_P14, PWMA_CH3N_P15 = 0<<15|2<<12|1<<11|0<<9|IO_P15,
    PWMA_CH3P_P04 = 0<<15|2<<12|0<<11|1<<9|IO_P04, PWMA_CH3N_P05 = 0<<15|2<<12|1<<11|1<<9|IO_P05,
    PWMA_CH3P_P24 = 0<<15|2<<12|0<<11|2<<9|IO_P24, PWMA_CH3N_P25 = 0<<15|2<<12|1<<11|2<<9|IO_P25,
																
    PWMA_CH4P_P16 = 0<<15|3<<12|0<<11|0<<9|IO_P16, PWMA_CH4N_P17 = 0<<15|3<<12|1<<11|0<<9|IO_P17,
    PWMA_CH4P_P06 = 0<<15|3<<12|0<<11|1<<9|IO_P06, PWMA_CH4N_P07 = 0<<15|3<<12|1<<11|1<<9|IO_P07,
    PWMA_CH4P_P26 = 0<<15|3<<12|0<<11|2<<9|IO_P26, PWMA_CH4N_P27 = 0<<15|3<<12|1<<11|2<<9|IO_P27,

//	PWMA_CH5_P33  = 0x040000 | IO_P33,
//	PWMA_CH5_P44  = 0x040100 | IO_P44,
//	PWMA_CH5_P56  = 0x040200 | IO_P56,
//		
//	PWMA_CH6_P34  = 0x050000 | IO_P34,
//	PWMA_CH6_P47  = 0x050100 | IO_P47,
//	PWMA_CH6_P57  = 0x050200 | IO_P57,

    // 以下是PWMB通道，初始化只能有一个频率
    // 同一组PWM，同一时刻，只能有同一个PWM输出。
    // 例如:PWMB_CH1_P01 和 PWMB_CH1_P21 不能同时输出
    // 但是不同的通道可以同一时刻输出。
    // 例如:PWMB_CH1_P01 和 PWMB_CH2_P13 可以同时输出
    PWMB_CH1_P01 = 1<<15|0<<12|0<<11|0<<9|IO_P01,
    PWMB_CH1_P11 = 1<<15|0<<12|0<<11|1<<9|IO_P11,
    PWMB_CH1_P21 = 1<<15|0<<12|0<<11|2<<9|IO_P21,
    PWMB_CH1_P50 = 1<<15|0<<12|0<<11|3<<9|IO_P50,

    PWMB_CH2_P03 = 1<<15|1<<12|0<<11|0<<9|IO_P03,
    PWMB_CH2_P13 = 1<<15|1<<12|0<<11|1<<9|IO_P13,
    PWMB_CH2_P23 = 1<<15|1<<12|0<<11|2<<9|IO_P23,
    PWMB_CH2_P51 = 1<<15|1<<12|0<<11|3<<9|IO_P51,

    PWMB_CH3_P05 = 1<<15|2<<12|0<<11|0<<9|IO_P05,
    PWMB_CH3_P15 = 1<<15|2<<12|0<<11|1<<9|IO_P15,
    PWMB_CH3_P25 = 1<<15|2<<12|0<<11|2<<9|IO_P25,
    PWMB_CH3_P52 = 1<<15|2<<12|0<<11|3<<9|IO_P52,

    PWMB_CH4_P07 = 1<<15|3<<12|0<<11|0<<9|IO_P07,
    PWMB_CH4_P17 = 1<<15|3<<12|0<<11|1<<9|IO_P17,
    PWMB_CH4_P27 = 1<<15|3<<12|0<<11|2<<9|IO_P27,
    PWMB_CH4_P53 = 1<<15|3<<12|0<<11|3<<9|IO_P53,
    
} pwm_channel_enum;



#define PWM_DUTY_MAX 10000

void    pwm_set_duty    (pwm_channel_enum pin, uint32 duty);
void    pwm_set_freq    (pwm_channel_enum pin, uint32 freq, uint32 duty);
void    pwm_init        (pwm_channel_enum pin, uint32 freq, uint32 duty);

#endif
