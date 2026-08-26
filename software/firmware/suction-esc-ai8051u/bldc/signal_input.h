/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		pit_timer
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ790875685)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2024-01-22
 ********************************************************************************************************************/


#ifndef _signal_input_h
#define _signal_input_h



#include "zf_common_typedef.h"


typedef struct
{
    uint16 frequency;   // 信号频率
    uint16 period;      // 信号周期
    uint16 high_value;  // 信号高电平计数值
    uint16 high_time;   // 信号高电平时间 us
    uint16 throttle;    // 输出油门大小
}pwmin_struct;


extern pwmin_struct pwmin;

void pwm_input_init(void);

#endif
