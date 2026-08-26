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
#include "zf_common_typedef.h"
#include "zf_common_clock.h"

#include "bldc_config.h"
#include "motor_control.h"
#include "pwm_out.h"
#include "signal_input.h"

#define PWMIN_PIN   P21

#define PWMIN_MAX_HIGH_TIME_US      (300u)
#define PWMIN_THROTTLE_STEPS        (500u)
#define PWMIN_DEADBAND_US           (4u)

pwmin_struct pwmin;

 
uint8 pwm_input_timeout_count = 0;
//-------------------------------------------------------------------------------------------------------------------
//  @brief      PWMB输入捕获中断
//  @param      void                        
//  @return     void          
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void pwmb_isr()interrupt 27
{
    uint16 temp;
	if(PWMB_SR1 & 0x02)
	{
		pwmin.period = (PWMB_CCR5H << 8) + PWMB_CCR5L;	    // CC1捕获周期宽度
		PWMB_SR1 = 0;
        
        // 计算输入PWM信号的频率
        pwmin.frequency = system_clock / (PWMB_PSCRL + 1) / pwmin.period;
	}
	
	if(PWMB_SR1 & 0x04)
	{
		pwmin.high_value = (PWMB_CCR6H << 8) + PWMB_CCR6L;   // CC2捕获高电平宽度
		PWMB_SR1 = 0;
        
        // 频率在合理的范围内才计算 (1000Hz模式支持)
        if((30 < pwmin.frequency) && (1100 > pwmin.frequency))
        {
            // 1000Hz模式: 0~300us 高电平时间对应油门 0~100%
            pwmin.high_time = pwmin.high_value;
            
            if(pwmin.high_time > PWMIN_MAX_HIGH_TIME_US)
            {
                pwmin.high_time = PWMIN_MAX_HIGH_TIME_US;
            }
            
            if(pwmin.high_time < PWMIN_DEADBAND_US)
            {
                temp = 0;
            }
            else
            {
                temp = (uint16)(((uint32)pwmin.high_time * PWMIN_THROTTLE_STEPS) / PWMIN_MAX_HIGH_TIME_US);
            }
            pwmin.throttle = temp;
			
			pwm_input_timeout_count = 0;
        }
        
        // 更新占空比（BLDC_USR_DUTY>0 时为宏固定占空比，见 bldc_config.h）
#if (BLDC_USR_DUTY == 0)
        motor.duty = (uint32)pwmin.throttle * BLDC_PWM_ARR_MAX / 500;
#endif
	}
    
    if(PWMB_SR1 & 0x01)
    {
        PWMB_SR1 = 0;
				
#if (BLDC_USR_DUTY == 0)
        // 未检测到输入信号则输出油门都清零
		if(motor.duty > 0)
		{
			if(++pwm_input_timeout_count >= 2)
			{
				pwm_input_timeout_count = 0;
				
				pwmin.throttle = 0;
				motor.duty = 0;
			}
		}
#endif
    }

#if (BLDC_USR_DUTY > 0)
    // 宏固定占空比：1~100 表示目标 1%~100%，不读外部捕获、不做无信号清零
    motor.duty = (uint32)BLDC_USR_DUTY * (uint32)BLDC_PWM_ARR_MAX / 100u;
    pwm_input_timeout_count = 0;
#endif
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      PWMB输入捕获初始化
//  @param      void                        
//  @return     void          
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void pwm_input_init(void)
{
    gpio_init(IO_P21, GPI, 0, GPI_IMPEDANCE);
    gpio_init(IO_P23, GPI, 0, GPI_IMPEDANCE);
    
    PWMB_PS = 0x0A;		// 通道引脚切换
    PWMB_CCMR1 = 0x01;	// CC5为输入模式,且映射到TI5FP5上
	PWMB_CCMR2 = 0x02;	// CC6为输入模式,且映射到TI5FP6上
    
	// CC5E 开启输入捕获
	// CC5P 捕获发生在TI5F的上升沿
	// CC6E 开启输入捕获
	// CC6P 捕获发生在TI5F的下降沿
    PWMB_CCER1 = 0x31;
    
    PWMB_PSCRH = 0;		// 分频值
	PWMB_PSCRL = system_clock / 1000000 - 1;    // 分频值
    PWMB_SMCR = 0x54;	// TS=TI1FP1,SMS=TI1上升沿复位模式
	PWMB_CR1 = 0x01;	// 启动PWMB，向上计数
	PWMB_IER = 0x07;	// 使能CC1、CC2、UIE中断

    pwmin.period = 0;
    pwmin.high_value = 0;
    pwmin.high_time = 0;

#if (BLDC_USR_DUTY > 0)
    motor.duty = (uint32)BLDC_USR_DUTY * (uint32)BLDC_PWM_ARR_MAX / 100u;
#endif
}
