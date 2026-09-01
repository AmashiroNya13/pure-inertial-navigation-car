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

#include "zf_driver_pwm.h"
#include "zf_driver_gpio.h"
#include "zf_driver_delay.h"
#include "zf_driver_exti.h"
#include "zf_driver_timer.h"
#include "zf_driver_pit.h"

#include "comparator.h"
#include "bldc_config.h"
#include "pwm_out.h"
#include "battery.h"
#include "motor_control.h"


 motor_struct motor;

 static uint16 temp_commutation_time;

 static uint8 xc_flag = 0;	
// 0-换相
// 1-消磁
// 2-消磁结束

//-------------------------------------------------------------------------------------------------------------------
//  @brief      电机step加一
//  @param      void
//  @return
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void motor_next_step(void)
{
    if(++motor.step >= 6) motor.step = 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      定时器4重新配置
//  @param      void
//  @return     uint16          返回当前计时器的时间
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint16 tim4_reconfig(void)
{
    uint16 temp;
    // 获取换相时间
    T4T3M &= ~0x80; // 停止定时器
    temp = (T4H << 8) | T4L;
    T4L = 0;
    T4H = 0;
    T4T3M |= 0x80;  // 开启定时器
    return temp;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      定时器4中断函数
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void TM4_Isr(void) interrupt 20
{
    TIM4_CLEAR_FLAG;
    // 换相超时
    if(MOTOR_CLOSE_LOOP == motor.run_flag && (500) < motor.commutation_num)
    {
        // 正在运行的时候 进入此中断应该立即关闭输出
        motor.run_flag = MOTOR_STOP_STALL;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      定时器0换相中断
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void TM0_Isr(void) interrupt TMR0_VECTOR
{	
	//	硬件自动清除中断标志位
	uint16 tim_com; 	

	if(xc_flag == 0)
	{
		if(MOTOR_CLOSE_LOOP == motor.run_flag)
		{
			// step+1
			motor_next_step();
			// 换相
			pwm_x_output[motor.step]();
			xc_flag = 1;

			tim_com = 0xffff - (motor.filter_commutation_time_sum / 36);

			// 停止定时器0
			TR0 = 0;         
			TL0 = (uint8)tim_com;
			TH0 = (uint8)(tim_com >> 8); 	
			TR0 = 1; 		// 启动定时器
			ET0 = 1; 		// 使能定时器中断
			
			// 去掉最早的数据
			motor.commutation_time_sum -= motor.commutation_time[motor.step];
			// 保存换相时间
			motor.commutation_time[motor.step] = temp_commutation_time;
			// 叠加新的换相时间，求6次换相总时长
			motor.commutation_time_sum += temp_commutation_time;
			motor.commutation_num++;
			// 一阶低通滤波
			motor.filter_commutation_time_sum = (motor.filter_commutation_time_sum * 7 + motor.commutation_time_sum * 1) >> 3;
			
			// 等待稳定，再开始换相错误判断
			if((BLDC_CLOSE_LOOP_WAIT) < motor.commutation_num)
			{
				// 本次换向60度的时间，在上一次换向一圈时间的30度到90度，否则认为换向错误
				if((temp_commutation_time > (motor.filter_commutation_time_sum * 30 / 360)) && (temp_commutation_time < (motor.filter_commutation_time_sum * 90 / 360)))
				{
					// 延时减去换向失败计数器
					if((motor.commutation_failed_num))
					{
						motor.commutation_failed_num--;
					}
				}
				else
				{
					// 只有在占空比大于10%的时候，才进行换相错误判断。
	//				if(motor.duty_register >= (BLDC_PWM_ARR_MAX / 10))
					{
						motor.commutation_failed_num ++;
						if(BLDC_COMMUTATION_FAILED_MAX < motor.commutation_failed_num)
						{
							motor_stop();
							motor.run_flag = MOTOR_STOP_STALL;
						}
					}

				}
                
//                // 换向时间超过1000us，这种是不可能的。
//                if(motor.filter_commutation_time_sum > (2000*2))
//                {
//                    motor_stop();
//                    motor.run_flag = MOTOR_STOP_STALL;
//                }
			}
            
           
		}
	}
	else if(xc_flag == 1)
	{
		xc_flag = 2;
		
//		// 等待消磁, 10度
//		while(((TH0 << 8) | TL0) < (motor.filter_commutation_time_sum / 36));

		comparator_open_isr();
		
		ET0 = 0; 		// 关闭定时器0中断
		TR0 = 0; 		// 关闭定时器0
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      比较器中断函数
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void comparator_isr(void) interrupt 21		// 比较器中断函数, 检测到反电动势过0事件
{
	// BLDC_MOTOR_ANGLE度换相
	uint16 tim_com = 0xffff - (motor.filter_commutation_time_sum * BLDC_MOTOR_ANGLE / 360); 	
	
    // 获取换相时间
	temp_commutation_time = (T4H << 8) | T4L;

	xc_flag = 0;

//	// 去除反电动势毛刺
//    if((temp_commutation_time < (motor.commutation_time_sum / 36) && (BLDC_CLOSE_LOOP_WAIT) < motor.commutation_num))
//    {
//        return;
//    }
		
	T4T3M &= ~0x80; 		// 停止定时器4
	T4L = 0;
	T4H = 0;
	T4T3M |= 0x80;  		// 开启定时器

	// 停止定时器0
	TR0 = 0;         
	TL0 = (uint8)tim_com;
	TH0 = (uint8)(tim_com >> 8); 	
	TR0 = 1; 		// 启动定时器
	ET0 = 1; 		// 使能定时器中断
	

	// 失能比较器中断
	comparator_close_isr();
	
	
	// 比较器清除中断标志位
	CMPCR1 &= ~0x40;
}


#define MUSIC_DELAY_MS   250
//-------------------------------------------------------------------------------------------------------------------
//  @brief      电机上电鸣叫
//  @param      volume          鸣叫音量大小
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void motor_power_on_beep(uint16 volume)
{
	uint16  frequency_spectrum[6] = {0, 523, 587, 659, 698, 783};
    uint16 beep_duty;
    beep_duty = volume;
    // 保护限制，避免设置过大烧毁电机
    if(100 < beep_duty)
    {
        beep_duty = 100;
    }
	
	// A上桥PWM B下桥常开
    PWM_A_H_PIN = 0;
    PWM_A_L_PIN = 0;
    PWM_B_H_PIN = 0;
    PWM_B_L_PIN = 1;
    PWM_C_H_PIN = 0;
    PWM_C_L_PIN = 0;

//	pwm_out_duty_update(motor.duty_register);
	pwm_init(PWMA_CH1P_P00, frequency_spectrum[1], beep_duty);
	PWMA_ENO = 1<<0;
	system_delay_ms(MUSIC_DELAY_MS);

    
	// B上桥PWM C下桥常开
	PWM_A_H_PIN = 0;
    PWM_A_L_PIN = 0;
    PWM_B_H_PIN = 0;
    PWM_B_L_PIN = 0;
    PWM_C_H_PIN = 0;
    PWM_C_L_PIN = 1;
	pwm_init(PWMA_CH2P_P02, frequency_spectrum[2], beep_duty);
	PWMA_ENO = 1<<2;
	system_delay_ms(MUSIC_DELAY_MS);

    
	// C上桥PWM A下桥常开
	PWM_A_H_PIN = 0;
    PWM_A_L_PIN = 1;
    PWM_B_H_PIN = 0;
    PWM_B_L_PIN = 0;
    PWM_C_H_PIN = 0;
    PWM_C_L_PIN = 0;
	pwm_init(PWMA_CH3P_P04, frequency_spectrum[3], beep_duty);
	PWMA_ENO = 1<<4;
	system_delay_ms(MUSIC_DELAY_MS);

    
    PWM_A_H_PIN = 0;
    PWM_A_L_PIN = 0;
    PWM_B_H_PIN = 0;
    PWM_B_L_PIN = 0;
    PWM_C_H_PIN = 0;
    PWM_C_L_PIN = 0;
    
	PWMA_ENO = 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      电机停止
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void motor_stop(void)
{
    pwm_out_duty_update(0);
	pwm_close_output();
    comparator_close_isr();
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      电机初始化
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void motor_init(void)
{
    // 初始化为推挽输出
    gpio_init(IO_P00, GPO, 0, GPO_PUSH_PULL);
    gpio_init(IO_P01, GPO, 0, GPO_PUSH_PULL);
    gpio_init(IO_P02, GPO, 0, GPO_PUSH_PULL);
    gpio_init(IO_P03, GPO, 0, GPO_PUSH_PULL);
    gpio_init(IO_P04, GPO, 0, GPO_PUSH_PULL);
    gpio_init(IO_P05, GPO, 0, GPO_PUSH_PULL);
    
    
    // 变量清零
    motor.duty = 0;
    motor.duty_register = 0;
    motor.run_flag = 0;
    motor.motor_start_delay = 0;
    motor.motor_start_wait = 0;
    motor.restart_delay = 0;
    motor.commutation_time_sum = 0;
    motor.commutation_num = 0;
	
	T4T3M = 0;
	
    T4T3M |= 1<<7;	// 定时器4使能
    T4T3M |= 1<<5;	// T4 1T模式
	
	
	AUXR |= 1<<7;     // 1T模式
	TMOD = 0x00; 	// 模式 0
	TL0 = 0; 	
	TH0 = 0;
//	TR0 = 1; 		// 启动定时器
//	ET0 = 1; 		// 使能定时器中断
	TM0PS = (system_clock / 1000000 / 2) - 1;	// 设置分频系数，时基为0.5us

    T4L = 0;
    T4H = 0;
    TM4PS = (system_clock / 1000000 / 2) - 1;	// 设置分频系数，时基为0.5us

    IP = 0;
    IPH = 0;
    IP2 = 0;
    IP2H = 0;
	
    // 设置定时器0优先级为 3 ,最高优先级为3
    IP  |= 1<<1;
    IPH |= 1<<1;
   
	// 设置比较器优先级 为 3 ,最高优先级为3
	IP2  |= 1<<5;
	IP2H |= 1<<5;

	// 设置PWMB输入捕获高优先级 为 2 ,最高优先级为3
    IP2  |= 0<<3;
    IP2H |= 1<<3;
   


#if (1 == BLDC_BEEP_ENABLE)
    // 电机鸣叫表示初始化完成
    motor_power_on_beep(BLDC_BEEP_VOLUME);
#endif
}
