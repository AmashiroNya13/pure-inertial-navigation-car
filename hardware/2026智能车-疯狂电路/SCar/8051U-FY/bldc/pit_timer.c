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

#include "zf_driver_timer.h"
#include "zf_driver_gpio.h"
#include "zf_driver_delay.h"
#include "zf_driver_pit.h"

#include "bldc_config.h"
#include "pwm_out.h"
#include "signal_input.h"
#include "motor_control.h"
#include "battery.h"
#include "pit_timer.h"
#include "comparator.h"
#include "motor_control.h"

#include "sine_control.h"


#pragma warning disable = 183


#define ERR_LED P37
#define RUN_LED P36

static uint32 pit_count = 0;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      LED灯光控制
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void led_control(void)
{
    #define TICK_TO_MS(ms)  (ms * 20)
    // 50us进一次该函数
    // LED状态显示
    if(BYTE_LOW_VOLTAGE == motor.run_flag)
    {
        // 电池电压过低，ERROR LED慢闪
        ERR_LED = 0;
        RUN_LED = 1;
    }
    else if(MOTOR_RESTART == motor.run_flag)
    {
        // 由于堵转造成的停止
        if(0 == (pit_count%TICK_TO_MS(50)))
        {
            ERR_LED = !ERR_LED;
            RUN_LED = 1;
        }
    }
    else if(MOTOR_IDLE == motor.run_flag)
    {
        // 空闲状态
        if(0 == (pit_count%TICK_TO_MS(200)))
        {
            RUN_LED = !RUN_LED;
            ERR_LED = 1;
        }
    }
    else if(MOTOR_START == motor.run_flag || MOTOR_OPEN_LOOP == motor.run_flag || MOTOR_CLOSE_LOOP == motor.run_flag)
    {
        // 电机转动状态
        ERR_LED = 1;
        RUN_LED = 0;
    }
  
}

void pit_motor_control()
{
    static uint16 stall_time_out_check = 0;
	static uint8 gpio_state = 0;
    uint8 pin_state = 0;
	static uint8 oled_pin_state = 0;
	
    if(motor.duty == 0)
    {
        ET0 = 0;
        IE2 &= ~0x40;
        motor_stop();
        motor.duty_register = 0;
        stall_time_out_check = 0;
        motor.commutation_num = 0;
        motor.commutation_failed_num = 0;
        motor.run_flag = MOTOR_IDLE;
        return;
    }
    
    switch(motor.run_flag)
    {
		case MOTOR_START:
		{
			uint16 sine_delay;
			ET0 = 0; 					// 关闭定时器0中断
			IE2 &= ~0x40;			  	// 关闭定时器4中断
			comparator_close_isr();		// 关闭比较器中断
			stall_time_out_check = 0;	// 堵转检测计数器设置为0
			motor.commutation_num = 0;	// 电机换相计数器设置为0
			
            PWMA_ENO = 0x3F;            // 打开所有PWM通道
            
			#if BLDC_USE_SINE_START

			sine_init();
			do
			{
                
                motor.duty_register = motor.duty;
        
                if (motor.duty_register < ((uint32)BLDC_MIN_DUTY * BLDC_PWM_ARR_MAX / 100))
                {
                    motor.duty_register = (uint32)BLDC_MIN_DUTY * BLDC_PWM_ARR_MAX / 100;
                }
                else if (motor.duty_register > ((uint32)BLDC_MAX_DUTY * BLDC_PWM_ARR_MAX / 100))
                {
                    motor.duty_register = ((uint32)BLDC_MAX_DUTY * BLDC_PWM_ARR_MAX / 100);
                }

				sine_start(motor.duty_register);
				
				T4T3M &= ~0x80; // 停止定时器
				T4L = 0;
				T4H = 0;
				T4T3M |= 0x80;  // 开启定时器
				
				sine_delay = 420 / BLDC_POLES;
				while(((T4H << 8) | T4L) < sine_delay);
			}while(motor_a_position != 0);
			
			motor.step = 2;
			motor.commutation_time[0] = 6000;
			motor.commutation_time[1] = 6000;
			motor.commutation_time[2] = 6000;
			motor.commutation_time[3] = 6000;
			motor.commutation_time[4] = 6000;
			motor.commutation_time[5] = 6000;
			motor.commutation_time_sum = motor.commutation_time[0] + motor.commutation_time[1] + motor.commutation_time[2] + \
										 motor.commutation_time[3] + motor.commutation_time[4] + motor.commutation_time[5] ;
			motor.filter_commutation_time_sum = motor.commutation_time_sum;
			// 清空定时器计数器值
			T4T3M &= ~0x80; // 停止定时器
			T4L = 0;
			T4H = 0;
			T4T3M |= 0x80;  // 开启定时器
			
			IE2 &= ~0x20; 	// 关闭定时器3中断

			motor.run_flag = MOTOR_OPEN_LOOP;

			// 设置占空比
			motor.duty_register = motor.duty;
            
            if (motor.duty_register < ((uint32)BLDC_MIN_DUTY * BLDC_PWM_ARR_MAX / 100))
            {
                motor.duty_register = (uint32)BLDC_MIN_DUTY * BLDC_PWM_ARR_MAX / 100;
            }
            else if (motor.duty_register > ((uint32)BLDC_MAX_DUTY * BLDC_PWM_ARR_MAX / 100))
            {
                motor.duty_register = ((uint32)BLDC_MAX_DUTY * BLDC_PWM_ARR_MAX / 100);
            }
            
			pwm_out_duty_update(motor.duty_register);
			
            // 换相
			motor_next_step();
			pwm_x_output[motor.step]();
            
			#else

			// 设置启动占空比
			motor.duty_register = motor.duty;
                   
            if (motor.duty_register < (BLDC_MIN_DUTY * BLDC_PWM_ARR_MAX / 100))
            {
                motor.duty_register = BLDC_MIN_DUTY * BLDC_PWM_ARR_MAX / 100;
            }
            else if (motor.duty_register > (BLDC_MAX_DUTY * BLDC_PWM_ARR_MAX / 100))
            {
                motor.duty_register = (BLDC_MAX_DUTY * BLDC_PWM_ARR_MAX / 100);
            }
            
			pwm_out_duty_update(motor.duty_register);
			if(((T4H << 8) | T4L) > 20*1000)
			{
				motor.commutation_time[0] = 6000;
				motor.commutation_time[1] = 6000;
				motor.commutation_time[2] = 6000;
				motor.commutation_time[3] = 6000;
				motor.commutation_time[4] = 6000;
				motor.commutation_time[5] = 6000;
				motor.commutation_time_sum = motor.commutation_time[0] + motor.commutation_time[1] + motor.commutation_time[2] + \
											 motor.commutation_time[3] + motor.commutation_time[4] + motor.commutation_time[5] ;
				motor.filter_commutation_time_sum = motor.commutation_time_sum;
				// 清空定时器计数器值
				T4T3M &= ~0x80; // 停止定时器
				T4L = 0;
				T4H = 0;
				T4T3M |= 0x80;  // 开启定时器
				
				IE2 &= ~0x20; 	// 关闭定时器3中断
				
				motor_next_step();
				pwm_x_output[motor.step]();
				motor.run_flag = MOTOR_OPEN_LOOP;
			}
			#endif
		}
		break;
		case MOTOR_OPEN_LOOP:
		{
			uint8 motor_next_step_flag = 0;
			uint16 temp_commutation_time = (T4H << 8) | T4L;
            
            if (motor.duty_register < ((uint32)BLDC_MIN_DUTY * BLDC_PWM_ARR_MAX / 100))
            {
                motor.duty_register = (uint32)BLDC_MIN_DUTY * BLDC_PWM_ARR_MAX / 100;
            }
            else if (motor.duty_register > ((uint32)BLDC_MAX_DUTY * BLDC_PWM_ARR_MAX / 100))
            {
                motor.duty_register = ((uint32)BLDC_MAX_DUTY * BLDC_PWM_ARR_MAX / 100);
            }
            
            
			// 通过位移的方式，将电平数据存入变量中
			gpio_state = (gpio_state << 1) | (CMPCR1 & 0x01);
			if (!(motor.step % 2))
			{
				//下降沿
				if(gpio_state == 0)
				{
					motor_next_step_flag = 1;
					gpio_state = 0x0F;
				}
			}
			else
			{
				// 上升沿
				if(gpio_state == 0xFF)
				{
					motor_next_step_flag = 1;
					gpio_state = 0xF0;
				}
			}
			
			if(motor_next_step_flag)
			{
				T4T3M &= ~0x80; // 停止定时器
				T4L = 0;
				T4H = 0;
				T4T3M |= 0x80;  // 开启定时器
				// 堵转计数清空
				stall_time_out_check = 0;
				// 去掉最早的数据
				motor.commutation_time_sum -= motor.commutation_time[motor.step];
				// 保存换相时间
				motor.commutation_time[motor.step] = temp_commutation_time;
				// 叠加新的换相时间，求6次换相总时长
				motor.commutation_time_sum += motor.commutation_time[motor.step];
				// 换相次数增加一
				motor.commutation_num++;
				// 一阶低通滤波
				motor.filter_commutation_time_sum = (motor.filter_commutation_time_sum * 3 + motor.commutation_time_sum * 1) >> 2;
				// 等待15度
				while(((T4H << 8) | T4L) < (motor.filter_commutation_time_sum / 24));
				// 电机step加一
				motor_next_step();
				// 电机换相
				pwm_x_output[motor.step]();
			}
			else
			{
				// 堵转检测
				if((10 * 200) < stall_time_out_check++)
				{
					stall_time_out_check = 0;
					motor.commutation_num = 0;
					motor.run_flag = MOTOR_STOP_STALL;
				}
			}
			
			// 闭环状态切换
			//if((motor.commutation_num > (BLDC_OPEN_LOOP_WAIT)) && (motor.filter_commutation_time_sum < 30000))
			if(motor.commutation_num > BLDC_OPEN_LOOP_WAIT)
			{
				// 清空计数器
				motor.commutation_failed_num = 0;
				// 清除一些变量
				motor.commutation_num = 0;
				stall_time_out_check = 0;
				// 打开比较器中断
				comparator_open_isr();
				// 使能定时器4中断
				IE2 |= 0x40;
				// 使能定时器4
				T4T3M |= 0x80;
				// 切换为闭环状态
				motor.run_flag = MOTOR_CLOSE_LOOP;
			}
			
			// 超过20ms没有检测到跳变，则从进入电机开始状态
			if(((T4H << 8) | T4L) > 20*1000)
			{
				T4T3M &= ~0x80; // 停止定时器
				T4L = 0;
				T4H = 0;
				T4T3M |= 0x80;  // 开启定时器
				// 电机step加一
				motor_next_step();
				// 电机换相
				pwm_x_output[motor.step]();
				motor.run_flag = MOTOR_START;
			}
		}
		break;
		case MOTOR_CLOSE_LOOP:
		{
			pin_state = (CMPCR1 & 0x01);
			// 通过PA2引脚的电平状态，进行堵转检测
			if(oled_pin_state != pin_state)
			{
				stall_time_out_check = 0;
				oled_pin_state = pin_state;
			}
			else
			{
				// 如果引脚一直是一个状态，且超过了 BLDC_STALL_TIME_OUT ms 则认为堵转了
				if(stall_time_out_check++ > (10 * 200))          // 10KHZ，0.1ms计数一次
				{
					stall_time_out_check = 0;
					motor.run_flag = MOTOR_STOP_STALL;
				}
			}
			
//			// 闭环缓慢加速		
//			if(motor.commutation_num < (BLDC_CLOSE_LOOP_WAIT))
//            {
//                 if (motor.duty_register < BLDC_MIN_DUTY)
//                 {
//                     motor.duty_register = BLDC_MIN_DUTY;
//                 }
//                 if (motor.duty_register > (BLDC_PWM_ARR_MAX / 10 + BLDC_MIN_DUTY))
//                 {
//                     motor.duty_register = BLDC_PWM_ARR_MAX / 10 + BLDC_MIN_DUTY;
//                 }
//            }
			

			// 缓慢加减速
			// BLDC_PWM_ARR_MAX = 833
			// 50us进一次PIT中断，一毫秒进20次。
			// 如果每一次进来，就修改一次duty的值，那么41.65ms就能从0%拉到100%
            // 等待稳定。
            if((BLDC_CLOSE_LOOP_WAIT) < motor.commutation_num)
			if((motor.duty_register != motor.duty) && (pit_count % BLDC_SPEED_INCREMENTAL == 0))
			{
				if(motor.duty > motor.duty_register)
				{
					motor.duty_register ++;
					if(BLDC_PWM_ARR_MAX < motor.duty_register)
					{
						motor.duty_register = BLDC_PWM_ARR_MAX;
					}
				}
				else
				{
					if(motor.duty_register > 0)
					{
						motor.duty_register--;
					}
				}
				
//				// 只有在减速的时候打开互补PWM
//				if((int16)((int16)motor.duty_register - (int16)motor.duty) >= (BLDC_PWM_ARR_MAX >> 4))
//				{
//					g_use_complementary = 1;
//				}
//				else
//				{
//					g_use_complementary = 0;
//				}
				
				pwm_out_duty_update(motor.duty_register);
			}
			
	
		}
		break;
		case MOTOR_STOP_STALL:
		{
			motor_stop();
			IE2 &= ~0x40;				// 关闭定时器4中断
			comparator_close_isr();		// 关闭比较器中断
			motor.run_flag = MOTOR_RESTART;
			motor.restart_delay = BLDC_START_DELAY;
		}
		break;
		case MOTOR_RESTART:
		{
			if(motor.restart_delay)
			{
				// 延时启动时间减减
				motor.restart_delay--;
			}
			else
			{
				motor.run_flag = MOTOR_IDLE;
			}
		}
		break;
		case MOTOR_IDLE:
		case BYTE_LOW_VOLTAGE:
		{
			ET0 = 0; 					// 关闭定时器0中断
			IE2 &= ~0x40;			  	// 关闭定时器4中断
			comparator_close_isr();		// 关闭比较器中断
			motor_stop();
		}
		break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      定时器1中断
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void TM1_Isr() interrupt 3
{
    pit_count++;
    if(battery_voltage_get())
    {
        motor.run_flag = BYTE_LOW_VOLTAGE;
    }
    // 电池电压检测为最高优先级，当电压过低，就不转
    if(motor.run_flag != BYTE_LOW_VOLTAGE)
    {
        // 输入的占空比为0，则为空闲状态
        if(motor.duty == 0)
        {
            motor.run_flag = MOTOR_IDLE;
        }
        else
        {
            // 输入的占空比不为0，则从空闲状态转为电机开始
            if(motor.run_flag == MOTOR_IDLE)
            {
                motor.run_flag = MOTOR_START;
            }
        }
    }
    led_control();
    pit_motor_control();
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      周期定时器初始化
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
   void pit_timer_init(void)
{
//	uint8 freq_div = 0;                
//    uint16 period_temp = 0;               
//    uint16 temp = 0;
//	uint16 tim_us = 50;		// 设置为0.5ms中断一次,200hz频率

//	AUXR |= 0x40;													// 设置为1T模式
//	freq_div = ((tim_us * system_clock / 1000000) / (1 << 16));          // 计算预分频
//	period_temp = ((tim_us * system_clock / 1000000) / (freq_div));  // 计算自动重装载值
//	temp = (uint16)65536 - period_temp;

//	TM1PS = freq_div;	// 设置分频值
//	TMOD |= 0x00; 		// 模式 0
//	TL1 = temp;
//	TH1 = temp >> 8;
//	TR1 = 1; 			// 启动定时器
//	ET1 = 1; 			// 使能定时器中断
	pit_us_init(TIM1_PIT, 50);
	pit_count = 0;
    motor.run_flag = MOTOR_IDLE;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      LED初始化
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void led_init(void)
{
	gpio_init(IO_P36, GPO, 0, GPO_PUSH_PULL);
	gpio_init(IO_P37, GPO, 0, GPO_PUSH_PULL);
    
    system_delay_ms(20);
    
    RUN_LED = 1;
    ERR_LED = 1;
}
