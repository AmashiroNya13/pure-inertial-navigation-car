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

#ifndef _pwm_out_h_
#define _pwm_out_h_


#include "zf_common_typedef.h"

#define PWM_A_H_PIN     P00
#define PWM_A_L_PIN     P01
                         
#define PWM_B_H_PIN     P02
#define PWM_B_L_PIN     P03
                         
#define PWM_C_H_PIN     P04
#define PWM_C_L_PIN     P05
typedef void(pwm_x_output_func)(void);
extern pwm_x_output_func* pwm_x_output[6];


extern uint8 g_use_complementary;

void pwm_brake(void);
void pwm_close_output(void);
void pwm_a_bn_output();
void pwm_a_cn_output();
void pwm_b_cn_output();
void pwm_b_an_output();
void pwm_c_an_output();
void pwm_c_bn_output();
void pwm_isr_close(void);
void pwm_isr_open(void);
void pwm_out_duty_update(uint16 duty);
void pwm_out_init(void);

#endif
