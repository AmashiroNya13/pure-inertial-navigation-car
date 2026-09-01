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

#include "intrins.h"
#include "zf_driver_gpio.h"
#include "bldc_config.h"
#include "comparator.h"
#include "pwm_out.h"


uint8 g_use_complementary = 0;



pwm_x_output_func* pwm_x_output[6] = 
{
    pwm_a_bn_output,
    pwm_a_cn_output,
    pwm_b_cn_output,
    pwm_b_an_output,
    pwm_c_an_output,
    pwm_c_bn_output
};



//-------------------------------------------------------------------------------------------------------------------
//  @brief      软延时
//  @param      void                        
//  @return     void          
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void delay_500ns(void)
{
	_nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
	_nop_();
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      刹车（关闭输出并开启所有下桥）
//  @param      void                        
//  @return     void          
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void pwm_brake(void)
{
    PWMA_ENO = 0;
    PWM_A_L_PIN = 1;
    PWM_B_L_PIN = 1;
    PWM_C_L_PIN = 1;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      关闭输出
//  @param      void                        
//  @return     void          
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void pwm_close_output(void)
{
    PWM_A_H_PIN = 0;
    PWM_B_H_PIN = 0;
    PWM_C_H_PIN = 0;

    PWM_A_L_PIN = 0;
    PWM_B_L_PIN = 0;
    PWM_C_L_PIN = 0;
    PWMA_ENO = 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      开启A上B下
//  @param      void                        
//  @return     void          
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void pwm_a_bn_output()
{
#if BLDC_USR_COMPLEMENTARY
    PWMA_ENO = 0x03;
#else
    PWMA_ENO = 0x01;
#endif

    PWM_A_L_PIN = 0;
    PWM_C_L_PIN = 0;
    PWM_B_L_PIN = 1;
    
    CMP_SELECT_C;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      开启A上C下
//  @param      void                        
//  @return     void          
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void pwm_a_cn_output()
{
#if BLDC_USR_COMPLEMENTARY
    PWMA_ENO = 0x03;
#else
    PWMA_ENO = 0x01;
#endif

    PWM_A_L_PIN = 0;
    PWM_B_L_PIN = 0;
    PWM_C_L_PIN = 1;
    
    CMP_SELECT_B;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      开启B上C下
//  @param      void                        
//  @return     void          
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void pwm_b_cn_output()
{
#if BLDC_USR_COMPLEMENTARY
    PWMA_ENO = 0x0C;
#else
    PWMA_ENO = 0x04;
#endif
    
    PWM_A_L_PIN = 0;
    PWM_B_L_PIN = 0;
    PWM_C_L_PIN = 1;
    
    CMP_SELECT_A;

}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      开启B上A下
//  @param      void                        
//  @return     void          
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void pwm_b_an_output()
{
#if BLDC_USR_COMPLEMENTARY
    PWMA_ENO = 0x0C;
#else
    PWMA_ENO = 0x04;
#endif
    
    PWM_B_L_PIN = 0;
    PWM_C_L_PIN = 0;
    PWM_A_L_PIN = 1;
    
    CMP_SELECT_C;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      开启C上A下
//  @param      void                        
//  @return     void          
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void pwm_c_an_output()
{	
#if BLDC_USR_COMPLEMENTARY
    PWMA_ENO = 0x30;
#else
    PWMA_ENO = 0x10;
#endif
    
    PWM_B_L_PIN = 0;
    PWM_C_L_PIN = 0;
    PWM_A_L_PIN = 1;
    
    CMP_SELECT_B;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      开启C上B下
//  @param      void                        
//  @return     void          
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void pwm_c_bn_output()
{
#if BLDC_USR_COMPLEMENTARY
    PWMA_ENO = 0x30;
#else
    PWMA_ENO = 0x10;
#endif
    
    PWM_A_L_PIN = 0;
    PWM_C_L_PIN = 0;
    PWM_B_L_PIN = 1;
    
    CMP_SELECT_A;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      更新PWM占空比
//  @param      void                        
//  @return     void          
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void pwm_out_duty_update(uint16 duty)
{
    uint8 temp_h, temp_l;

    temp_h = (duty >> 8) & 0xFF;
    temp_l = (uint8)duty;
    
    PWMA_CCR1H = temp_h;
    PWMA_CCR1L = temp_l;
    
    PWMA_CCR2H = temp_h;
    PWMA_CCR2L = temp_l;
    
    PWMA_CCR3H = temp_h;
    PWMA_CCR3L = temp_l;

}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      PWM初始化（中心对齐方式）
//  @param      void                        
//  @return     void          
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void pwm_out_init(void)
{
    PWM_A_H_PIN = 0;
	PWM_A_L_PIN = 0;
	PWM_B_H_PIN = 0;
	PWM_B_L_PIN = 0;
	PWM_C_H_PIN = 0;
	PWM_C_L_PIN = 0;
    
	PWMA_ENO = 0;
    
	PWMA_CCER1  = 0;
	PWMA_CCER2  = 0;
	PWMA_SR1    = 0;
	PWMA_SR2    = 0;
    PWMA_ENO    = 0;
	PWMA_IER    = 0;
    
    // 设置PWM引脚
    PWMA_PS     = 0x55;
    
    PWMA_CCMR1  = 0x78;		// 通道模式配置, PWM模式2, 预装载允许
	PWMA_CCR1H  = 0;
    PWMA_CCR1L  = 0;
	PWMA_CCER1 |= 0x0F;		// 开启比较输出, 低电平有效
    
	PWMA_CCMR2  = 0x78;		// 通道模式配置, PWM模式2, 预装载允许
	PWMA_CCR2H  = 0;
    PWMA_CCR2L  = 0;
	PWMA_CCER1 |= 0xF0;		// 开启比较输出, 低电平有效
    
    PWMA_CCMR3  = 0x78;		// 通道模式配置, PWM模式2, 预装载允许
	PWMA_CCR3H  = 0;
    PWMA_CCR3L  = 0;
	PWMA_CCER2 |= 0x0F;		// 开启比较输出, 低电平有效
   
//    PWMA_IER    = 0x10;     // 开启更新中
    //PWMA_ENO = 0xff;//0X15;
    
    // 预分频
    PWMA_PSCRH = 0;
    PWMA_PSCRL = 0;
    
    // 设置周期
    PWMA_ARRH = (uint8)((BLDC_PWM_ARR_MAX-1) >> 8);
    PWMA_ARRL = (uint8)((BLDC_PWM_ARR_MAX-1) & 0xff);
    
    PWMA_DTR    = BLDC_PWM_DEADTIME;
    PWMA_BKR    = 0x80;		// 主输出使能 相当于总开关
	PWMA_CR1    = 0x85;		// 使能计数器, 允许自动重装载寄存器缓冲, 边沿对齐, 向上计数, 只有计数器上下溢出才触发更新中断,  bit7=1:写自动重装载寄存器缓冲(本周期不会被打扰), =0:直接写自动重装载寄存器本(周期可能会乱掉)
	PWMA_EGR    = 0x01;		// 产生一次更新事件, 清除计数器和与分频计数器, 装载预分频寄存器的值
    
//    // 刚开始使用刹车将
//    PWM_A_L_PIN = 1;
//    PWM_B_L_PIN = 1;
//    PWM_C_L_PIN = 1;
}
