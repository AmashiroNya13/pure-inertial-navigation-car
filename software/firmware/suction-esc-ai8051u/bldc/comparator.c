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

#include "zf_driver_gpio.h"
#include "comparator.h"
#include "motor_control.h"

#define COMPARATOR_MID_PIN  IO_P44

#define COMPARATOR_A_PIN    IO_P46
#define COMPARATOR_B_PIN    IO_P50
#define COMPARATOR_C_PIN    IO_P51


#define CMP_DC_LAG    (0 << 6)

//-------------------------------------------------------------------------------------------------------------------
//  @brief      设置比较器上升沿触发中断
//  @param      void                        
//  @return     void          
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void comparator_rising(void)
{
    // 上升沿
    CMPCR1 = 0xA0 | 0x02;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      设置比较器下降沿触发中断
//  @param      void                        
//  @return     void          
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void comparator_falling(void)
{
    //下降沿
    CMPCR1 = 0x90 | 0x02;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取比较结果
//  @param      void                        
//  @return     void          
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint8 comparator_result_get(void)
{
    return (CMPCR1&0x01);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      关闭比较器中断
//  @param      void                        
//  @return     void          
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void comparator_close_isr(void)
{
    CMPCR1 = 0x80 | 0x02;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      打开比较器中断
//  @param      void                        
//  @return     void          
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void comparator_open_isr(void)
{
	// 需软件清除中断标志位
	CMPCR1 &= ~0x40;	
	
	// 打开比较器中断
	if(motor.step % 2)
	{
		comparator_rising();
	}
	else
	{
		comparator_falling();
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      比较器初始化
//  @param      void                        
//  @return     void          
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void comparator_init(void)
{
    gpio_init(IO_P45, GPI, 0, GPI_IMPEDANCE);

    gpio_init(COMPARATOR_MID_PIN, GPI, 0, GPI_IMPEDANCE); // 中性点
    gpio_init(COMPARATOR_A_PIN  , GPI, 0, GPI_IMPEDANCE); // A
	gpio_init(COMPARATOR_B_PIN  , GPI, 0, GPI_IMPEDANCE); // B
	gpio_init(COMPARATOR_C_PIN  , GPI, 0, GPI_IMPEDANCE); // C
    
    CMPCR1 = 0x8C | 1 << 1;	// 1000 1100 
	CMPCR2 = 63 | 1 << 6;	// 63个时钟滤波，关闭0.1uf滤波电容
    CMPO_S = 1;             // 比较器输出脚选择位 0->P45 1->P41

}

