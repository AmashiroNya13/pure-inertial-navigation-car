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


#ifndef _conparator_h_
#define _conparator_h_

#include "zf_common_typedef.h"

//-------------------------------------------------------------------------------------------------------------------
//  @brief      比较器选择A通道为输入
//  @param      void                        
//  @return     void          
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
#define CMP_SELECT_A    (CMPEXCFG = 0x00)


//-------------------------------------------------------------------------------------------------------------------
//  @brief      比较器选择B通道为输入
//  @param      void                        
//  @return     void          
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
#define CMP_SELECT_B    (CMPEXCFG = 0x01)

//-------------------------------------------------------------------------------------------------------------------
//  @brief      比较器选择C通道为输入
//  @param      void                        
//  @return     void          
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
#define CMP_SELECT_C    (CMPEXCFG = 0x02)

extern uint16 motor_commutation_time;

void  comparator_rising(void);
void  comparator_falling(void);
uint8 comparator_result_get(void);
void  comparator_close_isr(void);
void  comparator_open_isr(void);
void  comparator_init(void);

#endif
