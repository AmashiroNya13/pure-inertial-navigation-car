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

#ifndef _motor_control_h_
#define _motor_control_h_

#include "zf_common_typedef.h"




typedef struct
{
    vuint8   step;                   			// 电机运行的步骤
    uint16  duty;                   			// PWM占空比       用户改变电机速度时修改此变量
    uint16  duty_register;          			// PWM占空比寄存器 用户不可操作
    vuint8  run_flag;               			// 电机正在运行标志位 0:已停止 1：正在启动 2：切入闭环之后再维持一段运行时间 3：启动完成正在运行
    float   motor_start_delay;      			// 开环启动的换相的延时时间
    uint16  motor_start_wait;       			// 开环启动时，换相时间已经降低到最小值后，统计换相的次数
    uint16  restart_delay;          			// 电机延时重启
    uint16  commutation_failed_num; 			// 换相错误次数
    uint16  commutation_time[6];    			// 最近6次换相时间
    uint32  commutation_time_sum;   			// 最近6次换相时间总和
    uint32  commutation_num;        			// 统计换相次数
    uint32  filter_commutation_time_sum;        // 滤波后的换相时间
}motor_struct;



typedef enum
{
    MOTOR_IDLE = 0,
    MOTOR_START,
    MOTOR_OPEN_LOOP,
    MOTOR_CLOSE_LOOP ,
    MOTOR_STOP_STALL,
    MOTOR_RESTART,

    BYTE_LOW_VOLTAGE,

}run_state_enum;



extern  motor_struct motor;


void motor_power_on_beep(uint16 volume);
void motor_stop(void);
void motor_init(void);
void motor_commutation_isr_open(void);

void motor_next_step(void);
//void motor_commutation(void);





#endif 