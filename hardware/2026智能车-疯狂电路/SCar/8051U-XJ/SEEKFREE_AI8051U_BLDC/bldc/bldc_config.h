/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file        		pit_timer
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ790875685)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date        	2024-01-22
 ********************************************************************************************************************/

#ifndef _bldc_config_h_
#define _bldc_config_h_


// 40000000 / 43333 = 923.08
// 40000000 为系统频率40Mhz
// 43333为43.333Khz

#define BLDC_PWM_ARR_MAX                (923)           // PWM的最大占空比，需要自行根据PWM初始化去计算

#define BLDC_MAX_DUTY                   (15)            // 启动最大占空比，单位百分之一

#define BLDC_MIN_DUTY                   (5)             // 启动最小占空比，单位百分之一

#define BLDC_PWM_DEADTIME               (5)             // PWM输出死区

#define BLDC_MIN_BATTERY                (11000)         // 最小电压，单位mv，当检测到电压低于此值电机停转
    
#define BLDC_COMMUTATION_FAILED_MAX     (300)           // 换相错误最大次数 大于这个次数后认为电机堵转
    
#define BLDC_START_DELAY                (10000)         // 当遇到堵转后电机停止时间，之后电机会再次尝试启动（单位0.05ms）

#define BLDC_CLOSE_LOOP_WAIT            (200 )          // 闭环等待换向次数

#define BLDC_OPEN_LOOP_WAIT             (200 )          // 闭环等待换向次数

#define BLDC_SPEED_INCREMENTAL          (4)            // 加减速响应 1-20，可修改这个提高加速响应，
                                                        // 设置20响应速度最慢，设置1响应速度最快。
                                                        // 响应速度越快，越容易出现换相错误，所以建议从20开始，一点一点减小。

#define BLDC_POLES                      (6)             // 电机极对数

#define BLDC_BEEP_ENABLE                (1 )            // 1:使能上电电机鸣叫功能 0：禁用
    
#define BLDC_BEEP_VOLUME                (60)            // 电机鸣叫声音大小 0-100

#define BLDC_MOTOR_ANGLE                (15)             // 马达进角，进入比较器中断后，延时多少度进行换相
                                                        // 范围0-30度

#define BLDC_USE_SINE_START             (1)             // 使用正弦启动

#define BLDC_USR_COMPLEMENTARY          (1)             // 1:使能互补输出 0：禁用

#define BLDC_USR_DUTY                   (0)             // 目标占空比来源：0=外部 PWM 捕获；1~100=宏固定百分比
                                                        // 0        = 由输入捕获换算油门，无有效 PWM 时占空比清零
                                                        // 1~100    = 忽略外部 PWM，目标占空比恒为本宏数值（1%~100%），便于调试

#endif
