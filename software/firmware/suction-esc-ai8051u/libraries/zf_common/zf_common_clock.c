/*********************************************************************************************************************
* AI8051U Opensourec Library 即（AI8051U 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是STC 开源库的一部分
*
* AI8051U 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK FOR C251
* 适用平台          AI8051U
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2024-08-01        大W            first version
********************************************************************************************************************/

#include "zf_common_typedef.h"
#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_common_interrupt.h"

#include "zf_driver_delay.h"

//内核频率
int32 system_clock = SYSTEM_CLOCK_40M;


#define T22M_ADDR CHIPID11 //22.1184MHz
#define T24M_ADDR CHIPID12 //24MHz
#define T27M_ADDR CHIP ID13 //27MHz

#define T30M_ADDR CHIPID14 //30MHz
#define T33M_ADDR CHIPID15 //33.1776MHz
#define T35M_ADDR CHIPID16 //35MHz
#define T36M_ADDR CHIPID17 //36.864MHz
#define T40M_ADDR CHIPID18 //40MHz
#define T44M_ADDR CHIPID19 //4 4.2368 MHz
#define T48M_ADDR CHIPID20 //48 MHz
#define VRT6M_ADDR CHIPID21 //VRTRIM_6M
#define VRT10M_ADDR CHIPID22 //VRTRIM_10M
#define VRT27M_ADDR CHIPID23 //VRTRIM_27M
#define VRT44M_ADDR CHIPID24 //VRTRIM_44M


void clock_init (uint32 clock)                                              // 核心时钟初始化
{
	P_SW2 = 0x80;           // 开启特殊地址访问
    CKCON = 0x00;           // 设置外部数据总线为最快
    WTST = 0;               // 设置程序代码等待参数，赋值为0可将CPU执行程序的速度设置为最快



//	USBCLK &= ~(3 << 5);
//	USBCLK |= (2 << 5);
//	USBCLK |= (1 << 7);
//	_nop_();
//    _nop_();
//    _nop_();
//    _nop_();
//    _nop_();
//	CLKSEL |= (1 << 7);
//	CLKSEL |= (1 << 6);
//	MCLKOCR = 0x01;
//	IRC48MCR = 0x80;
//	while (!(IRC48MCR & 1));
//	IRCBAND &= ~(3 << 6);
//	IRCBAND |= (2 << 6);
//	DMAIR = 0x3F; 		//TPU时钟 120M
//	HSCLKDIV = 0x00;

    
//	// 外部晶振
//    XOSCCR = 0xc0;          // 启动外部晶振
//    while (!(XOSCCR & 1));  // 等待时钟稳定
//    CLKDIV = 0x00;          // 时钟不分频
//    CLKSEL = 0x01;          // 选择外部晶振
//	CLKDIV = 0x00;          // 40MHz主频，分频设置
//	

//    // 选择44.2368Mhz
//    CLKDIV = 0x04;
//    IRTRIM = T44M_ADDR;
//    VRTRIM = VRT44M_ADDR;
//    IRCBAND |= 0x03;
//    CLKDIV = 0x00;
    


	system_clock = clock;
    

    
//    P0M0 = 0x00;
//    P0M1 = 0x00;
//    P1M0 = 0x00;
//    P1M1 = 0x00;
//    P2M0 = 0x00;
//    P2M1 = 0x00;
//    P3M0 = 0x00;
//    P3M1 = 0x00;
//    P4M0 = 0x00;
//    P4M1 = 0x00;
//    P5M0 = 0x00;
//    P5M1 = 0x00;
//    P6M0 = 0x00;
//    P6M1 = 0x00;
//    P7M0 = 0x00;
//    P7M1 = 0x00;
	
	system_delay_init();    // 延时函数初始化
    
    ADCCFG = 0x00;
    AUXR = 0x00;
    SCON = 0x00;
    S2CON = 0x00;
    S3CON = 0x00;
    S4CON = 0x00;
	P_SW1 = 0x00;
	P_SW3 = 0x00;
    IE2 = 0x00;
    TMOD = 0x00;
	PWMA_ENO = 0x00;
	PWMB_ENO = 0x00;
	
	HSCLKDIV = 0x00;
	SPI_CLKDIV = 0x00;
	
	
	interrupt_global_enable();
}


