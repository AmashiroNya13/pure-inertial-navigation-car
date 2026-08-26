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

#include "zf_driver_adc.h"
#include "zf_driver_gpio.h"

#include "bldc_config.h"
#include "battery.h"


#define	ADC_CH		1						/* 1~16, ADC转换通道数, 需同步修改 DMA_ADC_CHSW 转换通道 */
#define	ADC_DATA	6						/* 6~n, 每个通道ADC转换数据总数, 2*转换次数+4, 需同步修改 DMA_ADC_CFG2 转换次数 */
#define	DMA_ADDR	0x800					/* DMA数据存放地址 */
static uint8 xdata adc_dma_buff[ADC_CH][ADC_DATA] _at_ DMA_ADDR;

uint16 battery_voltage;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      电池电压获取
//  @param      void                        
//  @return     uint8 	0-正常 1-电池电压过低          
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint8 battery_voltage_get(void)
{
    uint16 pin_voltage;
    static uint32 low_power_num = 0;
    uint16 adc_reg_value;
	
    adc_reg_value = adc_dma_buff[0][0];
    pin_voltage = (uint32)adc_reg_value * 5000 / 256;       // 将ADC值转换为实际的电压
    battery_voltage = (uint32)pin_voltage * 57 / 10;    	// 根据硬件分压电阻的值计算电池电压
	
	// 开启下一次ADC_DMA转换
	DMA_ADC_CR = 0xc0;
	
    if((BLDC_MIN_BATTERY > battery_voltage))
    {
        low_power_num++;
        if((2000 ) < (low_power_num / 20))		// 2s都超过电压保护值，则进行停机保护
        {
           return 1;   
        }
    }
    else
    {
        low_power_num = 0;
    }

	return 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      电池电压检测初始化
//  @param      void                        
//  @return     void          
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void battery_init(void)
{
	
	gpio_init(IO_P15, GPI, 0, GPI_IMPEDANCE);
	
	ADC_CONTR = 0x80;						// ADC使能
	ADCTIM = 0x3f;  						// 设置通道选择时间、保持时间、采样时间
	ADCCFG = 0x0F;							// 左对齐，ADC转换时间最大

	DMA_ADC_STA = 0x00;
	DMA_ADC_CFG = 0x0F;		
	DMA_ADC_RXAH = (uint8)(DMA_ADDR >> 8);	//ADC转换数据存储地址
	DMA_ADC_RXAL = (uint8)DMA_ADDR;
	DMA_ADC_CFG2 = 0x00;					// 每个通道ADC转换次数:1
	DMA_ADC_CHSW0 = 1<<5;				    // 
	DMA_ADC_CHSW1 = 0;					    // ADC通道使能寄存器
	DMA_ADC_CR = 0xc0;						// 使能ADC_DMA,ADC开始转换。
	
    // 初始化的时候先采集一次电压
    battery_voltage_get();
}