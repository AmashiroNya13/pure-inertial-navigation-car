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

#ifndef __ZF_DRIVER_UART_H
#define __ZF_DRIVER_UART_H


#include "zf_common_typedef.h"
#include "zf_driver_timer.h"
#include "zf_driver_gpio.h"


#define	UART1_CLEAR_RX_FLAG (SCON  &= ~0x01)
#define	UART2_CLEAR_RX_FLAG (S2CON &= ~0x01)
#define	UART3_CLEAR_RX_FLAG (S3CON &= ~0x01)
#define	UART4_CLEAR_RX_FLAG (S4CON &= ~0x01)

#define	UART1_CLEAR_TX_FLAG (SCON  &= ~0x02)
#define	UART2_CLEAR_TX_FLAG (S2CON &= ~0x02)
#define	UART3_CLEAR_TX_FLAG (S3CON &= ~0x02)
#define	UART4_CLEAR_TX_FLAG (S4CON &= ~0x02)


#define UART1_GET_RX_FLAG   (SCON  & 0x01)
#define UART2_GET_RX_FLAG   (S2CON & 0x01)
#define UART3_GET_RX_FLAG   (S3CON & 0x01)
#define UART4_GET_RX_FLAG   (S4CON & 0x01)

#define UART1_GET_TX_FLAG   (SCON  & 0x02)
#define UART2_GET_TX_FLAG   (S2CON & 0x02)
#define UART3_GET_TX_FLAG   (S3CON & 0x02)
#define UART4_GET_TX_FLAG   (S4CON & 0x02)


//此枚举定义不允许用户修改
typedef enum //枚举串口号
{
    UART_1,
    UART_2,
    UART_3,
    UART_4,
	UART_RESERVE,	//仅占位使用
} uart_index_enum;


typedef enum
{
    UART_FUNCTION_INIT = 0,                  // 功能未初始化    
    UART_FUNCTION_UART,                    	 // 用作 UART
    UART_FUNCTION_SPI,                       // 用作 SPI
}uart_function_enum;



//此枚举定义不允许用户修改
typedef enum //枚举串口引脚
{
    UART1_RX_P30 = 0x0000 | IO_P30, UART1_TX_P31 = 0x0000 | IO_P31,		//只能使用同一行的RX和TX引脚号。不允许混用
    UART1_RX_P36 = 0x0100 | IO_P36, UART1_TX_P37 = 0x0100 | IO_P37,		//例如:UART1_RX_P30,UART1_TX_P37。这样不行。
    UART1_RX_P16 = 0x0200 | IO_P16, UART1_TX_P17 = 0x0200 | IO_P17,
    UART1_RX_P43 = 0x0300 | IO_P43, UART1_TX_P44 = 0x0300 | IO_P44,

    UART2_RX_P12 = 0x1000 | IO_P12, UART2_TX_P13 = 0x1000 | IO_P13,
    UART2_RX_P42 = 0x1100 | IO_P42, UART2_TX_P43 = 0x1100 | IO_P43,

    UART3_RX_P00 = 0x2000 | IO_P00, UART3_TX_P01 = 0x2000 | IO_P01,
    UART3_RX_P50 = 0x2100 | IO_P50, UART3_TX_P51 = 0x2100 | IO_P51,

    UART4_RX_P02 = 0x3000 | IO_P02, UART4_TX_P03 = 0x3000 | IO_P03,
    UART4_RX_P52 = 0x3100 | IO_P52, UART4_TX_P53 = 0x3100 | IO_P53,
} uart_pin_enum;



typedef struct
{
	uint32 dma_urxt_cfg;
	uint32 dma_urxt_cr;
	uint32 dma_urxt_sta;
	uint32 dma_urxt_amt;
	uint32 dma_urxt_done;
	uint32 dma_urxt_txah;
	uint32 dma_urxt_txal;
	
	uint32 dma_urxt_amth;

	
	uint32 dma_urxr_cfg;
	uint32 dma_urxr_cr;
	uint32 dma_urxr_sta;
	uint32 dma_urxr_amt;
	uint32 dma_urxr_done;
	uint32 dma_urxr_rxah;
	uint32 dma_urxr_rxal;

	uint32 dma_urxr_amth;
	
}uart_addr_struct;




extern uint8 xdata uart_rx_buff[UART_RESERVE][1];

uint8 	uart_funciton_check 	(uart_index_enum index, uart_function_enum mode);
void 	uart_rx_start_buff		(uart_index_enum uart_n);

void    uart_write_byte         (uart_index_enum uart_n, const uint8 dat);
void    uart_write_buffer       (uart_index_enum uart_n, const uint8 *buff, uint16 len);
void    uart_write_string       (uart_index_enum uart_n, const char *str);

uint8   uart_read_byte          (uart_index_enum uart_n);
uint8   uart_query_byte         (uart_index_enum uart_n, uint8 *dat);

//void    uart_tx_interrupt       (uart_index_enum uart_n, uint8 status); 暂不支持TX中断
void    uart_rx_interrupt       (uart_index_enum uart_n, uint8 status);

void    uart_init               (uart_index_enum uart_n, uint32 baud, uart_pin_enum tx_pin, uart_pin_enum rx_pin);


#endif