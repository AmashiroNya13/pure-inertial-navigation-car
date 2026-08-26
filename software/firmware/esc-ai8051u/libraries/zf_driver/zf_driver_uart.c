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

#include "stdlib.h"
#include "zf_common_clock.h"
#include "zf_common_debug.h"

#include "zf_driver_uart.h"
#include "zf_driver_timer.h"
#include "zf_driver_gpio.h"

uint8 xdata uart_rx_buff[UART_RESERVE][1] = {0};

#define DMA_URXT_CFG(uart_n)		(*(unsigned char volatile far *)(0x7efa30 + uart_n*0x10))
#define DMA_URXT_CR(uart_n)     	(*(unsigned char volatile far *)(0x7efa31 + uart_n*0x10))
#define DMA_URXT_STA(uart_n)    	(*(unsigned char volatile far *)(0x7efa32 + uart_n*0x10))
#define DMA_URXT_AMT(uart_n)    	(*(unsigned char volatile far *)(0x7efa33 + uart_n*0x10))
#define DMA_URXT_DONE(uart_n)   	(*(unsigned char volatile far *)(0x7efa34 + uart_n*0x10))
#define DMA_URXT_TXAH(uart_n)   	(*(unsigned char volatile far *)(0x7efa35 + uart_n*0x10))
#define DMA_URXT_TXAL(uart_n)   	(*(unsigned char volatile far *)(0x7efa36 + uart_n*0x10))
#define DMA_URXR_CFG(uart_n)    	(*(unsigned char volatile far *)(0x7efa38 + uart_n*0x10))
#define DMA_URXR_CR(uart_n)     	(*(unsigned char volatile far *)(0x7efa39 + uart_n*0x10))
#define DMA_URXR_STA(uart_n)    	(*(unsigned char volatile far *)(0x7efa3a + uart_n*0x10))
#define DMA_URXR_AMT(uart_n)    	(*(unsigned char volatile far *)(0x7efa3b + uart_n*0x10))
#define DMA_URXR_DONE(uart_n)   	(*(unsigned char volatile far *)(0x7efa3c + uart_n*0x10))
#define DMA_URXR_RXAH(uart_n)   	(*(unsigned char volatile far *)(0x7efa3d + uart_n*0x10))
#define DMA_URXR_RXAL(uart_n)   	(*(unsigned char volatile far *)(0x7efa3e + uart_n*0x10))

#define DMA_URXT_AMTH(uart_n)   	(*(unsigned char volatile far *)(0x7efa88 + uart_n*0x04))
#define DMA_URXT_DONEH(uart_n)  	(*(unsigned char volatile far *)(0x7efa89 + uart_n*0x04))
#define DMA_URXR_AMTH(uart_n)   	(*(unsigned char volatile far *)(0x7efa8a + uart_n*0x04))
#define DMA_URXR_DONEH(uart_n)  	(*(unsigned char volatile far *)(0x7efa8b + uart_n*0x04))


// 该数组禁止修改，内部使用,用户无需关心
static uart_function_enum uart_function_state[4] =
{
    UART_FUNCTION_INIT,
    UART_FUNCTION_INIT,
    UART_FUNCTION_INIT,
    UART_FUNCTION_INIT,
};

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART 外设确认功能状态 库内部调用
// 参数说明     index           UART 外设模块号
// 参数说明     mode            需要确的功能模块
// 返回参数     uint8           1-可以使用 0-不可以使用
// 使用示例     zf_assert(uart_funciton_check(UART_2, UART_FUNCTION_UART));
//-------------------------------------------------------------------------------------------------------------------
uint8 uart_funciton_check (uart_index_enum index, uart_function_enum mode)
{
    uint8 return_state = 1;
    if(UART_FUNCTION_INIT == uart_function_state[index])
    {
        uart_function_state[index] = mode;
    }
    else if(uart_function_state[index] == mode)
    {
        return_state = 1;
    }
    else
    {
        return_state = 0;
    }
    return return_state;
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     串口发送数组
// 参数说明     uart_n       串口通道
// 参数说明     buff        要发送的数组地址
// 参数说明     len         数据长度
// 返回参数     void
// 使用示例     uart_write_buffer(UART_1, buff, 10);     //串口1发送10个buff数组。
//-------------------------------------------------------------------------------------------------------------------
void uart_write_buffer(uart_index_enum uart_n, const uint8 *buff, uint16 len)
{
    #define BUFF_LEN 64
	
	// 因UART_DMA只能操作xdata区域的数据，所以，这里新建一个数组，搬移。
	uint8 xdata tmp_buff[BUFF_LEN] = {0};
    uint16 tmp_len;
	while(len)
	{
        tmp_len = (len > BUFF_LEN) ? BUFF_LEN : len;        // 计算长度
        memcpy(tmp_buff, buff, tmp_len);                    // 拷贝
        len -= tmp_len;                                     // 去掉已经发送的长度
        buff += tmp_len;                                    // 指针指向后面
        
        //	DMA_URXT_CFG(uart_n)  = 0x00; 		            // DMA优先级低
        DMA_URXT_STA(uart_n) = 0;				            // 清空标志位

        DMA_URXT_AMT(uart_n)  = (tmp_len - 1) & 0xff;		// 设置传输总字节数(低8位)：n+1
        DMA_URXT_AMTH(uart_n) = (tmp_len - 1) >> 8;		    // 设置传输总字节数(高8位)：n+1
        DMA_URXT_TXAH(uart_n) = (uint8)((uint16)tmp_buff >> 8);
        DMA_URXT_TXAL(uart_n) = (uint8)((uint16)tmp_buff);
        DMA_URXT_CR(uart_n) = 0xC0; 			            // 使能DMA TX功能

        while(!(DMA_URXT_STA(uart_n) & 0x01));	            // 等待发送完成

        DMA_URXT_CR(uart_n) = 0x00;				            // 关闭DMA TX
	}
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     串口发送字符串
// 参数说明     uart_n       串口通道
// 参数说明     str         字符串首地址
// 返回参数     void
// 使用示例     uart_putstr(UART_1, (uint8 *)"12345")   //串口1发送12345这个字符串
//-------------------------------------------------------------------------------------------------------------------
void uart_write_string(uart_index_enum uart_n, const char *str)
{
    uint16 len = strlen(str);
    uart_write_buffer(uart_n, str, len);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     串口发送一个字节
// 参数说明     uart_n       串口通道
// 参数说明     dat         串口数据
// 返回参数     void
// 使用示例     uart_write_byte(UART_1, 0x43);        //串口1发送0x43。
//-------------------------------------------------------------------------------------------------------------------
void uart_write_byte(uart_index_enum uart_n, const uint8 dat)
{
    uart_write_buffer(uart_n, &dat, 1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     串口开始接收数据
// 参数说明     uart_n       串口通道
// 返回参数     void
// 使用示例     uart_rx_start_buff(UART_1);
//-------------------------------------------------------------------------------------------------------------------
void uart_rx_start_buff(uart_index_enum uart_n)
{
    DMA_URXR_AMT(uart_n) = (1 - 1);										// 设置接收的字节数
    DMA_URXR_AMTH(uart_n) = (1 - 1) >> 8;								// 设置接收的字节数
    DMA_URXR_RXAL(uart_n) = (uint8)((uint16)uart_rx_buff[uart_n]);		// 设置接收缓冲地址
    DMA_URXR_RXAH(uart_n) = (uint8)((uint16)uart_rx_buff[uart_n] >> 8);	// 设置接收缓冲地址
    DMA_URXR_CR(uart_n) = 0xA1;											// 开启DMA RX接收中断，开启DMA RX接收，清空FIFO
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     打开串口接收中断
// 参数说明     uart_n       串口通道
// 参数说明     status      使能或者失能
// 返回参数     void
// 使用示例     uart_rx_irq(UART_1, ENABLE);        //打开串口1接收中断
//-------------------------------------------------------------------------------------------------------------------
void uart_rx_interrupt (uart_index_enum uart_n, uint8 status)
{
	if(status)
	{
		DMA_URXR_CFG(uart_n) |= 0x80;
	}
	else
	{
		DMA_URXR_CFG(uart_n) &= ~0x80;
	} 
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取串口接收的数据（whlie等待）
// 参数说明     uart_n           串口模块号(UART_1 - UART_4)
// 参数说明     *dat            接收数据的地址
// 返回参数     void
// 使用示例     dat = uart_read_byte(USART_1);       // 接收串口1数据  存在在dat变量里
//-------------------------------------------------------------------------------------------------------------------
uint8 uart_read_byte(uart_index_enum uart_n)
{
	uint8 dat;
	
	// 等待串口有数据
	while(!(DMA_URXR_STA(uart_n)&0x03));

	// 读取数据
	dat = uart_rx_buff[uart_n][0];
		
	// 清空标志位
	DMA_URXR_STA(uart_n) = 0x00;
	
	// 开始下一次接收
	uart_rx_start_buff(uart_n);
	return dat;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取串口接收的数据（查询接收）
// 参数说明     uart_n           串口模块号(UART_1 - UART_8)
// 参数说明     *dat            接收数据的地址
// 返回参数     uint8           1：接收成功   0：未接收到数据
// 使用示例     uint8 dat; uart_query_byte(USART_1,&dat);       // 接收串口1数据  存在在dat变量里
//-------------------------------------------------------------------------------------------------------------------
uint8 uart_query_byte(uart_index_enum uart_n, uint8 *dat)
{
	uint8 ret = 0; 

	// 读取数据
	*dat = uart_rx_buff[uart_n][0];
	
	if(DMA_URXR_STA(uart_n) & 0x03)
	{
		ret = 1;
		
		DMA_URXR_STA(uart_n) &= ~0x03;
	}
	else
	{
		ret = 0;
	}
	
	// 开始下一次接收
	uart_rx_start_buff(uart_n);

	return ret;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     串口DMA初始化
// 参数说明     uart_n       串口通道
// 返回参数     void
// 使用示例     uart_dma_init(UART_1);
//-------------------------------------------------------------------------------------------------------------------
void uart_dma_init(uart_index_enum uart_n)
{
	DMA_URXT_CFG(uart_n)  = 0x00;	// DMA TX数据访问优先级最低，关闭DMA发送中断，
	DMA_URXT_STA(uart_n)  = 0x00;	// 清除DMA TX状态
	DMA_URXT_CR(uart_n)   = 0x00;	// 关闭DMA TX
		
	DMA_URXR_CFG(uart_n)  = 0x00;	// 关闭DMA接收中断
	DMA_URXR_STA(uart_n)  = 0x00;	// 清除DMA RX状态
	DMA_URXR_CR(uart_n)   = 0x00;	// 关闭DMA RX

	DMA_URXR_AMT(uart_n)  = (1 - 1);									// 设置接收的字节数
	DMA_URXR_AMTH(uart_n) = (1 - 1)>>8;									// 设置接收的字节数
	DMA_URXR_RXAL(uart_n) = (uint8)((uint16)uart_rx_buff[uart_n]);		// 设置接收缓冲地址
	DMA_URXR_RXAH(uart_n) = (uint8)((uint16)uart_rx_buff[uart_n] >> 8);	// 设置接收缓冲地址
	DMA_URXR_CFG(uart_n)  = 0x0F;										// 中断优先级最高，DMA优先级最高
	DMA_URXR_CR(uart_n)   = 0xA1;										// 开启DMA RX，清空FIFO
 
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     串口初始化
// 参数说明     uart_n       串口通道
// 参数说明     baud        波特率
// 参数说明     tx_pin      串口发送引脚号
// 参数说明     rx_pin      串口接收引脚号
// 返回参数     void
// 使用示例     uart_init(UART_2, 115200, UART2_TX_P11, UART2_RX_P10); //串口2初始化引脚号,TX为P11,RX为P10
// 备注信息     所有的串口，都只能使用定时器2 做波特率发生器，所有的串口只能是同一个波特率。
//-------------------------------------------------------------------------------------------------------------------
void uart_init(uart_index_enum uart_n, uint32 baud, uart_pin_enum tx_pin, uart_pin_enum rx_pin)
{
    uint16 brt;
    
	// 如果程序在输出了断言信息 并且提示出错位置在这里
    // 就去查看你在什么地方调用这个函数 检查你的传入参数
    // 这里是检查是否有重复使用UART1 和 UART2功能
    // 比如初始化了 UART1 然后又初始化成 SPI1 这种用法是不允许的
	// UART1和SPI1使用同一个寄存器，要么用UART1要么使用SPI1,只能是二选一。
	// UART2和SPI2使用同一个寄存器，要么用UART2要么使用SPI2,只能是二选一。
    zf_assert(uart_funciton_check(uart_n, UART_FUNCTION_UART));
	
	    
	// 如果程序在输出了断言信息 并且提示出错位置在这里
    // 就去查看你在什么地方调用这个函数 检查你的传入参数
    // 这里是检查是否有重复使用定时器
	// TIM2已经给串口用作波特率发生器了。不能再初始化为其他的。
	zf_assert(timer_funciton_check(TIM_2, TIMER_FUNCTION_UART));
	
	
	// 串口的RX和TX必须是一组切换，如果在这里进行报错，
	// 则说明你选到不是同一组的引脚了
	zf_assert(((tx_pin >> 8) & 0x0f) == ((rx_pin >> 8) & 0x0f));
	
	// 初始化GPIO
	gpio_init(tx_pin&0xFF, GPO, 1, GPO_PUSH_PULL);
	gpio_init(rx_pin&0xFF, GPI, 1, GPI_FLOATING_IN);
	
    brt = (uint16)(65536 - (system_clock / (baud + 2) / 4));
    
    switch(uart_n)
    {
        case UART_1:
        {
            //            if(TIM_1 == tim_n)
            //            {
            //                SCON |= 0x50;
            //                TMOD |= 0x00;
            //                TL1 = brt;
            //                TH1 = brt >> 8;
            //                AUXR |= 0x40;
            //                TR1 = 0;	//关闭发送中断
            //
            //            }
            //            else if(TIM_2 == tim_n)
            {
                SCON |= 0x50;
                T2L = brt;
                T2H = brt >> 8;
                AUXR |= 0x15;
                TR1 = 0;	//关闭发送中断
            }
            
            P_SW1 &= ~(0x03 << 6);
            
            if((UART1_RX_P30 == rx_pin) && (UART1_TX_P31 == tx_pin))
            {
                P_SW1 |= 0x00;
            }
            else if((UART1_RX_P36 == rx_pin) && (UART1_TX_P37 == tx_pin))
            {
                P_SW1 |= 0x40;
            }
            else if((UART1_RX_P16 == rx_pin) && (UART1_TX_P17 == tx_pin))
            {
                P_SW1 |= 0x80;
            }
            else if((UART1_RX_P43 == rx_pin) && (UART1_TX_P44 == tx_pin))
            {
                P_SW1 |= 0xc0;
            }
            
            //            ES = 1;	//允许串行口1中断
            break;
        }
        
        case UART_2:
        {
            //            if(TIM_2 == tim_n)
            {
                S2CON |= 0x50;
                T2L = brt;
                T2H = brt >> 8;
                AUXR |= 0x14;
            }
            
            P_SW2 &= ~(0x01 << 0);
            
            if((UART2_RX_P12 == rx_pin) && (UART2_TX_P13 == tx_pin))
            {
                P_SW2 |= 0x00;
            }
            else if((UART2_RX_P42 == rx_pin) && (UART2_TX_P43 == tx_pin))
            {
                P_SW2 |= 0x01;
            }
            
            //            IE2 |= 0x01 << 0;	//允许串行口2中断
            
            break;
        }
        
        case UART_3:
        {
            //            if(TIM_2 == tim_n)
            {
                S3CON |= 0x10;
                T2L = brt;
                T2H = brt >> 8;
                AUXR |= 0x14;
            }
            //            else if(TIM_3 == tim_n)
            //            {
            //                S3CON |= 0x50;
            //                T3L = brt;
            //                T3H = brt >> 8;
            //                T4T3M |= 0x0a;
            //            }
            
            P_SW2 &= ~(0x01 << 1);
            
            if((UART3_RX_P00 == rx_pin) && (UART3_TX_P01 == tx_pin))
            {
                P_SW2 |= 0x00;
            }
            else if((UART3_RX_P50 == rx_pin) && (UART3_TX_P51 == tx_pin))
            {
                P_SW2 |= 0x02;
            }
            
            //            IE2 |= 0x01 << 3;	//允许串行口3中断
            
            break;
        }
        
        case UART_4:
        {
            //            if(TIM_2 == tim_n)
            {
                S4CON |= 0x10;
                T2L = brt;
                T2H = brt >> 8;
                AUXR |= 0x14;
            }
            //            else if(TIM_4 == tim_n)
            //            {
            //                S4CON |= 0x50;
            //                T4L = brt;
            //                T4H = brt >> 8;
            //                T4T3M |= 0xa0;
            //            }
            
            P_SW2 &= ~(0x01 << 2);
            
            if((UART4_RX_P02 == rx_pin) && (UART4_TX_P03 == tx_pin))
            {
                P_SW2 |= 0x00;
            }
            else if((UART4_RX_P52 == rx_pin) && (UART4_TX_P53 == tx_pin))
            {
//                P5M0 = 0x00;
//                P5M1 = 0x01 << 2; //P5.2 需要设置为高阻
                P_SW2 |= 0x04;
            }
            
            //            IE2 |= 0x01 << 4;	//允许串行口4中断
            
            break;
        }
        
    }
    
	// uart dma 初始化
//    uart_dma_init(uart_n);
    
}
