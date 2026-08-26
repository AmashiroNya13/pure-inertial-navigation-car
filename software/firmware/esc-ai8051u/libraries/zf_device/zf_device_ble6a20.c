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
/*********************************************************************************************************************
* 接线定义：
*                   ------------------------------------
*                   模块管脚            单片机管脚
*                   RX                  查看 zf_device_wireless_ble6a20.h 中 BLE6A20_RX_PIN  宏定义
*                   TX                  查看 zf_device_wireless_ble6a20.h 中 BLE6A20_TX_PIN  宏定义
*                   RTS                 查看 zf_device_wireless_ble6a20.h 中 BLE6A20_RTS_PIN 宏定义
*                   VCC                 3.3V电源
*                   GND                 电源地
*                   其余引脚悬空
*                   ------------------------------------
*********************************************************************************************************************/

#include "zf_device_ble6a20.h"
#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_common_fifo.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_uart.h"
#include "zf_device_type.h"

#pragma warning disable = 183
#pragma warning disable = 177

static  fifo_struct                                     ble6a20_fifo;
static  uint8                                           ble6a20_buffer[BLE6A20_BUFFER_SIZE];

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蓝牙转串口模块 发送数据
// 参数说明     data            8bit 数据
// 返回参数     uint32          剩余发送长度 0-发送完毕 1-未发送完成
// 使用示例     ble6a20_send_byte(data);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint32 ble6a20_send_byte (const uint8 dat)
{
    uint16 time_cnt = BLE6A20_TIMEOUT_COUNT;
    
    while (time_cnt && gpio_get_level(BLE6A20_RTS_PIN)) 
	{
        system_delay_ms(1);
        time_cnt--;
    }
    
    if (time_cnt) 
	{
        uart_write_byte(BLE6A20_INDEX, dat);
    }
    
    return time_cnt == 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蓝牙转串口模块 发送数据块
// 参数说明     *buff           发送缓冲区
// 参数说明     len             发送数据长度
// 返回参数     uint32          剩余发送长度
// 使用示例     ble6a20_send_buffer(buff, 64);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint32 ble6a20_send_buffer (const uint8 *buff, uint32 len)
{
    #define PKT_SIZE 64
    #define DELAY_MS 1
    
    uint16 time_cnt = 0;
	uint16 send_len = 0;
    zf_assert(NULL != buff);  // 确保缓冲区非空
    
    while(len && time_cnt < BLE6A20_TIMEOUT_COUNT)
    {
        if(!gpio_get_level(BLE6A20_RTS_PIN))	// RTS低电平，可发送
        {
            send_len = len < PKT_SIZE ? len : PKT_SIZE;
            uart_write_buffer(BLE6A20_INDEX, buff, send_len);
            buff += send_len;
            len -= send_len;
            time_cnt = 0;	 // 重置超时计数
        }
		else	// RTS高电平，模块忙
		{
			system_delay_ms(DELAY_MS);
			time_cnt++;
		}
    }
    
    return len;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蓝牙转串口模块 发送字符串
// 参数说明     *str            要发送的字符串地址
// 返回参数     uint32          剩余发送长度
// 使用示例     ble6a20_send_string("Believe in yourself.");
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint32 ble6a20_send_string (const char *str)
{
    uint32 len = strlen(str);
    zf_assert(NULL != str);
	
    return  ble6a20_send_buffer(str, len);
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蓝牙转串口模块 读取缓冲
// 参数说明     *buff           接收缓冲区
// 参数说明     len             读取数据长度
// 返回参数     uint32          实际读取数据长度
// 使用示例     ble6a20_read_buffer(buff, 32);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint32 ble6a20_read_buffer (uint8 *buff, uint32 len)
{
    uint32 data_len = len;
    zf_assert(NULL != buff);
    fifo_read_buffer(&ble6a20_fifo, buff, &data_len, FIFO_READ_AND_CLEAN);
    return data_len;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蓝牙转串口模块 串口中断回调函数
// 参数说明     void
// 返回参数     void
// 使用示例     ble6a20_callback();
// 备注信息     该函数在 ISR 文件 串口中断程序被调用
//              由串口中断服务函数调用 wireless_module_uart_handler() 函数
//              再由 wireless_module_uart_handler() 函数调用本函数
//-------------------------------------------------------------------------------------------------------------------
void ble6a20_callback (uint8 uart_dat)
{
//    uart_query_byte(BLE6A20_INDEX, &ble6a20_data);
    fifo_write_buffer(&ble6a20_fifo, &uart_dat, 1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     蓝牙转串口模块 初始化
// 参数说明     void
// 返回参数     void
// 使用示例     ble6a20_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 ble6a20_init (void)
{
    uint8 return_state = 0;
    
    // 等待模块初始化
    system_delay_ms(50);
    
    set_wireless_type(BLE6A20, BLE6A20_INDEX, ble6a20_callback);
    
    fifo_init(&ble6a20_fifo, FIFO_DATA_8BIT, ble6a20_buffer, BLE6A20_BUFFER_SIZE);
    gpio_init(BLE6A20_RTS_PIN, GPIO, GPIO_HIGH, GPIO_NO_PULL);
    
    // 本函数使用的波特率为115200 为蓝牙转串口模块的默认波特率 如需其他波特率请自行配置模块并修改串口的波特率
    uart_init (BLE6A20_INDEX, BLE6A20_BUAD_RATE, BLE6A20_RX_PIN, BLE6A20_TX_PIN);   // 初始化串口
    uart_rx_interrupt(BLE6A20_INDEX, 1);
    
    return return_state;
}
