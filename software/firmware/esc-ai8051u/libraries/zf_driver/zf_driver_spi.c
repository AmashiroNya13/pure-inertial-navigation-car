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
#pragma warning disable = 47

#include "zf_common_debug.h"
#include "zf_common_clock.h"
#include "zf_driver_gpio.h"
#include "zf_driver_spi.h"

#pragma warning disable = 183
#pragma warning disable = 177

/*
#define SPI0_WRITE_DAT(dat) 		\
		SPSTAT = 0xc0;              \
		SPDAT = dat;				\
		while (!(SPSTAT & 0x80));

#define SPI1_WRITE_DAT(dat) 		\
		TI = 0x0;              		\
		SBUF = dat;					\
		while (!TI)

#define SPI2_WRITE_DAT(dat) 		\
		S2TI = 0x0;              	\
		S2BUF = dat;				\
		while (!S2TI)
		
#define SPI_WRITE_DAT(spi_n, dat)	\
	switch (spi_n)					\
	{                               \
		case SPI_0:                 \
			SPI0_WRITE_DAT(dat);    \
			break;                  \
		case SPI_1:                 \
			SPI1_WRITE_DAT(dat);    \
			break;                  \
		case SPI_2:                 \
			SPI2_WRITE_DAT(dat);    \
			break;                  \
		default:                    \
			zf_assert(0);           \
			break;                  \
	}

*/

#define SPI_WRITE_DAT(spi_n, dat) spi_write_dat(spi_n, dat)
void spi_write_dat(spi_index_enum spi_n, const uint8 dat)
{
	switch (spi_n)					
	{                               
		case SPI_0:                 
			SPSTAT = 0xc0;              
			SPDAT = dat;				
			while (!(SPSTAT & 0x80));  
			break;
		case SPI_1:                 
			TI = 0x0;              		
			SBUF = dat;					
			while (!TI);
			break;                  
		case SPI_2:
			S2TI = 0x0;              	
			S2BUF = dat;				
			while (!S2TI);
			break;          
		default:                    
			zf_assert(0);           
			break;                  
	}
}
	
#define SPI_READ_DAT(spi_n, dat) spi_read_dat(spi_n, dat)
uint8 spi_read_dat(spi_index_enum spi_n, const uint8 dat)
{
	uint8 ret = 0xFF;
	switch (spi_n)					
	{                               
		case SPI_0:                 
			SPSTAT = 0xc0;              
			SPDAT = dat;				
			while (!(SPSTAT & 0x80));  
			ret = SPDAT;
			break;
		case SPI_1:                 
			TI = 0x0;              		
			SBUF = dat;					
			while (!TI);
			ret = SBUF;
			break;                  
		case SPI_2:
			S2TI = 0x0;              	
			S2BUF = dat;				
			while (!S2TI);
			ret = S2BUF;
			break;          
		default:                    
			zf_assert(0);           
			break;                  
	}
	
	return ret;
}



//-------------------------------------------------------------------------------------------------------------------
// 函数简介      SPI 接口写 8bit 数据
// 参数说明     spi_n           SPI 模块号 参照 zf_driver_spi.h 内 spi_index_enum 枚举体定义
// 参数说明     data            数据
// 返回参数     void
// 使用示例     spi_write_8bit(SPI_1,0x11);
//-------------------------------------------------------------------------------------------------------------------
void spi_write_8bit (spi_index_enum spi_n, const uint8 dat)
{
	SPI_WRITE_DAT(spi_n, dat);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      SPI 接口写 8bit 数组
// 参数说明     spi_n           SPI 模块号 参照 zf_driver_spi.h 内 spi_index_enum 枚举体定义
// 参数说明     *data           数据存放缓冲区
// 参数说明     len             缓冲区长度
// 返回参数     void
// 使用示例     spi_write_8bit_array(SPI_1,data,64);
//-------------------------------------------------------------------------------------------------------------------
void spi_write_8bit_array (spi_index_enum spi_n, const uint8 *dat, uint32 len)
{
	while(len--)
	{
		SPI_WRITE_DAT(spi_n, *dat++);
	}
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      SPI 接口写 16bit 数据
// 参数说明     spi_n           SPI 模块号 参照 zf_driver_spi.h 内 spi_index_enum 枚举体定义
// 参数说明     data            数据
// 返回参数     void
// 使用示例     spi_write_16bit(SPI_1,0x1101);
//-------------------------------------------------------------------------------------------------------------------
void spi_write_16bit (spi_index_enum spi_n, const uint16 dat)
{
	SPI_WRITE_DAT(spi_n, (uint8)((dat & 0xFF00) >> 8));
	SPI_WRITE_DAT(spi_n, (uint8)(dat & 0x00FF));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      SPI 接口写 16bit 数组
// 参数说明     spi_n           SPI 模块号 参照 zf_driver_spi.h 内 spi_index_enum 枚举体定义
// 参数说明     *data           数据存放缓冲区
// 参数说明     len             缓冲区长度
// 返回参数     void
// 使用示例     spi_write_16bit_array(SPI_1,data,64);
//-------------------------------------------------------------------------------------------------------------------
void spi_write_16bit_array (spi_index_enum spi_n, const uint16 *dat, uint32 len)
{
    while(len--)
    {
		SPI_WRITE_DAT(spi_n, (uint8)((*dat & 0xFF00) >> 8));
		SPI_WRITE_DAT(spi_n, (uint8)(*dat++ & 0x00FF));
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      SPI 接口向传感器的寄存器写 8bit 数据
// 参数说明     spi_n           SPI 模块号 参照 zf_driver_spi.h 内 spi_index_enum 枚举体定义
// 参数说明     register_name   寄存器地址
// 参数说明     data            数据
// 返回参数     void
// 使用示例     spi_write_8bit_register(SPI_1,0x11,0x01);
//-------------------------------------------------------------------------------------------------------------------
void spi_write_8bit_register (spi_index_enum spi_n, const uint8 register_name, const uint8 dat)
{
	SPI_WRITE_DAT(spi_n, register_name);
	SPI_WRITE_DAT(spi_n, dat);
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介      SPI 接口向传感器的寄存器写 8bit 数组
// 参数说明     spi_n           SPI 模块号 参照 zf_driver_spi.h 内 spi_index_enum 枚举体定义
// 参数说明     register_name   寄存器地址
// 参数说明     *data           数据存放缓冲区
// 参数说明     len             缓冲区长度
// 返回参数     void
// 使用示例     spi_write_8bit_registers(SPI_1,0x11,data,32);
//-------------------------------------------------------------------------------------------------------------------
void spi_write_8bit_registers (spi_index_enum spi_n, const uint8 register_name, const uint8 *dat, uint32 len)
{
	SPI_WRITE_DAT(spi_n, register_name);
    while(len--)
    {
		SPI_WRITE_DAT(spi_n, *dat++);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     SPI 接口向传感器的寄存器写 16bit 数据
// 参数说明     spi_n           SPI 模块号 参照 zf_driver_spi.h 内 spi_index_enum 枚举体定义
// 参数说明     register_name   寄存器地址
// 参数说明     data            数据
// 返回参数     void
// 使用示例     spi_write_16bit_register(SPI_1,0x1011,0x0101);
//-------------------------------------------------------------------------------------------------------------------
void spi_write_16bit_register (spi_index_enum spi_n, const uint16 register_name, const uint16 dat)
{
	SPI_WRITE_DAT(spi_n, (uint8)((register_name & 0xFF00) >> 8));
	SPI_WRITE_DAT(spi_n, (uint8)(register_name & 0x00FF));
	SPI_WRITE_DAT(spi_n, (uint8)((dat & 0xFF00) >> 8));
	SPI_WRITE_DAT(spi_n, (uint8)(dat & 0x00FF));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      SPI 接口向传感器的寄存器写 16bit 数组
// 参数说明     spi_n           SPI 模块号 参照 zf_driver_spi.h 内 spi_index_enum 枚举体定义
// 参数说明     register_name   寄存器地址
// 参数说明     *data           数据存放缓冲区
// 参数说明     len             缓冲区长度
// 返回参数     void
// 使用示例     spi_write_16bit_registers(SPI_1,0x1011,data,32);
//-------------------------------------------------------------------------------------------------------------------
void spi_write_16bit_registers (spi_index_enum spi_n, const uint16 register_name, const uint16 *dat, uint32 len)
{
	SPI_WRITE_DAT(spi_n, (uint8)((register_name & 0xFF00) >> 8));
	SPI_WRITE_DAT(spi_n, (uint8)(register_name & 0x00FF));
	SPI_WRITE_DAT(spi_n, (uint8)((*dat & 0xFF00) >> 8));
	SPI_WRITE_DAT(spi_n, (uint8)(*dat++ & 0x00FF));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      SPI 接口读 8bit 数据
// 参数说明     spi_n           SPI 模块号 参照 zf_driver_spi.h 内 spi_index_enum 枚举体定义
// 参数说明     register_name   寄存器地址
// 返回参数     uint8         数据
// 使用示例     spi_read_8bit(SPI_1);
//-------------------------------------------------------------------------------------------------------------------
uint8 spi_read_8bit (spi_index_enum spi_n)
{
    return SPI_READ_DAT(spi_n, 0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      SPI 接口读 8bit 数组
// 参数说明     spi_n           SPI 模块号 参照 zf_driver_spi.h 内 spi_index_enum 枚举体定义
// 参数说明     *data           数据存放缓冲区
// 参数说明     len             发送缓冲区长度
// 返回参数     void
// 使用示例     spi_read_8bit_array(SPI_1,data,64);
//-------------------------------------------------------------------------------------------------------------------
void spi_read_8bit_array (spi_index_enum spi_n, uint8 *dat, uint32 len)
{
    while(len--)
    {
        *dat++ = SPI_READ_DAT(spi_n, 0);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      SPI 接口读 16bit 数据
// 参数说明     spi_n           SPI 模块号 参照 zf_driver_spi.h 内 spi_index_enum 枚举体定义
// 参数说明     register_name   寄存器地址
// 返回参数     uint16        数据
// 使用示例     spi_read_16bit(SPI_1);
//-------------------------------------------------------------------------------------------------------------------
uint16 spi_read_16bit (spi_index_enum spi_n)
{
    uint16 dat = 0;

    dat = SPI_READ_DAT(spi_n, 0);

    dat = ((dat << 8) | SPI_READ_DAT(spi_n, 0));
    
    return dat;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      SPI 接口读 16bit 数组
// 参数说明     spi_n           SPI 模块号 参照 zf_driver_spi.h 内 spi_index_enum 枚举体定义
// 参数说明     *data           数据存放缓冲区
// 参数说明     len             发送缓冲区长度
// 返回参数     void
// 使用示例     spi_read_16bit_array(SPI_1,data,64);
//-------------------------------------------------------------------------------------------------------------------
void spi_read_16bit_array (spi_index_enum spi_n, uint16 *dat, uint32 len)
{
    while(len--)
    {
        *dat = SPI_READ_DAT(spi_n, 0);
        *dat = ((*dat << 8) | SPI_READ_DAT(spi_n, 0));
        dat++;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      SPI 接口从传感器的寄存器读 8bit 数据
// 参数说明     spi_n           SPI 模块号 参照 zf_driver_spi.h 内 spi_index_enum 枚举体定义
// 参数说明     register_name   寄存器地址
// 返回参数     uint8         数据
// 使用示例     spi_read_8bit_register(SPI_1,0x11);
//-------------------------------------------------------------------------------------------------------------------
uint8 spi_read_8bit_register (spi_index_enum spi_n, const uint8 register_name)
{
	SPI_WRITE_DAT(spi_n, register_name);
    return SPI_READ_DAT(spi_n, 0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      SPI 接口从传感器的寄存器读 8bit 数组
// 参数说明     spi_n           SPI 模块号 参照 zf_driver_spi.h 内 spi_index_enum 枚举体定义
// 参数说明     register_name   寄存器地址
// 参数说明     *data           数据存放缓冲区
// 参数说明     len             发送缓冲区长度
// 返回参数     void
// 使用示例     spi_read_8bit_registers(SPI_1,0x11,data,32);
//-------------------------------------------------------------------------------------------------------------------
void spi_read_8bit_registers (spi_index_enum spi_n, const uint8 register_name, uint8 *dat, uint32 len)
{
	SPI_WRITE_DAT(spi_n, register_name);
    while(len--)
    {
        *dat++ = SPI_READ_DAT(spi_n, 0);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      SPI 接口从传感器的寄存器读 16bit 数据
// 参数说明     spi_n           SPI 模块号 参照 zf_driver_spi.h 内 spi_index_enum 枚举体定义
// 参数说明     register_name   寄存器地址
// 返回参数     uint16        数据
// 使用示例     spi_read_16bit_register(SPI_1,0x1011);
//-------------------------------------------------------------------------------------------------------------------
uint16 spi_read_16bit_register (spi_index_enum spi_n, const uint16 register_name)
{
    uint16 dat = 0;
    
	SPI_WRITE_DAT(spi_n, (uint8)((register_name & 0xFF00) >> 8));
	SPI_WRITE_DAT(spi_n, (uint8)(register_name & 0x00FF));
	
	dat = SPI_READ_DAT(spi_n, 0);
	dat = ((dat << 8) |  SPI_READ_DAT(spi_n, 0));

    return dat;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      SPI 接口从传感器的寄存器读 16bit 数组
// 参数说明     spi_n           SPI 模块号 参照 zf_driver_spi.h 内 spi_index_enum 枚举体定义
// 参数说明     register_name   寄存器地址
// 参数说明     *dat           数据存放缓冲区
// 参数说明     len             发送缓冲区长度
// 返回参数     void
// 使用示例     spi_read_16bit_registers(SPI_1,0x1101,dat,32);
//-------------------------------------------------------------------------------------------------------------------
void spi_read_16bit_registers (spi_index_enum spi_n, const uint16 register_name, uint16 *dat, uint32 len)
{
	SPI_WRITE_DAT(spi_n, (uint8)((register_name & 0xFF00) >> 8));
	SPI_WRITE_DAT(spi_n, (uint8)(register_name & 0x00FF));
    
    while(len--)
    {
		*dat = SPI_READ_DAT(spi_n, 0);
		*dat = ((*dat << 8) |  SPI_READ_DAT(spi_n, 0));
        dat++;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      SPI 8bit 数据传输 发送与接收数据是同时进行的
// 参数说明     spi_n           SPI 模块号 参照 zf_driver_spi.h 内 spi_index_enum 枚举体定义
// 参数说明     write_buffer    发送的数据缓冲区地址
// 参数说明     read_buffer     发送数据时接收到的数据的存储地址(不需要接收则传 NULL)
// 参数说明     len             缓冲区长度
// 返回参数     void
// 使用示例     spi_transfer_8bit(SPI_1,buf,buf,1);
//-------------------------------------------------------------------------------------------------------------------
void spi_transfer_8bit (spi_index_enum spi_n, const uint8 *write_buffer, uint8 *read_buffer, uint32 len)
{
    while(len--)
    {
        if(read_buffer != NULL)
        {
            *read_buffer++ = SPI_READ_DAT(spi_n, *(write_buffer++));
        }
		else
		{
			SPI_WRITE_DAT(spi_n, *(write_buffer++));
		}
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      SPI 16bit 数据传输 发送与接收数据是同时进行的
// 参数说明     spi_n           SPI 模块号 参照 zf_driver_spi.h 内 spi_index_enum 枚举体定义
// 参数说明     write_buffer    发送的数据缓冲区地址
// 参数说明     read_buffer     发送数据时接收到的数据的存储地址(不需要接收则传 NULL)
// 参数说明     len             缓冲区长度
// 返回参数     void
// 使用示例     spi_transfer_16bit(SPI_1,buf,buf,1);
//-------------------------------------------------------------------------------------------------------------------
void spi_transfer_16bit (spi_index_enum spi_n, const uint16 *write_buffer, uint16 *read_buffer, uint32 len)
{
    while(len--)
    {
		if(read_buffer != NULL)
        {
            *read_buffer = SPI_READ_DAT(spi_n, (uint8)((*write_buffer & 0xFF00) >> 8));
			*read_buffer = *read_buffer << 8 | (uint8)(SPI_READ_DAT(spi_n, (*write_buffer & 0x00FF)));
			read_buffer++;
        }
		else
		{
			SPI_WRITE_DAT(spi_n, (uint8)((*write_buffer & 0xFF00) >> 8));
			SPI_WRITE_DAT(spi_n, (uint8)(*write_buffer & 0x00FF));
		}
        write_buffer++;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     SPI 接口初始化
// 参数说明     spi_n           SPI 模块号 参照 zf_driver_spi.h 内 spi_index_enum 枚举体定义
// 参数说明     mode            SPI 模式 参照 zf_driver_spi.h 内 spi_mode_enum 枚举体定义
// 参数说明     baud            设置 SPI 的波特率 不超过系统时钟的一半 部分速率会被适配成相近的速率
// 参数说明     sck_pin         选择 SCK 引脚 参照 zf_driver_spi.h 内 spi_pin_enum 枚举体定义
// 参数说明     mosi_pin        选择 MOSI 引脚 参照 zf_driver_spi.h 内 spi_pin_enum 枚举体定义
// 参数说明     miso_pin        选择 MISO 引脚 参照 zf_driver_spi.h 内 spi_pin_enum 枚举体定义
// 参数说明     cs_pin          选择 CS 引脚 参照 zf_driver_gpio.h 内 gpio_pin_enum 枚举体定义
// 返回参数     void
// 使用示例     spi_init(SPI_1, 0, 1*1000*1000, SPI1_SCK_A5, SPI1_MOSI_A7, SPI1_MISO_A6, A4);  //硬件SPI初始化  模式0 波特率为1Mhz
//-------------------------------------------------------------------------------------------------------------------
void spi_init(spi_index_enum spi_n, spi_mode_enum mode, uint32 baud, spi_pin_enum sck_pin, spi_pin_enum mosi_pin, spi_pin_enum miso_pin, gpio_pin_enum cs_pin)
{
    uint32 spi_psc = 0;
    
    if((sck_pin & 0xFF00) != (mosi_pin & 0xFF00))
    {
        zf_assert(0);
        // SPI只能使用同一组引脚，不允许混用引脚
    }
    
    gpio_init(sck_pin  & 0xFF, GPO, 1, GPO_PUSH_PULL);
    gpio_init(mosi_pin & 0xFF, GPO, 1, GPO_PUSH_PULL);
    
    if(miso_pin != SPI_NULL_PIN)
    {
        gpio_init(miso_pin & 0xFF, GPIO, 1, GPIO_NO_PULL);
    }
    
    if(cs_pin != SPI_NULL_PIN)
    {
        gpio_init(cs_pin & 0xFF, GPO, 0, GPO_PUSH_PULL);
    }

	// 波特率大于等于10M就需要设置GPIO电平转换速度
	if(baud >= (10*1000*1000U))
	{
		gpio_set_level_speed(sck_pin  & 0xFF, GPIO_SPEED_FAST);
		gpio_set_level_speed(mosi_pin  & 0xFF, GPIO_SPEED_FAST);
		if(miso_pin != SPI_NULL_PIN)
		{
			gpio_set_level_speed(miso_pin & 0xFF, GPIO_SPEED_FAST);
		}
		if(cs_pin != SPI_NULL_PIN)
		{
			gpio_set_level_speed(cs_pin & 0xFF, GPIO_SPEED_FAST);
		}
	}
    
    if(SPI_0 == spi_n)
    {
        P_SW1 &= ~(0x03 << 2); //清除SPI功能脚选择位
        
        switch((sck_pin >> 8) & 0x03)
        {
            case 0:
                P_SW1 |= (0x00 << 2);
                break;
                
            case 1:
                P_SW1 |= (0x01 << 2);
                break;
                
            case 2:
                P_SW1 |= (0x02 << 2);
                break;
                
            case 3:
                P_SW1 |= (0x03 << 2);
                break;
        }
        
        SPCTL = 0;
        SPCTL |= 1 << 7;	//忽略SS引脚功能，使用MSTR确定器件是主机还是从机
        SPCTL |= 1 << 4;	//主机模式
        
        // 设置SPI速率
        spi_psc = system_clock / baud;
		
        SPCTL &= ~(0x03 << 0);
		
		if(spi_psc > 8)
        {
            SPCTL |= 0x02;	// SPI输入时钟/16
        }
        else if(spi_psc > 4)
        {
            SPCTL |= 0x01;	// SPI输入时钟/8
        }
        else if(spi_psc > 2)
        {
            SPCTL |= 0x00;	// SPI输入时钟/4
        }
        else
        {
            SPCTL |= 0x03;	// SPI输入时钟/2
        }
		
        // 设置SPI极性和相位
        switch(mode)
        {
            case SPI_MODE0:
                break;
                
            case SPI_MODE1:
                SPCTL |= 0x01 << 2;
                break;
                
            case SPI_MODE2:
                SPCTL |= 0x02 << 2;
                break;
                
            case SPI_MODE3:
                SPCTL |= 0x03 << 2;
                break;
        }
        
        SPCTL |= 1 << 6;	// 使能SPI功能
    }
    else if(SPI_1 == spi_n)
    {
		
		// 如果程序在输出了断言信息 并且提示出错位置在这里
		// 就去查看你在什么地方调用这个函数 检查你的传入参数
		// 这里是检查是否有重复使用UART1 和 UART2功能
		// 比如初始化了 UART1 然后又初始化成 SPI1 这种用法是不允许的
		// UART1和SPI1使用同一个寄存器，要么用UART1要么使用SPI1,只能是二选一。
		// UART2和SPI2使用同一个寄存器，要么用UART2要么使用SPI2,只能是二选一。
		zf_assert(uart_funciton_check(UART_1, UART_FUNCTION_SPI));
		
		
        P_SW3 &= ~(0x03 << 2); //清除SPI功能脚选择位
        
        switch((sck_pin >> 8) & 0x03)
        {
            case 0:
                P_SW3 |= (0x00 << 2);
                break;
                
            case 1:
                P_SW3 |= (0x01 << 2);
                break;
                
            case 2:
                P_SW3 |= (0x02 << 2);
                break;
                
            case 3:
                P_SW3 |= (0x03 << 2);
                break;
        }
        
        SCON = 0x10;		// 接收使能
        USARTCR1 = 0x10; 	// 使能SPI模式，主机功能
        
        
        // 设置SPI速率
        spi_psc = system_clock / baud;
        

		
        if(spi_psc > 8)
        {
            USARTCR4 |= 0x02;	// SPI输入时钟/16
        }
        else if(spi_psc > 4)
        {
            USARTCR4 |= 0x01;	// SPI输入时钟/8
        }
        else if(spi_psc > 2)
        {
            USARTCR4 |= 0x00;	// SPI输入时钟/4
        }
        else
        {
            USARTCR4 |= 0x03;	// SPI输入时钟/2
        }
        
        
        // 设置SPI极性和相位
        switch(mode)
        {
            case SPI_MODE0:
                USARTCR1 |= 0x00;
                break;
                
            case SPI_MODE1:
                USARTCR1 |= 0x01;
                break;
                
            case SPI_MODE2:
                USARTCR1 |= 0x02;
                break;
                
            case SPI_MODE3:
                USARTCR1 |= 0x03;
                break;
        }
        
        USARTCR1 |= 0x08; // 使能SPI功能
    }
    else if(SPI_2 == spi_n)
    {
		
		// 如果程序在输出了断言信息 并且提示出错位置在这里
		// 就去查看你在什么地方调用这个函数 检查你的传入参数
		// 这里是检查是否有重复使用UART1 和 UART2功能
		// 比如初始化了 UART1 然后又初始化成 SPI1 这种用法是不允许的
		// UART1和SPI1使用同一个寄存器，要么用UART1要么使用SPI1,只能是二选一。
		// UART2和SPI2使用同一个寄存器，要么用UART2要么使用SPI2,只能是二选一。
		zf_assert(uart_funciton_check(UART_2, UART_FUNCTION_SPI));

        P_SW3 &= ~(0x03 << 4); //清除SPI功能脚选择位
        
        switch((sck_pin >> 8) & 0x03)
        {
            case 0:
                P_SW3 |= (0x00 << 4);
                break;
                
            case 1:
                P_SW3 |= (0x01 << 4);
                break;
                
            case 2:
                P_SW3 |= (0x02 << 4);
                break;
                
            case 3:
                P_SW3 |= (0x03 << 4);
                break;
        }
        
        S2CON = 0x10;		// 接收使能
        USART2CR1 = 0x10;	// 使能SPI模式，主机功能
        
        
        // 设置SPI速率
        spi_psc = system_clock / baud;

        if(spi_psc > 8)
        {
            USART2CR4 |= 0x02;	// SPI输入时钟/16
        }
        else if(spi_psc > 4)
        {
            USART2CR4 |= 0x01;	// SPI输入时钟/8
        }
        else if(spi_psc > 2)
        {
            USART2CR4 |= 0x00;	// SPI输入时钟/4
        }
        else
        {
            USART2CR4 |= 0x03;	// SPI输入时钟/2
        }
        
        
        // 设置SPI极性和相位
        switch(mode)
        {
            case SPI_MODE0:
                USART2CR1 |= 0x00;
                break;
                
            case SPI_MODE1:
                USART2CR1 |= 0x01;
                break;
                
            case SPI_MODE2:
                USART2CR1 |= 0x02;
                break;
                
            case SPI_MODE3:
                USART2CR1 |= 0x03;
                break;
        }
        
        USART2CR1 |= 0x08; // 使能SPI功能
    }
}



