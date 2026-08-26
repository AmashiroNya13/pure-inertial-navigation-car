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
* 接线定义:
*                   ------------------------------------
*                   模块管脚                                        单片机管脚
*                   //------------------硬件 SPI 引脚------------------//
*                   SCL/SPC             查看 zf_device_icm20602.h 中 ICM20602_SPC_PIN 宏定义
*                   SDA/DSI             查看 zf_device_icm20602.h 中 ICM20602_SDI_PIN 宏定义
*                   SA0/SDO             查看 zf_device_icm20602.h 中 ICM20602_SDO_PIN 宏定义
*                   CS                  查看 zf_device_icm20602.h 中 IPS114_CS_PIN 宏定义
*                   //------------------硬件 SPI 引脚------------------//
*                   //------------------软件 IIC 引脚------------------//
*                   SCL/SPC             查看 zf_device_icm20602.h 中 ICM20602_SCL_PIN 宏定义
*                   SDA/DSI             查看 zf_device_icm20602.h 中 ICM20602_SDA_PIN 宏定义
*                   //------------------软件 IIC 引脚------------------//
*                   电源引脚
*                   VCC                 3.3V电源
*                   GND                 电源地
*                   ------------------------------------
********************************************************************************************************************/

#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_driver_delay.h"
#include "zf_driver_spi.h"
#include "zf_driver_soft_iic.h"
#include "zf_driver_soft_spi.h"

#include "zf_device_icm20602.h"

#pragma warning disable = 183
#pragma warning disable = 177

int16 icm20602_gyro_x = 0, icm20602_gyro_y = 0, icm20602_gyro_z = 0;            // 三轴陀螺仪数据      gyro (陀螺仪)
int16 icm20602_acc_x = 0, icm20602_acc_y = 0, icm20602_acc_z = 0;               // 三轴加速度计数据    acc (accelerometer 加速度计)
float icm20602_transition_factor[2] = {4096, 16.4};

#if (ICM20602_USE_INTERFACE==HARDWARE_SPI)                               // 这两段 颜色正常的才是正确的 颜色灰的就是没有用的

	//-------------------------------------------------------------------------------------------------------------------
	// 函数简介     ICM20602 写寄存器
	// 参数说明     reg             寄存器地址
	// 参数说明     data            数据
	// 返回参数     void
	// 使用示例     icm20602_write_register(ICM20602_PWR_MGMT_1, 0x80);
	// 备注信息     内部调用
	//-------------------------------------------------------------------------------------------------------------------
	static void icm20602_write_register (uint8 reg, uint8 dat)
	{
		ICM20602_CS(0);
		spi_write_8bit_register(ICM20602_SPI, reg | ICM20602_SPI_W, dat);
		ICM20602_CS(1);
	}

	//-------------------------------------------------------------------------------------------------------------------
	// 函数简介     ICM20602 读寄存器
	// 参数说明     reg             寄存器地址
	// 返回参数     uint8           数据
	// 使用示例     icm20602_read_register(ICM20602_WHO_AM_I);
	// 备注信息     内部调用
	//-------------------------------------------------------------------------------------------------------------------
	static uint8 icm20602_read_register (uint8 reg)
	{
		uint8 dat = 0;
		ICM20602_CS(0);
		dat = spi_read_8bit_register(ICM20602_SPI, reg | ICM20602_SPI_R);
		ICM20602_CS(1);
		return dat;
	}

	//-------------------------------------------------------------------------------------------------------------------
	// 函数简介     ICM20602 读数据
	// 参数说明     reg             寄存器地址
	// 参数说明     data            数据缓冲区
	// 参数说明     len             数据长度
	// 返回参数     void
	// 使用示例     icm20602_read_registers(ICM20602_ACCEL_XOUT_H, dat, 6);
	// 备注信息     内部调用
	//-------------------------------------------------------------------------------------------------------------------
	static void icm20602_read_registers (uint8 reg, uint8 *dat, uint32 len)
	{
		ICM20602_CS(0);
		spi_read_8bit_registers(ICM20602_SPI, reg | ICM20602_SPI_R, dat, len);
		ICM20602_CS(1);
	}

#elif (ICM20602_USE_INTERFACE==SOFT_SPI)
	#define ICM20602_SCK(x)				(ICM20602_SPC_PIN 	= x)
	#define ICM20602_MOSI(x) 			(ICM20602_SDI_PIN 	= x)
	#define ICM20602_MISO    			(ICM20602_SDO_PIN	   )
	#define ICM20602_CS(x)              (ICM20602_CS_PIN 	= x)
	//-------------------------------------------------------------------------------------------------------------------
	//  @brief      通过SPI写一个byte,同时读取一个byte
	//  @param      byte        发送的数据
	//  @return     uint8 edata       return 返回status状态
	//  @since      v1.0
	//  Sample usage:
	//-------------------------------------------------------------------------------------------------------------------
	static uint8 icm20602_simspi_wr_byte(uint8 byte)
	{
		uint8 i;
		for(i=0; i<8; i++)
		{
			ICM20602_SCK (0);
			ICM20602_MOSI(byte&0x80);
			byte <<= 1;
			ICM20602_SCK (1);
			byte |= ICM20602_MISO;
		}
		return(byte);
	}
	
	//-------------------------------------------------------------------------------------------------------------------
	//  @brief      将val写入cmd对应的寄存器地址,同时返回status字节
	//  @param      cmd         命令字
	//  @param      val         待写入寄存器的数值
	//  @since      v1.0
	//  Sample usage:
	//-------------------------------------------------------------------------------------------------------------------
	static void icm20602_simspi_w_reg_byte(uint8 cmd, uint8 val)
	{
		cmd |= ICM20602_SPI_W;
		icm20602_simspi_wr_byte(cmd);
		icm20602_simspi_wr_byte(val);
	}

	//-------------------------------------------------------------------------------------------------------------------
	//  @brief      读取cmd所对应的寄存器地址
	//  @param      cmd         命令字
	//  @param      *val        存储读取的数据地址
	//  @since      v1.0
	//  Sample usage:
	//-------------------------------------------------------------------------------------------------------------------
	static void icm20602_simspi_r_reg_byte(uint8 cmd, uint8 *val)
	{
		cmd |= ICM20602_SPI_R;
		icm20602_simspi_wr_byte(cmd);
		*val = icm20602_simspi_wr_byte(0);
	}

	//-------------------------------------------------------------------------------------------------------------------
	//  @brief      读取cmd所对应的寄存器地址
	//  @param      cmd         命令字
	//  @param      *val        存储读取的数据地址
	//  @param      num         读取的数量
	//  @since      v1.0
	//  Sample usage:
	//-------------------------------------------------------------------------------------------------------------------
	static void icm20602_simspi_r_reg_bytes(uint8 cmd, uint8 *val, uint32 num)
	{
		uint32 edata i = 0;
		cmd |= ICM20602_SPI_R;
		icm20602_simspi_wr_byte(cmd);
		while(num--)
		{
			*val++ = icm20602_simspi_wr_byte(0);
		}
	}

	//-------------------------------------------------------------------------------------------------------------------
	// 函数简介     IMU660RA 写寄存器
	// 参数说明     reg             寄存器地址
	// 参数说明     dat            数据
	// 返回参数     void
	// 使用示例     icm20602_write_register(ICM20602_PWR_CONF, 0x00);                   // 关闭高级省电模式
	// 备注信息     内部调用
	//-------------------------------------------------------------------------------------------------------------------
	static void icm20602_write_register(uint8 reg, uint8 dat)
	{
		ICM20602_CS(0);
		icm20602_simspi_w_reg_byte(reg | ICM20602_SPI_W, dat);
		ICM20602_CS(1);
	}

	//-------------------------------------------------------------------------------------------------------------------
	// 函数简介     IMU660RA 读寄存器
	// 参数说明     reg             寄存器地址
	// 返回参数     uint8 edata           数据
	// 使用示例     icm20602_read_register(ICM20602_CHIP_ID);
	// 备注信息     内部调用
	//-------------------------------------------------------------------------------------------------------------------
	static uint8 icm20602_read_register(uint8 reg)
	{
		uint8 dat;
		ICM20602_CS(0);
		icm20602_simspi_r_reg_byte(reg | ICM20602_SPI_R, &dat);
		ICM20602_CS(1);
		return dat;
	}

	//-------------------------------------------------------------------------------------------------------------------
	// 函数简介     IMU660RA 读数据
	// 参数说明     reg             寄存器地址
	// 参数说明     dat            数据缓冲区
	// 参数说明     len             数据长度
	// 返回参数     void
	// 使用示例     icm20602_read_registers(ICM20602_ACC_ADDRESS, dat, 6);
	// 备注信息     内部调用
	//-------------------------------------------------------------------------------------------------------------------
	static void icm20602_read_registers(uint8 reg, uint8 *dat, uint32 len)
	{
		ICM20602_CS(0);
		icm20602_simspi_r_reg_bytes(reg | ICM20602_SPI_R, dat, len);
		ICM20602_CS(1);
	}

#elif (ICM20602_USE_INTERFACE==SOFT_IIC)

	static soft_iic_info_struct icm20602_iic_struct;

	//-------------------------------------------------------------------------------------------------------------------
	// 函数简介     ICM20602 写寄存器
	// 参数说明     reg             寄存器地址
	// 参数说明     dat            数据
	// 返回参数     void
	// 使用示例     icm20602_write_register(ICM20602_PWR_MGMT_1, 0x80);
	// 备注信息     内部调用
	//-------------------------------------------------------------------------------------------------------------------
	#define icm20602_write_register(reg, dat)      (soft_iic_write_8bit_register(&icm20602_iic_struct, (reg), (dat)))

	//-------------------------------------------------------------------------------------------------------------------
	// 函数简介     ICM20602 读寄存器
	// 参数说明     reg             寄存器地址
	// 返回参数     uint8           数据
	// 使用示例     icm20602_read_register(ICM20602_WHO_AM_I);
	// 备注信息     内部调用
	//-------------------------------------------------------------------------------------------------------------------
	#define icm20602_read_register(reg)             (soft_iic_read_8bit_register(&icm20602_iic_struct, (reg)))

	//-------------------------------------------------------------------------------------------------------------------
	// 函数简介     ICM20602 读数据
	// 参数说明     reg             寄存器地址
	// 参数说明     dat            数据缓冲区
	// 参数说明     len             数据长度
	// 返回参数     void
	// 使用示例     icm20602_read_registers(ICM20602_ACCEL_XOUT_H, dat, 6);
	// 备注信息     内部调用
	//-------------------------------------------------------------------------------------------------------------------
	#define icm20602_read_registers(reg, dat, len) (soft_iic_read_8bit_registers(&icm20602_iic_struct, (reg), (dat), (len)))

#endif


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ICM20602 自检
// 参数说明     void
// 返回参数     uint8           1-自检失败 0-自检成功
// 使用示例     icm20602_self_check();
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8 icm20602_self_check (void)
{
    uint8 dat = 0, return_state = 0;
    uint16 timeout_count = 0;
    
    while(0x12 != dat)                                                          // 判断 ID 是否正确
    {
        if(ICM20602_TIMEOUT_COUNT < timeout_count ++)
        {
            return_state =  1;
            break;
        }
        
        dat = icm20602_read_register(ICM20602_WHO_AM_I);
        system_delay_ms(10);
    }
    
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 ICM20602 加速度计数据
// 参数说明     void
// 返回参数     void
// 使用示例     icm20602_get_acc();                                             // 执行该函数后，直接查看对应的变量即可
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void icm20602_get_acc (void)
{
    uint8 dat[6];
    
    icm20602_read_registers(ICM20602_ACCEL_XOUT_H, dat, 6);
    icm20602_acc_x = (int16)(((uint16)dat[0] << 8 | dat[1]));
    icm20602_acc_y = (int16)(((uint16)dat[2] << 8 | dat[3]));
    icm20602_acc_z = (int16)(((uint16)dat[4] << 8 | dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取ICM20602陀螺仪数据
// 参数说明     void
// 返回参数     void
// 使用示例     icm20602_get_gyro();                                            // 执行该函数后，直接查看对应的变量即可
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void icm20602_get_gyro (void)
{
    uint8 dat[6];
    
    icm20602_read_registers(ICM20602_GYRO_XOUT_H, dat, 6);
    icm20602_gyro_x = (int16)(((uint16)dat[0] << 8 | dat[1]));
    icm20602_gyro_y = (int16)(((uint16)dat[2] << 8 | dat[3]));
    icm20602_gyro_z = (int16)(((uint16)dat[4] << 8 | dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 ICM20602
// 参数说明     void
// 返回参数     uint8           1-初始化失败 0-初始化成功
// 使用示例     icm20602_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 icm20602_init (void)
{
    uint8 val = 0x0, return_state = 0;
    uint16 timeout_count = 0;
    
    system_delay_ms(10);                                                        // 上电延时
    
#if (ICM20602_USE_INTERFACE==HARDWARE_SPI)
	
	spi_init(ICM20602_SPI, SPI_MODE0, ICM20602_SPI_SPEED, ICM20602_SPC_PIN, ICM20602_SDI_PIN, ICM20602_SDO_PIN, SPI_CS_NULL);
    gpio_init(ICM20602_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
	
#elif (ICM20602_USE_INTERFACE==SOFT_SPI)
	
	// 默认使用双向IO，不需要初始化。
//	soft_spi_init(ICM20602_SPI, SPI_MODE0, 0, ICM20602_SPC_PIN, ICM20602_SDI_PIN, ICM20602_SDO_PIN, ICM20602_CS_PIN);

#elif (ICM20602_USE_INTERFACE==SOFT_IIC)
	
	soft_iic_init(&icm20602_iic_struct, ICM20602_DEV_ADDR, ICM20602_SOFT_IIC_DELAY, ICM20602_SCL_PIN, ICM20602_SDA_PIN);

#endif
	
    
    do
    {
        if(icm20602_self_check())
        {
            // 如果程序在输出了断言信息 并且提示出错位置在这里
            // 那么就是 ICM20602 自检出错并超时退出了
            // 检查一下接线有没有问题 如果没问题可能就是坏了
            printf( "icm20602 self check error.\r\n");
            return_state = 1;
            break;
        }
        
        icm20602_write_register(ICM20602_PWR_MGMT_1, 0x80);                     // 复位设备
        system_delay_ms(2);
        
        do
        {
            // 等待复位成功
            val = icm20602_read_register(ICM20602_PWR_MGMT_1);
            
            if(ICM20602_TIMEOUT_COUNT < timeout_count ++)
            {
                // 如果程序在输出了断言信息 并且提示出错位置在这里
                // 那么就是 ICM20602 自检出错并超时退出了
                // 检查一下接线有没有问题 如果没问题可能就是坏了
                printf( "icm20602 reset error.\r\n");
                return_state = 1;
                break;
            }
        }
        while(0x41 != val);
        
        if(1 == return_state)
        {
            break;
        }
        
        icm20602_write_register(ICM20602_PWR_MGMT_1,     0x01);                 // 时钟设置
        icm20602_write_register(ICM20602_PWR_MGMT_2,     0x00);                 // 开启陀螺仪和加速度计
        icm20602_write_register(ICM20602_CONFIG,         0x01);                 // 176HZ 1KHZ
        icm20602_write_register(ICM20602_SMPLRT_DIV,     0x07);                 // 采样速率 SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
        
        // ICM20602_ACCEL_CONFIG 寄存器
        // 设置为 0x00 加速度计量程为 ±2  g   获取到的加速度计数据除以 16384  可以转化为带物理单位的数据 单位 g(m/s^2)
        // 设置为 0x08 加速度计量程为 ±4  g   获取到的加速度计数据除以 8192   可以转化为带物理单位的数据 单位 g(m/s^2)
        // 设置为 0x10 加速度计量程为 ±8  g   获取到的加速度计数据除以 4096   可以转化为带物理单位的数据 单位 g(m/s^2)
        // 设置为 0x18 加速度计量程为 ±16 g   获取到的加速度计数据除以 2048   可以转化为带物理单位的数据 单位 g(m/s^2)
        switch(ICM20602_ACC_SAMPLE_DEFAULT)
        {
            default:
            {
                printf( "ICM20602_ACC_SAMPLE_DEFAULT set error.\r\n");
                return_state = 1;
            }
            break;
            
            case ICM20602_ACC_SAMPLE_SGN_2G:
            {
                icm20602_write_register(ICM20602_ACCEL_CONFIG, 0x00);
                icm20602_transition_factor[0] = 16384;
            }
            break;
            
            case ICM20602_ACC_SAMPLE_SGN_4G:
            {
                icm20602_write_register(ICM20602_ACCEL_CONFIG, 0x08);
                icm20602_transition_factor[0] = 8192;
            }
            break;
            
            case ICM20602_ACC_SAMPLE_SGN_8G:
            {
                icm20602_write_register(ICM20602_ACCEL_CONFIG, 0x10);
                icm20602_transition_factor[0] = 4096;
            }
            break;
            
            case ICM20602_ACC_SAMPLE_SGN_16G:
            {
                icm20602_write_register(ICM20602_ACCEL_CONFIG, 0x18);
                icm20602_transition_factor[0] = 2048;
            }
            break;
        }
        
        if(1 == return_state)
        {
            break;
        }
        
        // ICM20602_GYRO_CONFIG 寄存器
        // 设置为 0x00 陀螺仪量程为 ±250  dps    获取到的陀螺仪数据除以 131     可以转化为带物理单位的数据 单位为 °/s
        // 设置为 0x08 陀螺仪量程为 ±500  dps    获取到的陀螺仪数据除以 65.5    可以转化为带物理单位的数据 单位为 °/s
        // 设置为 0x10 陀螺仪量程为 ±1000 dps    获取到的陀螺仪数据除以 32.8    可以转化为带物理单位的数据 单位为 °/s
        // 设置为 0x18 陀螺仪量程为 ±2000 dps    获取到的陀螺仪数据除以 16.4    可以转化为带物理单位的数据 单位为 °/s
        switch(ICM20602_GYRO_SAMPLE_DEFAULT)
        {
            default:
            {
                printf( "ICM20602_GYRO_SAMPLE_DEFAULT set error.\r\n");
                return_state = 1;
            }
            break;
            
            case ICM20602_GYRO_SAMPLE_SGN_250DPS:
            {
                icm20602_write_register(ICM20602_GYRO_CONFIG, 0x00);
                icm20602_transition_factor[1] = 131.0;
            }
            break;
            
            case ICM20602_GYRO_SAMPLE_SGN_500DPS:
            {
                icm20602_write_register(ICM20602_GYRO_CONFIG, 0x08);
                icm20602_transition_factor[1] = 65.5;
            }
            break;
            
            case ICM20602_GYRO_SAMPLE_SGN_1000DPS:
            {
                icm20602_write_register(ICM20602_GYRO_CONFIG, 0x10);
                icm20602_transition_factor[1] = 32.8;
            }
            break;
            
            case ICM20602_GYRO_SAMPLE_SGN_2000DPS:
            {
                icm20602_write_register(ICM20602_GYRO_CONFIG, 0x18);
                icm20602_transition_factor[1] = 16.4;
            }
            break;
        }
        
        if(1 == return_state)
        {
            break;
        }
        
        icm20602_write_register(ICM20602_ACCEL_CONFIG_2, 0x03);                 // Average 4 samples   44.8HZ   //0x23 Average 16 samples
    }
    while(0);
    
    return return_state;
}
