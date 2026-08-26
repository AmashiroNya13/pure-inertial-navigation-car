/*********************************************************************************************************************
* STC32G Opensourec Library 即（STC32G 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是STC 开源库的一部分
*
* STC32G 开源库 是免费软件
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
*                   // 硬件 SPI 引脚
*                   SCL/SPC           查看 zf_device_imu660rb.h 中 IMU660RB_SPC_PIN 宏定义
*                   SDA/DSI           查看 zf_device_imu660rb.h 中 IMU660RB_SDI_PIN 宏定义
*                   SA0/SDO           查看 zf_device_imu660rb.h 中 IMU660RB_SDO_PIN 宏定义
*                   CS                查看 zf_device_imu660rb.h 中 IMU660RB_CS_PIN 宏定义
*                   VCC               3.3V电源
*                   GND               电源地
*                   其余引脚悬空
*
*                   // 软件 IIC 引脚
*                   SCL/SPC           查看 zf_device_imu660rb.h 中 IMU660RB_SCL_PIN 宏定义
*                   SDA/DSI           查看 zf_device_imu660rb.h 中 IMU660RB_SDA_PIN 宏定义
*                   VCC               3.3V电源
*                   GND               电源地
*                   其余引脚悬空
*                   ------------------------------------
********************************************************************************************************************/

#include "zf_common_debug.h"
#include "zf_driver_delay.h"
#include "zf_driver_spi.h"
#include "zf_driver_gpio.h"
#include "zf_driver_soft_iic.h"
#include "zf_device_config.h"
#include "zf_device_imu660rb.h"

int16 imu660rb_gyro_x = 0, imu660rb_gyro_y = 0, imu660rb_gyro_z = 0;            // 三轴陀螺仪数据   gyro (陀螺仪)
int16 imu660rb_acc_x = 0, imu660rb_acc_y = 0, imu660rb_acc_z = 0;               // 三轴加速度计数据 acc  (accelerometer 加速度计)


#if (IMU660RB_USE_INTERFACE==HARDWARE_SPI) 

	//-------------------------------------------------------------------------------------------------------------------
	// 函数简介     IMU660RB 写寄存器
	// 参数说明     reg             寄存器地址
	// 参数说明     dat            数据
	// 返回参数     void
	// 使用示例     imu660rb_write_register(IMU660RB_PWR_CONF, 0x00);                   // 关闭高级省电模式
	// 备注信息     内部调用
	//-------------------------------------------------------------------------------------------------------------------
	static void imu660rb_write_register(uint8 reg, uint8 dat)
	{
		IMU660RB_CS(0);
		spi_write_8bit_register(IMU660RB_SPI, reg | IMU660RB_SPI_W, dat);
		IMU660RB_CS(1);
	}

	//-------------------------------------------------------------------------------------------------------------------
	// 函数简介     IMU660RB 写数据
	// 参数说明     reg             寄存器地址
	// 参数说明     dat            数据
	// 返回参数     void
	// 使用示例     imu660rb_write_registers(IMU660RB_INIT_DATA, imu660rb_config_file, sizeof(imu660rb_config_file));
	// 备注信息     内部调用
	//-------------------------------------------------------------------------------------------------------------------
	// static void imu660rb_write_registers(uint8 reg, const uint8 *dat, uint32 len)
	// {
	//     IMU660RB_CS(0);
	//     spi_write_8bit_registers(IMU660RB_SPI, reg | IMU660RB_SPI_W, dat, len);
	//     IMU660RB_CS(1);
	// }

	//-------------------------------------------------------------------------------------------------------------------
	// 函数简介     IMU660RB 读寄存器
	// 参数说明     reg             寄存器地址
	// 返回参数     uint8           数据
	// 使用示例     imu660rb_read_register(IMU660RB_CHIP_ID);
	// 备注信息     内部调用
	//-------------------------------------------------------------------------------------------------------------------
	static uint8 imu660rb_read_register(uint8 reg)
	{
		uint8 dat;
		IMU660RB_CS(0);
		dat = spi_read_8bit_register(IMU660RB_SPI, reg | IMU660RB_SPI_R);
		IMU660RB_CS(1);
		return dat;
	}

	//-------------------------------------------------------------------------------------------------------------------
	// 函数简介     IMU660RB 读数据
	// 参数说明     reg             寄存器地址
	// 参数说明     dat            数据缓冲区
	// 参数说明     len             数据长度
	// 返回参数     void
	// 使用示例     imu660rb_read_registers(IMU660RB_ACC_ADDRESS, dat, 6);
	// 备注信息     内部调用
	//-------------------------------------------------------------------------------------------------------------------
	static void imu660rb_read_registers(uint8 reg, uint8 *dat, uint32 len)
	{
		IMU660RB_CS(0);
		spi_read_8bit_registers(IMU660RB_SPI, reg | IMU660RB_SPI_R, dat, len);
		IMU660RB_CS(1);

	}

#elif (IMU660RB_USE_INTERFACE==SOFT_SPI)
	
	
	#define IMU660RB_SCK(x)				IMU660RB_SPC_PIN  = x
	#define IMU660RB_MOSI(x) 			IMU660RB_SDI_PIN = x
	#define IMU660RB_MISO    			IMU660RB_SDO_PIN
	#define IMU660RB_CS(x)  			IMU660RB_CS_PIN  = x

	//-------------------------------------------------------------------------------------------------------------------
	//  @brief      通过SPI写一个byte,同时读取一个byte
	//  @param      byte        发送的数据
	//  @return     uint8 edata       return 返回status状态
	//  @since      v1.0
	//  Sample usage:
	//-------------------------------------------------------------------------------------------------------------------
	static uint8 imu660rb_simspi_wr_byte(uint8 byte)
	{
		uint8 i;
		for(i=0; i<8; i++)
		{
			IMU660RB_SCK (0);
			IMU660RB_MOSI(byte&0x80);
			byte <<= 1;
			IMU660RB_SCK (1);
			byte |= IMU660RB_MISO;
		}
		IMU660RB_SCK (0);
		return(byte);
	}
	
	//-------------------------------------------------------------------------------------------------------------------
	//  @brief      将val写入cmd对应的寄存器地址,同时返回status字节
	//  @param      cmd         命令字
	//  @param      val         待写入寄存器的数值
	//  @since      v1.0
	//  Sample usage:
	//-------------------------------------------------------------------------------------------------------------------
	static void imu660rb_simspi_w_reg_byte(uint8 cmd, uint8 val)
	{
		cmd |= IMU660RB_SPI_W;
		imu660rb_simspi_wr_byte(cmd);
		imu660rb_simspi_wr_byte(val);
	}

	//-------------------------------------------------------------------------------------------------------------------
	//  @brief      读取cmd所对应的寄存器地址
	//  @param      cmd         命令字
	//  @param      *val        存储读取的数据地址
	//  @since      v1.0
	//  Sample usage:
	//-------------------------------------------------------------------------------------------------------------------
	static void imu660rb_simspi_r_reg_byte(uint8 cmd, uint8 *val)
	{
		IMU660RB_SCK(0);
		cmd |= IMU660RB_SPI_R;
		imu660rb_simspi_wr_byte(cmd);
		*val = imu660rb_simspi_wr_byte(0);
		IMU660RB_SCK(1);
	}
	

//	//-------------------------------------------------------------------------------------------------------------------
//	//  @brief      将val写入cmd对应的寄存器地址
//	//  @param      cmd         命令字
//	//  @param      val         待写入寄存器的数值
//	//  @since      v1.0
//	//  Sample usage:
//	//-------------------------------------------------------------------------------------------------------------------
//	static void imu660rb_simspi_w_reg_bytes(uint8 cmd, uint8 *dat_addr, uint32 len)
//	{
//		cmd |= IMU660RB_SPI_W;
//		imu660rb_simspi_wr_byte(cmd);
//		while(len--)
//		{
//			imu660rb_simspi_wr_byte(*dat_addr++);
//		}
//	}

	//-------------------------------------------------------------------------------------------------------------------
	//  @brief      读取cmd所对应的寄存器地址
	//  @param      cmd         命令字
	//  @param      *val        存储读取的数据地址
	//  @param      num         读取的数量
	//  @since      v1.0
	//  Sample usage:
	//-------------------------------------------------------------------------------------------------------------------
	static void imu660rb_simspi_r_reg_bytes(uint8 cmd, uint8 *val, uint32 num)
	{
		cmd |= IMU660RB_SPI_R;
		imu660rb_simspi_wr_byte(cmd);
		while(num--)
		{
			*val++ = imu660rb_simspi_wr_byte(0);
		}
	}
	
	//-------------------------------------------------------------------------------------------------------------------
	// 函数简介     IMU660RB 写寄存器
	// 参数说明     reg             寄存器地址
	// 参数说明     dat            数据
	// 返回参数     void
	// 使用示例     imu660rb_write_register(IMU660RB_PWR_CONF, 0x00);                   // 关闭高级省电模式
	// 备注信息     内部调用
	//-------------------------------------------------------------------------------------------------------------------
	static void imu660rb_write_register(uint8 reg, uint8 dat)
	{
		IMU660RB_CS(0);
		imu660rb_simspi_w_reg_byte(reg | IMU660RB_SPI_W, dat);
		IMU660RB_CS(1);
	}

	//-------------------------------------------------------------------------------------------------------------------
	// 函数简介     IMU660RB 读寄存器
	// 参数说明     reg             寄存器地址
	// 返回参数     uint8           数据
	// 使用示例     imu660rb_read_register(IMU660RB_CHIP_ID);
	// 备注信息     内部调用
	//-------------------------------------------------------------------------------------------------------------------
	static uint8 imu660rb_read_register(uint8 reg)
	{
		uint8 dat;
		IMU660RB_CS(0);
		imu660rb_simspi_r_reg_byte(reg | IMU660RB_SPI_R, &dat);
		IMU660RB_CS(1);
		return dat;
	}

	//-------------------------------------------------------------------------------------------------------------------
	// 函数简介     IMU660RB 读数据
	// 参数说明     reg             寄存器地址
	// 参数说明     dat            数据缓冲区
	// 参数说明     len             数据长度
	// 返回参数     void
	// 使用示例     imu660rb_read_registers(IMU660RB_ACC_ADDRESS, dat, 6);
	// 备注信息     内部调用
	//-------------------------------------------------------------------------------------------------------------------
	static void imu660rb_read_registers(uint8 reg, uint8 *dat, uint32 len)
	{
		IMU660RB_CS(0);
		imu660rb_simspi_r_reg_bytes(reg | IMU660RB_SPI_R, dat, len);
		IMU660RB_CS(1);
	}
	
	

#elif (IMU660RB_USE_INTERFACE==SOFT_IIC)
	
	static soft_iic_info_struct imu660rb_iic_struct;

	#define imu660rb_write_register(reg, dat)        (soft_iic_write_8bit_register (&imu660rb_iic_struct, (reg), (dat)))
	#define imu660rb_write_registers(reg, dat, len)  (soft_iic_write_8bit_registers(&imu660rb_iic_struct, (reg), (dat), (len)))
	#define imu660rb_read_register(reg)               (soft_iic_read_8bit_register  (&imu660rb_iic_struct, (reg)))
	#define imu660rb_read_registers(reg, dat, len)   (soft_iic_read_8bit_registers (&imu660rb_iic_struct, (reg), (dat), (len)))

#endif

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU660RB 自检
// 参数说明     void
// 返回参数     uint8           1-自检失败 0-自检成功
// 使用示例     imu660rb_self_check();
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu660rb_self_check (void)
{
    uint8 dat = 0, return_state = 0;
    uint16 timeout_count = 0;
    do
    {
        if(timeout_count ++ > IMU660RB_TIMEOUT_COUNT)
        {
            return_state =  1;
            break;
        }
        dat = imu660rb_read_register(IMU660RB_CHIP_ID);
        system_delay_ms(1);
    }while(0x6B != dat);                                                        // 读取设备ID是否等于0X24，如果不是0X24则认为没检测到设备
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 IMU660RB 加速度计数据
// 参数说明     void
// 返回参数     void
// 使用示例     imu660rb_get_acc();                                             // 执行该函数后，直接查看对应的变量即可
// 备注信息     使用 SPI 的采集时间为69us
//             使用 IIC 的采集时间为126us        采集加速度计的时间与采集陀螺仪的时间一致的原因是都只是读取寄存器数据
//-------------------------------------------------------------------------------------------------------------------
void imu660rb_get_acc (void)
{
    uint8 dat[6];

    imu660rb_read_registers(IMU660RB_ACC_ADDRESS, dat, 6);
    imu660rb_acc_x = (int16)(((uint16)dat[1]<<8 | dat[0]));
    imu660rb_acc_y = (int16)(((uint16)dat[3]<<8 | dat[2]));
    imu660rb_acc_z = (int16)(((uint16)dat[5]<<8 | dat[4]));
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 IMU660RB 陀螺仪数据
// 参数说明     void
// 返回参数     void
// 使用示例     imu660rb_get_gyro();                                            // 执行该函数后，直接查看对应的变量即可
// 备注信息     使用 SPI 的采集时间为69us
//             使用 IIC 的采集时间为126us
//-------------------------------------------------------------------------------------------------------------------
void imu660rb_get_gyro (void)
{
    uint8 dat[6];

    imu660rb_read_registers(IMU660RB_GYRO_ADDRESS, dat, 6);
    imu660rb_gyro_x = (int16)(((uint16)dat[1]<<8 | dat[0]));
    imu660rb_gyro_y = (int16)(((uint16)dat[3]<<8 | dat[2]));
    imu660rb_gyro_z = (int16)(((uint16)dat[5]<<8 | dat[4]));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 IMU660RB 加速度计数据转换为实际物理数据
// 参数说明     gyro_value      任意轴的加速度计数据
// 返回参数     void
// 使用示例     float dat = imu660rb_acc_transition(imu660rb_acc_x);           // 单位为 g(m/s^2)
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
float imu660rb_acc_transition (int16 acc_value)
{
    float acc_data = 0;
    switch(IMU660RB_ACC_SAMPLE)
    {
        case 0x30: acc_data = (float)acc_value / 16393; break;                  // 0x30 加速度量程为:±2G      获取到的加速度计数据 除以 16393 ，可以转化为带物理单位的数据，单位：g(m/s^2)
        case 0x38: acc_data = (float)acc_value / 8197;  break;                  // 0x38 加速度量程为:±4G      获取到的加速度计数据 除以 8197 ， 可以转化为带物理单位的数据，单位：g(m/s^2)
        case 0x3C: acc_data = (float)acc_value / 4098;  break;                  // 0x3C 加速度量程为:±8G      获取到的加速度计数据 除以 4098 ， 可以转化为带物理单位的数据，单位：g(m/s^2)
        case 0x34: acc_data = (float)acc_value / 2049;  break;                  // 0x34 加速度量程为:±16G     获取到的加速度计数据 除以 2049 ， 可以转化为带物理单位的数据，单位：g(m/s^2)
        default: break;
    }
    return acc_data;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 IMU660RB 陀螺仪数据转换为实际物理数据
// 参数说明     gyro_value      任意轴的陀螺仪数据
// 返回参数     void
// 使用示例     float dat = imu660rb_gyro_transition(imu660rb_gyro_x);         // 单位为°/s
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
float imu660rb_gyro_transition (int16 gyro_value)
{
    float gyro_data = 0;
    switch(IMU660RB_GYR_SAMPLE)
    {
        case 0x52: gyro_data = (float)gyro_value / 228.6f;  break;              //  0x52 陀螺仪量程为:±125dps  获取到的陀螺仪数据除以 228.6，   可以转化为带物理单位的数据，单位为：°/s
        case 0x50: gyro_data = (float)gyro_value / 114.3f;  break;              //  0x50 陀螺仪量程为:±250dps  获取到的陀螺仪数据除以 114.3，   可以转化为带物理单位的数据，单位为：°/s
        case 0x54: gyro_data = (float)gyro_value / 57.1f;   break;              //  0x54 陀螺仪量程为:±500dps  获取到的陀螺仪数据除以 57.1，    可以转化为带物理单位的数据，单位为：°/s
        case 0x58: gyro_data = (float)gyro_value / 28.6f;   break;              //  0x58 陀螺仪量程为:±1000dps 获取到的陀螺仪数据除以 28.6，    可以转化为带物理单位的数据，单位为：°/s
        case 0x5C: gyro_data = (float)gyro_value / 14.3f;   break;              //  0x5C 陀螺仪量程为:±2000dps 获取到的陀螺仪数据除以 14.3，    可以转化为带物理单位的数据，单位为：°/s
        case 0x51: gyro_data = (float)gyro_value / 7.1f;    break;              //  0x51 陀螺仪量程为:±4000dps 获取到的陀螺仪数据除以 7.1，     可以转化为带物理单位的数据，单位为：°/s
        default: break;
    }
    return gyro_data;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 IMU660RB
// 参数说明     void
// 返回参数     uint8           1-初始化失败 0-初始化成功
// 使用示例     imu660rb_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 imu660rb_init (void)
{
    uint8 return_state = 0;
    system_delay_ms(20);                                                        // 等待设备上电成功

#if (IMU660RB_USE_INTERFACE==HARDWARE_SPI)
	
    spi_init(IMU660RB_SPI, SPI_MODE0, IMU660RB_SPI_SPEED, IMU660RB_SPC_PIN, IMU660RB_SDI_PIN, IMU660RB_SDO_PIN, SPI_CS_NULL);   // 配置 IMU660RB 的 SPI 端口
    gpio_init(IMU660RB_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);                  // 配置 IMU660RB 的 CS 端口
//    imu660rb_read_register(IMU660RB_CHIP_ID);                                   // 读取一下设备ID 将设备设置为SPI模式
	
#elif (IMU660RB_USE_INTERFACE==SOFT_SPI)
	
// 默认使用双向IO，不需要初始化。
//	soft_spi_init(IMU660RB_SPI, SPI_MODE0, 0, IMU963RA_SPC_PIN, IMU963RA_SDI_PIN, IMU963RA_SDO_PIN, IMU963RA_CS_PIN);
	
#elif (IMU660RB_USE_INTERFACE==SOFT_IIC)
	
	soft_iic_init(&imu660rb_iic_struct, IMU660RB_DEV_ADDR, IMU660RB_SOFT_IIC_DELAY, IMU660RB_SCL_PIN, IMU660RB_SDA_PIN);        // 配置 IMU660RB 的 IIC 端口
	
#endif
	
	
    do{
        if(imu660rb_self_check())                                               // IMU660RB 自检
        {
            // 如果程序在输出了断言信息 并且提示出错位置在这里
            // 那么就是 IMU660RB 自检出错并超时退出了
            // 检查一下接线有没有问题 如果没问题可能就是坏了
            printf("imu660rb self check error.");
            return_state = 1;
            break;
        }

        imu660rb_write_register(IMU660RB_INT1_CTRL, 0x03);                      // 开启陀螺仪 加速度数据就绪中断
        imu660rb_write_register(IMU660RB_CTRL1_XL, IMU660RB_ACC_SAMPLE);        // 设置加速度计量程 ±8G 以及数据输出速率 52hz 以及加速度信息从第一级滤波器输出
        // IMU660RB_CTRL1_XL 寄存器
        // 设置为:0x30 加速度量程为:±2G      获取到的加速度计数据 除以16393，可以转化为带物理单位的数据，单位：g(m/s^2)
        // 设置为:0x38 加速度量程为:±4G      获取到的加速度计数据 除以8197， 可以转化为带物理单位的数据，单位：g(m/s^2)
        // 设置为:0x3C 加速度量程为:±8G      获取到的加速度计数据 除以4098， 可以转化为带物理单位的数据，单位：g(m/s^2)
        // 设置为:0x34 加速度量程为:±16G     获取到的加速度计数据 除以2049， 可以转化为带物理单位的数据，单位：g(m/s^2)

        imu660rb_write_register(IMU660RB_CTRL2_G, IMU660RB_GYR_SAMPLE);         // 设置陀螺仪计量程 ±2000dps 以及数据输出速率 208hz
        // IMU660RB_CTRL2_G 寄存器
        // 设置为:0x52 陀螺仪量程为:±125dps  获取到的陀螺仪数据除以228.6，   可以转化为带物理单位的数据，单位为：°/s
        // 设置为:0x50 陀螺仪量程为:±250dps  获取到的陀螺仪数据除以114.3，   可以转化为带物理单位的数据，单位为：°/s
        // 设置为:0x54 陀螺仪量程为:±500dps  获取到的陀螺仪数据除以57.1，    可以转化为带物理单位的数据，单位为：°/s
        // 设置为:0x58 陀螺仪量程为:±1000dps 获取到的陀螺仪数据除以28.6，    可以转化为带物理单位的数据，单位为：°/s
        // 设置为:0x5C 陀螺仪量程为:±2000dps 获取到的陀螺仪数据除以14.3，    可以转化为带物理单位的数据，单位为：°/s
        // 设置为:0x51 陀螺仪量程为:±4000dps 获取到的陀螺仪数据除以7.1，     可以转化为带物理单位的数据，单位为：°/s

        imu660rb_write_register(IMU660RB_CTRL3_C, 0x44);                        // 使能陀螺仪数字低通滤波器
        imu660rb_write_register(IMU660RB_CTRL4_C, 0x02);                        // 使能数字低通滤波器
        imu660rb_write_register(IMU660RB_CTRL5_C, 0x00);                        // 加速度计与陀螺仪四舍五入
        imu660rb_write_register(IMU660RB_CTRL6_C, 0x00);                        // 开启加速度计高性能模式 陀螺仪低通滤波 133hz
        imu660rb_write_register(IMU660RB_CTRL7_G, 0x00);                        // 开启陀螺仪高性能模式 关闭高通滤波
        imu660rb_write_register(IMU660RB_CTRL9_XL, 0x01);                       // 关闭I3C接口

    }while(0);
    return return_state;
}


