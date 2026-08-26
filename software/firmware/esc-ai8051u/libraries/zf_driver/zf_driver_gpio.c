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

#include "zf_common_debug.h"
#include "zf_driver_gpio.h"






// 内部使用，用户无需关心。
static void gpio_set_mode(gpio_pin_enum pin, gpio_mode_enum mode)
{
	#define PXPU(pin)  (*(unsigned char volatile far *)(0x7efe10 + (pin >> 4 - 1)))
	#define PXPD(pin)  (*(unsigned char volatile far *)(0x7efe40 + (pin >> 4 - 1)))

	
    if(GPI_FLOATING_IN == mode || GPIO_NO_PULL == mode)
    {
        PXPU(pin) &= ~(1 << (pin & 0x0F));
        PXPD(pin) &= ~(1 << (pin & 0x0F));
    }
    else if(GPI_PULL_DOWN == mode)
    {
        PXPD(pin) |= (1 << (pin & 0x0F));
    }
    else if(GPI_PULL_UP == mode || GPO_PUSH_PULL == mode)
    {
        PXPU(pin) |= (1 << (pin & 0x0F));
    }
	else if(GPO_OPEN_DTAIN == mode)
	{
		switch(pin & 0xF0)
		{
			case IO_P00:
			{
                P0M1 |= (1 << (pin & 0xF));
                P0M0 |= (1 << (pin & 0xF));
			}
			break;

			case IO_P10:
			{
				P1M1 |= (1 << (pin & 0xF));
                P1M0 |= (1 << (pin & 0xF));
			}
			break;
			
			case IO_P20:
			{
				P2M1 |= (1 << (pin & 0xF));
                P2M0 |= (1 << (pin & 0xF));
			}
			break;
			
			case IO_P30:
			{
				P3M1 |= (1 << (pin & 0xF));
                P3M0 |= (1 << (pin & 0xF));
			}
			break;
			
			case IO_P40:
			{
				P4M1 |= (1 << (pin & 0xF));
                P4M0 |= (1 << (pin & 0xF));
			}
			break;
			
			case IO_P50:
			{
				P5M1 |= (1 << (pin & 0xF));
                P5M0 |= (1 << (pin & 0xF));
			}
			break;
			default:
			{
			}	
			break;
		}
	}
	else
	{
		zf_assert(0);
		// 参数不匹配
	}
}

// 内部使用，用户无需关心。
void gpio_set_level_speed(gpio_pin_enum pin, gpio_speed_enum speed)
{
    switch(pin & 0xF0)
    {
        case IO_P00:
        {
            if(GPIO_SPEED_FAST == speed)
            {
				P0SR &= ~(1<<(pin & 0x0F));
            }
            else if(GPIO_SPEED_LOW == speed)
            {
				P0SR |= (1<<(pin & 0x0F));
            }
			else
			{
				zf_assert(0);
				// 没有这个参数
			}
        }
        break;
        
        case IO_P10:
        {
            if(GPIO_SPEED_FAST == speed)
            {
				P1SR &= ~(1<<(pin & 0x0F));
            }
            else if(GPIO_SPEED_LOW == speed)
            {
				P1SR |= (1<<(pin & 0x0F));
            }
			else
			{
				zf_assert(0);
				// 没有这个参数
			}
        }
        break;
        
        case IO_P20:
        {
            if(GPIO_SPEED_FAST == speed)
            {
				P2SR &= ~(1<<(pin & 0x0F));
            }
            else if(GPIO_SPEED_LOW == speed)
            {
				P2SR |= (1<<(pin & 0x0F));
            }
			else
			{
				zf_assert(0);
				// 没有这个参数
			}
        }
        break;
        
        case IO_P30:
        {
            if(GPIO_SPEED_FAST == speed)
            {
				P3SR &= ~(1<<(pin & 0x0F));
            }
            else if(GPIO_SPEED_LOW == speed)
            {
				P3SR |= (1<<(pin & 0x0F));
            }
			else
			{
				zf_assert(0);
				// 没有这个参数
			}
        }
        break;
        
        case IO_P40:
        {
			if(GPIO_SPEED_FAST == speed)
            {
				P4SR &= ~(1<<(pin & 0x0F));
            }
            else if(GPIO_SPEED_LOW == speed)
            {
				P4SR |= (1<<(pin & 0x0F));
            }
			else
			{
				zf_assert(0);
				// 没有这个参数
			}
        }
        break;
        
        case IO_P50:
        {
            if(GPIO_SPEED_FAST == speed)
            {
				P5SR &= ~(1<<(pin & 0x0F));
            }
            else if(GPIO_SPEED_LOW == speed)
            {
				P5SR |= (1<<(pin & 0x0F));
            }
			else
			{
				zf_assert(0);
				// 没有这个参数
			}
        }
        break;
        
        default:
        {
			zf_assert(0);
			// 没有这个引脚
        } break;
    }
}


// 内部使用，用户无需关心。
static void gpio_set_dir(gpio_pin_enum pin, gpio_dir_enum dir)
{
    switch(pin & 0xF0)
    {
        case IO_P00:
        {
            if(GPIO == dir)
            {
                P0M1 &= ~(1 << (pin & 0xF));
                P0M0 &= ~(1 << (pin & 0xF));
            }
//            else if(GPO_OPEN_DTAIN == dir)
//            {
//                P0M1 |= (1 << (pin & 0xF));
//                P0M0 |= (1 << (pin & 0xF));
//            }
            else if(GPO == dir)
            {
                P0M1 &= ~(1 << (pin & 0xF));
                P0M0 |= (1 << (pin & 0xF));
            }
            else if(GPI == dir)
            {
                P0M1 |= (1 << (pin & 0xF));
                P0M0 &= ~(1 << (pin & 0xF));
            }
        }
        break;
        
        case IO_P10:
        {
            if(GPIO == dir)
            {
                P1M1 &= ~(1 << (pin & 0xF));
                P1M0 &= ~(1 << (pin & 0xF));
            }
//            else if(GPO_OPEN_DTAIN == dir)
//            {
//                P1M1 |= (1 << (pin & 0xF));
//                P1M0 |= (1 << (pin & 0xF));
//            }
            else if(GPO == dir)
            {
                P1M1 &= ~(1 << (pin & 0xF));
                P1M0 |= (1 << (pin & 0xF));
            }
            else if(GPI == dir)
            {
                P1M1 |= (1 << (pin & 0xF));
                P1M0 &= ~(1 << (pin & 0xF));
            }
        }
        break;
        
        case IO_P20:
        {
            if(GPIO == dir)
            {
                P2M1 &= ~(1 << (pin & 0xF));
                P2M0 &= ~(1 << (pin & 0xF));
            }
//            else if(GPO_OPEN_DTAIN == dir)
//            {
//                P2M1 |= (1 << (pin & 0xF));
//                P2M0 |= (1 << (pin & 0xF));
//            }
            else if(GPO == dir)
            {
                P2M1 &= ~(1 << (pin & 0xF));
                P2M0 |= (1 << (pin & 0xF));
            }
            else if(GPI == dir)
            {
                P2M1 |= (1 << (pin & 0xF));
                P2M0 &= ~(1 << (pin & 0xF));
            }
        }
        break;
        
        case IO_P30:
        {
            if(GPIO == dir)
            {
                P3M1 &= ~(1 << (pin & 0xF));
                P3M0 &= ~(1 << (pin & 0xF));
            }
//            else if(GPO_OPEN_DTAIN == dir)
//            {
//                P3M1 |= (1 << (pin & 0xF));
//                P3M0 |= (1 << (pin & 0xF));
//            }
            else if(GPO == dir)
            {
                P3M1 &= ~(1 << (pin & 0xF));
                P3M0 |= (1 << (pin & 0xF));
            }
            else if(GPI == dir)
            {
                P3M1 |= (1 << (pin & 0xF));
                P3M0 &= ~(1 << (pin & 0xF));
            }
        }
        break;
        
        case IO_P40:
        {
            if(GPIO == dir)
            {
                P4M1 &= ~(1 << (pin & 0xF));
                P4M0 &= ~(1 << (pin & 0xF));
            }
//            else if(GPO_OPEN_DTAIN == dir)
//            {
//                P4M1 |= (1 << (pin & 0xF));
//                P4M0 |= (1 << (pin & 0xF));
//            }
            else if(GPO == dir)
            {
                P4M1 &= ~(1 << (pin & 0xF));
                P4M0 |= (1 << (pin & 0xF));
            }
            else if(GPI == dir)
            {
                P4M1 |= (1 << (pin & 0xF));
                P4M0 &= ~(1 << (pin & 0xF));
            }
        }
        break;
        
        case IO_P50:
        {
            if(GPIO == dir)
            {
                P5M1 &= ~(1 << (pin & 0xF));
                P5M0 &= ~(1 << (pin & 0xF));
            }
//            else if(GPO_OPEN_DTAIN == dir)
//            {
//                P5M1 |= (1 << (pin & 0xF));
//                P5M0 |= (1 << (pin & 0xF));
//            }
            else if(GPO == dir)
            {
                P5M1 &= ~(1 << (pin & 0xF));
                P5M0 |= (1 << (pin & 0xF));
            }
            else if(GPI == dir)
            {
                P5M1 |= (1 << (pin & 0xF));
                P5M0 &= ~(1 << (pin & 0xF));
            }
        }
        break;
        
        default:
        {
			zf_assert(0);
			// 没有这个引脚
        } break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     GPIO状态获取
// 参数说明     pin         选择的引脚 (可选择范围由 zf_driver_gpio.h 内gpio_pin_enum枚举值确定)
// 返回参数     uint8       0：低电平 1：高电平
// 使用示例     uint8 status = gpio_get_level(IO_P00);//获取P00引脚电平
//-------------------------------------------------------------------------------------------------------------------
uint8 gpio_get_level(gpio_pin_enum pin)
{
    uint8 status = 0;
    
    switch(pin & 0xF0)
    {
        case IO_P00:
        {
            status = P0 & (1 << (pin & 0x0F));
        }
        break;
        
        case IO_P10:
        {
            status = P1 & (1 << (pin & 0x0F));
        }
        break;
        
        case IO_P20:
        {
            status = P2 & (1 << (pin & 0x0F));
        }
        break;
        
        case IO_P30:
        {
            status = P3 & (1 << (pin & 0x0F));
        }
        break;
        
        case IO_P40:
        {
            status = P4 & (1 << (pin & 0x0F));
        }
        break;
        
        case IO_P50:
        {
            status = P5 & (1 << (pin & 0x0F));
        }
        break;
        
        default:
        {
			zf_assert(0);
			// 没有这个引脚
        } break;
    }
    
    return status ? 1 : 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     GPIO输出设置
// 参数说明     pin         引脚号选择的引脚 (可选择范围由 common.h 内GPIO_PIN_enum枚举值确定)
// 参数说明     dat         引脚的电平状态，输出时有效 0：低电平 1：高电平
// 返回参数     void
// 使用示例     gpio_set_level(D0, 0);//D0输出低电平
//-------------------------------------------------------------------------------------------------------------------
void gpio_set_level(gpio_pin_enum pin, uint8 dat)
{
    switch(pin & 0xF0)
    {
        case IO_P00:
        {
			if(dat)
			{
				P0 |= (1 << (pin & 0x0F));
			}
			else
			{
				P0 &= ~(1 << (pin & 0x0F));
			}
        }
        break;
        
        case IO_P10:
        {
			if(dat)
			{
				P1 |= (1 << (pin & 0x0F));
			}
			else
			{
				P1 &= ~(1 << (pin & 0x0F));
			}
        }
        break;
        
        case IO_P20:
        {
			if(dat)
			{
				P2 |= (1 << (pin & 0x0F));
			}
			else
			{
				P2 &= ~(1 << (pin & 0x0F));
			}
        }
        break;
        
        case IO_P30:
        {
			if(dat)
			{
				P3 |= (1 << (pin & 0x0F));
			}
			else
			{
				P3 &= ~(1 << (pin & 0x0F));
			}
        }
        break;
        
        case IO_P40:
        {
			if(dat)
			{
				P4 |= (1 << (pin & 0x0F));
			}
			else
			{
				P4 &= ~(1 << (pin & 0x0F));
			}
        }
        break;
        
        case IO_P50:
        {
			if(dat)
			{
				P5 |= (1 << (pin & 0x0F));
			}
			else
			{
				P5 &= ~(1 << (pin & 0x0F));
			}
        }
        break;
        
        default:
        {
			zf_assert(0);
			// 没有这个引脚
        } break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     GPIO输出设置
// 参数说明     pin         引脚号选择的引脚 (可选择范围由 common.h 内GPIO_PIN_enum枚举值确定)
// 参数说明     dat         引脚的电平状态，输出时有效 0：低电平 1：高电平
// 返回参数     void
// 使用示例     gpio_set_level(D0, 0);//D0输出低电平
//-------------------------------------------------------------------------------------------------------------------
void gpio_toggle_level(gpio_pin_enum pin)
{
	if(gpio_get_level(pin))
	{
		gpio_low(pin);
	}
	else
	{
		gpio_high(pin);
	}
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     GPIO设置高阻输入
// 参数说明     pin         引脚号选择的引脚 (可选择范围由 common.h 内GPIO_PIN_enum枚举值确定)
// 返回参数     void
// 使用示例     
//-------------------------------------------------------------------------------------------------------------------
void gpio_set_impedance(gpio_pin_enum pin)
{
    switch(pin & 0xF0)
    {
        case IO_P00:
        {
			P0M1 |=  (1 << (pin & 0xF));
			P0M0 &= ~(1 << (pin & 0xF));
        }
        break;
        
        case IO_P10:
        {
			P1M1 |=  (1 << (pin & 0xF));
			P1M0 &= ~(1 << (pin & 0xF));
        }
        break;
        
        case IO_P20:
        {
			P2M1 |=  (1 << (pin & 0xF));
			P2M0 &= ~(1 << (pin & 0xF));;
        }
        break;
        
        case IO_P30:
        {
			P3M1 |=  (1 << (pin & 0xF));
			P3M0 &= ~(1 << (pin & 0xF));
        }
        break;
        
        case IO_P40:
        {
			P4M1 |=  (1 << (pin & 0xF));
			P4M0 &= ~(1 << (pin & 0xF));
        }
        break;
        
        case IO_P50:
        {
			P5M1 |=  (1 << (pin & 0xF));
			P5M0 &= ~(1 << (pin & 0xF));
        }
        break;
        
        default:
        {
			zf_assert(0);
			// 没有这个引脚
        } break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     GPIO初始化
// 参数说明     pin         选择的引脚 (可选择范围由 common.h 内GPIO_PIN_enum枚举值确定)
// 参数说明     dir         引脚的方向   输出：GPO   输入：GPI
// 参数说明     dat         引脚初始化时设置的电平状态，输出时有效 0：低电平 1：高电平
// 参数说明     pinconf     引脚配置（可设置参数由zf_gpio.h文件内GPIOSPEED_enum与GPIOMODE_enum枚举值确定，多个条件使用 | 相或）
// 返回参数     void
// 使用示例     gpio_init(D0, GPO, 1, GPIO_PIN_CONFIG);//D0初始化为GPIO功能、输出模式、输出高电平、速度100MHZ 推挽输出
//-------------------------------------------------------------------------------------------------------------------
void gpio_init(gpio_pin_enum pin, gpio_dir_enum dir, const uint8 dat, gpio_mode_enum mode)
{
	zf_assert(dir == (mode >> 4));
    gpio_set_level(pin, dat);
	if(dir == GPIO)
	{
		gpio_set_dir(pin, GPIO);
		gpio_set_mode(pin, mode);
	}
	else if(dir == GPI)
	{
		if(mode == GPI_IMPEDANCE)
		{
			gpio_set_impedance(pin);
		}
		else
		{
			gpio_set_dir(pin, GPI);
			gpio_set_mode(pin, mode);
		}
	}
	else if(dir == GPO)
	{
		if(GPO_PUSH_PULL == mode)
		{
			gpio_set_dir(pin, GPO);
//			gpio_set_mode(pin, GPI_PULL_UP);
		}
		else if(GPO_OPEN_DTAIN == mode)
		{
			gpio_set_mode(pin, GPO_OPEN_DTAIN);
		}
	}
}

