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
*                   模块管脚             单片机管脚
*                   SCL                  查看 zf_device_tft180.h 中 TFT180_SCL_PIN 宏定义
*                   SDA                  查看 zf_device_tft180.h 中 TFT180_SDA_PIN 宏定义
*                   RES                  查看 zf_device_tft180.h 中 TFT180_RES_PIN 宏定义
*                   DC                   查看 zf_device_tft180.h 中 TFT180_DC_PIN 宏定义
*                   CS                   查看 zf_device_tft180.h 中 TFT180_CS_PIN 宏定义
*                   BL                   查看 zf_device_tft180.h 中 TFT180_BL_PIN 宏定义
*                   VCC                 3.3V电源
*                   GND                 电源地
*                   最大分辨率160*128
*                   ------------------------------------
********************************************************************************************************************/

#ifndef _zf_device_tft180_h_
#define _zf_device_tft180_h_

#include "zf_common_typedef.h"
#include "zf_device_type.h"

#define TFT180_USE_INTERFACE             (HARDWARE_SPI)                                         // 默认使用硬件 SPI 方式驱动 建议使用硬件 SPI 方式驱动
#if (TFT180_USE_INTERFACE==SOFT_SPI)                                                         // 这两段 颜色正常的才是正确的 颜色灰的就是没有用的
//====================================================软件 SPI 驱动====================================================
	// 暂不支持
//====================================================软件 SPI 驱动====================================================
#elif (TFT180_USE_INTERFACE==HARDWARE_SPI)
//====================================================硬件 SPI 驱动====================================================
	#define TFT180_SPI_SPEED                ((uint32)20 * 1000 * 1000U)             // 硬件 SPI 速率 这里设置为系统时钟二分频
	#define TFT180_SPI                      (SPI_0)                                 // 硬件 SPI 号
	#define TFT180_SCL_PIN                  (SPI0_CH2_SCLK_P27)                     // 硬件 SPI SCK 引脚
	#define TFT180_SDA_PIN                  (SPI0_CH2_MOSI_P25)                     // 硬件 SPI MOSI 引脚
//====================================================硬件 SPI 驱动====================================================
#endif

#define TFT180_RES_PIN                  (P20)                                    // 液晶复位引脚定义
#define TFT180_DC_PIN                   (P21)                                    // 液晶命令位引脚定义
#define TFT180_CS_PIN                   (P22)                                    // CS 片选引脚
#define TFT180_BL_PIN                   (P23)                                    // 液晶背光引脚定义

#define TFT180_DC(x)                    (x? (TFT180_DC_PIN = 1): (TFT180_DC_PIN = 0))
#define TFT180_RES(x)                   (x? (TFT180_RES_PIN = 1): (TFT180_RES_PIN = 0))
#define TFT180_CS(x)                    (x? (TFT180_CS_PIN = 1): (TFT180_CS_PIN = 0))
#define TFT180_BL(x)                    (x? (TFT180_BL_PIN = 1): (TFT180_BL_PIN = 0))

typedef enum
{
    TFT180_PORTAIT                      = 0,                                    // 竖屏模式
    TFT180_PORTAIT_180                  = 1,                                    // 竖屏模式  旋转180
    TFT180_CROSSWISE                    = 2,                                    // 横屏模式
    TFT180_CROSSWISE_180                = 3,                                    // 横屏模式  旋转180
}tft180_dir_enum;

#define TFT180_DEFAULT_DISPLAY_DIR      TFT180_PORTAIT                          // 默认的显示方向
#define TFT180_DEFAULT_PENCOLOR         RGB565_RED                              // 默认的画笔颜色
#define TFT180_DEFAULT_BGCOLOR          RGB565_WHITE                            // 默认的背景颜色

extern uint16 tft180_pencolor;
extern uint16 tft180_bgcolor;

extern tft180_dir_enum tft180_display_dir;
extern uint8 tft180_x_max;
extern uint8 tft180_y_max;

void    tft180_init                     (void);
void    tft180_clear                    (uint16 color);
void    tft180_set_dir                  (tft180_dir_enum dir);
void    tft180_set_color                (uint16 pen, uint16 bgcolor);
void    tft180_draw_point               (uint16 x, uint16 y, uint16 color);

void    tft180_show_char                (uint16 x, uint16 y, const char dat);
void    tft180_show_string              (uint16 x, uint16 y, const char dat[]);
void    tft180_show_int8                (uint16 x, uint16 y, int8 dat);
void    tft180_show_uint8               (uint16 x, uint16 y, uint8 dat);
void    tft180_show_int16               (uint16 x, uint16 y, int16 dat);
void    tft180_show_uint16              (uint16 x, uint16 y, uint16 dat);
void    tft180_show_int32               (uint16 x, uint16 y, int32 dat,uint8 num);
void    tft180_show_float               (uint16 x, uint16 y, double dat,uint8 num,uint8 pointnum);

void    tft180_show_wave                (uint16 x, uint16 y, uint8 *p, uint16 width, uint16 value_max, uint16 dis_width, uint16 dis_value_max);

#endif

