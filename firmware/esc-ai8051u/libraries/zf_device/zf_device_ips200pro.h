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
*      				模块管脚            单片机管脚
*      				CLK                 查看 zf_device_ips200pro.h 中 IPS200PRO_CLK_PIN  宏定义
*      				MOSI                查看 zf_device_ips200pro.h 中 IPS200PRO_MOSI_PIN 宏定义
*      				MISO                查看 zf_device_ips200pro.h 中 IPS200PRO_MISO_PIN 宏定义
*      				RST                 查看 zf_device_ips200pro.h 中 IPS200PRO_RST_PIN  宏定义
*      				INT                 查看 zf_device_ips200pro.h 中 IPS200PRO_INT_PIN  宏定义
*      				CS                  查看 zf_device_ips200pro.h 中 IPS200PRO_CS_PIN   宏定义
*      				GND                 核心板电源地 GND
*      				3V3                 核心板 3V3 电源
*                   最大分辨率160*128
*                   ------------------------------------
********************************************************************************************************************/

#ifndef _zf_device_ips200pro_h_
#define _zf_device_ips200pro_h_

#include "zf_common_typedef.h"


#define IPS200PRO_SPI_SPEED      			    ( 20*1000*1000      	)   // 硬件 SPI 速率
#define IPS200PRO_SPI_INDEX                	    ( SPI_0         		)   // 硬件 SPI 号
#define IPS200PRO_CLK_PIN                  	    ( SPI0_CH2_SCLK_P27  	)   // 硬件 SPI SCK 引脚
#define IPS200PRO_MOSI_PIN                 	    ( SPI0_CH2_MOSI_P25  	)   // 硬件 SPI MOSI 引脚
#define IPS200PRO_MISO_PIN                 	    ( SPI_MISO_NULL			)   // 硬件 SPI MISO 引脚  TFT没有MISO引脚，但是这里任然需要定义，在spi的初始化时需要使用
#define IPS200PRO_RST_PIN                  	    ( IO_P20             	)   // 液晶复位引脚定义
#define IPS200PRO_INT_PIN                  	    ( IO_P21             	)   // 液晶命令位引脚定义
#define IPS200PRO_CS_PIN                   	    ( IO_P22             	)   // CS 片选引脚

#define IPS200PRO_WAIT_TIME                	    ( 900               	)   // 通讯等待时长，内部是软件延时，因此这里没有时间单位
#define IPS200PRO_CRC_ENABLE                    ( 0                 	)   // 0：关闭CRC模式（通常关闭即可） 1：使能CRC模式，在传输的数据包中加入CRC校验，能提高屏幕的抗干扰的能力
#define IPS200PRO_DEFAULT_FORMAT                ( IPS200PRO_FORMAT_UTF8 )  	// 默认编码格式，会在初始化屏幕的时候进行设置
#define IPS200PRO_DEFAULT_FONT_SIZE             ( FONT_SIZE_16      	)   // 默认指定大小，会在初始化屏幕的时候进行设置
#define IPS200PRO_DEFAULT_OPTIMIZE              ( 1                 	)   // 0:关闭优化 1：开启优化，默认开启优化，会在初始化屏幕的时候进行设置

// RGB888转RGB565宏定义
#define IPS200PRO_RGB888_TO_RGB565(r8, g8, b8)  ((((r8 >> 3) & 0x1FU) << 11) | (((g8 >> 2) & 0x3FU) << 5) |  ((b8 >> 3) & 0x1FU))

typedef enum
{
    // 仅16、20、24号字体支持中文显示，其余字体仅支持英文显示
    FONT_SIZE_12,
    FONT_SIZE_14,
    FONT_SIZE_16,						    // 16字体支持中文显示
    FONT_SIZE_18,					
    FONT_SIZE_20,						    // 20字体支持中文显示
    FONT_SIZE_22,					
    FONT_SIZE_24,						    // 24字体支持中文显示
    FONT_SIZE_26,
    FONT_SIZE_28,
    FONT_SIZE_30,
    FONT_SIZE_32,
    FONT_SIZE_34,
    FONT_SIZE_36,
    FONT_SIZE_40,
}ips200pro_font_size_enum;

typedef enum
{
    COLOR_FOREGROUND,                  	    // 前景色
    COLOR_BACKGROUND,                  	    // 背景色      除IMAGE组件不支持其他都支持
    COLOR_BORDER,                      	    // 组件边线颜色 支持组件有LABEL TABLE METER CLOCK CALENDAR CONTAINER
		
    // 专用指令
    COLOR_PAGE_SELECTED_TEXT,          	    // 选中页面后的标题文字颜色
    COLOR_PAGE_SELECTED_BG,            	    // 选中页面后的标题背景颜色
		
    COLOR_TABLE_SELECTED_BG,           	    // 表格选中后的颜色
		
    COLOR_MRTER_INDICATOR,             	    // 仪表组件的指针颜色
    COLOR_MRTER_TICKS,                 	    // 仪表组件刻度颜色
		
    COLOR_CLOCK_HOUR,                  	    // 圆形时钟时针颜色
    COLOR_CLOCK_MINUTE,                	    // 圆形时钟分针颜色
    COLOR_CLOCK_SECOND,                	    // 圆形时钟秒针颜色
    COLOR_CLOCK_TICKS,                 	    // 圆形时钟组件刻度颜色
		
    COLOR_CALENDAR_YEAR,               	    // 年份颜色
    COLOR_CALENDAR_WEEK,               	    // 星期颜色
    COLOR_CALENDAR_TODAY,              	    // 今日颜色
}ips200pro_widgets_color_type_enum;	
	
typedef enum
{
    PAGE_ANIM_OFF,                          // 页面切换时关闭动画效果
    PAGE_ANIM_ON,                           // 页面切换时开启动画效果
}ips200pro_page_animations_enum;

typedef enum	
{	
    LABEL_AUTO,                        	    // 当宽度无法显示全部内容时，会自动换行显示
    LABEL_DOT,                         	    // 当无法全部显示的时候，末尾右下角显示...
    LABEL_SCROLL,                      	    // 当宽度无法显示全部内容时水平滚动显示，当高度无法显示全部内容时垂直滚动显示，只按照一个方向左右或者上下滚动显示，水平滚动优先
    LABEL_SCROLL_CIRCULAR,             	    // 当宽度无法显示全部内容时水平滚动显示，当高度无法显示全部内容时垂直滚动显示，只按照一个方向循环滚动显示，水平滚动优先
    LABEL_CLIP,                        	    // 将无法显示的内容裁剪掉
}ips200pro_label_mode_enum;	
	
typedef enum	
{	
    IPS200PRO_PORTRAIT           = 0x01,    // 竖屏模式
    IPS200PRO_PORTRAIT_180       = 0x02,	// 竖屏模式  旋转180
    IPS200PRO_CROSSWISE          = 0x03,	// 横屏模式
    IPS200PRO_CROSSWISE_180      = 0x04,	// 横屏模式  旋转180
}ips200pro_display_direction_enum;	
	
typedef enum	
{	
    IPS200PRO_TITLE_LEFT         = 0x00,	// 页面标题显示在左侧 如果不需要显示标题，则将标题高度设置为0即可
    IPS200PRO_TITLE_RIGHT        = 0x01,	// 页面标题显示在右侧
    IPS200PRO_TITLE_TOP          = 0x02,	// 页面标题显示在上侧
    IPS200PRO_TITLE_BOTTOM       = 0x03,	// 页面标题显示在底侧
}ips200pro_title_position_enum;	
	
typedef enum	
{	
    IPS200PRO_CALENDAR_CHINESE   = 0x01,	// 日历使用中文显示  仅16、20、24号字体支持中文显示
    IPS200PRO_CALENDAR_ENGLISH   = 0x02,	// 日历使用英文显示
}ips200pro_calendar_mode_enum;

typedef enum
{
    IMAGE_NULL                  = 0x00,     // 图像为空，用于纯边线显示
    IMAGE_GRAYSCALE             = 0x03,	    // 灰度 总钻风、小钻风解压后的图像使用此枚举定义
    IMAGE_RGB565,                      	    // RGB565彩色 凌瞳使用此枚举定义
}ips200pro_image_type_enum;

typedef enum
{
    IMAGE_LINE_TYPE_UINT8       = 0x01,	    // 线条数据是8位类型
    IMAGE_LINE_TYPE_UINT16      = 0x02,	    // 线条数据是16位类型
}ips200pro_image_line_type_enum;

typedef enum
{
    IPS200PRO_FORMAT_GBK         = 0x01,		// GBK编码，开源库默认的文件都是GBK编码
    IPS200PRO_FORMAT_UTF8        = 0x02,		// UTF-8编码
}ips200pro_format_enum;

typedef enum
{
    METER_ANGLE                 = 0x01,	    // 角度指示仪表，可设置字体、背景、边线、刻度、指针的颜色
    METER_SPEED,                       	    // 速度指示仪表，可设置字体、背景、边线、刻度、指针的颜色
}ips200pro_meter_style_enum;	
	
typedef enum	
{	
    CLOCK_DIGITAL               = 0x01,	    // 数字时钟，可设置字体、背景、边线的颜色
    CLOCK_ANALOG,                      	    // 指针时钟，可设置字体、背景、边线、刻度、指针的颜色
}ips200pro_clock_style_enum;	
	
// 图像叠加线条 uint8类型的线条结构体
typedef struct	
{	
    uint8 x;                          	    // 点的横坐标
    uint8 y;                          	    // 点的纵坐标
}ips200pro_image_line_uint8_struct;	
	
// 图像叠加线条 uint16类型的线条结构体
typedef struct	
{	
    uint16 x;                        	    	// 点的横坐标
    uint16 y;                        	    	// 点的纵坐标
}ips200pro_image_line_uint16_struct;

typedef struct
{
    uint16  id;                      	 	   	// 屏幕ID编号
    uint16  width;                   	 	   	// 屏幕最大显示宽度
    uint16  height;                  	 	   	// 屏幕最大显示高度
    uint8   version_major;           	 	   	// 固件版本-主版本
    uint8   version_middle;          	 	   	// 固件版本-中版本
    uint8   version_micro;           	 	   	// 固件版本-微版本
}ips200pro_information_struct;	
	
typedef struct	
{	
    uint16  year;                    	    // 年
    uint8   month;                   	    // 月
    uint8   day;                     	    // 日
    uint8   hour;                    	    // 时
    uint8   minute;                  	    // 分
    uint8   second;                  	    // 秒
    uint8   week;                    	    // 星期
}ips200pro_time_struct;

extern ips200pro_information_struct    ips200pro_information;
extern ips200pro_time_struct           ips200pro_time;

//------------------------------------功能函数一览表-------------------------------------------

//------------------------------------系统功能函数-------------------------------------------
// uint8  ips200pro_set_date        		(uint16 year, uint8 month, uint8 day);                                            	 	// 设置系统日期
// uint8  ips200pro_set_time            	(uint8 hour, uint8 minute, uint8 second);                                            	// 设置系统时间
// uint8  ips200pro_set_parent          	(uint16 child_id, uint16 parent_id);                                                	// 设置组件父对象
// uint8  ips200pro_set_format          	(ips200pro_format_enum format);                                                       	// 设置汉字编码格式
// uint8  ips200pro_set_backlight       	(uint8 backlight);                                                                   	// 设置屏幕背光亮度
// uint8  ips200pro_set_direction       	(ips200pro_display_direction_enum dir);                                               	// 设置屏幕显示方向
// uint8  ips200pro_set_default_font    	(ips200pro_font_size_enum font);                                                      	// 设置默认字体大小
// uint8  ips200pro_set_optimize            (uint8 state)                                                                           // 设置优化（默认是开启的）
//		
// uint8  ips200pro_get_date            	(ips200pro_time_struct *time);                                                        	// 获取系统日期
// uint8  ips200pro_get_time            	(ips200pro_time_struct *time);                                                        	// 获取系统时间
// uint8  ips200pro_get_information     	(ips200pro_information_struct *information);                                          	// 获取系统信息
// uint8  ips200pro_get_free_stack_size 	(uint32 *stack_size);                                                                	// 获取系统空闲栈大小
	
//-----------------------------------通用函数接口-------------------------------------------
// uint8  ips200pro_delete_widgets   	    (uint16 widgets_id);                                                                 	// 删除组件
// uint8  ips200pro_set_font         	    (uint16 widgets_id, ips200pro_font_size_enum font_size);                              	// 设置组件的字体
// uint8  ips200pro_set_color        	    (uint16 widgets_id, ips200pro_widgets_color_type_enum color_type, uint16 color);      	// 设置组件颜色
// uint8  ips200pro_set_position     	    (uint16 widgets_id, int16 x, int16 y);                                               	// 设置组件位置
// uint8  ips200pro_set_hidden       	    (uint16 widgets_id, uint8 state);                                                    	// 设置组件隐藏
	
//-----------------------------------PAGE页面操作接口-------------------------------------------
// uint16 ips200pro_page_create         	(char *str);                                                                         	// 页面创建
// uint8  ips200pro_page_switch         	(uint16 page_id, ips200pro_page_animations_enum anim_en);                               // 页面切换
// uint8  ips200pro_page_hidden         	(uint16 page_id, uint8 state)                                                           // 页面隐藏
// uint8  ips200pro_page_set_title_name 	(uint16 page_id, char *str);                                                            // 页面标题设置
	
//-----------------------------------文本标签操作接口-------------------------------------------
// uint16 ips200pro_label_create       	    (int16 x, int16 y, uint16 width, uint16 height);                                     	// 文本标签创建
// uint8  ips200pro_label_printf       	    (uint16 label_id, const char *format, ...);                                          	// 文本标签内容设置
// uint8  ips200pro_label_show_string       (uint16 label_id, const char *str)                                                      // 文本标签字符串显示
// uint8  ips200pro_label_mode         	    (uint16 label_id, ips200pro_label_mode_enum mode);                                    	// 文本标签模式设置
	
//-----------------------------------表格TABLE操作接口-------------------------------------------
// uint16 ips200pro_table_create        	(int16 x, int16 y, uint16 row_num, uint16 col_num);                                  	// 表格创建
// uint8  ips200pro_table_cell_printf   	(uint16 table_id, uint8 row, uint8 col, char *format, ...);                          	// 表格单元格内容设置
// uint8  ips200pro_table_set_col_width 	(uint16 table_id, uint8 col, uint16 width);                                          	// 表格列宽度设置
// uint8  ips200pro_table_select        	(uint16 table_id, uint8 row, uint8 col);                                             	// 单元格选中
	
//-----------------------------------仪表指示器操作接口-------------------------------------------
// uint16 ips200pro_meter_create      	    (int16 x, int16 y, uint16 size, ips200pro_meter_style_enum style);                    	// 仪表创建
// uint8  ips200pro_meter_set_value   	    (uint16 meter_id, int16 value);                                                      	// 仪表数值设置
	
//-----------------------------------时钟操作接口-------------------------------------------
// uint16 ips200pro_clock_create      	    (int16 x, int16 y, uint16 clock_size, ips200pro_clock_style_enum clock_type)          	// 时钟创建
	
//-----------------------------------进度条操作接口-------------------------------------------
// uint16 ips200pro_progress_bar_create     (int16 x, int16 y, uint16 width, uint16 height);                                     	// 进度条创建
// uint8  ips200pro_progress_bar_set_value  (uint16 progress_bar_id, uint8 start_value, uint8 end_value);                        	// 进度条数值设置
	
//-----------------------------------日历操作接口-------------------------------------------
// uint16 ips200pro_calendar_create    	    (int16 x, int16 y, uint16 width, uint16 height);                                     	// 日历创建
// uint8  ips200pro_calendar_display   	    (uint16 year, uint8 month, uint8 ips200pro_calendar_mode_enum);                       	// 日历显示设置
	
//-----------------------------------波形图操作接口-------------------------------------------
// uint16 ips200pro_waveform_create    	    (int16 x, int16 y, uint16 width, uint16 height);                                     	// 波形图创建
// uint8  ips200pro_waveform_add_value 	    (uint16 waveform_id, uint8 line_id, const uint16 *dat, uint16 length, uint16 color);	// 波形图添加数据
// uint8  ips200pro_waveform_line_state	    (uint16 waveform_id, uint16 line_id, uint16 line_state);                             	// 波形图线条状态设置
// uint8  ips200pro_waveform_line_type 	    (uint16 waveform_id, uint8 line_type);                                               	// 波形图线条类型设置
// uint8  ips200pro_waveform_clear     	    (uint16 waveform_id);                                                                	// 波形图数据清空

//-----------------------------------图像操作接口-------------------------------------------
// uint16 ips200pro_image_create        	(int16 x, int16 y, uint16 width, uint16 height);                                        // 图像组件创建
// uint8  ips200pro_image_display           (image_id, *image, width, height, image_type, threshold);                               // 图像数据更新
// uint8  ips200pro_image_draw_line         (image_id, line_id, *line_data, line_length, data_type, color);                         // 图像上画线 先发送ips200pro_image_draw_line再发送ips200pro_image_display
// uint8  ips200pro_image_draw_rectangle    (image_id, rectangle_id, x, y, rectangle_width, rectangle_height, color);               // 图像上画框 先发送ips200pro_image_draw_rectangle再发送ips200pro_image_display

//-----------------------------------容器操作接口-------------------------------------------
// uint16 ips200pro_container_create    	(int16 x, int16 y, uint16 width, uint16 height);                                     	// 容器组件创建
// uint8  ips200pro_container_radius    	(uint16 container_id, uint16 border_width, uint16 radius);                           	// 设置容器的边线宽度与圆角半径

//-----------------------------------屏幕初始化-------------------------------------------
// uint16 ips200pro_init               	    (char *str, ips200pro_title_position_enum title_position, uint8 title_size);            // 屏幕初始化


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置系统日期
// 参数说明     year            年
// 参数说明     month           月
// 参数说明     day             日
// 返回参数     uint8           状态 0：设置成功  1：设置失败
// 使用示例     ips200pro_set_date(2024, 10, 31);  // 2024年10月31日
//-------------------------------------------------------------------------------------------------------------------
uint8	ips200pro_set_date           	(uint16 year, uint8 month, uint8 day);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置系统时间
// 参数说明     hour            时   24小时制（仅支持24小时制）
// 参数说明     minute          分
// 参数说明     second          秒
// 返回参数     uint8           状态 0：设置成功  1：设置失败
// 使用示例     ips200pro_set_time(13, 19, 15);  // 13点19分15秒
//-------------------------------------------------------------------------------------------------------------------
uint8   ips200pro_set_time             	(uint8 hour, uint8 minute, uint8 second);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置组件父对象
// 参数说明     child_id        子组件ID
// 参数说明     parent_id       父组件ID
// 返回参数     uint8           状态 0：设置成功  1：设置失败
// 使用示例     ips200pro_set_parent(1, label_id, container_id);  // 将label显示到容器上 设置后label的坐标原点为容器的左上角，不再是屏幕的左上角，并且超出容器范围的内容不会进行显示
//-------------------------------------------------------------------------------------------------------------------
uint8   ips200pro_set_parent          	(uint16 child_id, uint16 parent_id);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置中文编码格式
// 参数说明     format          中文编码格式
// 返回参数     uint8           状态 0：设置成功  1：设置失败
// 使用示例     ips200pro_set_format(IPS200PRO_FORMAT_GBK);  // 将ID为1的表格显示在ID为2的页面上
//-------------------------------------------------------------------------------------------------------------------
uint8   ips200pro_set_format          	(ips200pro_format_enum format);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置背光亮度
// 参数说明     backlight       亮度1-255
// 返回参数     uint8           状态 0：设置成功  1：设置失败
// 使用示例     ips200pro_set_backlight(100);  // 背光亮度设置为100
//-------------------------------------------------------------------------------------------------------------------
uint8   ips200pro_set_backlight        	(uint8 backlight);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置屏幕显示方向
// 参数说明     dir             亮度1-255
// 返回参数     uint8           状态 0：设置成功  1：设置失败
// 使用示例     ips200pro_set_backlight(100);  // 背光亮度设置为100
//-------------------------------------------------------------------------------------------------------------------
uint8   ips200pro_set_direction       	(ips200pro_display_direction_enum dir);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置默认字体大小
// 参数说明     font            默认字体大小
// 返回参数     uint8           状态 0：设置成功  1：设置失败
// 使用示例     ips200pro_set_default_font(FONT_SIZE_20);  // 将默认字体设置为20号
//-------------------------------------------------------------------------------------------------------------------
uint8   ips200pro_set_default_font   	(ips200pro_font_size_enum font);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置优化（默认为开启状态）
// 参数说明     font            默认字体大小
// 返回参数     uint8           状态 0：关闭优化  1：开启优化
// 使用示例     ips200pro_set_optimize(1);  // 开启优化
// 备注说明     目前仅针对图像进行优化，当开启优化时，发送图像给屏幕进行显示，如果屏幕正在显示上次的图像，则本次图像直接丢弃，这样避免用户等待指令响应完成，从而尽量避免用户程序等待的问题
//             如果关闭优化，则等待上次图像显示完成之后，再处理新的图像
//-------------------------------------------------------------------------------------------------------------------
uint8   ips200pro_set_optimize           (uint8 state);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取系统日期
// 参数说明     *time           保存时间的指针
// 返回参数     uint8           状态 0：设置成功  1：设置失败
// 使用示例     ips200pro_get_date(&ips200pro_time);  //
//-------------------------------------------------------------------------------------------------------------------
uint8   ips200pro_get_date           	(ips200pro_time_struct *time);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取系统时间
// 参数说明     *time           保存时间的指针
// 返回参数     uint8           状态 0：设置成功  1：设置失败
// 使用示例     ips200pro_get_time(&ips200pro_time);  //
//-------------------------------------------------------------------------------------------------------------------
uint8   ips200pro_get_time           	(ips200pro_time_struct *time);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取空闲栈大小
// 参数说明     *information    保存系统信息的指针
// 返回参数     uint8           状态 0：设置成功  1：设置失败
// 使用示例     ips200pro_get_information(&ips200pro_information);  //
//-------------------------------------------------------------------------------------------------------------------
uint8   ips200pro_get_information    	(ips200pro_information_struct *information);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取系统信息
// 参数说明     *stack_size     保存空闲栈大小的指针
// 返回参数     uint8           状态 0：设置成功  1：设置失败
// 使用示例     ips200pro_get_free_stack_size(&stack_size);  //
//-------------------------------------------------------------------------------------------------------------------
uint8   ips200pro_get_free_stack_size 	(uint32 *stack_size);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     删除组件
// 参数说明     widgets_id      组件的编号
// 返回参数     uint8           状态 0：设置成功  1：设置失败
// 使用示例     ips200pro_delete_widgets(widgets_id);  // 将选中的组件删除
//-------------------------------------------------------------------------------------------------------------------
uint8   ips200pro_delete_widgets     	(uint16 widgets_id);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置组件字体大小
// 参数说明     widgets_id      组件的编号
// 参数说明     font_size       字体大小（仅16 20 24号字体支持中文显示）
// 返回参数     uint8           状态 0：设置成功  1：设置失败
// 使用示例     ips200pro_set_font(widgets_id, FONT_SIZE_20);  // 将选中的组件字体大小设置为20号
//-------------------------------------------------------------------------------------------------------------------
uint8   ips200pro_set_font           	(uint16 widgets_id, ips200pro_font_size_enum font_size);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     组件颜色设置
// 参数说明     widgets_id      组件的编号
// 参数说明     color_type      颜色类型
// 参数说明     color           RGB565数值
// 返回参数     uint8           状态 0：设置成功  1：设置失败
// 使用示例     ips200pro_set_color(widgets_id, COLOR_BORDER, RGB565_RED);  // 将选中的组件边线颜色设置为红色
//-------------------------------------------------------------------------------------------------------------------
uint8   ips200pro_set_color          	(uint16 widgets_id, ips200pro_widgets_color_type_enum color_type, uint16 color);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     组件位置设置
// 参数说明     widgets_id      组件的编号
// 参数说明     x               左上角横轴坐标（以像素为单位）
// 参数说明     y               左上角纵轴坐标（以像素为单位）
// 返回参数     uint8           状态 0：设置成功  1：设置失败
// 使用示例     ips200pro_set_position(widgets_id, 10, 10);  // 将选中的组件起始坐标设置为10,10
//-------------------------------------------------------------------------------------------------------------------
uint8   ips200pro_set_position        	(uint16 widgets_id, int16 x, int16 y);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     组件隐藏设置
// 参数说明     widgets_id      组件的编号
// 参数说明     state           0：取消隐藏  1：将组件隐藏
// 返回参数     uint8           状态 0：设置成功  1：设置失败
// 使用示例     ips200pro_set_hidden(widgets_id, 1);  // 将选中的组件隐藏显示
//-------------------------------------------------------------------------------------------------------------------
uint8   ips200pro_set_hidden          	(uint16 widgets_id, uint8 state);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     创建页面
// 参数说明     *str            页面名称 不需要名称可以直接填写 ""
// 返回参数     uint16          状态 0：创建失败  其他：创建成功，同时返回的数值表示此PAGE的ID
// 使用示例     uint16 page1_id = ips200pro_page_create("Test");  // 创建一个名称为Test的页面
//-------------------------------------------------------------------------------------------------------------------
uint16	ips200pro_page_create         	(char *str);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     切换页面
// 参数说明     page_id         页面ID
// 参数说明     anim_en         动画使能
// 返回参数     uint8           状态 0：设置成功  1：设置失败
// 使用示例     ips200pro_page_switch(page_id, PAGE_ANIM_ON);  // 切换到选中的页面进行显示 开启动画效果
//-------------------------------------------------------------------------------------------------------------------
uint8   ips200pro_page_switch         	(uint16 page_id, ips200pro_page_animations_enum anim_en);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     隐藏页面
// 参数说明     page_id         页面ID
// 参数说明     state           隐藏状态 1：将页面隐藏 0：取消隐藏
// 返回参数     uint8           状态 0：设置成功  1：设置失败
// 使用示例     ips200pro_page_hidden(page_id, 1);  // 将选中的页面隐藏
// 注意事项     ID设置为0，则表示对所有的页面同时进行设置
//-------------------------------------------------------------------------------------------------------------------
uint8   ips200pro_page_hidden			(uint16 page_id, uint8 state);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置页面名称
// 参数说明     page_id         页面ID
// 参数说明     *str            页面名称
// 返回参数     uint8           状态 0：设置成功  1：设置失败
// 使用示例     ips200pro_page_set_title_name(page_id, "Test");  // 将选中的页面，名称设置为Test
//-------------------------------------------------------------------------------------------------------------------
uint8   ips200pro_page_set_title_name	(uint16 page_id, char *str);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     创建文本标签
// 参数说明     x               左上角横轴坐标（以像素为单位）
// 参数说明     y               左上角纵轴坐标（以像素为单位）
// 参数说明     width           组件的宽度
// 参数说明     height          组件的高度
// 返回参数     uint16          状态 0：创建失败  其他：创建成功，同时返回的数值表示此PAGE的ID
// 使用示例     uint16 label_id = ips200pro_label_create(0, 0, 50, 20);  // 创建一个文本标签 左上角坐标0,0 宽度50 高度20
//-------------------------------------------------------------------------------------------------------------------
uint16	ips200pro_label_create     		(int16 x, int16 y, uint16 width, uint16 height);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     文本标签格式化并显示
// 参数说明     label_id        文本标签ID
// 参数说明     *format         字符串
// 参数说明     ...             可变参数列表，方法与printf相同
// 返回参数     uint8           状态 0：成功  1：失败
// 使用示例     ips200pro_label_printf(label_id, "tset=%d", test);
//-------------------------------------------------------------------------------------------------------------------
uint8	ips200pro_label_printf			(uint16 label_id, const char *format, ...);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     文本标签字符串显示
// 参数说明     label_id        文本标签ID
// 参数说明     *str            字符串首地址
// 返回参数     uint8           状态 0：成功  1：失败
// 使用示例     ips200pro_label_show_string(label_id, “This is a test”);
//-------------------------------------------------------------------------------------------------------------------
uint8 ips200pro_label_show_string(uint16 label_id, const char *str);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置文本标签长文本模式
// 参数说明     label_id        文本标签ID
// 参数说明     mode            长文本模式，当文本超过组件可显示范围时的处理方式
// 返回参数     uint8           状态 0：成功  1：失败
// 使用示例     ips200pro_label_mode(label_id, "tset=%d", test);
//-------------------------------------------------------------------------------------------------------------------
uint8	ips200pro_label_mode          	(uint16 label_id, ips200pro_label_mode_enum mode);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     创建表格标签
// 参数说明     x               左上角横轴坐标（以像素为单位）
// 参数说明     y               左上角纵轴坐标（以像素为单位）
// 参数说明     row_num         表格行数量
// 参数说明     col_num         表格列数量
// 返回参数     uint16          状态 0：创建失败  其他：创建成功，同时返回的数值表示此PAGE的ID
// 使用示例     uint16 table2_id = ips200pro_table_create(0, 0, 5, 4);  // 创建5行4列表格 左上角坐标0,0，如需调整宽度使用ips200pro_table_set_col_width函数单独调整
//-------------------------------------------------------------------------------------------------------------------
uint16	ips200pro_table_create        	(int16 x, int16 y, uint16 row_num, uint16 col_num);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     单元格格式化并显示
// 参数说明     table_id        表格ID
// 参数说明     row             单元格所在行 行号从1开始
// 参数说明     col             单元格所在列 列号从1开始
// 参数说明     *format         字符串
// 参数说明     ...             可变参数列表，方法与printf相同
// 返回参数     uint8           状态 0：成功  1：失败
// 使用示例     ips200pro_label_printf(table_id, 1, 1, "tset=%d", test);  // 格式化字符串，并显示在表格的第一行 第一列（即左上角的单元格）
//-------------------------------------------------------------------------------------------------------------------
uint8	ips200pro_table_cell_printf    	(uint16 table_id, uint8 row, uint8 col, char *format, ...);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置表格列宽度
// 参数说明     table_id        表格ID
// 参数说明     col             需要调整的列号 列号从1开始
// 参数说明     width           列宽度（以像素为单位）
// 返回参数     uint8           状态 0：成功  1：失败
// 使用示例     ips200pro_table_set_col_width(table_id, 1, 20);  // 将选中的表格 的第一列（即最左侧的一列）宽度设置为20
//-------------------------------------------------------------------------------------------------------------------
uint8	ips200pro_table_set_col_width	(uint16 table_id, uint8 col, uint16 width);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     表格选中
// 参数说明     table_id        表格ID
// 参数说明     row             需要选中的行号 行号从1开始
// 参数说明     col             需要选中的列号 列号从1开始
// 返回参数     uint8           状态 0：成功  1：失败
// 使用示例     ips200pro_table_select(table_id, 1, 2);  // 将选中的表格 的第一行、第一列（即最左上角）选中
// 备注说明     选中整列：指定col row设置为0x00  选中整行：指定row col设置为0x00 取消选中row col都设置为0x00
//-------------------------------------------------------------------------------------------------------------------
uint8	ips200pro_table_select        	(uint16 table_id, uint8 row, uint8 col);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     创建仪表指示器
// 参数说明     x               左上角横轴坐标（以像素为单位）
// 参数说明     y               左上角纵轴坐标（以像素为单位）
// 参数说明     size            仪表直径（以像素为单位）
// 参数说明     style           仪表样式
// 返回参数     uint16          状态 0：创建失败  其他：创建成功，同时返回的数值表示此PAGE的ID
// 使用示例     uint16 table2_id = ips200pro_meter_create(0, 0, 60, 1);  // 1：角度指示器 2：速度指示器
//-------------------------------------------------------------------------------------------------------------------
uint16	ips200pro_meter_create        	(int16 x, int16 y, uint16 size, ips200pro_meter_style_enum style);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     仪表指示器数值设置
// 参数说明     meter_id        仪表ID
// 参数说明     value           数值	角度类型数值范围0-360 速度类型设置范围0-100
// 返回参数     uint8           状态 0：成功  1：失败
// 使用示例     ips200pro_meter_set_value(meter_id, 50);  // 将选中的仪表数值设置为50
//-------------------------------------------------------------------------------------------------------------------
uint8	ips200pro_meter_set_value     	(uint16 meter_id, int16 value);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     创建时钟
// 参数说明     x               左上角横轴坐标（以像素为单位）
// 参数说明     y               左上角纵轴坐标（以像素为单位）
// 参数说明     clock_size      时钟大小，数字时钟时：表示字体大小，可设置范围查看ips200pro_font_size_enum枚举体
//                             指针时钟：表示时钟的直径，最小设置为80，最大240
// 参数说明     clock_type      时钟类型查看ips200pro_clock_style_enum枚举定义
// 返回参数     uint16          状态 0：创建失败  其他：创建成功，同时返回的数值表示此PAGE的ID
// 使用示例     uint16 clock2_id = ips200pro_clock_create(0, 0, 150, CLOCK_ANALOG);  //
// 备注说明     当宽度与高度一致时，时钟为圆形指针时钟，否则为数字时钟
// 备注说明     如果需要修改时间，使用ips200pro_set_time函数修改
//-------------------------------------------------------------------------------------------------------------------
uint16	ips200pro_clock_create        	(int16 x, int16 y, uint16 clock_size, ips200pro_clock_style_enum clock_type);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     创建进度条
// 参数说明     x               左上角横轴坐标（以像素为单位）
// 参数说明     y               左上角纵轴坐标（以像素为单位）
// 参数说明     width           组件的宽度
// 参数说明     height          组件的高度
// 返回参数     uint16          状态 0：创建失败  其他：创建成功，同时返回的数值表示此PAGE的ID
// 使用示例     uint16 bar2_id = ips200pro_progress_bar_create(0, 0, 60, 10);  //
// 备注说明     当宽度与高度一致时，为圆形进度条，否为长条形进度条
//-------------------------------------------------------------------------------------------------------------------
uint16	ips200pro_progress_bar_create  	(int16 x, int16 y, uint16 width, uint16 height);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     进度条数值设置
// 参数说明     progress_bar_id 进度条ID
// 参数说明     start_value     开始数值
// 参数说明     end_value       结束数值
// 返回参数     uint8           状态 0：成功  1：失败
// 使用示例     ips200pro_progress_bar_set_value(progress_bar_id, 10, 60);  //
//-------------------------------------------------------------------------------------------------------------------
uint8	ips200pro_progress_bar_set_value	(uint16 progress_bar_id, uint8 start_value, uint8 end_value);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     创建日历
// 参数说明     x               左上角横轴坐标（以像素为单位）
// 参数说明     y               左上角纵轴坐标（以像素为单位）
// 参数说明     width           组件的宽度
// 参数说明     height          组件的高度
// 返回参数     uint16          状态 0：创建失败  其他：创建成功，同时返回的数值表示此PAGE的ID
// 使用示例     uint16 test_id = ips200pro_calendar_create(0, 0, 200, 200);  //
//-------------------------------------------------------------------------------------------------------------------
uint16	ips200pro_calendar_create     	(int16 x, int16 y, uint16 width, uint16 height);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     日历显示设置
// 参数说明     year            需要显示的年份 如果填写0则显示当前日期的月份
// 参数说明     month           需要显示的月份 如果填写0则显示当前日期的月份
// 参数说明     mode            显示模式  中文显示（仅16、20、24号字体支持中文显示）
// 返回参数     uint8           状态 0：成功  1：失败
// 使用示例     ips200pro_calendar_display(2036, 1, IPS200PRO_CALENDAR_CHINESE);  // 日历显示2036年1月份 中文显示
//-------------------------------------------------------------------------------------------------------------------
uint8	ips200pro_calendar_display		(uint16 year, uint8 month, ips200pro_calendar_mode_enum mode);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     创建波形图
// 参数说明     x               左上角横轴坐标（以像素为单位）
// 参数说明     y               左上角纵轴坐标（以像素为单位）
// 参数说明     width           组件的宽度
// 参数说明     height          组件的高度
// 返回参数     uint16          状态 0：创建失败  其他：创建成功，同时返回的数值表示此PAGE的ID
// 使用示例     uint16 test_id = ips200pro_waveform_create(0, 0, 200, 200);  //
//-------------------------------------------------------------------------------------------------------------------
uint16	ips200pro_waveform_create     	(int16 x, int16 y, uint16 width, uint16 height);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     波形图添加点
// 参数说明     waveform_id     波形图组件ID
// 参数说明     line_id         线条ID    ID从1开始
// 参数说明     *dat           增加的点数据地址
// 参数说明     length          增加的点数量
// 参数说明     color           点颜色
// 返回参数     uint8           状态 0：成功  1：失败
// 使用示例     ips200pro_waveform_add_value(1, 1, &point[0], 10, RGB565_RED);  //
// 备注说明     波形图显示的点数量上限为组件宽度，当收到的点数量超过组件宽度时优先显示最近收到的点数据
//-------------------------------------------------------------------------------------------------------------------
uint8	ips200pro_waveform_add_value   	(uint16 waveform_id, uint8 line_id, uint16 *dat, uint16 length, uint16 color);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置波形图线条状态
// 参数说明     waveform_id     波形图组件ID
// 参数说明     line_id         线条ID    从1开始 最多支持叠加5条线条
// 参数说明     line_state      0：隐藏线条  1：显示线条
// 返回参数     uint8           状态 0：成功  1：失败
// 使用示例     ips200pro_waveform_line_state(waveform_id, 1, 0);  //
//-------------------------------------------------------------------------------------------------------------------
uint8	ips200pro_waveform_line_state  	(uint16 waveform_id, uint16 line_id, uint16 line_state);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置波形图线条类型
// 参数说明     waveform_id     波形图组件ID
// 参数说明     line_type       线条类型 0：散点图（点与点之间没有连线） 1：（所有的点连成一条线）
// 返回参数     uint8           状态 0：成功  1：失败
// 使用示例     ips200pro_waveform_line_type(waveform_id, 0);  //
//-------------------------------------------------------------------------------------------------------------------
uint8	ips200pro_waveform_line_type   	(uint16 waveform_id, uint8 line_type);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     波形图清除数据（即清空显示）
// 参数说明     waveform_id     波形图组件ID
// 返回参数     uint8           状态 0：成功  1：失败
// 使用示例     ips200pro_waveform_clear(waveform_id);  //
//-------------------------------------------------------------------------------------------------------------------
uint8   ips200pro_waveform_clear     	(uint16 waveform_id);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     创建图像组件
// 参数说明     x               左上角横轴坐标（以像素为单位）
// 参数说明     y               左上角纵轴坐标（以像素为单位）
// 参数说明     width           组件的宽度
// 参数说明     height          组件的高度
// 返回参数     uint16          状态 0：创建失败  其他：创建成功，同时返回的数值表示此PAGE的ID
// 使用示例     uint16 test_id = ips200pro_image_create(0, 0, 188, 120);  //
//-------------------------------------------------------------------------------------------------------------------
uint16	ips200pro_image_create        	(int16 x, int16 y, uint16 width, uint16 height);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     图像显示
// 参数说明     image_id        图像组件ID
// 参数说明     *image          图像地址    NULL则不发送图像，仅通知屏幕更新边线或矩形
// 参数说明     width           图像宽度    0则不发送图像数据，仅通知屏幕更新边线或矩形
// 参数说明     height          图像高度    0则不发送图像数据，仅通知屏幕更新边线或矩形
// 参数说明     image_type      图像类型    总钻风：IMAGE_GRAYSCALE  凌瞳：IMAGE_RGB565  小钻风：需要自行解压后选择IMAGE_GRAYSCALE  IMAGE_NULL：不发送图像数据，仅通知屏幕更新边线或矩形
// 参数说明     threshold       二值化阈值   仅在IMAGE_GRAYSCALE（灰度）模式下有效 0:不启用二值化 其他：启用二值化
// 返回参数     uint8           状态 0：成功  1：失败
// 使用示例     ips200pro_image_display(image_id, mt9v03x_image[0], 188, 120, IMAGE_GRAYSCALE, 0);  //
// 备注说明     如果图像宽度、高度与组件的宽度、高度不一致，则会自动缩放，但是缩放会导致屏幕性能下降，从而可能导致显示帧率下降
//-------------------------------------------------------------------------------------------------------------------
uint8	ips200pro_image_display      	(uint16 image_id, const void *image, uint16 width, uint16 height, ips200pro_image_type_enum image_type, uint8 threshold);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     图像叠加线条
// 参数说明     image_id        图像组件ID
// 参数说明     line_id         线条ID    从1开始 最多支持叠加10条线条
// 参数说明     *line_data      线条数据地址  只能使用ips200pro_image_line_uint8_struct或者ips200pro_image_line_uint16_struct类型
// 参数说明     line_length     线条长度
// 参数说明     data_type       线条的数据类型 8为类型与16为类型
// 参数说明     color           线条颜色
// 返回参数     uint8           状态 0：成功  1：失败
// 使用示例     ips200pro_image_draw_line(image_id, 1, line, 120, IMAGE_LINE_TYPE_UINT8, RGB565_RED);  //
// 备注说明     务必在ips200pro_image_display调用函数之前 调用本函数才能实现线条叠加显示
//-------------------------------------------------------------------------------------------------------------------
uint8   ips200pro_image_draw_line     	(uint16 image_id, uint8 line_id, void *line_data, uint16 line_length, ips200pro_image_line_type_enum data_type, uint16 color);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     图像叠加矩形框
// 参数说明     image_id        图像组件ID
// 参数说明     rectangle_id    矩形框ID    从1开始 最多支持叠加5个矩形框
// 参数说明     x               矩形框左上角横坐标
// 参数说明     y               矩形框左上角纵坐标
// 参数说明     rectangle_width 矩形宽度
// 参数说明     rectangle_height 矩形高度
// 参数说明     color           线条颜色
// 返回参数     uint8           状态 0：成功  1：失败
// 使用示例     ips200pro_image_draw_rectangle(image_id, 1, 10, 10, 20, 30, RGB565_RED);  //
// 备注说明     务必在ips200pro_image_display调用函数之前 调用本函数才能实现矩形框叠加显示
//-------------------------------------------------------------------------------------------------------------------
uint8	ips200pro_image_draw_rectangle	(uint16 image_id, uint8 rectangle_id, int16 x, int16 y, uint16 rectangle_width, uint16 rectangle_height, uint16 color);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     创建容器组件
// 参数说明     x               左上角横轴坐标（以像素为单位）
// 参数说明     y               左上角纵轴坐标（以像素为单位）
// 参数说明     width           组件的宽度
// 参数说明     height          组件的高度
// 返回参数     uint16          状态 0：创建失败  其他：创建成功，同时返回的数值表示此PAGE的ID
// 使用示例     uint16 test_id = ips200pro_container_create(0, 0, 60, 60);  //
//-------------------------------------------------------------------------------------------------------------------
uint16	ips200pro_container_create		(int16 x, int16 y, uint16 width, uint16 height);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置容器圆角半径
// 参数说明     container_id    容器的ID
// 参数说明     border_width    容器周围边线宽度
// 参数说明     radius          容器圆角半径
// 返回参数     uint8           状态 0：创建失败  其他：创建成功，同时返回的数值表示此PAGE的ID
// 使用示例     ips200pro_container_radius(container_id, 1, 10);  // 将选择的容器 线条宽度设置为1，圆角半径设置为10
// 注意事项     如果容器为宽度高度相同，圆角半径是高度的一半，则最终容易为圆形
//-------------------------------------------------------------------------------------------------------------------
uint8	ips200pro_container_radius		(uint16 container_id, uint16 border_width, uint16 radius);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     屏幕初始化
// 参数说明     str             页面标题名称  NULL:不创建页面  不为空：则创建一个页面，并将此字符串设置为页面名称
// 参数说明     title_position  标题显示的位置
// 参数说明     title_size      标题大小 如果不需要标题设置为0即可
// 返回参数     uint16          页面ID
// 使用示例     ips200pro_init("测试", IPS200PRO_TITLE_BOTTOM, 30);  // 初始化屏幕并创建一个标题为测试的页面、标题显示在底部、标题宽度为30
//-------------------------------------------------------------------------------------------------------------------
uint16  ips200pro_init					(char *str, ips200pro_title_position_enum title_position, uint8 title_size);



#endif

