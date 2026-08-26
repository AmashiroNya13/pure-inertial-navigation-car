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

#include "zf_common_debug.h"
#include "zf_common_function.h"
#include "zf_driver_delay.h"
#include "zf_driver_spi.h"
#include "zf_device_type.h"
#include "zf_device_ips200pro.h"

#define IPS200PRO_SPI_LENGTH    ( 4096 )    // 每次SPI通讯最大长度 不可修改

// 不可修改
#define MAX_ID_PAGE             ( 30 )
#define MAX_ID_LABEL            ( 50 )
#define MAX_ID_TABLE            ( 20 )
#define MAX_ID_METER            ( 10  )
#define MAX_ID_CLOCK            ( 1  )
#define MAX_ID_PROGRESS_BAR     ( 20 )
#define MAX_ID_CALENDAR         ( 1  )
#define MAX_ID_WAVEFORM         ( 5  )
#define MAX_ID_IMAGE            ( 5  )
#define MAX_ID_IMAGE_LINE       ( 10 )
#define MAX_ID_IMAGE_RECTANGLE  ( 5  )

ips200pro_information_struct    ips200pro_information;
ips200pro_time_struct           ips200pro_time;
static uint8                    ips200pro_page_num = 0;
static char                     ips200pro_printf_buffer[51];
typedef enum
{
    IPS200PRO_PARAMETER_SET    		= 0x01,	// 设置系统参数命令
    IPS200PRO_PARAMETER_GET,                // 获取系统参数命令
    IPS200PRO_WIDGETS_PAGE     		= 0x10,	// 页面组件
    IPS200PRO_WIDGETS_LABEL,                // 文本标签组件
    IPS200PRO_WIDGETS_TABLE,                // 表格组件
    IPS200PRO_WIDGETS_METER,                // 仪表组件
    IPS200PRO_WIDGETS_CLOCK,                // 时钟组件
    IPS200PRO_WIDGETS_BAR,                  // 进度条组件
    IPS200PRO_WIDGETS_CALENDAR,             // 日历组件
    IPS200PRO_WIDGETS_WAVEFORM,             // 波形组件
    IPS200PRO_WIDGETS_IMAGE,                // 图像组件
    IPS200PRO_WIDGETS_CONTAINER,            // 容器组件
    IPS200PRO_WIDGETS_MAX,                  // 占位使用
}ips200pro_command1_enum;

// 系统参数的子命令与屏幕组件的子命令
typedef enum
{
    // 可读可写的参数
    IPS200PRO_SYSTEM_DATE            = 0x01,// 系统日期
    IPS200PRO_SYSTEM_TIME,         	        // 系统时间
    IPS200PRO_SYSTEM_PARENT,               	// 父对象
    IPS200PRO_SYSTEM_CODED_FORMAT,         	// 编码格式
    IPS200PRO_SYSTEM_BACKLIGHT,            	// 背光亮度
    IPS200PRO_SYSTEM_DIRECTION,            	// 屏幕显示方向
    IPS200PRO_SYSTEM_CRC_STATE,            	// CRC使能状态
    IPS200PRO_SYSTEM_FONT_SIZE,            	// 全局字体
    SCREEN_SYSTEM_OPTIMIZE,                 // 优化 目前仅针对图像进行优化(默认优化开启)
    IPS200PRO_SYSTEM_THEME,                	// 系统主题
    IPS200PRO_SYSTEM_SET_MAX,              	// 占位使用

    // 仅可读的参数
    IPS200PRO_SYSTEM_INFORMATION   	= 0x10, // 屏幕ID编号、分辨率、固件版本
    IPS200PRO_SYSTEM_FREE_STACK,           	// 系统空闲栈大小
    IPS200PRO_SYSTEM_GET_MAX,              	// 占位使用

    // 通用操作命令
    IPS200PRO_COMMON_CREATE        	= 0x01, // 组件创建
    IPS200PRO_COMMON_DELETE,               	// 组件删除
    IPS200PRO_COMMON_FONT_SIZE,            	// 组件字体大小
    IPS200PRO_COMMON_COLOR,                	// 组件颜色
    IPS200PRO_COMMON_VALUE,                	// 组件数值 不同组件数据类型不同
    IPS200PRO_COMMON_POSITION,             	// 组件位置
    IPS200PRO_COMMON_HIDDEN,               	// 组件隐藏
    IPS200PRO_COMMON_MAX,                  	// 占位

    // PAGE组件专用命令
    IPS200PRO_PAGE_SWITCH          	= 0x10, // 切换页面
    IPS200PRO_PAGE_TITLE,                  	// 设置页面标题显示位置与宽度
    IPS200PRO_PAGE_MAX,

    // LABEL组件专用命令
    IPS200PRO_LABEL_LONG_MODE      	= 0x10, // 长文本模式
    IPS200PRO_LABEL_MAX,

    // TABLE组件专用命令
    IPS200PRO_TABLE_COL_WIDTH      	= 0x10, // 表格列宽度设置  行高由屏幕自动调整，无法设置
    IPS200PRO_TABLE_SELECT,                	// 表格中单元格选中
    IPS200PRO_TABLE_MAX,

    // WAVEFORM组件专用命令
    IPS200PRO_WAVEFORM_LINE_STATE  	= 0x10, // 隐藏指定线条
    IPS200PRO_WAVEFORM_LINE_TYPE,          	// 设置组件中线条的类型
    IPS200PRO_WAVEFORM_CLEAR,              	// 将所有数据清空
    IPS200PRO_WAVEFORM_MAX,

    // IMAGE组件专用命令
    IPS200PRO_IMAGE_DRAW_LINE      	= 0x10, // 图像画线
    IPS200PRO_IMAGE_DRAW_RECTANGLE,        	// 图像画框
    IPS200PRO_IMAGE_MAX,
}ips200pro_command2_enum;


// 这里使用宏定义的方式，目的是为了避免使用匿名结构体，导致部分IDE无法在线调试的时候查看匿名结构体的数据
#define IPS200PRO_HEADER   \
    uint8   command1;                   	/* 命令1     */ \
    uint8   command2;                   	/* 命令2     */ \
    uint8   check_crc8;                 	/* CRC校验、默认未启用   */ \
    uint8   widgets_id;                 	/* ID编号    */ \
    uint32  length                      	/* 数据包长度 */ \

typedef struct
{
    IPS200PRO_HEADER;
}ips200pro_header_struct;

//// 数据拆分联合体
//typedef union
//{
//    int8    int8_data[2];               	// 有符号字节数据
//    uint8   uint8_data[2];              	// 无符号字节数据
//    int16   int16_data;                 	// 有符号半字数据
//    uint16  uint16_data;                	// 无符号半字数据
//}data_split_union;

// 通用结构体
#define IPS200PRO_COMMON_STRUCT(nam, num) \
    struct common_packet\
    {\
        IPS200PRO_HEADER;  \
        uint16 dat[num]; \
    }nam;

#define ips200pro_write_8bit_data_spi_array(dat, len)                 (spi_write_8bit_array(IPS200PRO_SPI_INDEX, (dat), (len)))
#define ips200pro_transfer_8bit_data_spi_array(tx_data, rx_data, len)  (spi_transfer_8bit(IPS200PRO_SPI_INDEX, (tx_data), (rx_data), (len)))
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     屏幕等待函数
// 参数说明     wait_time       等待时间
// 返回参数     uint8           1：超时退出 0：未超时
// 使用示例
// 备注信息     内部使用，用户无需关心
//-------------------------------------------------------------------------------------------------------------------
static uint8 ips200pro_wait_idle (uint32 wait_time)
{
    wait_time = wait_time * 100;

    while(0 == gpio_get_level(IPS200PRO_INT_PIN) && (0 != wait_time))
    {
        func_soft_delay(1000);
        wait_time--;
    }
    return (!wait_time);
}

#if(1 == IPS200PRO_CRC_ENABLE)
static uint8 ips200pro_calculate_crc8(uint8 *dat, uint32 length)
{
    uint8 i, crc = 0;
    while(length--)
    {
        crc ^= *dat++;
        for (i = 0; i < 8; i++)
        {
            crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc <<= 1);
        }
    }
    return crc;
}
#endif

uint8 ips200pro_send_buffer(const void *buffer, uint32 length, uint32 time_out, uint8 end_flag)
{
    uint8 return_state = 1;

    if(     ((0 == time_out) || (0 == ips200pro_wait_idle(time_out)))  	// 等待未超时
        &&  (IPS200PRO_SPI_LENGTH >= length)                           	// 数据量未超过限制
        &&  (NULL != buffer))                                       	// 指针不为空
    {
        gpio_low(IPS200PRO_CS_PIN);
        ips200pro_write_8bit_data_spi_array((const uint8 *)buffer, length);
        if(1 == end_flag)
        {
            gpio_high(IPS200PRO_CS_PIN);
        }
        return_state = 0;
    }
    return return_state;
}


uint8 ips200pro_receive_buffer(void *buffer, uint32 length, uint32 time_out)
{
    uint8 return_state = 1;

    if(     ((0 == time_out) || (0 == ips200pro_wait_idle(time_out)))  	// 等待未超时
        &&  (IPS200PRO_SPI_LENGTH >= length)                           	// 数据量未超过限制
        &&  (NULL != buffer))                                       	// 指针不为空
    {
        gpio_low(IPS200PRO_CS_PIN);
        ips200pro_transfer_8bit_data_spi_array((const uint8 *)buffer, (uint8 *)buffer, length);
        gpio_high(IPS200PRO_CS_PIN);
        return_state = 0;
    }

    return return_state;
}

uint8 ips200pro_write_packet(ips200pro_command1_enum command1, ips200pro_command2_enum command2, uint8 widgets_id, ips200pro_header_struct *temp, uint32 length, const void *buffer, uint32 buffer_length)
{
	uint8 return_state  = 1;
	uint8 *p = (uint8 *)temp;
	uint8 tmp = 0;
	uint16 i = 0;
	const uint16 struct_len = sizeof(ips200pro_header_struct);
	
    temp->command1      = command1;
    temp->command2      = command2;
    temp->widgets_id    = widgets_id;
    temp->length        = length + buffer_length;
#if(1 == IPS200PRO_CRC_ENABLE)
    temp->check_crc8    = 0;
    temp->check_crc8    = ips200pro_calculate_crc8((uint8 *)temp, length);
#endif


	// 四位uint32长度进行大小端交换
    temp->length = ((temp->length & 0x000000FF) << 24) |
                   ((temp->length & 0x0000FF00) << 8)  |
                   ((temp->length & 0x00FF0000) >> 8)  |
                   ((temp->length & 0xFF000000) >> 24);

	// 两位uint16数据进行大小端交换
	for(i = struct_len; i < length; i += 2)
	{
		tmp = p[i];
		p[i] = p[i + 1];
		p[i + 1] = tmp;
	}
	
    return_state = ips200pro_send_buffer(temp, length, IPS200PRO_WAIT_TIME, NULL == buffer);
    if((0 == return_state) && (NULL != buffer) && (0 != buffer_length))
    {
        return_state = ips200pro_send_buffer(buffer, buffer_length, 0, 1);
    }
    else
    {
        gpio_high(IPS200PRO_CS_PIN);
    }
    return return_state;
}

uint8 ips200pro_read_parameter(ips200pro_command2_enum command2, ips200pro_header_struct *temp, uint8 length)
{
    uint8 return_state;

    temp->command1   = IPS200PRO_PARAMETER_GET;
    temp->command2   = command2;
    temp->length     = length;
    // 发送命令
    return_state    = ips200pro_send_buffer(temp, length, IPS200PRO_WAIT_TIME, 1);
    // 读取数据
    temp->command1  = 0x00;
    return_state    = ips200pro_receive_buffer(temp, length, IPS200PRO_WAIT_TIME);
    //*parameter = temp.dat[0].uint8_data[0];
    return return_state;
}



uint8 ips200pro_set_date(uint16 year, uint8 month, uint8 day)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 2);

    temp.dat[0]  = year;
    temp.dat[1]  = month || day << 8;

    return_state = ips200pro_write_packet(IPS200PRO_PARAMETER_SET, IPS200PRO_SYSTEM_DATE, 0, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    return return_state;
}

uint8 ips200pro_set_time(uint8 hour, uint8 minute, uint8 second)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 2);
	
    temp.dat[0] = hour || minute << 8;
    temp.dat[1] = second || 0 << 8;
	
    return_state = ips200pro_write_packet(IPS200PRO_PARAMETER_SET, IPS200PRO_SYSTEM_TIME, 0, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    return return_state;
}

uint8 ips200pro_set_parent(uint16 child_id, uint16 parent_id)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 2);

	temp.dat[0] = child_id;
	temp.dat[1] = parent_id;
	
    return_state = ips200pro_write_packet(IPS200PRO_PARAMETER_SET, IPS200PRO_SYSTEM_PARENT, 0, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    return return_state;
}

uint8 ips200pro_set_format(ips200pro_format_enum format)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 1);

	temp.dat[0] = format;
	
    return_state = ips200pro_write_packet(IPS200PRO_PARAMETER_SET, IPS200PRO_SYSTEM_CODED_FORMAT, 0, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    return return_state;
}

uint8 ips200pro_set_backlight(uint8 backlight)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 1);
	
	temp.dat[0]  = backlight;
	
    return_state = ips200pro_write_packet(IPS200PRO_PARAMETER_SET, IPS200PRO_SYSTEM_BACKLIGHT, 0, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    return return_state;
}

uint8 ips200pro_set_direction(ips200pro_display_direction_enum dir)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 1);

	temp.dat[0]  = dir;
	
    return_state = ips200pro_write_packet(IPS200PRO_PARAMETER_SET, IPS200PRO_SYSTEM_DIRECTION, 0, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    return return_state;
}

uint8 ips200pro_set_crc_state(uint8 crc_state)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 1);
	
	temp.dat[0]  = crc_state;
	
    return_state = ips200pro_write_packet(IPS200PRO_PARAMETER_SET, IPS200PRO_SYSTEM_CRC_STATE, 0, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    return return_state;
}

uint8 ips200pro_set_default_font(ips200pro_font_size_enum font)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 1);
	
	temp.dat[0]  = font;
	
    return_state = ips200pro_write_packet(IPS200PRO_PARAMETER_SET, IPS200PRO_SYSTEM_FONT_SIZE, 0, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    return return_state;
}

uint8 ips200pro_set_optimize(uint8 state)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 1);

	temp.dat[0]  = state;
	
    return_state = ips200pro_write_packet(IPS200PRO_PARAMETER_SET, SCREEN_SYSTEM_OPTIMIZE, 0, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    return return_state;
}

uint8 ips200pro_get_date(ips200pro_time_struct *time)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 2);

    return_state = ips200pro_read_parameter(IPS200PRO_SYSTEM_DATE, (ips200pro_header_struct *)&temp, sizeof(temp));
    if(0 == return_state)
    {
        memcpy(&time->year, &(temp.dat[0]), 4);
    }
    return return_state;
}

uint8 ips200pro_get_time(ips200pro_time_struct *time)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 2);

    return_state = ips200pro_read_parameter(IPS200PRO_SYSTEM_TIME, (ips200pro_header_struct *)&temp, sizeof(temp));
    if(0 == return_state)
    {
        memcpy(&time->hour, &(temp.dat[0]), 4);
    }
    return return_state;
}

uint8 ips200pro_get_information(ips200pro_information_struct *information)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 4);

    return_state = ips200pro_read_parameter(IPS200PRO_SYSTEM_INFORMATION, (ips200pro_header_struct *)&temp, sizeof(temp));
    if(0 == return_state)
    {
        information->version_major  = temp.widgets_id;              // 主版本
        memcpy(&information->id, &(temp.dat[0]), 8);
    }
    return return_state;
}

uint8 ips200pro_get_free_stack_size(uint32 *stack_size)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 2);

    return_state = ips200pro_read_parameter(IPS200PRO_SYSTEM_FREE_STACK, (ips200pro_header_struct *)&temp, sizeof(temp));
    if(0 == return_state)
    {
        *stack_size = *((uint32 *)&temp.dat[0]);
    }
    return return_state;
}

uint8 ips200pro_create_widgets(uint16 widgets_id, int16 x, int16 y, uint16 width, uint16 height)
{
    uint8 return_state = 1;
    IPS200PRO_COMMON_STRUCT(temp, 4);

    temp.dat[0]    = x;
    temp.dat[1]    = y;
    temp.dat[2]    = width;
    temp.dat[3]    = height;

    if(ips200pro_page_num || (IPS200PRO_WIDGETS_PAGE == (widgets_id >> 8)))
    {
        // 只有在页面已经创建后，才允许创建其他组件
        return_state = ips200pro_write_packet(widgets_id >> 8, IPS200PRO_COMMON_CREATE, (uint8)widgets_id, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    }

    return return_state;
}

uint8 ips200pro_delete_widgets(uint16 widgets_id)
{
    uint8 return_state;
    ips200pro_header_struct temp;

    return_state = ips200pro_write_packet(widgets_id >> 8, IPS200PRO_COMMON_DELETE, (uint8)widgets_id, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    return return_state;
}

uint8 ips200pro_set_font(uint16 widgets_id, ips200pro_font_size_enum font_size)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 1);

    temp.dat[0] = font_size;
    return_state = ips200pro_write_packet(widgets_id >> 8, IPS200PRO_COMMON_FONT_SIZE, (uint8)widgets_id, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    return return_state;
}

uint8 ips200pro_set_color(uint16 widgets_id, ips200pro_widgets_color_type_enum color_type, uint16 color)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 2);

    temp.dat[0] = color_type;
    temp.dat[1] = color;
    return_state = ips200pro_write_packet(widgets_id >> 8, IPS200PRO_COMMON_COLOR, (uint8)widgets_id, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    return return_state;
}

uint8 ips200pro_set_position(uint16 widgets_id, int16 x, int16 y)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 2);

    temp.dat[0]	= x;
    temp.dat[1]	= y;
    return_state = ips200pro_write_packet(widgets_id >> 8, IPS200PRO_COMMON_POSITION, (uint8)widgets_id, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    return return_state;
}

uint8 ips200pro_set_hidden(uint16 widgets_id, uint8 state)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 1);

    temp.dat[0] = state;
    return_state = ips200pro_write_packet(widgets_id >> 8, IPS200PRO_COMMON_HIDDEN, (uint8)widgets_id, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    return return_state;
}


uint16 ips200pro_page_create(char *str)
{
    uint8 return_state = 1;

    if(MAX_ID_PAGE > ips200pro_page_num)
    {
        return_state = ips200pro_create_widgets(++ips200pro_page_num | (IPS200PRO_WIDGETS_PAGE << 8), 0, 0, 0, 0);
        if(1 == return_state)
        {
            ips200pro_page_num--;
        }
        else if(NULL != str)
        {
           return_state = ips200pro_page_set_title_name(ips200pro_page_num, str);
        }
    }
    return return_state == 1 ? 0 : (ips200pro_page_num | (IPS200PRO_WIDGETS_PAGE << 8));
}

uint8 ips200pro_page_switch(uint16 page_id, ips200pro_page_animations_enum anim_en)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 1);

    temp.dat[0] = anim_en;
    return_state = ips200pro_write_packet(IPS200PRO_WIDGETS_PAGE, IPS200PRO_PAGE_SWITCH, (uint8)page_id, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    return return_state;
}

uint8 ips200pro_page_hidden(uint16 page_id, uint8 state)
{
    uint8 return_state;
    if(0 == page_id)
    {
        page_id = 0 | (IPS200PRO_WIDGETS_PAGE << 8);
    }
    return_state = ips200pro_set_hidden(page_id, state);
    return return_state;
}

uint8 ips200pro_page_set_title_name(uint16 page_id, char *str)
{
    uint8 return_state;
    ips200pro_header_struct temp;

    return_state = ips200pro_write_packet(IPS200PRO_WIDGETS_PAGE, IPS200PRO_COMMON_VALUE, (uint8)page_id, (ips200pro_header_struct *)&temp, sizeof(temp), str, strlen(str));
    return return_state;
}

uint8 ips200pro_page_set_title_position_width(ips200pro_title_position_enum title_position, uint8 title_width)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 2);

    temp.dat[0] = title_position; 
    temp.dat[1] = title_width;	 
    return_state = ips200pro_write_packet(IPS200PRO_WIDGETS_PAGE, IPS200PRO_PAGE_TITLE, 1, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    return return_state;
}

uint16 ips200pro_label_create(int16 x, int16 y, uint16 width, uint16 height)
{
    uint8 return_state = 1;
    static uint8 lebel_num = 0;

    if(MAX_ID_LABEL > lebel_num)
    {
        return_state = ips200pro_create_widgets(++lebel_num | (IPS200PRO_WIDGETS_LABEL << 8), x, y, width, height);
        if(1 == return_state)
        {
            lebel_num--;
        }
    }
    return return_state == 1 ? 0 : (lebel_num | (IPS200PRO_WIDGETS_LABEL << 8));
}

uint8 ips200pro_label_printf(uint16 label_id, const char *format, ...)
{
	int32 str_length; 
	uint8 return_state = 1;
	ips200pro_header_struct temp;
	
    va_list arg;
    va_start(arg, format);

	str_length = vsnprintf(ips200pro_printf_buffer, sizeof(ips200pro_printf_buffer) - 1, format, arg);
	if(0 <= str_length)
	{
		temp.length = (uint32)str_length;
		return_state = ips200pro_write_packet(IPS200PRO_WIDGETS_LABEL, IPS200PRO_COMMON_VALUE, (uint8)label_id, (ips200pro_header_struct *)&temp, sizeof(temp), ips200pro_printf_buffer, temp.length);
	}
	va_end(arg);

    return return_state;
}

uint8 ips200pro_label_show_string(uint16 label_id, const char *str)
{
    uint8 return_state;
    ips200pro_header_struct temp;

    temp.length  = strlen(str);
    return_state = ips200pro_write_packet(IPS200PRO_WIDGETS_LABEL, IPS200PRO_COMMON_VALUE, (uint8)label_id, (ips200pro_header_struct *)&temp, sizeof(temp), str, temp.length);

    return return_state;
}

uint8 ips200pro_label_mode(uint16 label_id, ips200pro_label_mode_enum mode)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 1);

    temp.dat[0]  = mode;
    return_state = ips200pro_write_packet(IPS200PRO_WIDGETS_LABEL, IPS200PRO_LABEL_LONG_MODE, (uint8)label_id, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    return return_state;
}


uint16 ips200pro_table_create(int16 x, int16 y, uint16 row_num, uint16 col_num)
{
    uint8 return_state = 1;
    static uint8 table_num = 0;
    if(MAX_ID_TABLE > table_num)
    {
        return_state = ips200pro_create_widgets(++table_num | (IPS200PRO_WIDGETS_TABLE << 8), x, y, row_num, col_num);
        if(1 == return_state)
        {
            table_num--;
        }
    }
    return return_state == 1 ? 0 : (table_num | (IPS200PRO_WIDGETS_TABLE << 8));
}

uint8 ips200pro_table_cell_printf(uint16 table_id, uint8 row, uint8 col, char *format, ...)
{
	int32 str_length = 0; 
    uint8 return_state = 1;
	va_list arg;
    IPS200PRO_COMMON_STRUCT(temp, 2);

	va_start(arg, format);
	
	str_length = vsnprintf(ips200pro_printf_buffer, sizeof(ips200pro_printf_buffer) - 1, format, arg);
	if(0 <= str_length)
	{
		temp.length = (uint32)str_length;
		temp.dat[0]	= row;
		temp.dat[1]	= col;
		return_state = ips200pro_write_packet(IPS200PRO_WIDGETS_TABLE, IPS200PRO_COMMON_VALUE, (uint8)table_id, (ips200pro_header_struct *)&temp, sizeof(temp), ips200pro_printf_buffer, temp.length);
	}
	va_end(arg);
	
    return return_state;
}


uint8 ips200pro_table_set_col_width(uint16 table_id, uint8 col, uint16 width)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 2);

    temp.dat[0] = width;
    temp.dat[1] = col;
    return_state = ips200pro_write_packet(IPS200PRO_WIDGETS_TABLE, IPS200PRO_TABLE_COL_WIDTH, (uint8)table_id, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    return return_state;
}

uint8 ips200pro_table_select(uint16 table_id, uint8 row, uint8 col)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 2);

    temp.dat[0]  = row;
    temp.dat[1]  = col;
    return_state = ips200pro_write_packet(IPS200PRO_WIDGETS_TABLE, IPS200PRO_TABLE_SELECT, (uint8)table_id, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    return return_state;
}

uint16 ips200pro_meter_create(int16 x, int16 y, uint16 siz, ips200pro_meter_style_enum style)
{
    uint8 return_state = 1;
    static uint8 meter_num = 0;
    if(MAX_ID_METER > meter_num)
    {
        return_state = ips200pro_create_widgets(++meter_num | (IPS200PRO_WIDGETS_METER << 8), x, y, siz, style);
        if(1 == return_state)
        {
            meter_num--;
        }
    }
    return return_state == 1 ? 0 : (meter_num | (IPS200PRO_WIDGETS_METER << 8));
}

uint8 ips200pro_meter_set_value(uint16 meter_id, int16 value)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 1);

    temp.dat[0] = value;
    return_state = ips200pro_write_packet(IPS200PRO_WIDGETS_METER, IPS200PRO_COMMON_VALUE, (uint8)meter_id, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    return return_state;
}

uint16 ips200pro_clock_create(int16 x, int16 y, uint16 clock_size, ips200pro_clock_style_enum clock_type)
{
    uint8 return_state = 1;
    static uint8 clock_num = 0;
    if(MAX_ID_CLOCK > clock_num)
    {
        return_state = ips200pro_create_widgets(++clock_num | (IPS200PRO_WIDGETS_CLOCK << 8), x, y, clock_size, clock_type);
        if(1 == return_state)
        {
            clock_num--;
        }
    }
    return return_state == 1 ? 0 : (clock_num | (IPS200PRO_WIDGETS_CLOCK << 8));
}


uint16 ips200pro_progress_bar_create(int16 x, int16 y, uint16 width, uint16 height)
{
    uint8 return_state = 1;
    static uint8 progress_bar_num = 0;
    if(MAX_ID_PROGRESS_BAR > progress_bar_num)
    {
        return_state = ips200pro_create_widgets(++progress_bar_num | (IPS200PRO_WIDGETS_BAR << 8), x, y, width, height);
        if(1 == return_state)
        {
            progress_bar_num--;
        }
    }
    return return_state == 1 ? 0 : (progress_bar_num | (IPS200PRO_WIDGETS_BAR << 8));
}

uint8 ips200pro_progress_bar_set_value(uint16 progress_bar_id, uint8 start_value, uint8 end_value)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 2);

    temp.dat[0] = start_value;
    temp.dat[1] = end_value;
    return_state = ips200pro_write_packet(IPS200PRO_WIDGETS_BAR, IPS200PRO_COMMON_VALUE, (uint8)progress_bar_id, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    return return_state;
}

uint16 ips200pro_calendar_create(int16 x, int16 y, uint16 width, uint16 height)
{
    uint8 return_state;
    // 只能创建一个日历控件
    return_state = ips200pro_create_widgets(1 | (IPS200PRO_WIDGETS_CALENDAR << 8), x, y, width, height);
    return return_state == 1 ? 0 : (1 | (IPS200PRO_WIDGETS_CALENDAR << 8));
}

uint8 ips200pro_calendar_display(uint16 year, uint8 month, ips200pro_calendar_mode_enum mode)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 2);

    temp.dat[0] = year;
    temp.dat[1] = month || mode << 8;

    return_state = ips200pro_write_packet(IPS200PRO_WIDGETS_CALENDAR, IPS200PRO_COMMON_VALUE, 1, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    return return_state;
}

uint16 ips200pro_waveform_create(int16 x, int16 y, uint16 width, uint16 height)
{
    uint8 return_state = 1;
    static uint8 waveform_num = 0;
    if(MAX_ID_WAVEFORM > waveform_num)
    {
        return_state = ips200pro_create_widgets(++waveform_num | (IPS200PRO_WIDGETS_WAVEFORM << 8), x, y, width, height);
        if(1 == return_state)
        {
            waveform_num--;
        }
    }
    return return_state == 1 ? 0 : (waveform_num | (IPS200PRO_WIDGETS_WAVEFORM << 8));
}

uint8 ips200pro_waveform_add_value(uint16 waveform_id, uint8 line_id,  uint16 *dat, uint16 length, uint16 color)
{
    uint8 return_state;
	uint16 i;
	
    IPS200PRO_COMMON_STRUCT(temp, 2);
	
	
    temp.dat[0] = line_id;
    temp.dat[1] = color;
	
	// 两位uint16数据进行大小端交换
	for(i = 0; i < length; i++)
	{
		dat[i] = ((dat[i] & 0xFF) << 8) | ((dat[i] & 0xFF00) >> 8);
	}
	
    return_state = ips200pro_write_packet(IPS200PRO_WIDGETS_WAVEFORM, IPS200PRO_COMMON_VALUE, (uint8)waveform_id, (ips200pro_header_struct *)&temp, sizeof(temp), dat, length * 2);
    return return_state;
}

uint8 ips200pro_waveform_line_state(uint16 waveform_id, uint16 line_id, uint16 line_state)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 2);

    temp.dat[0] = line_id;
    temp.dat[1] = line_state;
    return_state = ips200pro_write_packet(IPS200PRO_WIDGETS_WAVEFORM, IPS200PRO_WAVEFORM_LINE_STATE, (uint8)waveform_id, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    return return_state;
}

uint8 ips200pro_waveform_line_type(uint16 waveform_id, uint8 line_type)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 1);

    temp.dat[0]  = line_type;
    return_state = ips200pro_write_packet(IPS200PRO_WIDGETS_WAVEFORM, IPS200PRO_WAVEFORM_LINE_TYPE, (uint8)waveform_id, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    return return_state;
}

uint8 ips200pro_waveform_clear(uint16 waveform_id)
{
    uint8 return_state;
    ips200pro_header_struct temp;

    return_state = ips200pro_write_packet(IPS200PRO_WIDGETS_WAVEFORM, IPS200PRO_WAVEFORM_CLEAR, (uint8)waveform_id, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    return return_state;
}

uint16 ips200pro_image_create(int16 x, int16 y, uint16 width, uint16 height)
{
    uint8 return_state = 1;
    static uint8 image_num = 0;
    if(MAX_ID_IMAGE > image_num)
    {
        return_state = ips200pro_create_widgets(++image_num | (IPS200PRO_WIDGETS_IMAGE << 8), x, y, width, height);
        if(1 == return_state)
        {
            image_num--;
        }
    }
    return return_state == 1 ? 0 : (image_num | (IPS200PRO_WIDGETS_IMAGE << 8));
}

uint8 ips200pro_image_display(uint16 image_id, const void *image, uint16 width, uint16 height, ips200pro_image_type_enum image_type, uint8 threshold)
{
    uint8 return_state = 0;
    uint16 send_length;
    uint32 image_size;
    uint8 const *image_data;
    IPS200PRO_COMMON_STRUCT(temp, 4);

    image_data = (uint8 *)image;
    image_size = width * height * (IMAGE_RGB565 == image_type ? 2 : 1);
    // 任意条件满足，则表示不需要发送图像数据，仅通知屏幕更新边线或矩形
    if((NULL == image) || (!width) || (!height) || (IMAGE_NULL == image_type))
    {
		temp.dat[0]  = 0;
        temp.dat[1]  = 0;
        temp.dat[2]  = IMAGE_NULL << 8;
    }
    else
    {
		temp.dat[0] = width;
        temp.dat[1] = height;
        temp.dat[2] = image_type << 8;
    }
    temp.dat[2] = 1;            // 图像开始传输标志位
    temp.dat[3] = threshold;

    do
    {
        // 计算本次传输字节数
        send_length = image_size > (IPS200PRO_SPI_LENGTH - sizeof(temp)) ? (IPS200PRO_SPI_LENGTH - sizeof(temp)) : (uint16)image_size;
        return_state += ips200pro_write_packet(IPS200PRO_WIDGETS_IMAGE, IPS200PRO_COMMON_VALUE, (uint8)image_id, (ips200pro_header_struct *)&temp, sizeof(temp), image_data, send_length);
        image_data += send_length;
        image_size -= send_length;
        temp.dat[2] = 0;
    }while(image_size);

    return return_state;
}

uint8 ips200pro_image_draw_line(uint16 image_id, uint8 line_id, void *line_data, uint16 line_length, ips200pro_image_line_type_enum data_type, uint16 color)
{
    uint8 return_state = 1;
    IPS200PRO_COMMON_STRUCT(temp, 2);

    if(MAX_ID_IMAGE_LINE >= line_id)
    {
        temp.length                 = data_type * line_length * 2 + sizeof(temp);
        temp.dat[0]  = line_id || data_type << 8;
        temp.dat[1]  = color;
        return_state = ips200pro_write_packet(IPS200PRO_WIDGETS_IMAGE, IPS200PRO_IMAGE_DRAW_LINE, (uint8)image_id, (ips200pro_header_struct *)&temp, sizeof(temp), line_data, data_type * line_length * 2);
    }
    return return_state;
}

uint8 ips200pro_image_draw_rectangle(uint16 image_id, uint8 rectangle_id, int16 x, int16 y, uint16 rectangle_width, uint16 rectangle_height, uint16 color)
{
    uint8 return_state = 1;
    IPS200PRO_COMMON_STRUCT(temp, 6);
    if(MAX_ID_IMAGE_RECTANGLE >= rectangle_id)
    {
        temp.dat[0] = rectangle_id;
        temp.dat[1] = x;
        temp.dat[2] = y;
        temp.dat[3] = rectangle_width;
        temp.dat[4] = rectangle_height;
        temp.dat[5] = color;
        return_state = ips200pro_write_packet(IPS200PRO_WIDGETS_IMAGE, IPS200PRO_IMAGE_DRAW_RECTANGLE, (uint8)image_id, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    }
    return return_state;
}

uint16 ips200pro_container_create(int16 x, int16 y, uint16 width, uint16 height)
{
    uint8 return_state = 1;
    static uint8 container_num = 0;
    if(MAX_ID_IMAGE_RECTANGLE > container_num)
    {
        return_state = ips200pro_create_widgets(++container_num | (IPS200PRO_WIDGETS_CONTAINER << 8), x, y, width, height);
        if(1 == return_state)
        {
            container_num--;
        }
    }
    return return_state == 1 ? 0 : (container_num | (IPS200PRO_WIDGETS_CONTAINER << 8));
}

uint8 ips200pro_container_radius(uint16 container_id, uint16 border_width, uint16 radius)
{
    uint8 return_state;
    IPS200PRO_COMMON_STRUCT(temp, 2);

    temp.dat[0] = border_width;
    temp.dat[1] = radius;
    return_state = ips200pro_write_packet(IPS200PRO_WIDGETS_CONTAINER, IPS200PRO_COMMON_VALUE, (uint8)container_id, (ips200pro_header_struct *)&temp, sizeof(temp), NULL, 0);
    return return_state;
}

uint16 ips200pro_init(char *str, ips200pro_title_position_enum title_position, uint8 title_size)
{
    uint16 page_id = 0;
    spi_init(IPS200PRO_SPI_INDEX, SPI_MODE0, IPS200PRO_SPI_SPEED, IPS200PRO_CLK_PIN, IPS200PRO_MOSI_PIN, IPS200PRO_MISO_PIN, SPI_CS_NULL);
    gpio_init(IPS200PRO_RST_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(IPS200PRO_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    // 将屏幕应答引脚修改为上拉，这样即使屏幕在使用过程中被拔掉也不会导致程序卡主
    gpio_init(IPS200PRO_INT_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);

    gpio_set_level(IPS200PRO_RST_PIN, 0);
    system_delay_ms(5);
    gpio_set_level(IPS200PRO_RST_PIN, 1);
    system_delay_ms(20);



    // 某一些主板屏幕接口由于没有MISO引脚，所以无法读取屏幕信息
    // ips200pro_get_information(&ips200pro_information);
    // ips200pro_get_time(&ips200pro_time);

    // 设置时间会立即生效，因此并不建议在初始化中调用时间设置函数，如果在初始化中调用就会导致每次上电之后时间都会还原
    // ips200pro_set_time(15, 54, 30);

    // 特别注意、特别注意、特别注意
    // 设置页面标题显示位置以及标题宽度，需要注意仅在创建页面之前调用才能生效
    // 特别注意、特别注意、特别注意


    ips200pro_page_set_title_position_width(title_position, title_size);

    ips200pro_set_format(IPS200PRO_DEFAULT_FORMAT);
    ips200pro_set_default_font(IPS200PRO_DEFAULT_FONT_SIZE);
    ips200pro_set_optimize(IPS200PRO_DEFAULT_OPTIMIZE);



    // 设置默认参数并创建一个页面
    if(NULL != str)
    {
        page_id = ips200pro_page_create(str); // 创建一个页面
    }

#if(1 == IPS200PRO_CRC_ENABLE)
    ips200pro_set_crc_state(1);  // 使能CRC模式增强抗干扰能力，但是需要消耗主控一部分算力做CRC计算
#endif

    return page_id;
}
