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

#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_common_font.h"
#include "zf_common_function.h"
#include "zf_driver_delay.h"
#include "zf_driver_spi.h"

#include "zf_device_tft180.h"

uint16 tft180_pencolor = TFT180_DEFAULT_PENCOLOR;
uint16 tft180_bgcolor = TFT180_DEFAULT_BGCOLOR;

tft180_dir_enum tft180_display_dir = TFT180_DEFAULT_DISPLAY_DIR;
uint8 tft180_x_max = 240;
uint8 tft180_y_max = 135;

#if (TFT180_USE_INTERFACE==SOFT_SPI)  
	static soft_spi_info_struct             tft180_spi;
	#define tft180_write_8bit_data(dat)    soft_spi_write_8bit(&TFT180_spi, dat)
	#define tft180_write_16bit_data(dat)   soft_spi_write_16bit(&TFT180_spi, dat)
#elif (TFT180_USE_INTERFACE==HARDWARE_SPI)
	#define tft180_write_8bit_data(dat)    spi_write_8bit(TFT180_SPI, dat)
	#define tft180_write_16bit_data(dat)   spi_write_16bit(TFT180_SPI, dat)
#endif

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       写命令 内部调用
// @note        内部调用 用户无需关心
//-------------------------------------------------------------------------------------------------------------------
static void tft180_write_index (uint8 dat)
{
    TFT180_CS(1);
    TFT180_CS(0);
    TFT180_DC(0);
    tft180_write_8bit_data(dat);
    TFT180_CS(1);
    TFT180_DC(1);
    TFT180_CS(0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       设置显示区域 内部调用
// 参数说明       x1              起始x轴坐标
// 参数说明       y1              起始y轴坐标
// 参数说明       x2              结束x轴坐标
// 参数说明       y2              结束y轴坐标
// 返回参数      void
// @note        内部调用 用户无需关心
//-------------------------------------------------------------------------------------------------------------------
static void tft180_set_region (uint16 x1, uint16 y1, uint16 x2, uint16 y2)
{
    zf_assert(x1 < tft180_x_max);
    zf_assert(y1 < tft180_y_max);
    zf_assert(x2 < tft180_x_max);
    zf_assert(y2 < tft180_y_max);

    if(tft180_display_dir==TFT180_PORTAIT || tft180_display_dir==TFT180_PORTAIT_180)
    {
        tft180_write_index(0x2a);
        tft180_write_16bit_data(x1+2);
        tft180_write_16bit_data(x2+2);

        tft180_write_index(0x2b);
        tft180_write_16bit_data(y1+1);
        tft180_write_16bit_data(y2+1);
    }
    else
    {
        tft180_write_index(0x2a);
        tft180_write_16bit_data(x1+1);
        tft180_write_16bit_data(x2+1);

        tft180_write_index(0x2b);
        tft180_write_16bit_data(y1+2);
        tft180_write_16bit_data(y2+2);
    }
    tft180_write_index(0x2c);
}



//-------------------------------------------------------------------------------------------------------------------
// 函数简介       液晶清屏函数
// 参数说明       color           颜色格式 RGB565 或者可以使用 zf_common_font.h 内常用颜色宏定义
// 返回参数      void
// 使用示例                tft180_clear(YELLOW);
//-------------------------------------------------------------------------------------------------------------------
void tft180_clear (uint16 color)
{
    uint32 i = tft180_x_max*tft180_y_max;

    tft180_set_region(0,0,tft180_x_max-1,tft180_y_max-1);
    for( ; i>0; i--)
    {
        tft180_write_16bit_data(color);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       设置显示方向 这个函数只有在初始化屏幕之前调用才生效
// 参数说明       dir             显示方向  参照 zf_device_ips114.h 内 tft180_dir_enum 枚举体定义
// 返回参数      void
// 使用示例                tft180_set_dir(TFT180_CROSSWISE);
//-------------------------------------------------------------------------------------------------------------------
void tft180_set_dir (tft180_dir_enum dir)
{
    tft180_display_dir = dir;
    if(dir < 2)
    {
        tft180_x_max = 128;
        tft180_y_max = 160;
    }
    else
    {
        tft180_x_max = 160;
        tft180_y_max = 128;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       设置显示颜色
// 参数说明       pen             颜色格式 RGB565 或者可以使用 zf_common_font.h 内常用颜色宏定义
// 参数说明       bgcolor         颜色格式 RGB565 或者可以使用 zf_common_font.h 内常用颜色宏定义
// 返回参数      void
// 使用示例                tft180_set_color(WHITE,BLACK);
//-------------------------------------------------------------------------------------------------------------------
void tft180_set_color (uint16 pen, uint16 bgcolor)
{
    tft180_pencolor = pen;
    tft180_bgcolor = bgcolor;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       液晶画点
// 参数说明       x               坐标x方向的起点 参数范围 [0, tft180_x_max-1]
// 参数说明       y               坐标y方向的起点 参数范围 [0, tft180_y_max-1]
// 参数说明       dat             需要显示的颜色
// 返回参数      void
// 使用示例                tft180_draw_point(0,0,RED);                     // 坐标 0,0 画一个红色的点
//-------------------------------------------------------------------------------------------------------------------
void tft180_draw_point (uint16 x,uint16 y,uint16 color)
{
    zf_assert(x < tft180_x_max);
    zf_assert(y < tft180_y_max);

    tft180_set_region(x,y,x,y);
    tft180_write_16bit_data(color);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       液晶显示字符
// 参数说明       x               坐标x方向的起点 参数范围 [0, tft180_x_max-1]
// 参数说明       y               坐标y方向的起点 参数范围 [0, tft180_y_max-1]
// 参数说明       dat             需要显示的字符
// 返回参数      void
// 使用示例                tft180_show_char(0,0,'x');                      // 坐标 0,0 写一个字符 x
//-------------------------------------------------------------------------------------------------------------------
void tft180_show_char (uint16 x,uint16 y,const char dat)
{
    uint8 i,j;
    uint8 temp;
	
    zf_assert(x < tft180_x_max);
    zf_assert(y < tft180_y_max);
	
    for(i=0; i<16; i++)
    {
        tft180_set_region(x,y+i,x+7,y+i);
        temp = ascii_font_8x16[dat-32][i];                                            //减 32 因为是取模是从空格开始取得 空格在 ascii 中序号是 32
        for(j=0; j<8; j++)
        {
            if(temp&0x01)   
                tft180_write_16bit_data(tft180_pencolor);
            else
                tft180_write_16bit_data(tft180_bgcolor);
            temp>>=1;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       液晶显示字符串
// 参数说明       x               坐标x方向的起点 参数范围 [0, tft180_x_max-1]
// 参数说明       y               坐标y方向的起点 参数范围 [0, tft180_y_max-1]
// 参数说明       dat             需要显示的字符串
// 返回参数      void
// 使用示例                tft180_show_string(0,0,"seekfree");
//-------------------------------------------------------------------------------------------------------------------
void tft180_show_string (uint16 x,uint16 y,const char dat[])
{
    uint16 j = 0;
	
    zf_assert(x < tft180_x_max);
    zf_assert(y < tft180_y_max);
    

    while(dat[j] != '\0')
    {
        tft180_show_char(x+8*j,y,dat[j]);
        j++;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      液晶显示8位有符号
// 参数说明       x               坐标x方向的起点 参数范围 [0, tft180_x_max-1]
// 参数说明       y               坐标y方向的起点 参数范围 [0, tft180_y_max-1]
//  @param      dat             需要显示的变量，数据类型int8
//  @return     void
//  Sample usage:               tft180_show_int8(0,0,x);                        // x为 int8 类型
//-------------------------------------------------------------------------------------------------------------------
void tft180_show_int8(uint16 x,uint16 y,int8 dat)
{
    uint8 a[3];
    uint8 i;
	
    zf_assert(x < tft180_x_max);
    zf_assert(y < tft180_y_max);


    if(dat<0)
    {
        tft180_show_char(x,y,'-');
        dat = -dat;
    }
    else
        tft180_show_char(x,y,' ');
    
    a[0] = dat/100;
    a[1] = dat/10%10;
    a[2] = dat%10;
    i = 0;
    while(i<3)
    {
        tft180_show_char(x+(8*(i+1)),y,(uint8)('0' + a[i]));
        i++;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       液晶显示8位无符号
// 参数说明       x               坐标x方向的起点 参数范围 [0, tft180_x_max-1]
// 参数说明       y               坐标y方向的起点 参数范围 [0, tft180_y_max-1]
// 参数说明       dat             需要显示的变量 数据类型uint8
// 返回参数      void
// 使用示例                tft180_show_uint8(0,0,x);                       // x 为 uint8 类型
//-------------------------------------------------------------------------------------------------------------------
void tft180_show_uint8(uint16 x,uint16 y,uint8 dat)
{
    uint8 a[3];
    uint8 i;
	
    zf_assert(x < tft180_x_max);
    zf_assert(y < tft180_y_max);

    a[0] = dat/100;
    a[1] = dat/10%10;
    a[2] = dat%10;
    i = 0;
    while(i<3)
    {
        tft180_show_char(x+(8*i),y,(uint8)('0' + a[i]));
        i++;
    }
    
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       液晶显示16位有符号
// 参数说明       x               坐标x方向的起点 参数范围 [0, tft180_x_max-1]
// 参数说明       y               坐标y方向的起点 参数范围 [0, tft180_y_max-1]
// 参数说明       dat             需要显示的变量 数据类型int16
// 返回参数      void
// 使用示例                tft180_show_int16(0,0,x);                       // x 为 int16 类型
//-------------------------------------------------------------------------------------------------------------------
void tft180_show_int16(uint16 x,uint16 y,int16 dat)
{
    uint8 a[5];
    uint8 i;

    zf_assert(x < tft180_x_max);
    zf_assert(y < tft180_y_max);

    if(dat<0)
    {
        tft180_show_char(x,y,'-');
        dat = -dat;
    }
    else    tft180_show_char(x,y,' ');

    a[0] = dat/10000;
    a[1] = dat/1000%10;
    a[2] = dat/100%10;
    a[3] = dat/10%10;
    a[4] = dat%10;
    
    i = 0;
    while(i<5)
    {
        tft180_show_char(x+(8*(i+1)),y,(uint8)('0' + a[i]));
        i++;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       液晶显示16位无符号
// 参数说明       x               坐标x方向的起点 参数范围 [0, tft180_x_max-1]
// 参数说明       y               坐标y方向的起点 参数范围 [0, tft180_y_max-1]
// 参数说明       dat             需要显示的变量，数据类型uint16
// 返回参数      void
// 使用示例                tft180_show_uint16(0,0,x);                      // x 为 uint16 类型
//-------------------------------------------------------------------------------------------------------------------
void tft180_show_uint16(uint16 x,uint16 y,uint16 dat)
{
    uint8 a[5];
    uint8 i;
	
    zf_assert(x < tft180_x_max);
    zf_assert(y < tft180_y_max);

    a[0] = dat/10000;
    a[1] = dat/1000%10;
    a[2] = dat/100%10;
    a[3] = dat/10%10;
    a[4] = dat%10;
    
    i = 0;
    while(i<5)
    {
        tft180_show_char(x+(8*i),y,(uint8)('0' + a[i]));
        i++;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       液晶显示32位有符号(去除整数部分无效的0)
// 参数说明       x               坐标x方向的起点 参数范围 [0, tft180_x_max-1]
// 参数说明       y               坐标y方向的起点 参数范围 [0, tft180_y_max-1]
// 参数说明       dat             需要显示的变量，数据类型uint32
// 参数说明       num             需要显示的位数 最高10位  不包含正负号
// 返回参数      void
// 使用示例                tft180_showuint32(0,0,x,3);                     // x 可以为 int32 uint16 int16 uint8 int8 类型
// note:                        负数会显示一个 ‘-’号   正数显示一个空格
//-------------------------------------------------------------------------------------------------------------------
void tft180_show_int32(uint16 x,uint16 y,int32 dat,uint8 num)
{
    int8    buff[34];
    uint8   length;
	
    zf_assert(x < tft180_x_max);
    zf_assert(y < tft180_y_max);

    if(10<num)      num = 10;
    
    num++;
    if(0>dat)   length = zf_sprintf( &buff[0],(const int8 *)"%d",dat);        // 负数
    else
    {
        buff[0] = ' ';
        length = zf_sprintf( &buff[1],(const int8 *)"%d",dat);
        length++;
    }
    while(length < num)
    {
        buff[length] = ' ';
        length++;
    }
    buff[num] = '\0';

    tft180_show_string(x, y, (const char *)buff);                               // 显示数字
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       液晶显示浮点数(去除整数部分无效的0)
// 参数说明       x               坐标x方向的起点 参数范围 [0, tft180_x_max-1]
// 参数说明       y               坐标y方向的起点 参数范围 [0, tft180_y_max-1]
// 参数说明       dat             需要显示的变量，数据类型float或double
// 参数说明       num             整数位显示长度   最高10位
// 参数说明       pointnum        小数位显示长度   最高6位
// 返回参数      void
// 使用示例                tft180_showfloat(0,0,x,2,3);                    // 显示浮点数   整数显示2位   小数显示三位
// @note                        特别注意当发现小数部分显示的值与你写入的值不一样的时候，
//                              可能是由于浮点数精度丢失问题导致的，这并不是显示函数的问题，
//                              有关问题的详情，请自行百度学习   浮点数精度丢失问题。
//                              负数会显示一个 ‘-’号   正数显示一个空格
//-------------------------------------------------------------------------------------------------------------------
void tft180_show_float(uint16 x,uint16 y,double dat,uint8 num,uint8 pointnum)
{
    uint8   length;
    int8    buff[34];
    int8    start,end,point;
	
    zf_assert(x < tft180_x_max);
    zf_assert(y < tft180_y_max);
	
    if(6<pointnum)  pointnum = 6;
    if(10<num)      num = 10;
        
    if(0>dat)   length = zf_sprintf( &buff[0],(const int8 *)"%f",dat);        // 负数
    else
    {
        length = zf_sprintf( &buff[1],(const int8 *)"%f",dat);
        length++;
    }
    point = length - 7;                                                         // 计算小数点位置
    start = point - num - 1;                                                    // 计算起始位
    end = point + pointnum + 1;                                                 // 计算结束位
    while(0>start)                                                              // 整数位不够  末尾应该填充空格
    {
        buff[end] = ' ';
        end++;
        start++;
    }
    
    if(0>dat)   buff[start] = '-';
    else        buff[start] = ' ';
    
    buff[end] = '\0';

    tft180_show_string(x, y, (const char *)buff);                               // 显示数字
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       TFT180 显示波形2
// 参数说明       x               坐标x方向的起点 参数范围 [0, tft180_x_max-1]
// 参数说明       y               坐标y方向的起点 参数范围 [0, tft180_y_max-1]
// 参数说明       *p              波形数组指针
// 参数说明       width           波形实际宽度
// 参数说明       value_max       波形实际最大值
// 参数说明       dis_width       波形显示宽度 参数范围 [0, tft180_x_max]
// 参数说明       dis_value_max   波形显示最大值 参数范围 [0, tft180_y_max]
// 返回参数      void
// 使用示例                tft180_show_gray_image(0,0,camera_buffer_addr,MT9V03X_W,MT9V03X_H,240,135,128);
//-------------------------------------------------------------------------------------------------------------------
void tft180_show_wave(uint16 x, uint16 y, uint8 *p, uint16 width, uint16 value_max, uint16 dis_width, uint16 dis_value_max)
{
    uint32 i = 0, j = 0;
    uint32 width_index, value_max_index;
	
    zf_assert(x < tft180_x_max);
    zf_assert(y < tft180_y_max);
	
    tft180_set_region(x, y, x+dis_width-1, y+dis_value_max-1);                  // 设置显示区域
    for(i=0;i<dis_value_max;i++)
    {
        for(j=0;j<dis_width;j++)
        {
            tft180_write_16bit_data(tft180_bgcolor); 
        }
    }

    for(i=0;i<dis_width;i++)
    {
        width_index = i*width/dis_width;
        value_max_index = *(p+width_index)*(dis_value_max-1)/value_max;
        tft180_draw_point(i+x, (dis_value_max-1)-value_max_index+y, tft180_pencolor);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       1.8寸 TFT液晶初始化
// 返回参数      void
// 使用示例                tft180_init();
//-------------------------------------------------------------------------------------------------------------------
void tft180_init (void)
{
#if (TFT180_USE_INTERFACE==SOFT_SPI)  
    soft_spi_init(&tft180_spi, SPI_MODE0, TFT180_SOFT_SPI_DELAY, TFT180_SCL_PIN, TFT180_SDA_PIN, SOFT_SPI_PIN_NULL, SOFT_SPI_PIN_NULL);
#elif (TFT180_USE_INTERFACE==HARDWARE_SPI)
    spi_init(TFT180_SPI, SPI_MODE0, TFT180_SPI_SPEED, TFT180_SCL_PIN, TFT180_SDA_PIN, SPI_MISO_NULL, SPI_CS_NULL);
#endif

//    gpio_init(TFT180_DC_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
//    gpio_init(TFT180_RES_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
//    gpio_init(TFT180_CS_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
//    gpio_init(TFT180_BL_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    tft180_set_dir(tft180_display_dir);
    tft180_set_color(tft180_pencolor, tft180_bgcolor);

    TFT180_RES(0);
    system_delay_ms(10);

    TFT180_RES(1);
    system_delay_ms(120);

    tft180_write_index(0x11);
    system_delay_ms(120);

    tft180_write_index(0xB1);
    tft180_write_8bit_data(0x01);
    tft180_write_8bit_data(0x2C);
    tft180_write_8bit_data(0x2D);

    tft180_write_index(0xB2);
    tft180_write_8bit_data(0x01);
    tft180_write_8bit_data(0x2C);
    tft180_write_8bit_data(0x2D);

    tft180_write_index(0xB3);
    tft180_write_8bit_data(0x01);
    tft180_write_8bit_data(0x2C);
    tft180_write_8bit_data(0x2D);
    tft180_write_8bit_data(0x01);
    tft180_write_8bit_data(0x2C);
    tft180_write_8bit_data(0x2D);

    tft180_write_index(0xB4);
    tft180_write_8bit_data(0x07);

    tft180_write_index(0xC0);
    tft180_write_8bit_data(0xA2);
    tft180_write_8bit_data(0x02);
    tft180_write_8bit_data(0x84);
    tft180_write_index(0xC1);
    tft180_write_8bit_data(0xC5);

    tft180_write_index(0xC2);
    tft180_write_8bit_data(0x0A);
    tft180_write_8bit_data(0x00);

    tft180_write_index(0xC3);
    tft180_write_8bit_data(0x8A);
    tft180_write_8bit_data(0x2A);
    tft180_write_index(0xC4);
    tft180_write_8bit_data(0x8A);
    tft180_write_8bit_data(0xEE);

    tft180_write_index(0xC5);
    tft180_write_8bit_data(0x0E);

    tft180_write_index(0x36);
    switch(tft180_display_dir)                                                         // y x v
    {
        case 0: tft180_write_8bit_data(1<<7 | 1<<6 | 0<<5);  break;                          // 竖屏模式
        case 1: tft180_write_8bit_data(0<<7 | 0<<6 | 0<<5);  break;                          // 竖屏模式  旋转180
        case 2: tft180_write_8bit_data(1<<7 | 0<<6 | 1<<5);  break;                          // 横屏模式
        case 3: tft180_write_8bit_data(0<<7 | 1<<6 | 1<<5);  break;                          // 横屏模式  旋转180
    }

    tft180_write_index(0xe0);
    tft180_write_8bit_data(0x0f);
    tft180_write_8bit_data(0x1a);
    tft180_write_8bit_data(0x0f);
    tft180_write_8bit_data(0x18);
    tft180_write_8bit_data(0x2f);
    tft180_write_8bit_data(0x28);
    tft180_write_8bit_data(0x20);
    tft180_write_8bit_data(0x22);
    tft180_write_8bit_data(0x1f);
    tft180_write_8bit_data(0x1b);
    tft180_write_8bit_data(0x23);
    tft180_write_8bit_data(0x37);
    tft180_write_8bit_data(0x00);
    tft180_write_8bit_data(0x07);
    tft180_write_8bit_data(0x02);
    tft180_write_8bit_data(0x10);

    tft180_write_index(0xe1);
    tft180_write_8bit_data(0x0f);
    tft180_write_8bit_data(0x1b);
    tft180_write_8bit_data(0x0f);
    tft180_write_8bit_data(0x17);
    tft180_write_8bit_data(0x33);
    tft180_write_8bit_data(0x2c);
    tft180_write_8bit_data(0x29);
    tft180_write_8bit_data(0x2e);
    tft180_write_8bit_data(0x30);
    tft180_write_8bit_data(0x30);
    tft180_write_8bit_data(0x39);
    tft180_write_8bit_data(0x3f);
    tft180_write_8bit_data(0x00);
    tft180_write_8bit_data(0x07);
    tft180_write_8bit_data(0x03);
    tft180_write_8bit_data(0x10);

    tft180_write_index(0x2a);
    tft180_write_8bit_data(0x00);
    tft180_write_8bit_data(0x00+2);
    tft180_write_8bit_data(0x00);
    tft180_write_8bit_data(0x80+2);

    tft180_write_index(0x2b);
    tft180_write_8bit_data(0x00);
    tft180_write_8bit_data(0x00+3);
    tft180_write_8bit_data(0x00);
    tft180_write_8bit_data(0x80+3);

    tft180_write_index(0xF0);
    tft180_write_8bit_data(0x01);
    tft180_write_index(0xF6);
    tft180_write_8bit_data(0x00);

    tft180_write_index(0x3A);
    tft180_write_8bit_data(0x05);
    tft180_write_index(0x29);

    tft180_clear(tft180_bgcolor);
}