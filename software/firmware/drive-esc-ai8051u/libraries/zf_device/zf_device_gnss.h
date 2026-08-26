/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
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
* 文件名称          zf_device_gnss
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.9.20
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者               备注
* 2023-12-28       pudding            first version
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义：
*                   ------------------------------------
*                   模块管脚            单片机管脚
*                   RX                  查看 zf_device_gnss.h 中 GNSS_RX 宏定义
*                   TX                  查看 zf_device_gnss.h 中 GNSS_TX 宏定义
*                   VCC                 3.3V电源
*                   GND                 电源地
*                   ------------------------------------
********************************************************************************************************************/

#ifndef _zf_device_gnss_h_
#define _zf_device_gnss_h_

#include "zf_common_typedef.h"

//--------------------------------------------------------------------------------------------------
//引脚配置
//--------------------------------------------------------------------------------------------------
#define GNSS_UART           (UART_1)
#define GNSS_RX             (UART1_TX_P31)                                    // GPS RX引脚连接到单片机此
#define GNSS_TX             (UART1_RX_P30)                                    // GPS TX串口引脚

#define ANGLE_TO_RAD(x)     ( (x) * PI / 180.0 )                                // 角度转换为弧度
#define RAD_TO_ANGLE(x)     ( (x) * 180.0 / PI )                                // 弧度转换为角度
#define PI                  ( 3.1415926535898 )

typedef enum
{
    TAU1201 = 1,                                                                // 逐飞科技双频GPS模块
    GN42A   = 1,                                                                // 逐飞科技双频GPS模块 与TAU1201是一样的
    GN43RFA = 2,                                                                // 逐飞科技三频RTK模块
}gps_device_enum;

typedef struct
{
    uint16      year;  
    uint8       month; 
    uint8       day;
    uint8       hour;
    uint8       minute;
    uint8       second;
}gps_time_struct;

typedef struct
{
    gps_time_struct    time;                                                    // 时间
    
    uint8       state;                                                          // 有效状态  1：定位有效  0：定位无效
    
    uint16      latitude_degree;                                                // 度
    uint16      latitude_cent;                                                  // 分
    uint16      latitude_second;                                                // 秒    这里的秒是被放大了100倍的，主要是避免使用浮点数
    uint16      longitude_degree;                                               // 度
    uint16      longitude_cent;                                                 // 分
    uint16      longitude_second;                                               // 秒    这里的秒是被放大了100倍的，主要是避免使用浮点数
    
    double      latitude;                                                       // 纬度
    double      longitude;                                                      // 经度
    
    int8        ns;                                                             // 纬度半球 N（北半球）或 S（南半球）
    int8        ew;                                                             // 经度半球 E（东经）或 W（西经）
    
    float       speed;                                                          // 速度（公里/每小时）
    float       direction;                                                      // 地面航向（000.0~359.9 度，以真北方为参考基准）+
    uint8       antenna_direction_state;                                        // 双天线测向有效状态 1：测向有效  0：测向无效，无效时antenna_direction数据是无效的
    float       antenna_direction;                                              // 主天线指向从天线与真北构成的夹角（000.0~359.9 度）
    
    // 下面两个个信息从GNGGA语句中获取
    uint8       satellite_used;                                                 // 用于定位的卫星数量
    float       height;                                                         // 高度
}gnss_info_struct;

typedef enum
{
    GPS_STATE_RECEIVING,                                                        // 正在接收数据
    GPS_STATE_RECEIVED,                                                         // 数据接收完成
    GPS_STATE_PARSING,                                                          // 正在解析
}gps_state_enum;

extern gnss_info_struct gnss;
extern uint8            gnss_flag;


double      get_two_points_distance     (double lat1, double lng1, double lat2, double lng2);
double      get_two_points_azimuth      (double lat1, double lon1, double lat2, double lon2);
uint8       gnss_data_parse             (void);
void        gnss_uart_callback          (void);
void        gnss_init                   (gps_device_enum gps_device);

#endif
