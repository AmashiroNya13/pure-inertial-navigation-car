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
#include "zf_driver_adc.h"


static uint8 adc_resolution = {ADC_12BIT};


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ADC转换N次，平均值滤波
// 参数说明     ch              选择ADC通道
// 参数说明     resolution      分辨率(8位 10位 12位)
// 参数说明     count           转换次数
// 返回参数     void
// 使用示例     adc_mean_filter(ADC_IN0_A0, ADC_8BIT,5);  //采集A0端口返回8位分辨率的AD值，采集五次取平均值
//-------------------------------------------------------------------------------------------------------------------
uint16 adc_mean_filter_convert (adc_channel_enum ch, const uint8 count)
{
    uint8 i;
    uint32 sum;

	zf_assert(count);//断言次数不能为0
	
    sum = 0;
    for(i=0; i<count; i++)
    {
        sum += adc_convert(ch);
    }

    sum = sum/count;
    return sum;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ADC转换一次
//  @param      adcn            选择ADC通道
//  @param      resolution      分辨率
//  @return     void
//  Sample usage:               adc_convert(ADC_P10, ADC_10BIT);
//-------------------------------------------------------------------------------------------------------------------
uint16 adc_convert(adc_channel_enum ch)
{
    uint16 adc_value;
    
    ADC_CONTR &= (0xF0);			//清除ADC_CHS[3:0] ： ADC 模拟通道选择位
    ADC_CONTR |= ch;
    
    ADC_CONTR |= 0x40;  			// 启动 AD 转换
    
    while (!(ADC_CONTR & 0x20));  	// 查询 ADC 完成标志
    
    ADC_CONTR &= ~0x20;  			// 清完成标志
    
    adc_value = ADC_RES;  			//存储 ADC 的 12 位结果的高 4 位
    adc_value <<= 8;
    adc_value |= ADC_RESL;  		//存储 ADC 的 12 位结果的低 8 位
    
    ADC_RES = 0;
    ADC_RESL = 0;
    
    adc_value >>= adc_resolution;	//取多少位
    
    
    return adc_value;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ADC初始化
// 参数说明     ch      			ch(可选择范围由zf_adc.h内adc_channel_enum枚举值确定)
// 参数说明     resolution       resolution(可选择范围由zf_adc.h内adc_resolution_enum枚举值确定)
// 返回参数     void
// 使用示例     adc_init(ADC_P10, ADC_12BIT);//初始化P10引脚为ADC功能，12位分辨率
//-------------------------------------------------------------------------------------------------------------------
void adc_init(adc_channel_enum ch, adc_resolution_enum resolution)
{
    ADC_CONTR |= 1 << 7;				//1 ：打开 ADC 电源
    
    ADC_CONTR &= (0xF0);			//清除ADC_CHS[3:0] ： ADC 模拟通道选择位
    ADC_CONTR |= ch;
    
    if((ch >> 3) == 1) //P0.0
    {
        //IO口需要设置为高阻输入
        P0M0 &= ~(1 << (ch & 0x07));
        P0M1 |= (1 << (ch & 0x07));
    }
    else if((ch >> 3) == 0) //P1.0
    {
        //IO口需要设置为高阻输入
        P1M0 &= ~(1 << (ch & 0x07));
        P1M1 |= (1 << (ch & 0x07));
    }
    
    ADCCFG |= ADC_SYSclk_DIV_16 & 0x0F;			//ADC时钟频率SYSclk/16&0x0F;
    
    ADCCFG |= 1 << 5;							//转换结果右对齐。 ADC_RES 保存结果的高 2 位， ADC_RESL 保存结果的低 8 位。
	
	adc_resolution = resolution;           // 记录ADC精度 将在采集时使用
}