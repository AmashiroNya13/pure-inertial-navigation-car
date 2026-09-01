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

#ifndef _zf_common_fifo_h_
#define _zf_common_fifo_h_

#include "zf_common_typedef.h"

typedef enum
{
    FIFO_SUCCESS,                                                               // FIFO 操作成功

    FIFO_RESET_UNDO,                                                            // FIFO 重置操作未执行
    FIFO_CLEAR_UNDO,                                                            // FIFO 清空操作未执行
    FIFO_BUFFER_NULL,                                                           // FIFO 用户缓冲区异常
    FIFO_WRITE_UNDO,                                                            // FIFO 写入操作未执行
    FIFO_SPACE_NO_ENOUGH,                                                       // FIFO 写入操作 缓冲区空间不足
    FIFO_READ_UNDO,                                                             // FIFO 读取操作未执行
    FIFO_DATA_NO_ENOUGH,                                                        // FIFO 读取操作 数据长度不足
}fifo_state_enum;                                                               // FIFO 操作结果

// 操作逻辑
// 整体重置操作   将会强制清空 FIFO 谨慎使用
// 数据写入操作   不能在重置以及写入操作时进行
// 顺序读取操作   不能在清空和重置操作时进行
// 尾部读取操作   不能在清空和重置以及写入操作时进行
// 读取清空操作   不能在清空和重置以及读取操作时进行
// 这是为了防止中断嵌套导致数据混乱
typedef enum
{
    FIFO_IDLE       = 0x00,                                                     // 空闲状态

    FIFO_RESET      = 0x01,                                                     // 正在执行重置缓冲区
    FIFO_CLEAR      = 0x02,                                                     // 正在执行清空缓冲区
    FIFO_WRITE      = 0x04,                                                     // 正在执行写入缓冲区
    FIFO_READ       = 0x08,                                                     // 正在执行读取缓冲区
}fifo_execution_enum;                                                           // FIFO 操作状态 为嵌套使用预留 无法完全避免误操作

typedef enum
{
    FIFO_READ_AND_CLEAN,                                                        // FIFO 读操作模式 读取后清空释放对应缓冲区
    FIFO_READ_ONLY,                                                             // FIFO 读操作模式 仅读取
}fifo_operation_enum;

typedef enum
{
    FIFO_DATA_8BIT,                                                             // FIFO 数据位宽 8bit
    FIFO_DATA_16BIT,                                                            // FIFO 数据位宽 16bit
    FIFO_DATA_32BIT,                                                            // FIFO 数据位宽 32bit
}fifo_data_type_enum;

typedef struct 
{
    uint8               execution;                                              // 执行步骤
    fifo_data_type_enum type;                                                   // 数据类型
    void                *buffer;                                                // 缓存指针
    uint32              head;                                                   // 缓存头指针 总是指向空的缓存
    uint32              end;                                                    // 缓存尾指针 总是指向非空缓存（缓存全空除外）
    uint32              siz;                                                   // 缓存剩余大小
    uint32              max;                                                    // 缓存总大小
}fifo_struct;

fifo_state_enum fifo_clear              (fifo_struct *fifo);
uint32          fifo_used               (fifo_struct *fifo);

//fifo_state_enum fifo_write_element      (fifo_struct *fifo, uint32 dat);
fifo_state_enum fifo_write_buffer       (fifo_struct *fifo, void *dat, uint32 length);
//fifo_state_enum fifo_read_element       (fifo_struct *fifo, void *dat, fifo_operation_enum flag);
fifo_state_enum fifo_read_buffer        (fifo_struct *fifo, void *dat, uint32 *length, fifo_operation_enum flag);
//fifo_state_enum fifo_read_tail_buffer   (fifo_struct *fifo, void *dat, uint32 *length, fifo_operation_enum flag);

fifo_state_enum fifo_init               (fifo_struct *fifo, fifo_data_type_enum type, void *buffer_addr, uint32 siz);

#endif
