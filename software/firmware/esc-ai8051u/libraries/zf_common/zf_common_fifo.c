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



#include "zf_common_fifo.h"
#include "zf_common_debug.h"

#pragma warning disable = 188

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     FIFO 头指针位移
// 参数说明     *fifo               FIFO 对象指针
// 参数说明     offset              偏移量
// 返回参数     void
// 使用示例     fifo_head_offset(fifo, 1);
// 备注信息     本函数在文件内部调用 用户不用关注 也不可修改
//-------------------------------------------------------------------------------------------------------------------
static void fifo_head_offset (fifo_struct *fifo, uint32 offset)
{
    fifo->head += offset;
    
    while(fifo->max <= fifo->head)                                              // 如果范围超过则减缓冲区大小 直到小于最大缓冲区大小
    {
        fifo->head -= fifo->max;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     FIFO 尾指针位移
// 参数说明     *fifo               FIFO 对象指针
// 参数说明     offset              偏移量
// 返回参数     void
// 使用示例     fifo_end_offset(fifo, 1);
// 备注信息     本函数在文件内部调用 用户不用关注 也不可修改
//-------------------------------------------------------------------------------------------------------------------
static void fifo_end_offset (fifo_struct *fifo, uint32 offset)
{
    fifo->end += offset;
    
    while(fifo->max <= fifo->end)                                               // 如果范围超过则减缓冲区大小 直到小于最大缓冲区大小
    {
        fifo->end -= fifo->max;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     FIFO 重置缓冲器
// 参数说明     *fifo               FIFO 对象指针
// 返回参数     void
// 使用示例     fifo_clear(fifo);
// 备注信息     清空当前 FIFO 对象的内存
//-------------------------------------------------------------------------------------------------------------------
fifo_state_enum fifo_clear (fifo_struct *fifo)
{
	fifo_state_enum return_state = FIFO_SUCCESS;                                // 操作结果初值
    zf_assert(NULL != fifo);
	
    do
    {
        if(FIFO_IDLE != fifo->execution)                                        // 判断是否当前 FIFO 是否空闲
        {
            return_state = FIFO_RESET_UNDO;                                     // 重置操作未完成
            break;
        }
        fifo->execution |= FIFO_RESET;                                          // 重置操作置位
        fifo->head      = 0;                                                    // 重置 FIFO 所有数值复位
        fifo->end       = 0;                                                    // 重置 FIFO 所有数值复位
        fifo->siz      = fifo->max;                                            // 重置 FIFO 所有数值复位
        switch(fifo->type)
        {
            case FIFO_DATA_8BIT:    memset(fifo->buffer, 0, fifo->max);     break;
            case FIFO_DATA_16BIT:   memset(fifo->buffer, 0, fifo->max * 2); break;
            case FIFO_DATA_32BIT:   memset(fifo->buffer, 0, fifo->max * 4); break;
        }
        fifo->execution = FIFO_IDLE;                                            // 操作状态复位
    }while(0);
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     FIFO 查询当前数据个数
// 参数说明     *fifo               FIFO 对象指针
// 返回参数     uint32              已使用长度
// 使用示例     uint32 len = fifo_used(fifo);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint32 fifo_used (fifo_struct *fifo)
{
    zf_assert(fifo != NULL);
    return (fifo->max - fifo->siz);                                            // 返回当前 FIFO 缓冲区中数据个数
}

////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     向 FIFO 中写入数据
//// 参数说明     *fifo               FIFO 对象指针
//// 参数说明     dat                 数据
//// 返回参数     fifo_state_enum     操作状态
//// 使用示例     zf_log(fifo_write_element(&fifo, data) == FIFO_SUCCESS, "fifo_write_byte error");
//// 备注信息
////-------------------------------------------------------------------------------------------------------------------
//fifo_state_enum fifo_write_element (fifo_struct *fifo, uint32 dat)
//{
//    zf_assert(NULL != fifo);
//    fifo_state_enum return_state = FIFO_SUCCESS;                                // 操作结果初值

//    do
//    {
//        if((FIFO_RESET | FIFO_WRITE) & fifo->execution)                         // 不在写入与重置状态 避免写入竞争与指向错误
//        {
//            return_state = FIFO_WRITE_UNDO;                                     // 写入操作未完成
//            break;
//        }
//        fifo->execution |= FIFO_WRITE;                                          // 写入操作置位

//        if(1 <= fifo->siz)                                                     // 剩余空间足够装下本次数据
//        {
//            switch(fifo->type)
//            {
//                case FIFO_DATA_8BIT:    ((uint8 *)fifo->buffer)[fifo->head]  = dat;  break;
//                case FIFO_DATA_16BIT:   ((uint16 *)fifo->buffer)[fifo->head] = dat; break;
//                case FIFO_DATA_32BIT:   ((uint32 *)fifo->buffer)[fifo->head] = dat; break;
//            }
//            fifo_head_offset(fifo, 1);                                          // 头指针偏移
//            fifo->siz -= 1;                                                    // 缓冲区剩余长度减小
//        }
//        else
//        {
//            return_state = FIFO_SPACE_NO_ENOUGH;                                // 当前 FIFO 缓冲区满 不能再写入数据 返回空间不足
//        }
//        fifo->execution &= ~FIFO_WRITE;                                         // 写入操作复位
//    }while(0);

//    return return_state;
//}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     向 FIFO 中写入数据
// 参数说明     *fifo               FIFO 对象指针
// 参数说明     *dat                数据来源缓冲区指针
// 参数说明     length              需要写入的数据长度
// 返回参数     fifo_state_enum     操作状态
// 使用示例     zf_log(fifo_write_buffer(&fifo, data, 32) == FIFO_SUCCESS, "fifo_write_buffer error");
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
fifo_state_enum fifo_write_buffer (fifo_struct *fifo, void *dat, uint32 length)
{
    fifo_state_enum return_state = FIFO_SUCCESS;                                // 操作结果初值
    uint32 temp_length = 0;
	zf_assert(NULL != fifo);
	
    do
    {
        if(NULL == dat)
        {
            return_state = FIFO_BUFFER_NULL;                                    // 用户缓冲区异常
            break;
        }
        if((FIFO_RESET | FIFO_WRITE) & fifo->execution)                         // 不在写入与重置状态 避免写入竞争与指向错误
        {
            return_state = FIFO_WRITE_UNDO;                                     // 写入操作未完成
            break;
        }
        fifo->execution |= FIFO_WRITE;                                          // 写入操作置位

        if(length <= fifo->siz)                                                // 剩余空间足够装下本次数据
        {
            temp_length = fifo->max - fifo->head;                               // 计算头指针距离缓冲区尾还有多少空间

            if(length > temp_length)                                            // 距离缓冲区尾长度不足写入数据 环形缓冲区分段操作
            {
                switch(fifo->type)
                {
                    case FIFO_DATA_8BIT:
                    {
                        memcpy(
                            &(((uint8 *)fifo->buffer)[fifo->head]),
                            dat, temp_length);                                  // 拷贝第一段数据
                        fifo_head_offset(fifo, temp_length);                    // 头指针偏移
                        memcpy(
                            &(((uint8 *)fifo->buffer)[fifo->head]),
                            &(((uint8 *)dat)[temp_length]),
                            length - temp_length);                              // 拷贝第二段数据
                        fifo_head_offset(fifo, length - temp_length);           // 头指针偏移
                    }break;
                    case FIFO_DATA_16BIT:
                    {
                        memcpy(
                            &(((uint16 *)fifo->buffer)[fifo->head]),
                            dat, temp_length * 2);                              // 拷贝第一段数据
                        fifo_head_offset(fifo, temp_length);                    // 头指针偏移
                        memcpy(
                            &(((uint16 *)fifo->buffer)[fifo->head]),
                            &(((uint16 *)dat)[temp_length]),
                            (length - temp_length) * 2);                        // 拷贝第二段数据
                        fifo_head_offset(fifo, length - temp_length);           // 头指针偏移
                    }break;
                    case FIFO_DATA_32BIT:
                    {
                        memcpy(
                            &(((uint32 *)fifo->buffer)[fifo->head]),
                            dat, temp_length * 4);                              // 拷贝第一段数据
                        fifo_head_offset(fifo, temp_length);                    // 头指针偏移
                        memcpy(
                            &(((uint32 *)fifo->buffer)[fifo->head]),
                            &(((uint32 *)dat)[temp_length]),
                            (length - temp_length) * 4);                        // 拷贝第二段数据
                        fifo_head_offset(fifo, length - temp_length);           // 头指针偏移
                    }break;
                }
            }
            else
            {
                switch(fifo->type)
                {
                    case FIFO_DATA_8BIT:
                    {
                        memcpy(
                            &(((uint8 *)fifo->buffer)[fifo->head]),
                            dat, length);                                       // 一次完整写入
                        fifo_head_offset(fifo, length);                         // 头指针偏移
                    }break;
                    case FIFO_DATA_16BIT:
                    {
                        memcpy(
                            &(((uint16 *)fifo->buffer)[fifo->head]),
                            dat, length * 2);                                   // 一次完整写入
                        fifo_head_offset(fifo, length);                         // 头指针偏移
                    }break;
                    case FIFO_DATA_32BIT:
                    {
                        memcpy(
                            &(((uint32 *)fifo->buffer)[fifo->head]),
                            dat, length * 4);                                   // 一次完整写入
                        fifo_head_offset(fifo, length);                         // 头指针偏移
                    }break;
                }
            }

            fifo->siz -= length;                                               // 缓冲区剩余长度减小
        }
        else
        {
            return_state = FIFO_SPACE_NO_ENOUGH;                                // 当前 FIFO 缓冲区满 不能再写入数据 返回空间不足
        }
        fifo->execution &= ~FIFO_WRITE;                                         // 写入操作复位
    }while(0);

    return return_state;
}

////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     从 FIFO 读取数据
//// 参数说明     *fifo               FIFO 对象指针
//// 参数说明     *dat                目标缓冲区指针
//// 参数说明     flag                是否变更 FIFO 状态 可选择是否清空读取的数据
//// 返回参数     fifo_state_enum     操作状态
//// 使用示例     zf_log(fifo_read_element(&fifo, data, FIFO_READ_ONLY) == FIFO_SUCCESS, "fifo_read_byte error");
//// 备注信息
////-------------------------------------------------------------------------------------------------------------------
//fifo_state_enum fifo_read_element (fifo_struct *fifo, void *dat, fifo_operation_enum flag)
//{
//    zf_assert(NULL != fifo);
//    fifo_state_enum return_state = FIFO_SUCCESS;                                // 操作结果初值

//    do
//    {
//        if(NULL == dat)
//        {
//            return_state = FIFO_BUFFER_NULL;                                    // 用户缓冲区异常
//        }
//        else
//        {
//            if((FIFO_RESET | FIFO_CLEAR) & fifo->execution)                     // 判断是否当前 FIFO 是否在执行清空或重置操作
//            {
//                return_state = FIFO_READ_UNDO;                                  // 读取操作未完成
//                break;
//            }

//            if(1 > fifo_used(fifo))
//            {
//                return_state = FIFO_DATA_NO_ENOUGH;                             // 缓冲区没有数据 返回数据长度不足
//                break;                                                          // 直接退出操作
//            }

//            fifo->execution |= FIFO_READ;                                       // 读操作置位
//            switch(fifo->type)
//            {
//                case FIFO_DATA_8BIT:    *((uint8 *)dat) = ((uint8 *)fifo->buffer)[fifo->end];   break;
//                case FIFO_DATA_16BIT:   *((uint16 *)dat) = ((uint16 *)fifo->buffer)[fifo->end]; break;
//                case FIFO_DATA_32BIT:   *((uint32 *)dat) = ((uint32 *)fifo->buffer)[fifo->end]; break;
//            }
//            fifo->execution &= ~FIFO_READ;                                      // 读操作复位
//        }

//        if(FIFO_READ_AND_CLEAN == flag)                                         // 如果选择读取并更改 FIFO 状态
//        {
//            if((FIFO_RESET | FIFO_CLEAR | FIFO_READ) == fifo->execution)        // 不在 重置 清空 读取 状态 避免异常
//            {
//                return_state = FIFO_CLEAR_UNDO;                                 // 清空操作未完成
//                break;
//            }
//            fifo->execution |= FIFO_CLEAR;                                      // 清空作置位
//            fifo_end_offset(fifo, 1);                                           // 移动 FIFO 头指针
//            fifo->siz += 1;                                                    // 释放对应长度空间
//            fifo->execution &= ~FIFO_CLEAR;                                     // 清空作复位
//        }
//    }while(0);

//    return return_state;
//}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从 FIFO 读取数据
// 参数说明     *fifo               FIFO 对象指针
// 参数说明     *dat                目标缓冲区指针
// 参数说明     *length             读取的数据长度 如果没有这么多数据这里会被修改
// 参数说明     flag                是否变更 FIFO 状态 可选择是否清空读取的数据
// 返回参数     fifo_state_enum     操作状态
// 使用示例     zf_log(fifo_read_buffer(&fifo, data, &length, FIFO_READ_ONLY) == FIFO_SUCCESS, "fifo_read_buffer error");
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
fifo_state_enum fifo_read_buffer (fifo_struct *fifo, void *dat, uint32 *length, fifo_operation_enum flag)
{
    fifo_state_enum return_state = FIFO_SUCCESS;                                // 操作结果初值
    uint32 temp_length = 0;
    uint32 fifo_data_length = 0;
	
    zf_assert(NULL != fifo);
    zf_assert(NULL != length);
	
    do
    {
        if(NULL == dat)
        {
            return_state = FIFO_BUFFER_NULL;
        }
        else
        {
            if((FIFO_RESET | FIFO_CLEAR) & fifo->execution)                     // 判断是否当前 FIFO 是否在执行清空或重置操作
            {
                *length = fifo_data_length;                                     // 纠正读取的长度
                return_state = FIFO_READ_UNDO;                                  // 读取操作未完成
                break;
            }

            fifo_data_length = fifo_used(fifo);                                 // 获取当前数据有多少
            if(*length > fifo_data_length)                                      // 判断长度是否足够
            {
                *length = fifo_data_length;                                     // 纠正读取的长度
                return_state = FIFO_DATA_NO_ENOUGH;                             // 标志数据不够
                if(0 == fifo_data_length)                                       // 如果没有数据 就直接退出
                {
                    fifo->execution &= ~FIFO_READ;                              // 读操作复位
                    break;
                }
            }

            fifo->execution |= FIFO_READ;                                       // 读操作置位
            temp_length = fifo->max - fifo->end;                                // 计算尾指针距离缓冲区尾还有多少空间
            if(*length <= temp_length)                                          // 足够一次性读取完毕
            {
                switch(fifo->type)
                {
                    case FIFO_DATA_8BIT:    memcpy(dat, &(((uint8 *)fifo->buffer)[fifo->end]), *length);        break;
                    case FIFO_DATA_16BIT:   memcpy(dat, &(((uint16 *)fifo->buffer)[fifo->end]), *length * 2);   break;
                    case FIFO_DATA_32BIT:   memcpy(dat, &(((uint32 *)fifo->buffer)[fifo->end]), *length * 4);   break;
                }
            }
            else
            {
                switch(fifo->type)
                {
                    case FIFO_DATA_8BIT:
                    {
                        memcpy(dat, &(((uint8 *)fifo->buffer)[fifo->end]), temp_length);
                        memcpy(&(((uint8 *)dat)[temp_length]), fifo->buffer, *length - temp_length);
                    }break;
                    case FIFO_DATA_16BIT:
                    {
                        memcpy(dat, &(((uint16 *)fifo->buffer)[fifo->end]), temp_length * 2);
                        memcpy(&(((uint16 *)dat)[temp_length]), fifo->buffer, (*length - temp_length) * 2);
                    }break;
                    case FIFO_DATA_32BIT:
                    {
                        memcpy(dat, &(((uint32 *)fifo->buffer)[fifo->end]), temp_length * 4);
                        memcpy(&(((uint32 *)dat)[temp_length]), fifo->buffer, (*length - temp_length) * 4);
                    }break;
                }
            }
            fifo->execution &= ~FIFO_READ;                                      // 读操作复位
        }

        if(FIFO_READ_AND_CLEAN == flag)                                         // 如果选择读取并更改 FIFO 状态
        {
            if((FIFO_RESET | FIFO_CLEAR | FIFO_READ) == fifo->execution)        // 不在 重置 清空 读取 状态 避免异常
            {
                return_state = FIFO_CLEAR_UNDO;                                 // 清空操作未完成
                break;
            }
            fifo->execution |= FIFO_CLEAR;                                      // 清空作置位
            fifo_end_offset(fifo, *length);                                     // 移动 FIFO 头指针
            fifo->siz += *length;                                              // 释放对应长度空间
            fifo->execution &= ~FIFO_CLEAR;                                     // 清空作复位
        }
    }while(0);

    return return_state;
}

////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     从 FIFO 尾部读取指定长度 buffer
//// 参数说明     *fifo               FIFO 对象指针
//// 参数说明     *dat                目标缓冲区指针
//// 参数说明     *length             读取的数据长度 如果没有这么多数据这里会被修改
//// 参数说明     flag                是否变更 FIFO 状态 可选择是否清空读取的数据
//// 返回参数     fifo_state_enum     操作状态
//// 使用示例     zf_log(fifo_read_tail_buffer(&fifo, data, &length, FIFO_READ_ONLY) == FIFO_SUCCESS, "fifo_read_buffer error");
//// 备注信息     如果使用 FIFO_READ_AND_CLEAN 操作 将会丢弃所有数据并清空整个 FIFO
////              如果使用 FIFO_READ_AND_CLEAN 操作 将会丢弃所有数据并清空整个 FIFO
////              如果使用 FIFO_READ_AND_CLEAN 操作 将会丢弃所有数据并清空整个 FIFO
////-------------------------------------------------------------------------------------------------------------------
//fifo_state_enum fifo_read_tail_buffer (fifo_struct *fifo, void *dat, uint32 *length, fifo_operation_enum flag)
//{
//    zf_assert(NULL != fifo);
//    zf_assert(NULL != length);
//    fifo_state_enum return_state = FIFO_SUCCESS;                                // 操作结果初值
//    uint32 temp_length = 0;
//    uint32 fifo_data_length = 0;

//    do
//    {
//        if(NULL == dat)
//        {
//            return_state = FIFO_BUFFER_NULL;
//        }
//        else
//        {
//            if((FIFO_RESET | FIFO_CLEAR | FIFO_WRITE) & fifo->execution)        // 判断是否当前 FIFO 是否在执行清空或重置操作
//            {
//                *length = fifo_data_length;                                     // 纠正读取的长度
//                return_state = FIFO_READ_UNDO;                                  // 读取操作未完成
//                break;
//            }

//            fifo_data_length = fifo_used(fifo);                                 // 获取当前数据有多少
//            if(*length > fifo_data_length)                                      // 判断长度是否足够
//            {
//                *length = fifo_data_length;                                     // 纠正读取的长度
//                return_state = FIFO_DATA_NO_ENOUGH;                             // 标志数据不够
//                if(0 == fifo_data_length)                                       // 如果没有数据 就直接退出
//                {
//                    fifo->execution &= ~FIFO_READ;                              // 读操作复位
//                    break;
//                }
//            }

//            fifo->execution |= FIFO_READ;                                       // 读操作置位
//            if((fifo->head > fifo->end) || (fifo->head >= *length))
//            {
//                switch(fifo->type)
//                {
//                    case FIFO_DATA_8BIT:    memcpy(dat, &(((uint8 *)fifo->buffer)[fifo->head - *length]), *length);     break;
//                    case FIFO_DATA_16BIT:   memcpy(dat, &(((uint16 *)fifo->buffer)[fifo->head - *length]), *length * 2);break;
//                    case FIFO_DATA_32BIT:   memcpy(dat, &(((uint32 *)fifo->buffer)[fifo->head - *length]), *length * 4);break;
//                }
//            }
//            else
//            {
//                temp_length = *length - fifo->head;                             // 计算尾指针距离缓冲区尾还有多少空间
//                switch(fifo->type)
//                {
//                    case FIFO_DATA_8BIT:
//                    {
//                        memcpy(dat, &(((uint8 *)fifo->buffer)[fifo->max - temp_length]), temp_length);
//                        memcpy(&(((uint8 *)dat)[temp_length]), &(((uint8 *)fifo->buffer)[fifo->head - *length]), (*length - temp_length));
//                    }break;
//                    case FIFO_DATA_16BIT:
//                    {
//                        memcpy(dat, &(((uint16 *)fifo->buffer)[fifo->max - temp_length]), temp_length * 2);
//                        memcpy(&(((uint16 *)dat)[temp_length]), &(((uint16 *)fifo->buffer)[fifo->head - *length]), (*length - temp_length) * 2);
//                    }break;
//                    case FIFO_DATA_32BIT:
//                    {
//                        memcpy(dat, &(((uint32 *)fifo->buffer)[fifo->max - temp_length]), temp_length * 4);
//                        memcpy(&(((uint32 *)dat)[temp_length]), &(((uint32 *)fifo->buffer)[fifo->head - *length]), (*length - temp_length) * 4);
//                    }break;
//                }
//            }
//            fifo->execution &= ~FIFO_READ;                                      // 读操作复位
//        }

//        if(FIFO_READ_AND_CLEAN == flag)                                         // 如果选择读取并更改 FIFO 状态
//        {
//            if((FIFO_RESET | FIFO_CLEAR | FIFO_READ) == fifo->execution)        // 不在 重置 清空 读取 状态 避免异常
//            {
//                return_state = FIFO_CLEAR_UNDO;                                 // 清空操作未完成
//                break;
//            }
//            fifo_clear(fifo);
//        }
//    }while(0);

//    return return_state;
//}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     FIFO 初始化 挂载对应缓冲区
// 参数说明     *fifo               FIFO 对象指针
// 参数说明     type                FIFO 数据位数
// 参数说明     *buffer_addr        要挂载的缓冲区
// 参数说明     siz                缓冲区大小
// 返回参数     fifo_state_enum     操作状态
// 使用示例     fifo_init(&user_fifo, user_buffer, 64);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
fifo_state_enum fifo_init (fifo_struct *fifo, fifo_data_type_enum type, void *buffer_addr, uint32 siz)
{
    zf_assert(NULL != fifo);
    do
    {
        fifo->buffer    = buffer_addr;
        fifo->execution = FIFO_IDLE;
        fifo->type      = type;
        fifo->head      = 0;
        fifo->end       = 0;
        fifo->siz      = siz;
        fifo->max       = siz;
    }while(0);
    return FIFO_SUCCESS;
}
