#include "test_device_debug.h"
#include "device_debug.h"
#include "tools_print.h"
#include "Ifx_Fifo.h"

#define TEST_DEBUG_SEND_DELAY    3000000u
#define TEST_DEBUG_RECV_BUF_SIZE 64



static uint8 recv_buf[TEST_DEBUG_RECV_BUF_SIZE];
static uint8 recv_len;

static uint8 recv_buf_fifo[TEST_DEBUG_RECV_BUF_SIZE];
static uint8 recv_len_fifo;

void test_device_debug_send(void)
{
    for (volatile uint32 i = 0; i < TEST_DEBUG_SEND_DELAY; i++) { }
    tools_print((const uint8*)"hello\r\n");
}

void test_device_debug_recv_callback(void)
{
    uint8 ch;

    for (uint8 i = 0; i < TEST_DEBUG_RECV_BUF_SIZE; i++)
    {
        ch = 0xFF;
        device_debug_query_byte(DEVICE_DEBUG_1, &ch);

        if (ch == 0xFF)
        {
            break;
        }

        if ((ch == '\r') || (ch == '\n'))
        {
            if (recv_len > 0u)
            {
                recv_buf[recv_len] = '\0';

                if ((recv_len == 5u) &&
                    (recv_buf[0] == 'w') &&
                    (recv_buf[1] == 'o') &&
                    (recv_buf[2] == 'r') &&
                    (recv_buf[3] == 'l') &&
                    (recv_buf[4] == 'd'))
                {
                    tools_print((const uint8*)"ok\r\n");
                }
                else
                {
                    tools_print((const uint8*)"no\r\n");
                }

                recv_len = 0u;
            }
        }
        else
        {
            if (recv_len < (TEST_DEBUG_RECV_BUF_SIZE - 1u))
            {
                recv_buf[recv_len] = ch;
                recv_len++;
            }
        }
    }
}

void test_device_debug_recv_callback_fifo(void)
{
    device_debug_runtime_t *rt = device_debug_runtime_table_get();
    Ifx_Fifo *fifo = (Ifx_Fifo *)rt[DEVICE_DEBUG_1].rxFifo_Buffer;
    IfxAsclin_Asc *asclin = &rt[DEVICE_DEBUG_1].asclin_asc_runtime.asclin_asc_modulehn;

    Ifx_SizeT count = fifo->shared.count;

    if (count == 0u)
    {
        return;
    }

    uint8 *buf = (uint8 *)fifo->buffer;
    Ifx_SizeT size = fifo->size;
    Ifx_SizeT start = fifo->startIndex;

    for (Ifx_SizeT i = 0; i < count; i++)
    {
        uint8 ch = buf[(start + i) % size];

        if ((ch == '\r') || (ch == '\n'))
        {
            if (recv_len_fifo > 0u)
            {
                if ((recv_len_fifo == 5u) &&
                    (recv_buf_fifo[0] == 'w') &&
                    (recv_buf_fifo[1] == 'o') &&
                    (recv_buf_fifo[2] == 'r') &&
                    (recv_buf_fifo[3] == 'l') &&
                    (recv_buf_fifo[4] == 'd'))
                {
                    tools_print((const uint8*)"ok\r\n");
                }
                else
                {
                    tools_print((const uint8*)"no\r\n");
                }

                recv_len_fifo = 0u;
                IfxAsclin_Asc_clearRx(asclin);
            }
        }
        else
        {
            if (recv_len_fifo < (TEST_DEBUG_RECV_BUF_SIZE - 1u))
            {
                recv_buf_fifo[recv_len_fifo] = ch;
                recv_len_fifo++;
            }
        }
    }

    if (recv_len_fifo > 0u)
    {
        tools_print((const uint8*)"...\r\n");
    }
}

void test_device_debug_recv_callback_check(void)
{
    device_debug_runtime_t *rt = device_debug_runtime_table_get();
    Ifx_Fifo *fifo = (Ifx_Fifo *)rt[DEVICE_DEBUG_1].rxFifo_Buffer;
    IfxAsclin_Asc *asclin = &rt[DEVICE_DEBUG_1].asclin_asc_runtime.asclin_asc_modulehn;

    Ifx_SizeT count = fifo->shared.count;

    if (count == 0u)
    {
        return;
    }

    static uint32 total = 0u;
    static uint32 errors = 0u;

    uint8 *buf = (uint8 *)fifo->buffer;
    Ifx_SizeT size = fifo->size;
    Ifx_SizeT start = fifo->startIndex;

    for (Ifx_SizeT i = 0; i < count; i++)
    {
        uint8 ch = buf[(start + i) % size];

        total++;
        if (ch != 0x55u)
        {
            errors++;
        }

        if ((total & 0x1Fu) == 0u)
        {
            uint32 rate = (errors * 1000u) / total;

            tools_printf("ERR=%lu/%lu %lu.%lu%%\r\n",
                         (unsigned long)errors,
                         (unsigned long)total,
                         (unsigned long)(rate / 10u),
                         (unsigned long)(rate % 10u));
        }
    }

    IfxAsclin_Asc_clearRx(asclin);
}
