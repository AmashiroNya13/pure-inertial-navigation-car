#include "../../../../inc/middleware/tools/tools_host/tools_host.h"

#include <string.h>

IFX_INLINE uint8 tools_host_split(uint8* line, uint8* argv[]);
IFX_INLINE void tools_host_dispatch(uint8* line);

static uint8 tools_host_command_table[TOOLS_HOST_COMMAND_COUNT][TOOLS_HOST_COMMAND_LENGTH];
static tools_host_callback_t tools_host_callback_table[TOOLS_HOST_COMMAND_COUNT];
static uint8 tools_host_command_registered_count;
static uint8 tools_host_line_buffer[TOOLS_HOST_LINE_LENGTH];
static uint8 tools_host_line_length;
static uint8 tools_host_pending_line_queue[TOOLS_HOST_PENDING_LINE_COUNT][TOOLS_HOST_LINE_LENGTH];
static volatile uint8 tools_host_pending_line_head;
static volatile uint8 tools_host_pending_line_tail;
static tools_host_binary_callback_t tools_host_binary_callback;
static uint8 tools_host_binary_frame[TOOLS_HOST_BINARY_FRAME_MAX_LENGTH];
static uint16 tools_host_binary_frame_length;
static uint16 tools_host_binary_expected_length;
static uint8 tools_host_binary_magic_match;
static uint8 tools_host_pending_binary_frame_queue[TOOLS_HOST_PENDING_BINARY_FRAME_COUNT]
                                                   [TOOLS_HOST_BINARY_FRAME_MAX_LENGTH];
static uint16 tools_host_pending_binary_frame_length[TOOLS_HOST_PENDING_BINARY_FRAME_COUNT];
static volatile uint8 tools_host_pending_binary_frame_head;
static volatile uint8 tools_host_pending_binary_frame_tail;

static const uint8 tools_host_binary_magic[4] = {0x00u, 0xA5u, 0x5Au, 0xC3u};

IFX_INLINE boolean tools_host_pending_line_push(const uint8* line);
IFX_INLINE boolean tools_host_pending_line_pop(uint8* line);
IFX_INLINE boolean tools_host_pending_binary_frame_push(const uint8* frame, uint16 length);
IFX_INLINE boolean tools_host_pending_binary_frame_pop(uint8* frame, uint16* length);
IFX_INLINE boolean tools_host_binary_byte_process(uint8 ch);

void tools_host_init(void)
{
    tools_host_command_registered_count = 0u;
    tools_host_line_length = 0u;
    tools_host_pending_line_head = 0u;
    tools_host_pending_line_tail = 0u;
    tools_host_binary_callback = NULL_PTR;
    tools_host_binary_frame_length = 0u;
    tools_host_binary_expected_length = 0u;
    tools_host_binary_magic_match = 0u;
    tools_host_pending_binary_frame_head = 0u;
    tools_host_pending_binary_frame_tail = 0u;

    for (uint8 i = 0u; i < TOOLS_HOST_COMMAND_COUNT; i++)
    {
        tools_host_command_table[i][0] = '\0';
        tools_host_callback_table[i] = NULL_PTR;
    }
}

void tools_host_init_all(void)
{
    tools_host_init();
}

boolean tools_host_register(const uint8* command, tools_host_callback_t callback)
{
    uint8 command_length;

    if ((command == NULL_PTR) || (callback == NULL_PTR))
    {
        return FALSE;
    }

    command_length = (uint8)strlen((const char*)command);

    if ((command_length == 0u) || (command_length >= TOOLS_HOST_COMMAND_LENGTH))
    {
        return FALSE;
    }

    for (uint8 i = 0u; i < tools_host_command_registered_count; i++)
    {
        if (strcmp((const char*)command, (const char*)tools_host_command_table[i]) == 0)
        {
            tools_host_callback_table[i] = callback;
            return TRUE;
        }
    }

    if (tools_host_command_registered_count >= TOOLS_HOST_COMMAND_COUNT)
    {
        return FALSE;
    }

    (void)memcpy(tools_host_command_table[tools_host_command_registered_count], command, command_length + 1u);
    tools_host_callback_table[tools_host_command_registered_count] = callback;
    tools_host_command_registered_count++;

    return TRUE;
}

void tools_host_rx_callback(void)
{
    tools_host_cfg_t* tools_host_cfg = tools_host_cfg_table_get();
    device_debug_runtime_t* debug_runtime = device_debug_runtime_table_get();
    Ifx_Fifo* fifo = (Ifx_Fifo*)debug_runtime[tools_host_cfg[TOOLS_HOST_1].debug_id].rxFifo_Buffer;

    Ifx_SizeT count = fifo->shared.count;

    if (count == 0u)
    {
        return;
    }

    uint8* buffer = (uint8*)fifo->buffer;
    Ifx_SizeT size = fifo->size;
    Ifx_SizeT start = fifo->startIndex;

    for (Ifx_SizeT i = 0u; i < count; i++)
    {
        uint8 ch = buffer[(start + i) % size];

        if (tools_host_binary_byte_process(ch) != FALSE)
        {
            continue;
        }

        if ((ch == '\r') || (ch == '\n'))
        {
            if (tools_host_line_length > 0u)
            {
                tools_host_line_buffer[tools_host_line_length] = '\0';
                if (tools_host_pending_line_push(tools_host_line_buffer) == FALSE)
                {
                    tools_host_line_length = 0u;
                    continue;
                }
                tools_host_line_length = 0u;
            }

            continue;
        }

        if (tools_host_line_length < (TOOLS_HOST_LINE_LENGTH - 1u))
        {
            tools_host_line_buffer[tools_host_line_length++] = ch;
        }
        else
        {
            tools_host_line_length = 0u;
        }
    }

    fifo->shared.count -= count;
    fifo->startIndex = (fifo->startIndex + count) % fifo->size;
}

void tools_host_binary_register(tools_host_binary_callback_t callback)
{
    tools_host_binary_callback = callback;
}

void tools_host_process(void)
{
    uint8 line[TOOLS_HOST_LINE_LENGTH];
    uint8 binary_frame[TOOLS_HOST_BINARY_FRAME_MAX_LENGTH];
    uint16 binary_length;

    if (tools_host_pending_binary_frame_pop(binary_frame, &binary_length) != FALSE)
    {
        if (tools_host_binary_callback != NULL_PTR)
        {
            tools_host_binary_callback(binary_frame, binary_length);
        }
    }

    if (tools_host_pending_line_pop(line) == FALSE)
    {
        return;
    }

    tools_host_dispatch(line);
}

IFX_INLINE boolean tools_host_binary_byte_process(uint8 ch)
{
    if (tools_host_binary_frame_length > 0u)
    {
        if (tools_host_binary_frame_length >= TOOLS_HOST_BINARY_FRAME_MAX_LENGTH)
        {
            tools_host_binary_frame_length = 0u;
            tools_host_binary_expected_length = 0u;
            return TRUE;
        }

        tools_host_binary_frame[tools_host_binary_frame_length++] = ch;
        if (tools_host_binary_frame_length == 8u)
        {
            uint16 payload_length = (uint16)tools_host_binary_frame[6]
                                  | ((uint16)tools_host_binary_frame[7] << 8u);
            uint32 expected_length = 8u + (uint32)payload_length + 4u;

            if ((expected_length > TOOLS_HOST_BINARY_FRAME_MAX_LENGTH) || (payload_length < 12u))
            {
                tools_host_binary_frame_length = 0u;
                tools_host_binary_expected_length = 0u;
                return TRUE;
            }
            tools_host_binary_expected_length = (uint16)expected_length;
        }

        if ((tools_host_binary_expected_length > 0u)
            && (tools_host_binary_frame_length == tools_host_binary_expected_length))
        {
            (void)tools_host_pending_binary_frame_push(tools_host_binary_frame,
                                                       tools_host_binary_frame_length);
            tools_host_binary_frame_length = 0u;
            tools_host_binary_expected_length = 0u;
        }
        return TRUE;
    }

    if (ch == tools_host_binary_magic[tools_host_binary_magic_match])
    {
        tools_host_binary_magic_match++;
        if (tools_host_binary_magic_match == (uint8)sizeof(tools_host_binary_magic))
        {
            (void)memcpy(tools_host_binary_frame,
                         tools_host_binary_magic,
                         sizeof(tools_host_binary_magic));
            tools_host_binary_frame_length = (uint16)sizeof(tools_host_binary_magic);
            tools_host_binary_expected_length = 0u;
            tools_host_binary_magic_match = 0u;
        }
        return TRUE;
    }

    tools_host_binary_magic_match = (ch == tools_host_binary_magic[0]) ? 1u : 0u;
    return (ch == tools_host_binary_magic[0]) ? TRUE : FALSE;
}

IFX_INLINE boolean tools_host_pending_binary_frame_push(const uint8* frame, uint16 length)
{
    uint8 tail = tools_host_pending_binary_frame_tail;
    uint8 next_tail = (uint8)(tail + 1u);

    if ((frame == NULL_PTR) || (length == 0u) || (length > TOOLS_HOST_BINARY_FRAME_MAX_LENGTH))
    {
        return FALSE;
    }
    if (next_tail >= TOOLS_HOST_PENDING_BINARY_FRAME_COUNT)
    {
        next_tail = 0u;
    }
    if (next_tail == tools_host_pending_binary_frame_head)
    {
        return FALSE;
    }

    (void)memcpy(tools_host_pending_binary_frame_queue[tail], frame, length);
    tools_host_pending_binary_frame_length[tail] = length;
    tools_host_pending_binary_frame_tail = next_tail;
    return TRUE;
}

IFX_INLINE boolean tools_host_pending_binary_frame_pop(uint8* frame, uint16* length)
{
    uint8 head = tools_host_pending_binary_frame_head;
    uint8 next_head = (uint8)(head + 1u);

    if ((frame == NULL_PTR) || (length == NULL_PTR)
        || (head == tools_host_pending_binary_frame_tail))
    {
        return FALSE;
    }
    if (next_head >= TOOLS_HOST_PENDING_BINARY_FRAME_COUNT)
    {
        next_head = 0u;
    }

    *length = tools_host_pending_binary_frame_length[head];
    (void)memcpy(frame, tools_host_pending_binary_frame_queue[head], *length);
    tools_host_pending_binary_frame_head = next_head;
    return TRUE;
}

IFX_INLINE boolean tools_host_pending_line_push(const uint8* line)
{
    uint8 tail = tools_host_pending_line_tail;
    uint8 next_tail = (uint8)(tail + 1u);

    if (next_tail >= TOOLS_HOST_PENDING_LINE_COUNT)
    {
        next_tail = 0u;
    }

    if (next_tail == tools_host_pending_line_head)
    {
        return FALSE;
    }

    (void)memcpy(tools_host_pending_line_queue[tail], line, TOOLS_HOST_LINE_LENGTH);
    tools_host_pending_line_tail = next_tail;
    return TRUE;
}

IFX_INLINE boolean tools_host_pending_line_pop(uint8* line)
{
    uint8 head = tools_host_pending_line_head;
    uint8 next_head = (uint8)(head + 1u);

    if (head == tools_host_pending_line_tail)
    {
        return FALSE;
    }

    if (next_head >= TOOLS_HOST_PENDING_LINE_COUNT)
    {
        next_head = 0u;
    }

    (void)memcpy(line, tools_host_pending_line_queue[head], TOOLS_HOST_LINE_LENGTH);
    tools_host_pending_line_head = next_head;
    return TRUE;
}

IFX_INLINE uint8 tools_host_split(uint8* line, uint8* argv[])
{
    uint8 argc = 0u;
    uint8 i = 0u;

    while (line[i] != '\0')
    {
        while (line[i] == ' ')
        {
            i++;
        }

        if (line[i] == '\0')
        {
            break;
        }

        if (argc >= TOOLS_HOST_ARG_COUNT)
        {
            break;
        }

        argv[argc++] = &line[i];

        while ((line[i] != '\0') && (line[i] != ' '))
        {
            i++;
        }

        if (line[i] == '\0')
        {
            break;
        }

        line[i++] = '\0';
    }

    return argc;
}

IFX_INLINE void tools_host_dispatch(uint8* line)
{
    uint8* argv[TOOLS_HOST_ARG_COUNT];
    uint8 argc = tools_host_split(line, argv);

    if (argc == 0u)
    {
        return;
    }

    for (uint8 i = 0u; i < tools_host_command_registered_count; i++)
    {
        if (strcmp((const char*)argv[0], (const char*)tools_host_command_table[i]) == 0)
        {
            tools_host_callback_table[i](argc, argv);
            return;
        }
    }
}

