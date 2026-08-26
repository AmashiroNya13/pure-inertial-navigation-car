#include "../../../../inc/middleware/tools/tools_log/tools_log.h"

#include "../../../../inc/middleware/tools/tools_print/tools_print.h"

#define TOOLS_LOG_MAGIC 0xA5u
#define TOOLS_LOG_FLAG_LAST 0x01u
#define TOOLS_LOG_PAGE_DATA_SIZE 5u
#define TOOLS_LOG_LINE_BUFFER_SIZE 128u

IFX_INLINE void tools_log_write_record(tools_log_id_t log_id, const uint8* payload, uint8 payload_length, boolean is_last);
IFX_INLINE boolean tools_log_print_sector(driver_flash_sector_t* sector, uint8* line_buffer, uint8* line_length);

void tools_log_init(tools_log_id_t log_id)
{
    tools_log_cfg_t* log_cfg = tools_log_cfg_table_get();
    tools_log_runtime_t* log_runtime = tools_log_runtime_table_get();
    driver_flash_init(&log_cfg[log_id].flash_cfg, &log_runtime[log_id].flash_runtime);
}

void tools_log_init_all(void)
{
    tools_log_init(TOOLS_LOG_1);
}

void tools_log(tools_log_id_t log_id, const uint8* string)
{
    uint32 length = 0u;
    uint32 offset = 0u;

    if (string == NULL_PTR)
    {
        return;
    }

    while (string[length] != '\0')
    {
        length++;
    }

    if (length == 0u)
    {
        return;
    }

    while (offset < length)
    {
        uint8 payload_length = (uint8)(length - offset);

        if (payload_length > TOOLS_LOG_PAGE_DATA_SIZE)
        {
            payload_length = TOOLS_LOG_PAGE_DATA_SIZE;
        }

        tools_log_write_record(log_id,
                               &string[offset],
                               payload_length,
                               (boolean)((offset + payload_length) >= length));

        offset += payload_length;
    }
}

void tools_log_erase(tools_log_id_t log_id)
{
    tools_log_runtime_t* log_runtime = tools_log_runtime_table_get();
    driver_flash_eraseGroup(&log_runtime[log_id].flash_runtime);
}

void tools_log_print_all(tools_log_id_t log_id)
{
    tools_log_runtime_t* log_runtime = tools_log_runtime_table_get();
    uint8 line_buffer[TOOLS_LOG_LINE_BUFFER_SIZE];
    uint8 line_length = 0u;
    uint64 sector_group = log_runtime[log_id].flash_runtime.sector_group;

    for (uint32 sector_index = 0u; sector_group != 0u; sector_index++)
    {
        if ((sector_group & 1u) != 0u)
        {
            if (tools_log_print_sector(driver_flash_sector[sector_index], line_buffer, &line_length) == FALSE)
            {
                break;
            }
        }

        sector_group >>= 1u;
    }

    if (line_length > 0u)
    {
        line_buffer[line_length] = '\0';
        tools_println(line_buffer);
    }
}

IFX_INLINE void tools_log_write_record(tools_log_id_t log_id,
                                   const uint8* payload,
                                   uint8 payload_length,
                                   boolean is_last)
{
    tools_log_runtime_t* log_runtime = tools_log_runtime_table_get();
    uint8 page_buffer[8] = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu};

    page_buffer[0] = TOOLS_LOG_MAGIC;
    page_buffer[1] = payload_length;
    page_buffer[2] = (is_last != FALSE) ? TOOLS_LOG_FLAG_LAST : 0u;

    for (uint8 i = 0u; i < payload_length; i++)
    {
        page_buffer[3u + i] = payload[i];
    }

    uint32 word_l, word_u;
    word_l = page_buffer[0]         |
             page_buffer[1] << 8    |
             page_buffer[2] << 16   |
             page_buffer[3] << 24;

    word_u = page_buffer[4]         |
             page_buffer[5] << 8    |
             page_buffer[6] << 16   |
             page_buffer[7] << 24;

    driver_flash_autoDFlashGroupWrite(&log_runtime[log_id].flash_runtime, word_l, word_u);
}

IFX_INLINE boolean tools_log_print_sector(driver_flash_sector_t* sector, uint8* line_buffer, uint8* line_length)
{
    uint32 start_address = sector->start_address;
    uint32 end_address = sector->end_address;
    uint32 page_length = sector->page_length;
    uint8 page_buffer[8];

    while ((start_address + page_length - 1u) <= end_address)
    {
        driver_flash_read(start_address, page_buffer, 8u);

        if (page_buffer[0] != TOOLS_LOG_MAGIC)
        {
            return FALSE;
        }

        if (page_buffer[1] > TOOLS_LOG_PAGE_DATA_SIZE)
        {
            return FALSE;
        }
        
        for (uint8 i = 0u; i < page_buffer[1]; i++)
        {
            if (*line_length < (TOOLS_LOG_LINE_BUFFER_SIZE - 1u))
            {
                line_buffer[(*line_length)++] = page_buffer[3u + i];
            }
        }

        if ((page_buffer[2] & TOOLS_LOG_FLAG_LAST) != 0u)
        {
            line_buffer[*line_length] = '\0';
            tools_println(line_buffer);
            *line_length = 0u;
        }

        start_address += page_length;
    }

    return TRUE;
}



