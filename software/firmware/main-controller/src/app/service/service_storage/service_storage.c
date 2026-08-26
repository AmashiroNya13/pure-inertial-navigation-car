/**
 * @file service_storage.c
 * @brief Upper storage service wrapper implementation for internal flash access.
 */

#include "../../../../inc/app/service/service_storage/service_storage.h"

IFX_EXTERN service_storage_cfg_t service_storage_cfg_table[SERVICE_STORAGE_COUNT];
IFX_EXTERN service_storage_runtime_t service_storage_runtime_table[SERVICE_STORAGE_COUNT];

static service_storage_cfg_t* storage_cfg = service_storage_cfg_table;
static service_storage_runtime_t* storage_runtime = service_storage_runtime_table;


IFX_INLINE service_storage_buffer_id_t service_storage_getNextBufferId(service_storage_buffer_id_t buffer_id);
IFX_INLINE boolean service_storage_lockAccessMode(service_storage_runtime_t* runtime,
                                                  service_storage_access_mode_t access_mode);
IFX_INLINE boolean service_storage_isRangeValid(service_storage_runtime_t* runtime,
                                                uint32 group_offset,
                                                uint32 length);
IFX_INLINE boolean service_storage_isPageAligned(uint32 value);
IFX_INLINE void service_storage_page_packData(const uint8* data,
                                              uint32 data_offset,
                                              uint32 (*page_word_l)[4],
                                              uint32 (*page_word_u)[4]);
IFX_INLINE void service_storage_page_pack(service_storage_id_t storage_id,
                                        service_storage_buffer_id_t buffer_id,
                                        uint32 buffer_offset,
                                        uint32 (*page_word_l)[4],
                                       uint32 (*page_word_u)[4]);
IFX_INLINE void service_storage_pushLastWrite(service_storage_id_t storage_id);
IFX_INLINE void service_storage_pullFirstRead(service_storage_id_t storage_id);

/**
 * @brief Initialize storage service runtime buffers and job state.
 * @param[in] void No parameter.
 * @return void
 */
void service_storage_init(service_storage_id_t storage_id)
{
    device_int_flash_runtime_t* int_flash_runtime = device_int_flash_runtime_table_get();

    storage_runtime[storage_id].storage_id = storage_id;
    storage_runtime[storage_id].int_flash_runtime = &int_flash_runtime[storage_cfg[storage_id].int_flash_id];
    storage_runtime[storage_id].buffer_length[SERVICE_STORAGE_BUFFER_1] = 0u;
    storage_runtime[storage_id].buffer_length[SERVICE_STORAGE_BUFFER_2] = 0u;
    storage_runtime[storage_id].buffer_group_offset[SERVICE_STORAGE_BUFFER_1] = 0u;
    storage_runtime[storage_id].buffer_group_offset[SERVICE_STORAGE_BUFFER_2] = 0u;
    storage_runtime[storage_id].buffer_state[SERVICE_STORAGE_BUFFER_1] = SERVICE_STORAGE_BUFFER_STATE_PENDING;
    storage_runtime[storage_id].buffer_state[SERVICE_STORAGE_BUFFER_2] = SERVICE_STORAGE_BUFFER_STATE_PENDING;
    storage_runtime[storage_id].current_buffer_id = SERVICE_STORAGE_BUFFER_1;
    storage_runtime[storage_id].current_job = SERVICE_STORAGE_JOB_NONE;
    storage_runtime[storage_id].access_mode = SERVICE_STORAGE_ACCESS_MODE_NONE;
    storage_runtime[storage_id].write_close_requested = FALSE;
}

void service_storage_init_all(void)
{
    service_storage_init(SERVICE_STORAGE_1);
}

void service_storage_setJobTypeAsync(service_storage_id_t storage_id, service_storage_job_type_t job_type)
{
    service_storage_job_type_t current_job;

    if (service_storage_lockAccessMode(&storage_runtime[storage_id], SERVICE_STORAGE_ACCESS_MODE_ASYNC) == FALSE)
    {
        return;
    }

    current_job = storage_runtime[storage_id].current_job;
    if (current_job == SERVICE_STORAGE_JOB_NONE)
    {
        if (job_type == SERVICE_STORAGE_JOB_WRITE)
        {
            storage_runtime[storage_id].current_job = SERVICE_STORAGE_JOB_WRITE;
            storage_runtime[storage_id].buffer_state[SERVICE_STORAGE_BUFFER_1] = SERVICE_STORAGE_BUFFER_STATE_PENDING;
            storage_runtime[storage_id].buffer_state[SERVICE_STORAGE_BUFFER_2] = SERVICE_STORAGE_BUFFER_STATE_IDLE;
            storage_runtime[storage_id].current_buffer_id = SERVICE_STORAGE_BUFFER_1;
            storage_runtime[storage_id].write_close_requested = FALSE;
        }
        else if (job_type == SERVICE_STORAGE_JOB_READ)
        {
            service_storage_pullFirstRead(storage_id);
            storage_runtime[storage_id].current_job = SERVICE_STORAGE_JOB_READ;
        }
    }
    else if (current_job == SERVICE_STORAGE_JOB_WRITE)
    {
        if (job_type == SERVICE_STORAGE_JOB_NONE)
        {
            service_storage_pushLastWrite(storage_id);
        }
    }
    else if (current_job == SERVICE_STORAGE_JOB_READ)
    {
        if (job_type == SERVICE_STORAGE_JOB_NONE)
        {
            storage_runtime[storage_id].current_job = SERVICE_STORAGE_JOB_NONE;
        }
    }
}

uint32 service_storage_writeSync(service_storage_id_t storage_id,
                                 uint32 group_offset,
                                 const uint8* data,
                                 uint32 length)
{
    service_storage_runtime_t* runtime = &storage_runtime[storage_id];
    uint32 page_word_l[4];
    uint32 page_word_u[4];
    uint32 start_address;
    uint32 data_offset;

    if (service_storage_lockAccessMode(runtime, SERVICE_STORAGE_ACCESS_MODE_SYNC) == FALSE)
    {
        return 0u;
    }

    if ((data == NULL_PTR) || (length == 0u))
    {
        return 0u;
    }

    if ((service_storage_isPageAligned(group_offset) == FALSE)
        || (service_storage_isPageAligned(length) == FALSE)
        || (service_storage_isRangeValid(runtime, group_offset, length) == FALSE))
    {
        return 0u;
    }

    start_address = device_int_flash_start_address_get(runtime->int_flash_runtime->int_flash_id) + group_offset;

    for (data_offset = 0u; data_offset < length; data_offset += IFXFLASH_PFLASH_PAGE_LENGTH)
    {
        service_storage_page_packData(data, data_offset, &page_word_l, &page_word_u);

        if (device_int_flash_writePFlashPageData(runtime->int_flash_runtime->int_flash_id,
                                                 start_address + data_offset,
                                                 &page_word_l,
                                                 &page_word_u) == FALSE)
        {
            return data_offset;
        }
    }

    return length;
}

uint32 service_storage_readSync(service_storage_id_t storage_id,
                                uint32 group_offset,
                                uint8* data,
                                uint32 length)
{
    service_storage_runtime_t* runtime = &storage_runtime[storage_id];

    if (service_storage_lockAccessMode(runtime, SERVICE_STORAGE_ACCESS_MODE_SYNC) == FALSE)
    {
        return 0u;
    }

    if ((data == NULL_PTR) || (length == 0u))
    {
        return 0u;
    }

    if (service_storage_isRangeValid(runtime, group_offset, length) == FALSE)
    {
        return 0u;
    }

    device_int_flash_read(runtime->int_flash_runtime->int_flash_id,
                          device_int_flash_start_address_get(runtime->int_flash_runtime->int_flash_id) + group_offset,
                          data,
                          length);

    return length;
}

boolean service_storage_erase(service_storage_id_t storage_id)
{
    service_storage_runtime_t* runtime = &storage_runtime[storage_id];

    device_int_flash_eraseGroup(runtime->int_flash_runtime->int_flash_id);
    device_int_flash_init(runtime->int_flash_runtime->int_flash_id);
    service_storage_init(storage_id);

    return TRUE;
}
/**
 * @brief Execute one background storage service step.
 * @param[in] void No parameter.
 * @return void
 */
void service_storage_runAsync(service_storage_id_t storage_id)
{
    service_storage_runtime_t* runtime = &storage_runtime[storage_id];
    service_storage_buffer_id_t background_buffer_id;
    uint32 page_word_l[4];
    uint32 page_word_u[4];
    uint32 buffer_offset;
    uint32 read_length;

    if (storage_runtime[storage_id].current_job == SERVICE_STORAGE_JOB_WRITE)
    {
        background_buffer_id = service_storage_getNextBufferId(runtime->current_buffer_id);

        if ((runtime->buffer_state[background_buffer_id] != SERVICE_STORAGE_BUFFER_STATE_FLUSHING)
            && (runtime->write_close_requested == TRUE)
            && (runtime->buffer_length[runtime->current_buffer_id] > 0u)
            && (runtime->buffer_state[runtime->current_buffer_id] == SERVICE_STORAGE_BUFFER_STATE_PENDING))
        {
            runtime->buffer_state[runtime->current_buffer_id] = SERVICE_STORAGE_BUFFER_STATE_FLUSHING;
            runtime->buffer_state[background_buffer_id] = SERVICE_STORAGE_BUFFER_STATE_IDLE;
            runtime->buffer_length[background_buffer_id] = 0u;
            runtime->current_buffer_id = background_buffer_id;
            background_buffer_id = service_storage_getNextBufferId(runtime->current_buffer_id);
        }

        if (runtime->buffer_state[background_buffer_id] != SERVICE_STORAGE_BUFFER_STATE_FLUSHING)
        {
            if ((runtime->write_close_requested == TRUE)
                && (runtime->buffer_length[runtime->current_buffer_id] == 0u)
                && (runtime->buffer_state[runtime->current_buffer_id] != SERVICE_STORAGE_BUFFER_STATE_FLUSHING))
            {
                runtime->current_job = SERVICE_STORAGE_JOB_NONE;
                runtime->write_close_requested = FALSE;
            }

            return;
        }

        if (background_buffer_id != SERVICE_STORAGE_BUFFER_COUNT)
        {
            for (buffer_offset = 0u; buffer_offset < runtime->buffer_length[background_buffer_id]; buffer_offset += IFXFLASH_PFLASH_PAGE_LENGTH)
            {
                service_storage_page_pack(storage_id,
                                          background_buffer_id,
                                          buffer_offset,
                                          &page_word_l,
                                          &page_word_u);
                device_int_flash_autoPFlashWrite(runtime->int_flash_runtime->int_flash_id, &page_word_l, &page_word_u);
            }

            runtime->buffer_length[background_buffer_id] = 0u;
            runtime->buffer_state[background_buffer_id] = SERVICE_STORAGE_BUFFER_STATE_IDLE;

            if ((runtime->write_close_requested == TRUE)
                && (runtime->buffer_length[runtime->current_buffer_id] == 0u)
                && (runtime->buffer_state[runtime->current_buffer_id] != SERVICE_STORAGE_BUFFER_STATE_FLUSHING))
            {
                runtime->current_job = SERVICE_STORAGE_JOB_NONE;
                runtime->write_close_requested = FALSE;
            }
        }
    }

    if (storage_runtime[storage_id].current_job == SERVICE_STORAGE_JOB_READ)
    {
        background_buffer_id = service_storage_getNextBufferId(runtime->current_buffer_id); 

        if (runtime->buffer_state[background_buffer_id] != SERVICE_STORAGE_BUFFER_STATE_IDLE)
        {
            return ;
        }

        if (background_buffer_id != SERVICE_STORAGE_BUFFER_COUNT)
        {
            if (runtime->buffer_group_offset[background_buffer_id] < runtime->int_flash_runtime->flash_runtime.group_capacity)
            {
                read_length = runtime->int_flash_runtime->flash_runtime.group_capacity - runtime->buffer_group_offset[background_buffer_id];

                if (read_length > SERVICE_STORAGE_DOUBLE_BUFFER_CAPACITY)
                {
                    read_length = SERVICE_STORAGE_DOUBLE_BUFFER_CAPACITY;
                }

                device_int_flash_read(runtime->int_flash_runtime->int_flash_id,
                                      device_int_flash_start_address_get(runtime->int_flash_runtime->int_flash_id)
                                          + runtime->buffer_group_offset[background_buffer_id],
                                      runtime->buffer[background_buffer_id],
                                      read_length);
                runtime->buffer_length[background_buffer_id] = read_length;
                runtime->buffer_state[background_buffer_id] = SERVICE_STORAGE_BUFFER_STATE_FLUSHING;
            }
        }
    }
}

/**
 * @brief Write data into storage service staging buffer.
 * @param[in] buffer_id Staging buffer id.
 * @param[in] buffer_offset Buffer offset, unit: byte.
 * @param[in] data Source buffer pointer.
 * @param[in] length Write length, unit: byte.
 * @return TRUE if buffer write succeeds, otherwise FALSE.
 */
uint32 service_storage_writeAsync(service_storage_id_t storage_id, const uint8* data, uint32 length)
{
    service_storage_runtime_t* runtime = &storage_runtime[storage_id];
    service_storage_buffer_id_t current_buffer_id;
    service_storage_buffer_id_t next_buffer_id;
    uint32 buffer_length;
    uint32 data_offset;

    if ((runtime->current_job != SERVICE_STORAGE_JOB_WRITE) || (data == NULL_PTR) || (length == 0u))
    {
        return 0u;
    }

    if (runtime->write_close_requested == TRUE)
    {
        return 0u;
    }

    current_buffer_id = runtime->current_buffer_id;

    if (runtime->buffer_state[current_buffer_id] == SERVICE_STORAGE_BUFFER_STATE_IDLE)
    {
        runtime->buffer_length[current_buffer_id] = 0u;
        runtime->buffer_state[current_buffer_id] = SERVICE_STORAGE_BUFFER_STATE_PENDING;
    }

    if (runtime->buffer_state[runtime->current_buffer_id] != SERVICE_STORAGE_BUFFER_STATE_PENDING)
    {
        return 0u;
    }

    for (data_offset = 0u; data_offset < length; data_offset++)
    {
        buffer_length = runtime->buffer_length[current_buffer_id];

        if (buffer_length >= SERVICE_STORAGE_DOUBLE_BUFFER_CAPACITY)
        {
            next_buffer_id = service_storage_getNextBufferId(current_buffer_id);

            if (runtime->buffer_state[next_buffer_id] != SERVICE_STORAGE_BUFFER_STATE_IDLE)
            {
                return data_offset;
            }

            runtime->buffer_state[current_buffer_id] = SERVICE_STORAGE_BUFFER_STATE_FLUSHING;
            runtime->buffer_state[next_buffer_id] = SERVICE_STORAGE_BUFFER_STATE_PENDING;
            runtime->buffer_length[next_buffer_id] = 0u;
            runtime->current_buffer_id = next_buffer_id;
            current_buffer_id = next_buffer_id;
            buffer_length = 0u;
        }

        runtime->buffer[current_buffer_id][buffer_length] = data[data_offset];
        runtime->buffer_length[current_buffer_id] = buffer_length + 1u;
    }

    return length;
}

/**
 * @brief Read data from storage service staging buffer.
 * @param[in] buffer_id Staging buffer id.
 * @param[in] buffer_offset Buffer offset, unit: byte.
 * @param[out] data Destination buffer pointer.
 * @param[in] length Requested read length, unit: byte.
 * @return Actual read length, unit: byte.
 */
uint32 service_storage_readAsync(service_storage_id_t storage_id,
                                 uint32 group_offset,
                                 uint8* data,
                                 uint32 length)
{
    service_storage_runtime_t* runtime = &storage_runtime[storage_id];
    uint32 group_capacity = runtime->int_flash_runtime->flash_runtime.group_capacity;
    service_storage_buffer_id_t current_buffer_id;
    service_storage_buffer_id_t next_buffer_id;
    uint32 buffer_length;
    uint32 buffer_offset;
    uint32 data_offset;
    int diff;

    if ((runtime->current_job != SERVICE_STORAGE_JOB_READ) || (data == NULL_PTR) || (length == 0u))
    {
        return 0u;
    }

    if ((group_offset >= group_capacity) || ((group_offset + length) > group_capacity))
    {
        return 0u;
    }

    if (runtime->buffer_state[runtime->current_buffer_id] == SERVICE_STORAGE_BUFFER_STATE_FLUSHING)
    {
        runtime->buffer_state[runtime->current_buffer_id] = SERVICE_STORAGE_BUFFER_STATE_PENDING;
    }

    if (runtime->buffer_state[runtime->current_buffer_id] != SERVICE_STORAGE_BUFFER_STATE_PENDING)
    {
        return 0u;
    }

    for (data_offset = 0u; data_offset < length; data_offset++)
    {
        current_buffer_id = runtime->current_buffer_id;

        diff = group_offset - runtime->buffer_group_offset[current_buffer_id];

        if (diff < 0)
        {
            return data_offset;
        }

        if (diff >= SERVICE_STORAGE_DOUBLE_BUFFER_CAPACITY)
        {
            next_buffer_id = service_storage_getNextBufferId(current_buffer_id);

            if (runtime->buffer_state[next_buffer_id] != SERVICE_STORAGE_BUFFER_STATE_FLUSHING)
            {
                return data_offset;
            }

            runtime->buffer_state[current_buffer_id] = SERVICE_STORAGE_BUFFER_STATE_IDLE;
            runtime->buffer_group_offset[current_buffer_id] += 2 * SERVICE_STORAGE_DOUBLE_BUFFER_CAPACITY;
            runtime->buffer_state[next_buffer_id] = SERVICE_STORAGE_BUFFER_STATE_PENDING;
            runtime->buffer_group_offset[next_buffer_id] = (group_offset / SERVICE_STORAGE_DOUBLE_BUFFER_CAPACITY) * SERVICE_STORAGE_DOUBLE_BUFFER_CAPACITY;
            buffer_length = group_capacity - runtime->buffer_group_offset[next_buffer_id];

            if (buffer_length > SERVICE_STORAGE_DOUBLE_BUFFER_CAPACITY)
            {
                buffer_length = SERVICE_STORAGE_DOUBLE_BUFFER_CAPACITY;
            }

            runtime->buffer_length[next_buffer_id] = buffer_length;
            runtime->current_buffer_id = next_buffer_id;
            current_buffer_id = next_buffer_id;
        }

        buffer_offset = group_offset - runtime->buffer_group_offset[current_buffer_id];

        if (buffer_offset >= runtime->buffer_length[current_buffer_id])
        {
            return data_offset;
        }

        data[data_offset] = runtime->buffer[current_buffer_id][buffer_offset];
        group_offset++;
    }

    return length;
}

IFX_INLINE boolean service_storage_lockAccessMode(service_storage_runtime_t* runtime,
                                                  service_storage_access_mode_t access_mode)
{
    if (runtime->access_mode == SERVICE_STORAGE_ACCESS_MODE_NONE)
    {
        runtime->access_mode = access_mode;
        return TRUE;
    }

    return (boolean)(runtime->access_mode == access_mode);
}

IFX_INLINE boolean service_storage_isRangeValid(service_storage_runtime_t* runtime,
                                                uint32 group_offset,
                                                uint32 length)
{
    uint32 group_capacity = runtime->int_flash_runtime->flash_runtime.group_capacity;

    if (length == 0u)
    {
        return FALSE;
    }

    if (group_offset >= group_capacity)
    {
        return FALSE;
    }

    if ((group_offset + length) < group_offset)
    {
        return FALSE;
    }

    return (boolean)((group_offset + length) <= group_capacity);
}

IFX_INLINE boolean service_storage_isPageAligned(uint32 value)
{
    return (boolean)((value % IFXFLASH_PFLASH_PAGE_LENGTH) == 0u);
}

IFX_INLINE void service_storage_page_packData(const uint8* data,
                                              uint32 data_offset,
                                              uint32 (*page_word_l)[4],
                                              uint32 (*page_word_u)[4])
{
    for (uint32 i = 0u; i < 4u; i++)
    {
        (*page_word_l)[i] = ((uint32)data[data_offset + (i * 8u)])
                          | (((uint32)data[data_offset + (i * 8u) + 1u]) << 8u)
                          | (((uint32)data[data_offset + (i * 8u) + 2u]) << 16u)
                          | (((uint32)data[data_offset + (i * 8u) + 3u]) << 24u);

        (*page_word_u)[i] = ((uint32)data[data_offset + (i * 8u) + 4u])
                          | (((uint32)data[data_offset + (i * 8u) + 5u]) << 8u)
                          | (((uint32)data[data_offset + (i * 8u) + 6u]) << 16u)
                          | (((uint32)data[data_offset + (i * 8u) + 7u]) << 24u);
    }
}

/**
 * @brief Pack one staged PFlash page from buffer data.
 * @param[in] buffer_id Staging buffer id.
 * @param[in] buffer_offset Buffer offset, unit: byte.
 * @param[in] length Valid byte length inside this page.
 * @param[out] page_word_l Lower word array for PFlash programming.
 * @param[out] page_word_u Upper word array for PFlash programming.
 * @return void
 */
IFX_INLINE void service_storage_page_pack(service_storage_id_t storage_id,
                                      service_storage_buffer_id_t buffer_id,
                                      uint32 buffer_offset,
                                      uint32 (*page_word_l)[4],
                                      uint32 (*page_word_u)[4])
{
    uint8 page_buffer[IFXFLASH_PFLASH_PAGE_LENGTH];

    for (uint32 i = 0u; i < IFXFLASH_PFLASH_PAGE_LENGTH; i++)
    {
        if ((buffer_offset + i) < storage_runtime[storage_id].buffer_length[buffer_id])
        {
            page_buffer[i] = storage_runtime[storage_id].buffer[buffer_id][buffer_offset + i];
        }
        else
        {
            page_buffer[i] = 0xFFu;
        }
    }

    for (uint32 i = 0u; i < 4u; i++)
    {
        (*page_word_l)[i] = ((uint32)page_buffer[i * 8u])
                          | (((uint32)page_buffer[(i * 8u) + 1u]) << 8u)
                          | (((uint32)page_buffer[(i * 8u) + 2u]) << 16u)
                          | (((uint32)page_buffer[(i * 8u) + 3u]) << 24u);

        (*page_word_u)[i] = ((uint32)page_buffer[(i * 8u) + 4u])
                          | (((uint32)page_buffer[(i * 8u) + 5u]) << 8u)
                          | (((uint32)page_buffer[(i * 8u) + 6u]) << 16u)
                          | (((uint32)page_buffer[(i * 8u) + 7u]) << 24u);
    }
}

IFX_INLINE service_storage_buffer_id_t service_storage_getNextBufferId(service_storage_buffer_id_t buffer_id)
{
    return (service_storage_buffer_id_t)((buffer_id + 1u) % SERVICE_STORAGE_BUFFER_COUNT);
}

IFX_INLINE void service_storage_pushLastWrite(service_storage_id_t storage_id)
{
    storage_runtime[storage_id].write_close_requested = TRUE;
}

IFX_INLINE void service_storage_pullFirstRead(service_storage_id_t storage_id)
{
    service_storage_runtime_t* runtime = &storage_runtime[storage_id];
    uint32 group_capacity = runtime->int_flash_runtime->flash_runtime.group_capacity;
    uint32 read_length;
    service_storage_buffer_id_t current_buffer_id = SERVICE_STORAGE_BUFFER_1;
    service_storage_buffer_id_t next_buffer_id = service_storage_getNextBufferId(current_buffer_id);

    runtime->current_buffer_id = current_buffer_id;
    runtime->buffer_group_offset[current_buffer_id] = 0u;
    runtime->buffer_group_offset[next_buffer_id] = SERVICE_STORAGE_DOUBLE_BUFFER_CAPACITY;
    runtime->buffer_length[current_buffer_id] = 0u;
    runtime->buffer_length[next_buffer_id] = 0u;
    runtime->buffer_state[current_buffer_id] = SERVICE_STORAGE_BUFFER_STATE_PENDING;
    runtime->buffer_state[next_buffer_id] = SERVICE_STORAGE_BUFFER_STATE_IDLE;

    if (group_capacity == 0u)
    {
        return;
    }

    read_length = group_capacity;

    if (read_length > SERVICE_STORAGE_DOUBLE_BUFFER_CAPACITY)
    {
        read_length = SERVICE_STORAGE_DOUBLE_BUFFER_CAPACITY;
    }

    device_int_flash_read(runtime->int_flash_runtime->int_flash_id,
                          device_int_flash_start_address_get(runtime->int_flash_runtime->int_flash_id),
                          runtime->buffer[current_buffer_id],
                          read_length);
    runtime->buffer_length[current_buffer_id] = read_length;
}
