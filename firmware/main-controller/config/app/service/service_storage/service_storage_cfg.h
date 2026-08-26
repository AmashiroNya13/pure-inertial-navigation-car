#ifndef MAD_CIRCUITS_APP_SERVICE_STORAGE_CFG_H
#define MAD_CIRCUITS_APP_SERVICE_STORAGE_CFG_H

#include "Ifx_Types.h"

#include "../../../../inc/device/device_int_flash/device_int_flash.h"

#define SERVICE_STORAGE_DOUBLE_BUFFER_CAPACITY 1024u

#if (SERVICE_STORAGE_DOUBLE_BUFFER_CAPACITY % IFXFLASH_PFLASH_PAGE_LENGTH) != 0u
#error "SERVICE_STORAGE_DOUBLE_BUFFER_CAPACITY must align with IFXFLASH_PFLASH_PAGE_LENGTH"
#endif

typedef enum
{
    SERVICE_STORAGE_1 = 0,
    SERVICE_STORAGE_COUNT = 1,
} service_storage_id_t;

typedef enum
{
    SERVICE_STORAGE_BUFFER_1 = 0,     /**< Buffer channel 1, used for staged writes. */
    SERVICE_STORAGE_BUFFER_2 = 1,     /**< Buffer channel 2, used for staged writes. */
    SERVICE_STORAGE_BUFFER_COUNT = 2, /**< Total buffer channel count. */
} service_storage_buffer_id_t;

typedef enum
{
    SERVICE_STORAGE_JOB_NONE = 0,
    SERVICE_STORAGE_JOB_WRITE = 2,
    SERVICE_STORAGE_JOB_READ = 3,
} service_storage_job_type_t;

typedef enum
{
    SERVICE_STORAGE_ACCESS_MODE_NONE = 0,
    SERVICE_STORAGE_ACCESS_MODE_ASYNC = 1,
    SERVICE_STORAGE_ACCESS_MODE_SYNC = 2,
} service_storage_access_mode_t;

typedef enum
{
    SERVICE_STORAGE_BUFFER_STATE_IDLE = 0,
    SERVICE_STORAGE_BUFFER_STATE_PENDING = 1,
    SERVICE_STORAGE_BUFFER_STATE_FLUSHING = 2,
} service_storage_buffer_state_t;

typedef struct
{
    service_storage_id_t storage_id;
    device_int_flash_id_t int_flash_id;
} service_storage_cfg_t;

typedef struct
{
    service_storage_id_t storage_id;
    device_int_flash_runtime_t* int_flash_runtime;
    uint8 buffer[SERVICE_STORAGE_BUFFER_COUNT][SERVICE_STORAGE_DOUBLE_BUFFER_CAPACITY];
    uint32 buffer_length[SERVICE_STORAGE_BUFFER_COUNT];
    uint32 buffer_group_offset[SERVICE_STORAGE_BUFFER_COUNT];
    service_storage_buffer_state_t buffer_state[SERVICE_STORAGE_BUFFER_COUNT];
    service_storage_buffer_id_t current_buffer_id;
    service_storage_job_type_t current_job;
    service_storage_access_mode_t access_mode;
    boolean write_close_requested;
} service_storage_runtime_t;

service_storage_cfg_t* service_storage_cfg_table_get(void);
service_storage_runtime_t* service_storage_runtime_table_get(void);

#endif
