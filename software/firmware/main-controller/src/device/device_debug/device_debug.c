#include "../../../inc/device/device_debug/device_debug.h"

void device_debug_init(device_debug_id_t debug_id)
{
    device_debug_cfg_t* debug_cfg = device_debug_cfg_table_get();
    device_debug_runtime_t* debug_runtime = device_debug_runtime_table_get();

    debug_cfg[debug_id].asclin_asc_cfg.asclin_asc_modulecfg.txBuffer = &debug_runtime[debug_id].txFifo_Buffer;
    debug_cfg[debug_id].asclin_asc_cfg.asclin_asc_modulecfg.rxBuffer = &debug_runtime[debug_id].rxFifo_Buffer;
    driver_asclin_asc_init(&debug_cfg[debug_id].asclin_asc_cfg, &debug_runtime[debug_id].asclin_asc_runtime);
}

void device_debug_init_all(void)
{
    device_debug_init(DEVICE_DEBUG_1);
}

void device_debug_register_callback(device_debug_id_t debug_id, void (*device_debug_callback)(void))
{
    device_debug_runtime_t* debug_runtime = device_debug_runtime_table_get();
    debug_runtime[debug_id].device_debug_callback = device_debug_callback;
}

void device_debug_write_byte(device_debug_id_t debug_id, IFX_CONST uint8 data)
{
    device_debug_runtime_t* debug_runtime = device_debug_runtime_table_get();
    driver_asclin_asc_write_byte(&debug_runtime[debug_id].asclin_asc_runtime, data);
}

void device_debug_write_string(device_debug_id_t debug_id, IFX_CONST uint8* string)
{
    device_debug_runtime_t* debug_runtime = device_debug_runtime_table_get();
    driver_asclin_asc_write_string(&debug_runtime[debug_id].asclin_asc_runtime, string);
}

void device_debug_write_buffer(device_debug_id_t debug_id, IFX_CONST uint8* buffer, uint32 length)
{
    device_debug_runtime_t* debug_runtime = device_debug_runtime_table_get();
    driver_asclin_asc_write_buffer(&debug_runtime[debug_id].asclin_asc_runtime, buffer, length);
}

void device_debug_read_byte(device_debug_id_t debug_id, uint8* data)
{
    device_debug_runtime_t* debug_runtime = device_debug_runtime_table_get();
    driver_asclin_asc_read_byte(&debug_runtime[debug_id].asclin_asc_runtime, data);
}

void device_debug_query_byte(device_debug_id_t debug_id, uint8* data)
{
    device_debug_runtime_t* debug_runtime = device_debug_runtime_table_get();
    driver_asclin_asc_query_byte(&debug_runtime[debug_id].asclin_asc_runtime, data);
}

