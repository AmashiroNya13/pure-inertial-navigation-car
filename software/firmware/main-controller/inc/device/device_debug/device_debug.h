#ifndef MAD_CIRCUITS_DEVICE_DEBUG_H
#define MAD_CIRCUITS_DEVICE_DEBUG_H

#include "../../../config/device/device_debug/device_debug_cfg.h"

void device_debug_init(device_debug_id_t debug_id);
void device_debug_init_all(void);
void device_debug_register_callback(device_debug_id_t debug_id, void (*device_debug_callback)(void));
void device_debug_write_byte(device_debug_id_t debug_id, IFX_CONST uint8 data);
void device_debug_write_string(device_debug_id_t debug_id, IFX_CONST uint8* string);
void device_debug_write_buffer(device_debug_id_t debug_id, IFX_CONST uint8* buffer, uint32 len);
void device_debug_read_byte(device_debug_id_t debug_id, uint8* data);
void device_debug_query_byte(device_debug_id_t debug_id, uint8* data);

#endif
