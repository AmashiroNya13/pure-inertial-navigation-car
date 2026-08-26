#ifndef MAD_CIRCUITS_DEVICE_KEY_H
#define MAD_CIRCUITS_DEVICE_KEY_H

#include "../../../config/device/device_key/device_key_cfg.h"

void device_key_init(device_key_id_t key_id);
void device_key_init_all(void);
void device_key_scanner(void);
device_key_state_t device_key_getState(device_key_id_t key_id);

device_key_flag_t device_key_getFlag(device_key_id_t key_id);
void device_key_clearFlag(device_key_id_t key_id);
void device_key_clearAllFlag(void);

#endif
