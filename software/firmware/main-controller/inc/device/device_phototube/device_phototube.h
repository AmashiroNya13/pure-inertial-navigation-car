#ifndef MAD_CIRCUITS_DEVICE_PHOTOTUBE_H
#define MAD_CIRCUITS_DEVICE_PHOTOTUBE_H

#include "../../../config/device/device_phototube/device_phototube_cfg.h"

void device_phototube_init(device_phototube_id_t phototube_id);
void device_phototube_start(device_phototube_id_t phototube_id);
void device_phototube_init_all(void);
void device_phototube_set_enabled(boolean enable);
boolean device_phototube_is_enabled(void);
void device_phototube_group_register_callback(device_phototube_group_id_t group_id, void (*device_phototube_group_callback)(void));
void device_phototube_group_result_buffer_update(device_phototube_group_id_t group_id);

#endif
