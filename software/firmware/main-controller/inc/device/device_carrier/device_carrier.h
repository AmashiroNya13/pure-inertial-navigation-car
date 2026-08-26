#ifndef MAD_CIRCUITS_DEVICE_CARRIER_H
#define MAD_CIRCUITS_DEVICE_CARRIER_H

#include "../../../config/device/device_carrier/device_carrier_cfg.h"

void device_carrier_init(device_carrier_id_t carrier_id);
void device_carrier_init_all(void);
void device_carrier_register_callback(device_carrier_id_t carrier_id, void (*device_carrier_callback) (void));
void device_carrier_setTask(device_carrier_id_t carrier_id, uint32 sourceAddress, uint32 destinationAddress);

#endif
