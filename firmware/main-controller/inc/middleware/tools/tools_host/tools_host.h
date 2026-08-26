#ifndef MAD_CIRCUITS_TOOLS_HOST_H
#define MAD_CIRCUITS_TOOLS_HOST_H

#include "../../../../config/middleware/tools/tools_host/tools_host_cfg.h"

typedef void (*tools_host_callback_t)(uint8 argc, uint8* argv[]);
typedef void (*tools_host_binary_callback_t)(const uint8* frame, uint16 length);

void tools_host_init(void);
void tools_host_init_all(void);
boolean tools_host_register(const uint8* command, tools_host_callback_t callback);
void tools_host_binary_register(tools_host_binary_callback_t callback);

void tools_host_rx_callback(void);
void tools_host_process(void);

#endif
