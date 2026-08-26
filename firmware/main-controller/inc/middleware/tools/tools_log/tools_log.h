#ifndef MAD_CIRCUITS_TOOLS_LOG_H
#define MAD_CIRCUITS_TOOLS_LOG_H

#include "../../../../config/middleware/tools/tools_log/tools_log_cfg.h"

void tools_log_init(tools_log_id_t tools_log_id);
void tools_log_init_all(void);

void tools_log(tools_log_id_t log_id, const uint8* string);
void tools_log_erase(tools_log_id_t log_id);
void tools_log_print_all(tools_log_id_t log_id);

#endif
