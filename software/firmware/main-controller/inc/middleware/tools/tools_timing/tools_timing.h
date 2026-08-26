#ifndef MAD_CIRCUITS_TOOLS_TIMING_H
#define MAD_CIRCUITS_TOOLS_TIMING_H

#include "../../../../config/middleware/tools/tools_timing/tools_timing_cfg.h"

typedef void (*tools_timing_callback_t)(void);

void tools_timing_init(tools_timing_id_t tools_timing_id);
void tools_timing_init_all(void);
uint64 tools_timing_getTick(tools_timing_id_t tools_timing_id);
uint64 tools_timing_getFrequencyHz(tools_timing_id_t tools_timing_id);
uint64 tools_timing_ticksToMicroseconds(tools_timing_id_t tools_timing_id, uint64 ticks);
uint64 tools_timing_ticksToMilliseconds(tools_timing_id_t tools_timing_id, uint64 ticks);
uint32 tools_timing_getTicksFromMicroseconds(tools_timing_id_t tools_timing_id, uint32 microseconds);
uint32 tools_timing_getTicksFromMilliseconds(tools_timing_id_t tools_timing_id, uint32 milliseconds);
void tools_timing_pair_and_print(tools_timing_id_t tools_timing_id,
                                 const uint8* tag);
void tools_timing_benchmark_and_print(tools_timing_id_t tools_timing_id,
                                      tools_timing_callback_t tools_timing_callback,
                                      uint32 loop_count,
                                      boolean disable_interrupts,
                                      const uint8* tag);

#endif
