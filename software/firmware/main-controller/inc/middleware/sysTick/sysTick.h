#ifndef MAD_CIRCUITS_SYSTICK_H
#define MAD_CIRCUITS_SYSTICK_H

#include "../../../config/middleware/sysTick/sysTick_cfg.h"

typedef struct
{
    boolean active;
    uint64 start_tick;
    uint32 duration_tick;
} sysTick_delay_t;

void sysTick_init(sysTick_id_t sysTick_id);
void sysTick_init_all(void);
void sysTick_register_callback(sysTick_id_t sysTick_id, void (*sysTick_callback)(void));
uint64 sysTick_getTick(sysTick_id_t sysTick_id);
uint64 sysTick_getFrequencyHz(sysTick_id_t sysTick_id);
uint64 sysTick_ticksToMicroseconds(sysTick_id_t sysTick_id, uint64 ticks);
uint64 sysTick_ticksToMilliseconds(sysTick_id_t sysTick_id, uint64 ticks);
uint32 sysTick_getTicksFromMicroseconds(sysTick_id_t sysTick_id, uint32 microseconds);
uint32 sysTick_getTicksFromMilliseconds(sysTick_id_t sysTick_id, uint32 milliseconds);
void sysTick_delay_blockMicroseconds(sysTick_id_t sysTick_id, uint32 microseconds);
void sysTick_delay_blockMilliseconds(sysTick_id_t sysTick_id, uint32 milliseconds);
void sysTick_delay_nonBlockingStartMicroseconds(sysTick_id_t sysTick_id,
                                                sysTick_delay_t* delay,
                                                uint32 microseconds);
void sysTick_delay_nonBlockingStartMilliseconds(sysTick_id_t sysTick_id,
                                                sysTick_delay_t* delay,
                                                uint32 milliseconds);
boolean sysTick_delay_isElapsed(sysTick_id_t sysTick_id, sysTick_delay_t* delay);
void sysTick_delay_reset(sysTick_delay_t* delay);

#endif
