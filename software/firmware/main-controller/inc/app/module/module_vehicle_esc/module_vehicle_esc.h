/**
 * @file module_vehicle_esc.h
 * @brief Vehicle three-channel ESC output interface.
 */

#ifndef MAD_CIRCUITS_APP_MODULE_MODULE_VEHICLE_ESC_H
#define MAD_CIRCUITS_APP_MODULE_MODULE_VEHICLE_ESC_H

#include "../../../../config/app/module/module_vehicle_esc/vehicle_esc_cfg.h"

typedef struct
{
    uint8 state;
    uint8 fault;
    boolean closed_loop;
    boolean fresh;
} module_vehicle_esc_role_status_t;

typedef struct
{
    sint32 duty_cycle[VEHICLE_ESC_ROLE_COUNT];
    sint32 last_sent_suction_duty;
    uint32 suction_refresh_counter;
    uint32 status_request_counter;
    uint32 status_sequence;
    uint32 status_stale_counter;
    module_vehicle_esc_role_status_t role_status[VEHICLE_ESC_ROLE_COUNT];
} module_vehicle_esc_state_t;

void module_vehicle_esc_init(void);

boolean module_vehicle_esc_set_duty(vehicle_esc_role_t role, sint32 duty_cycle);

void module_vehicle_esc_flush(boolean force_all);

boolean module_vehicle_esc_stop(vehicle_esc_role_t role);

void module_vehicle_esc_stop_all(void);

void module_vehicle_esc_play_music(uint8 song_id);

void module_vehicle_esc_stop_music(void);

sint32 module_vehicle_esc_clamp_duty(vehicle_esc_role_t role, sint32 duty_cycle);

device_esc_id_t module_vehicle_esc_device_id_get(vehicle_esc_role_t role);

const module_vehicle_esc_state_t* module_vehicle_esc_state_get(void);

boolean module_vehicle_esc_role_closed_loop(vehicle_esc_role_t role);

const module_vehicle_esc_role_status_t* module_vehicle_esc_role_status_get(vehicle_esc_role_t role);

void module_vehicle_esc_status_command(uint8 argc, uint8* argv[]);

#endif
