/**
 * @file module_vehicle_esc.c
 * @brief Vehicle three-channel ESC PWM output implementation.
 */

#include "../../../../inc/app/module/module_vehicle_esc/module_vehicle_esc.h"
#include "../../../../inc/device/device_esc/device_esc.h"
#include "../../../../inc/middleware/tools/tools_print/tools_print.h"

#define MODULE_VEHICLE_ESC_PWM_CLOSED_LOOP_STATE ((uint8)3u)

static module_vehicle_esc_state_t module_vehicle_esc_state;

static boolean module_vehicle_esc_role_valid(vehicle_esc_role_t role);
static const vehicle_esc_role_cfg_t* module_vehicle_esc_role_cfg_get(vehicle_esc_role_t role);
static uint32 module_vehicle_esc_pwm_duty(sint32 duty_cycle);
static void module_vehicle_esc_status_refresh(void);
static void module_vehicle_esc_status_set_role(vehicle_esc_role_t role,
                                               uint8 state,
                                               uint8 fault,
                                               boolean fresh);

void module_vehicle_esc_init(void)
{
    uint32 index;

    for (index = 0u; index < (uint32)VEHICLE_ESC_ROLE_COUNT; index++)
    {
        module_vehicle_esc_state.duty_cycle[index] =
            module_vehicle_esc_clamp_duty((vehicle_esc_role_t)index,
                                          module_vehicle_esc_role_cfg_get((vehicle_esc_role_t)index)->limit.stop_duty);
    }

    module_vehicle_esc_state.last_sent_suction_duty = -1;
    module_vehicle_esc_state.suction_refresh_counter = 0u;
    module_vehicle_esc_state.status_request_counter = 0u;
    module_vehicle_esc_state.status_sequence = 0u;
    module_vehicle_esc_state.status_stale_counter = 0u;
    module_vehicle_esc_status_refresh();
    module_vehicle_esc_stop_all();
}

boolean module_vehicle_esc_set_duty(vehicle_esc_role_t role, sint32 duty_cycle)
{
    if (module_vehicle_esc_role_valid(role) == FALSE)
    {
        return FALSE;
    }

    module_vehicle_esc_state.duty_cycle[(uint32)role] = module_vehicle_esc_clamp_duty(role, duty_cycle);

    return TRUE;
}

void module_vehicle_esc_flush(boolean force_all)
{
    sint32 left_duty = module_vehicle_esc_state.duty_cycle[(uint32)VEHICLE_ESC_ROLE_LEFT_DRIVE];
    sint32 right_duty = module_vehicle_esc_state.duty_cycle[(uint32)VEHICLE_ESC_ROLE_RIGHT_DRIVE];
    sint32 suction_duty = module_vehicle_esc_state.duty_cycle[(uint32)VEHICLE_ESC_ROLE_SUCTION];

    (void)force_all;

    device_esc_set_duty_cycle(module_vehicle_esc_device_id_get(VEHICLE_ESC_ROLE_LEFT_DRIVE),
                              module_vehicle_esc_pwm_duty(left_duty));
    device_esc_set_duty_cycle(module_vehicle_esc_device_id_get(VEHICLE_ESC_ROLE_RIGHT_DRIVE),
                              module_vehicle_esc_pwm_duty(right_duty));
    device_esc_set_duty_cycle(module_vehicle_esc_device_id_get(VEHICLE_ESC_ROLE_SUCTION),
                              module_vehicle_esc_pwm_duty(suction_duty));
    module_vehicle_esc_state.last_sent_suction_duty = suction_duty;
    module_vehicle_esc_status_refresh();
}

boolean module_vehicle_esc_stop(vehicle_esc_role_t role)
{
    if (module_vehicle_esc_role_valid(role) == FALSE)
    {
        return FALSE;
    }

    return module_vehicle_esc_set_duty(role, module_vehicle_esc_role_cfg_get(role)->limit.stop_duty);
}

void module_vehicle_esc_stop_all(void)
{
    (void)module_vehicle_esc_stop(VEHICLE_ESC_ROLE_LEFT_DRIVE);
    (void)module_vehicle_esc_stop(VEHICLE_ESC_ROLE_RIGHT_DRIVE);
    (void)module_vehicle_esc_stop(VEHICLE_ESC_ROLE_SUCTION);
    module_vehicle_esc_flush(TRUE);
}

void module_vehicle_esc_play_music(uint8 song_id)
{
    (void)song_id;
    module_vehicle_esc_stop_all();
}

void module_vehicle_esc_stop_music(void)
{
}

sint32 module_vehicle_esc_clamp_duty(vehicle_esc_role_t role, sint32 duty_cycle)
{
    const vehicle_esc_role_cfg_t* role_cfg = module_vehicle_esc_role_cfg_get(role);

    if (duty_cycle < role_cfg->limit.min_duty)
    {
        duty_cycle = role_cfg->limit.min_duty;
    }

    if (duty_cycle > role_cfg->limit.max_duty)
    {
        duty_cycle = role_cfg->limit.max_duty;
    }

    return duty_cycle;
}

device_esc_id_t module_vehicle_esc_device_id_get(vehicle_esc_role_t role)
{
    return module_vehicle_esc_role_cfg_get(role)->device_esc_id;
}

const module_vehicle_esc_state_t* module_vehicle_esc_state_get(void)
{
    return &module_vehicle_esc_state;
}

boolean module_vehicle_esc_role_closed_loop(vehicle_esc_role_t role)
{
    if (module_vehicle_esc_role_valid(role) == FALSE)
    {
        return FALSE;
    }

    return module_vehicle_esc_state.role_status[(uint32)role].closed_loop;
}

const module_vehicle_esc_role_status_t* module_vehicle_esc_role_status_get(vehicle_esc_role_t role)
{
    if (module_vehicle_esc_role_valid(role) == FALSE)
    {
        role = VEHICLE_ESC_ROLE_LEFT_DRIVE;
    }

    return &module_vehicle_esc_state.role_status[(uint32)role];
}

void module_vehicle_esc_status_command(uint8 argc, uint8* argv[])
{
    const module_vehicle_esc_role_status_t* left_status;
    const module_vehicle_esc_role_status_t* right_status;
    const module_vehicle_esc_role_status_t* suction_status;

    (void)argc;
    (void)argv;

    module_vehicle_esc_status_refresh();
    left_status = module_vehicle_esc_role_status_get(VEHICLE_ESC_ROLE_LEFT_DRIVE);
    right_status = module_vehicle_esc_role_status_get(VEHICLE_ESC_ROLE_RIGHT_DRIVE);
    suction_status = module_vehicle_esc_role_status_get(VEHICLE_ESC_ROLE_SUCTION);

    tools_printf("{vesc}l,%u,%u,%u,%u,r,%u,%u,%u,%u,s,%u,%u,%u,%u,seq,%u,stale,%u\r\n",
                 (unsigned int)left_status->state,
                 (unsigned int)left_status->fault,
                 (unsigned int)((left_status->closed_loop != FALSE) ? 1u : 0u),
                 (unsigned int)((left_status->fresh != FALSE) ? 1u : 0u),
                 (unsigned int)right_status->state,
                 (unsigned int)right_status->fault,
                 (unsigned int)((right_status->closed_loop != FALSE) ? 1u : 0u),
                 (unsigned int)((right_status->fresh != FALSE) ? 1u : 0u),
                 (unsigned int)suction_status->state,
                 (unsigned int)suction_status->fault,
                 (unsigned int)((suction_status->closed_loop != FALSE) ? 1u : 0u),
                 (unsigned int)((suction_status->fresh != FALSE) ? 1u : 0u),
                 (unsigned int)module_vehicle_esc_state.status_sequence,
                 (unsigned int)module_vehicle_esc_state.status_stale_counter);
}

static boolean module_vehicle_esc_role_valid(vehicle_esc_role_t role)
{
    return ((uint32)role < (uint32)VEHICLE_ESC_ROLE_COUNT) ? TRUE : FALSE;
}

static const vehicle_esc_role_cfg_t* module_vehicle_esc_role_cfg_get(vehicle_esc_role_t role)
{
    const vehicle_esc_cfg_t* esc_cfg = vehicle_esc_cfg_get();

    if (module_vehicle_esc_role_valid(role) == FALSE)
    {
        role = VEHICLE_ESC_ROLE_LEFT_DRIVE;
    }

    return &esc_cfg->role_cfg[(uint32)role];
}

static uint32 module_vehicle_esc_pwm_duty(sint32 duty_cycle)
{
    if (duty_cycle <= 0)
    {
        return 0u;
    }

    return (uint32)duty_cycle;
}

static void module_vehicle_esc_status_refresh(void)
{
    module_vehicle_esc_state.status_sequence++;
    module_vehicle_esc_state.status_stale_counter = 0u;
    module_vehicle_esc_status_set_role(VEHICLE_ESC_ROLE_LEFT_DRIVE,
                                       MODULE_VEHICLE_ESC_PWM_CLOSED_LOOP_STATE,
                                       0u,
                                       TRUE);
    module_vehicle_esc_status_set_role(VEHICLE_ESC_ROLE_RIGHT_DRIVE,
                                       MODULE_VEHICLE_ESC_PWM_CLOSED_LOOP_STATE,
                                       0u,
                                       TRUE);
    module_vehicle_esc_status_set_role(VEHICLE_ESC_ROLE_SUCTION,
                                       MODULE_VEHICLE_ESC_PWM_CLOSED_LOOP_STATE,
                                       0u,
                                       TRUE);
}

static void module_vehicle_esc_status_set_role(vehicle_esc_role_t role,
                                               uint8 state,
                                               uint8 fault,
                                               boolean fresh)
{
    module_vehicle_esc_role_status_t* role_status;

    if (module_vehicle_esc_role_valid(role) == FALSE)
    {
        return;
    }

    role_status = &module_vehicle_esc_state.role_status[(uint32)role];
    role_status->state = state;
    role_status->fault = fault;
    role_status->fresh = fresh;
    role_status->closed_loop = (fresh != FALSE) ? TRUE : FALSE;
}
