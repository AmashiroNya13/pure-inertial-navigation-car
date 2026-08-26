#ifndef MAD_CIRCUITS_DRIVER_ASCLIN_ASC_H
#define MAD_CIRCUITS_DRIVER_ASCLIN_ASC_H

#include "IfxAsclin_Asc.h"
#include "IfxAsclin_cfg.h"
#include "Ifx_Types.h"

typedef struct
{
    IfxAsclin_Index asclin_index;
    IfxAsclin_Asc_Config asclin_asc_modulecfg;
    IFX_CONST IfxAsclin_Asc_Pins asclin_asc_pins;
}driver_asclin_asc_cfg_t;

typedef struct
{
    IfxAsclin_Asc asclin_asc_modulehn;
}driver_asclin_asc_runtime_t;

void driver_asclin_asc_init(driver_asclin_asc_cfg_t* asclin_asc_cfg, driver_asclin_asc_runtime_t* asclin_asc_runtime);
void driver_asclin_asc_write_byte(driver_asclin_asc_runtime_t* asclin_asc_runtime, IFX_CONST uint8 data);
void driver_asclin_asc_write_string(driver_asclin_asc_runtime_t* asclin_asc_runtime, IFX_CONST uint8* str);
void driver_asclin_asc_write_buffer(driver_asclin_asc_runtime_t* asclin_asc_runtime, IFX_CONST uint8* buffer, uint32 len);
void driver_asclin_asc_read_byte(driver_asclin_asc_runtime_t* asclin_asc_runtime, uint8* data);
void driver_asclin_asc_query_byte(driver_asclin_asc_runtime_t* asclin_asc_runtime, uint8* data);

#endif
