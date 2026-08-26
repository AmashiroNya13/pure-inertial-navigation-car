#include "../../../inc/driver/driver_asclin/driver_asclin_asc.h"

void driver_asclin_asc_init(driver_asclin_asc_cfg_t* asclin_asc_cfg, driver_asclin_asc_runtime_t* asclin_asc_runtime)
{
    asclin_asc_cfg->asclin_asc_modulecfg.asclin = IfxAsclin_getAddress(asclin_asc_cfg->asclin_index);
    asclin_asc_cfg->asclin_asc_modulecfg.pins = &asclin_asc_cfg->asclin_asc_pins;
    IfxAsclin_Asc_initModule(&asclin_asc_runtime->asclin_asc_modulehn, &asclin_asc_cfg->asclin_asc_modulecfg);
}

void driver_asclin_asc_write_byte(driver_asclin_asc_runtime_t* asclin_asc_runtime, IFX_CONST uint8 data)
{

























    IfxAsclin_Asc_blockingWrite(&asclin_asc_runtime->asclin_asc_modulehn, data);

}

void driver_asclin_asc_write_string(driver_asclin_asc_runtime_t* asclin_asc_runtime, IFX_CONST uint8* str)
{
    while (*str)
    {
        driver_asclin_asc_write_byte(asclin_asc_runtime, *str++);
    }
}

void driver_asclin_asc_write_buffer(driver_asclin_asc_runtime_t* asclin_asc_runtime, IFX_CONST uint8* buffer, uint32 len)
{
    while (len > 0u)
    {
        Ifx_SizeT count = (len > (uint32)IFX_SIZET_MAX) ? (Ifx_SizeT)IFX_SIZET_MAX : (Ifx_SizeT)len;

        IfxAsclin_Asc_write(&asclin_asc_runtime->asclin_asc_modulehn, buffer, &count, TIME_INFINITE);
        buffer += (uint32)count;
        len -= (uint32)count;
    }
}

void driver_asclin_asc_read_byte(driver_asclin_asc_runtime_t* asclin_asc_runtime, uint8* data)
{
    while (IfxAsclin_getRxFifoFillLevel(asclin_asc_runtime->asclin_asc_modulehn.asclin) == 0)
    {
    }

    IfxAsclin_read8(asclin_asc_runtime->asclin_asc_modulehn.asclin, data, 1);
}

void driver_asclin_asc_query_byte(driver_asclin_asc_runtime_t* asclin_asc_runtime, uint8* data)
{
    if (IfxAsclin_getRxFifoFillLevel(asclin_asc_runtime->asclin_asc_modulehn.asclin) > 0)
    {
        IfxAsclin_read8(asclin_asc_runtime->asclin_asc_modulehn.asclin, data, 1);
    }
}
