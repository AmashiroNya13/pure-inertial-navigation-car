#include "../../../inc/driver/driver_src/driver_src.h"

void driver_src_init(driver_src_cfg_t* src_cfg, driver_src_runtime_t* src_runtime)
{
    IfxSrc_init(src_cfg->src, src_cfg->typOfService, src_cfg->priority);
    src_runtime->src = src_cfg->src;
    IfxSrc_enable(src_runtime->src);
}

void driver_src_SWTrap(driver_src_runtime_t* src_runtime)
{
    IfxSrc_setRequest(src_runtime->src);
}
