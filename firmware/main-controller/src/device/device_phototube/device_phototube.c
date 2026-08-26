#include "../../../inc/device/device_phototube/device_phototube.h"

IFX_INLINE void device_phototube_switch_component_init(device_phototube_switch_component_cfg_t *phototube_switch_component_cfg, device_phototube_switch_component_runtime_t *phototube_switch_component_runtime);
IFX_INLINE void device_phototube_switch_component_start(device_phototube_switch_component_cfg_t *phototube_switch_comonent_cfg, device_phototube_switch_component_runtime_t* phototube_switch_component_runtime);
IFX_INLINE void device_phototube_switch_component_stop(device_phototube_switch_component_runtime_t* phototube_switch_component_runtime);
IFX_INLINE void device_phototube_switch_component_set_enabled(device_phototube_switch_component_cfg_t *phototube_switch_component_cfg, device_phototube_switch_component_runtime_t *phototube_switch_component_runtime, boolean enable);
IFX_INLINE void device_phototube_vadc_module_init(void);
IFX_INLINE void device_phototube_vadc_group_init(uint32 group_index);
IFX_INLINE void device_phototube_vadc_channel_init(uint32 phototube_index);
IFX_INLINE uint32 device_phototube_group_representative_index_get(uint32 group_index);
IFX_INLINE void device_phototube_group_dma_init(uint32 group_index);
IFX_INLINE void device_phototube_group_dma_cfg_prepare(uint32 group_index);
IFX_INLINE void device_phototube_group_scan_prepare(uint32 group_index);
IFX_INLINE uint32 device_phototube_group_buffer_offset_get(uint32 group_index);
IFX_INLINE uint32 device_phototube_group_result_count_get(uint32 group_index);

void device_phototube_init(device_phototube_id_t phototube_id)
{
    device_phototube_cfg_t *phototube_cfg = device_phototube_cfg_table_get();
    device_phototube_runtime_t *phototube_runtime = device_phototube_runtime_table_get();

    device_phototube_switch_component_init(phototube_cfg[phototube_id].phototube_switch_component_cfg, phototube_runtime[phototube_id].phototube_switch_component_runtime);

    driver_vadc_init(&phototube_cfg[phototube_id].vadc_cfg, &phototube_runtime[phototube_id].vadc_runtime);
}

void device_phototube_start(device_phototube_id_t phototube_id)
{
    device_phototube_cfg_t *phototube_cfg = device_phototube_cfg_table_get();
    device_phototube_runtime_t *phototube_runtime = device_phototube_runtime_table_get();

    device_phototube_switch_component_start(phototube_cfg[phototube_id].phototube_switch_component_cfg, phototube_runtime[phototube_id].phototube_switch_component_runtime);
    device_phototube_switch_component_set_enabled(phototube_cfg[phototube_id].phototube_switch_component_cfg,
                                                  phototube_runtime[phototube_id].phototube_switch_component_runtime,
                                                  TRUE);
}

void device_phototube_init_all(void)
{
    device_phototube_cfg_t *phototube_cfg = device_phototube_cfg_table_get();
    device_phototube_runtime_t *phototube_runtime = device_phototube_runtime_table_get();

    device_phototube_switch_component_init(phototube_cfg[DEVICE_PHOTOTUBE_1].phototube_switch_component_cfg,
                                           phototube_runtime[DEVICE_PHOTOTUBE_1].phototube_switch_component_runtime);

    device_phototube_vadc_module_init();

    for (uint32 group_index = 0; group_index < DEVICE_PHOTOTUBE_VADC_GROUP_COUNT; ++group_index)
    {
        device_phototube_vadc_group_init(group_index);
    }

    for (uint32 index = 0; index < DEVICE_PHOTOTUBE_COUNT; ++index)
    {
        device_phototube_vadc_channel_init(index);
    }

    for (uint32 group_index = 0; group_index < DEVICE_PHOTOTUBE_VADC_GROUP_COUNT; ++group_index)
    {
        device_phototube_group_scan_prepare(group_index);
    }

    device_phototube_set_enabled(FALSE);
}

void device_phototube_set_enabled(boolean enable)
{
    device_phototube_cfg_t *phototube_cfg = device_phototube_cfg_table_get();
    device_phototube_runtime_t *phototube_runtime = device_phototube_runtime_table_get();

    if (enable != FALSE)
    {
        device_phototube_switch_component_start(
            phototube_cfg[DEVICE_PHOTOTUBE_1].phototube_switch_component_cfg,
            phototube_runtime[DEVICE_PHOTOTUBE_1].phototube_switch_component_runtime);
    }

    device_phototube_switch_component_set_enabled(
        phototube_cfg[DEVICE_PHOTOTUBE_1].phototube_switch_component_cfg,
        phototube_runtime[DEVICE_PHOTOTUBE_1].phototube_switch_component_runtime,
        enable);

    if (enable == FALSE)
    {
        device_phototube_switch_component_stop(
            phototube_runtime[DEVICE_PHOTOTUBE_1].phototube_switch_component_runtime);
    }
}

boolean device_phototube_is_enabled(void)
{
    device_phototube_runtime_t *phototube_runtime = device_phototube_runtime_table_get();

    return phototube_runtime[DEVICE_PHOTOTUBE_1].phototube_switch_component_runtime->enabled;
}

void device_phototube_group_register_callback(device_phototube_group_id_t group_id, void (*device_phototube_group_callback)(void))
{
    device_phototube_runtime_t *phototube_runtime = device_phototube_runtime_table_get();

    if (group_id >= DEVICE_PHOTOTUBE_GROUP_COUNT)
    {
        return;
    }

    phototube_runtime[device_phototube_group_representative_index_get(group_id)].phototube_group_runtime->device_phototube_group_callback = device_phototube_group_callback;
}

void device_phototube_group_result_buffer_update(device_phototube_group_id_t group_id)
{
    device_phototube_cfg_t *phototube_cfg = device_phototube_cfg_table_get();
    device_phototube_runtime_t *phototube_runtime = device_phototube_runtime_table_get();
    uint32 representative_index;
    uint32 buffer_offset;
    uint32 result_count;
    uint32 index;
    volatile Ifx_VADC_G *group;

    if (group_id >= DEVICE_PHOTOTUBE_GROUP_COUNT)
    {
        return;
    }

    representative_index = device_phototube_group_representative_index_get(group_id);
    buffer_offset = device_phototube_group_buffer_offset_get(group_id);
    result_count = device_phototube_group_result_count_get(group_id);
    group = phototube_runtime[representative_index].vadc_runtime.vadc_grouphn->group;

    for (index = 0u; index < result_count; ++index)
    {
        uint32 phototube_index = buffer_offset + index;
        uint32 result_index =
            (uint32)phototube_cfg[phototube_index].vadc_cfg.vadc_channelcfg->resultRegister;

        device_phototube_dma_receive_buffer[phototube_index] = (uint16)group->RES[result_index].B.RESULT;
    }
}

IFX_INLINE void device_phototube_switch_component_init(device_phototube_switch_component_cfg_t *phototube_switch_component_cfg, device_phototube_switch_component_runtime_t *phototube_switch_component_runtime)
{
    phototube_switch_component_runtime->tbu_channel = phototube_switch_component_cfg->tbu_channel;
    phototube_switch_component_runtime->switch_started = FALSE;
    phototube_switch_component_runtime->enabled = FALSE;

    driver_gtm_atom_pwm_init(&phototube_switch_component_cfg->switch_atom_pwm_cfg, &phototube_switch_component_runtime->switch_atom_pwm_runtime);
    driver_gtm_atom_pwm_init(&phototube_switch_component_cfg->sample_atom_pwm_cfg, &phototube_switch_component_runtime->sample_atom_pwm_runtime);
}

IFX_INLINE void device_phototube_switch_component_start(device_phototube_switch_component_cfg_t *phototube_switch_comonent_cfg, device_phototube_switch_component_runtime_t* phototube_switch_component_runtime)
{
    uint32 period = phototube_switch_comonent_cfg->switch_atom_pwm_cfg.atom_pwm_modulecfg.period;
    uint32 duty = 0u;
    uint32 offset = period - (period / 2 + duty / 2);
    if (phototube_switch_component_runtime->switch_started != FALSE)
    {
        return;
    }

    IfxGtm_Atom_Ch_setCounterValue(phototube_switch_component_runtime->switch_atom_pwm_runtime.atom_pwm_modulehn.atom, IfxGtm_Atom_Ch_0, 0u);
    IfxGtm_Atom_Ch_setCounterValue(phototube_switch_component_runtime->sample_atom_pwm_runtime.atom_pwm_modulehn.atom, IfxGtm_Atom_Ch_6, offset);

    IfxGtm_Atom_Agc_setTimeTrigger(phototube_switch_component_runtime->switch_atom_pwm_runtime.atom_pwm_modulehn.agc, phototube_switch_component_runtime->tbu_channel, 1000u);
    IfxGtm_Atom_Agc_setTimeTrigger(phototube_switch_component_runtime->sample_atom_pwm_runtime.atom_pwm_modulehn.agc, phototube_switch_component_runtime->tbu_channel, 1000u);

    IfxGtm_Atom_Agc_enableTimeTrigger(phototube_switch_component_runtime->switch_atom_pwm_runtime.atom_pwm_modulehn.agc, TRUE);
    IfxGtm_Atom_Agc_enableTimeTrigger(phototube_switch_component_runtime->sample_atom_pwm_runtime.atom_pwm_modulehn.agc, TRUE);
    driver_gtm_atom_pwm_start(&phototube_switch_component_runtime->switch_atom_pwm_runtime);
    driver_gtm_atom_pwm_start(&phototube_switch_component_runtime->sample_atom_pwm_runtime);
    phototube_switch_component_runtime->switch_started = TRUE;
}

IFX_INLINE void device_phototube_switch_component_stop(device_phototube_switch_component_runtime_t* phototube_switch_component_runtime)
{
    if ((phototube_switch_component_runtime == NULL_PTR)
        || (phototube_switch_component_runtime->switch_started == FALSE))
    {
        return;
    }

    driver_gtm_atom_pwm_stop(&phototube_switch_component_runtime->sample_atom_pwm_runtime, TRUE);
    driver_gtm_atom_pwm_stop(&phototube_switch_component_runtime->switch_atom_pwm_runtime, TRUE);
    phototube_switch_component_runtime->switch_started = FALSE;
}

IFX_INLINE void device_phototube_switch_component_set_enabled(device_phototube_switch_component_cfg_t *phototube_switch_component_cfg, device_phototube_switch_component_runtime_t *phototube_switch_component_runtime, boolean enable)
{
    uint32 period;
    uint32 duty;

    if ((phototube_switch_component_cfg == NULL_PTR)
        || (phototube_switch_component_runtime == NULL_PTR))
    {
        return;
    }

    period = phototube_switch_component_cfg->switch_atom_pwm_cfg.atom_pwm_modulecfg.period;
    duty = (enable != FALSE) ? 0u : ((period > 0u) ? (period - 1u) : 0u);
    driver_gtm_atom_pwm_setDutyCycle(&phototube_switch_component_runtime->switch_atom_pwm_runtime,
                                     duty);
    phototube_switch_component_runtime->enabled = (enable != FALSE) ? TRUE : FALSE;
}

IFX_INLINE void device_phototube_vadc_module_init(void)
{
    device_phototube_cfg_t *phototube_cfg = device_phototube_cfg_table_get();
    device_phototube_runtime_t *phototube_runtime = device_phototube_runtime_table_get();

    IfxVadc_Adc_initModule(phototube_runtime[DEVICE_PHOTOTUBE_1].vadc_runtime.vadc_modulehn,
                           phototube_cfg[DEVICE_PHOTOTUBE_1].vadc_cfg.vadc_modulecfg);
}

IFX_INLINE void device_phototube_vadc_group_init(uint32 group_index)
{
    device_phototube_cfg_t *phototube_cfg = device_phototube_cfg_table_get();
    device_phototube_runtime_t *phototube_runtime = device_phototube_runtime_table_get();
    uint32 representative_index = device_phototube_group_representative_index_get(group_index);

    phototube_cfg[representative_index].vadc_cfg.vadc_groupcfg->module =
        phototube_runtime[DEVICE_PHOTOTUBE_1].vadc_runtime.vadc_modulehn;
    IfxVadc_Adc_initGroup(phototube_runtime[representative_index].vadc_runtime.vadc_grouphn,
                          phototube_cfg[representative_index].vadc_cfg.vadc_groupcfg);
}

IFX_INLINE void device_phototube_vadc_channel_init(uint32 phototube_index)
{
    device_phototube_cfg_t *phototube_cfg = device_phototube_cfg_table_get();
    device_phototube_runtime_t *phototube_runtime = device_phototube_runtime_table_get();

    phototube_cfg[phototube_index].vadc_cfg.vadc_channelcfg->group =
        phototube_runtime[phototube_index].vadc_runtime.vadc_grouphn;
    IfxVadc_Adc_initChannel(phototube_runtime[phototube_index].vadc_runtime.vadc_channelhn,
                            phototube_cfg[phototube_index].vadc_cfg.vadc_channelcfg);
}

IFX_INLINE uint32 device_phototube_group_representative_index_get(uint32 group_index)
{
    return (group_index == DEVICE_PHOTOTUBE_GROUP_0) ? DEVICE_PHOTOTUBE_1 :
           ((group_index == DEVICE_PHOTOTUBE_GROUP_1) ? DEVICE_PHOTOTUBE_9 : DEVICE_PHOTOTUBE_15);
}

IFX_INLINE void device_phototube_group_dma_init(uint32 group_index)
{
    device_phototube_cfg_t *phototube_cfg = device_phototube_cfg_table_get();
    device_phototube_runtime_t *phototube_runtime = device_phototube_runtime_table_get();
    uint32 representative_index = device_phototube_group_representative_index_get(group_index);

    driver_dma_init(&phototube_cfg[representative_index].phototube_group_cfg->dma_cfg,
                    &phototube_runtime[representative_index].phototube_group_runtime->dma_runtime);
}

IFX_INLINE void device_phototube_group_dma_cfg_prepare(uint32 group_index)
{
    device_phototube_cfg_t *phototube_cfg = device_phototube_cfg_table_get();
    device_phototube_runtime_t *phototube_runtime = device_phototube_runtime_table_get();
    uint32 representative_index = device_phototube_group_representative_index_get(group_index);

    phototube_cfg[representative_index].phototube_group_cfg->dma_cfg.dma_channelcfg.sourceAddress =
        (uint32)&phototube_runtime[representative_index].vadc_runtime.vadc_grouphn->group->RES[0];
    phototube_cfg[representative_index].phototube_group_cfg->dma_cfg.dma_channelcfg.destinationAddress =
        (uint32)&device_phototube_dma_receive_buffer[representative_index];
}

IFX_INLINE void device_phototube_group_scan_prepare(uint32 group_index)
{
    device_phototube_cfg_t *phototube_cfg = device_phototube_cfg_table_get();
    device_phototube_runtime_t *phototube_runtime = device_phototube_runtime_table_get();
    uint32 representative_index = device_phototube_group_representative_index_get(group_index);

    driver_vadc_scanSetChannels(&phototube_runtime[representative_index].vadc_runtime,
                                phototube_cfg[representative_index].phototube_group_cfg->vadc_scan_channels,
                                phototube_cfg[representative_index].phototube_group_cfg->vadc_scan_channels);
}

IFX_INLINE uint32 device_phototube_group_buffer_offset_get(uint32 group_index)
{
    return (group_index == DEVICE_PHOTOTUBE_GROUP_0) ? 0u :
           ((group_index == DEVICE_PHOTOTUBE_GROUP_1) ? 8u : 14u);
}

IFX_INLINE uint32 device_phototube_group_result_count_get(uint32 group_index)
{
    return (group_index == DEVICE_PHOTOTUBE_GROUP_0) ? 8u :
           ((group_index == DEVICE_PHOTOTUBE_GROUP_1) ? 6u : 2u);
}
