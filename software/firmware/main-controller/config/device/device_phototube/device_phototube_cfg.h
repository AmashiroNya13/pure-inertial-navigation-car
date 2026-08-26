#ifndef MAD_CIRCUITS_DEVICE_PHOTOTUBE_CFG_H
#define MAD_CIRCUITS_DEVICE_PHOTOTUBE_CFG_H

#include "Ifx_Types.h"
#include "IfxGtm_Tbu.h"
#include "../../../inc/driver/driver_gtm/driver_gtm_atom_pwm.h"
#include "../../../inc/driver/driver_vadc/driver_vadc.h"
#include "../../../inc/driver/driver_dma/driver_dma.h"
#include "../../../isr/isr_config.h"

#define DEVICE_PHOTOTUBE_VADC_GROUP_COUNT 3u

typedef enum
{
    DEVICE_PHOTOTUBE_1 = 0,
    DEVICE_PHOTOTUBE_2 = 1,
    DEVICE_PHOTOTUBE_3 = 2,
    DEVICE_PHOTOTUBE_4 = 3,
    DEVICE_PHOTOTUBE_5 = 4,
    DEVICE_PHOTOTUBE_6 = 5,
    DEVICE_PHOTOTUBE_7 = 6,
    DEVICE_PHOTOTUBE_8 = 7,
    DEVICE_PHOTOTUBE_9 = 8,
    DEVICE_PHOTOTUBE_10 = 9,
    DEVICE_PHOTOTUBE_11 = 10,
    DEVICE_PHOTOTUBE_12 = 11,
    DEVICE_PHOTOTUBE_13 = 12,
    DEVICE_PHOTOTUBE_14 = 13,
    DEVICE_PHOTOTUBE_15 = 14,
    DEVICE_PHOTOTUBE_16 = 15,
    DEVICE_PHOTOTUBE_COUNT = 16,
} device_phototube_id_t;

typedef enum
{
    DEVICE_PHOTOTUBE_GROUP_0 = 0,
    DEVICE_PHOTOTUBE_GROUP_1 = 1,
    DEVICE_PHOTOTUBE_GROUP_2 = 2,
    DEVICE_PHOTOTUBE_GROUP_COUNT = DEVICE_PHOTOTUBE_VADC_GROUP_COUNT,
} device_phototube_group_id_t;

typedef struct
{
    IfxGtm_Tbu_Ts tbu_channel;
    driver_gtm_atom_pwm_cfg_t switch_atom_pwm_cfg;
    driver_gtm_atom_pwm_cfg_t sample_atom_pwm_cfg;
} device_phototube_switch_component_cfg_t;

typedef struct
{
    IfxGtm_Tbu_Ts tbu_channel;
    boolean switch_started;
    boolean enabled;
    driver_gtm_atom_pwm_runtime_t switch_atom_pwm_runtime;
    driver_gtm_atom_pwm_runtime_t sample_atom_pwm_runtime;
} device_phototube_switch_component_runtime_t;

typedef struct
{
    uint32 vadc_scan_channels;
    driver_dma_cfg_t dma_cfg;
} device_phototube_group_cfg_t;

typedef struct
{
    driver_dma_runtime_t dma_runtime;
    void (*device_phototube_group_callback)(void);
} device_phototube_group_runtime_t;

typedef struct
{
    device_phototube_id_t phototube_id;
    device_phototube_switch_component_cfg_t *phototube_switch_component_cfg;
    driver_vadc_cfg_t vadc_cfg;
    device_phototube_group_cfg_t *phototube_group_cfg;
} device_phototube_cfg_t;

typedef struct
{
    device_phototube_id_t phototube_id;
    device_phototube_switch_component_runtime_t *phototube_switch_component_runtime;
    driver_vadc_runtime_t vadc_runtime;
    device_phototube_group_runtime_t *phototube_group_runtime;
    uint16 *dma_receive_buffer;
} device_phototube_runtime_t;

extern IfxVadc_Adc_Config device_phototube_vadc_modulecfg;
extern IfxVadc_Adc_GroupConfig device_phototube_vadc_groupcfg_table[3];
extern IfxVadc_Adc_ChannelConfig device_phototube_vadc_channelcfg_table[DEVICE_PHOTOTUBE_COUNT];

extern IfxVadc_Adc device_phototube_vadc_modulehn;
extern IfxVadc_Adc_Group device_phototube_vadc_grouphn_table[3];
extern IfxVadc_Adc_Channel device_phototube_vadc_channelhn_table[DEVICE_PHOTOTUBE_COUNT];
extern uint16 device_phototube_dma_receive_buffer[DEVICE_PHOTOTUBE_COUNT];
extern device_phototube_group_cfg_t device_phototube_group_cfg_table[DEVICE_PHOTOTUBE_VADC_GROUP_COUNT];
extern device_phototube_group_runtime_t device_phototube_group_runtime_table[DEVICE_PHOTOTUBE_VADC_GROUP_COUNT];

device_phototube_cfg_t *device_phototube_cfg_table_get(void);
device_phototube_runtime_t *device_phototube_runtime_table_get(void);

#endif
