#ifndef MAD_CIRCUITS_APP_MODULE_MODULE_VEHICLE_PHOTOTUBE_H
#define MAD_CIRCUITS_APP_MODULE_MODULE_VEHICLE_PHOTOTUBE_H

#include "Ifx_Types.h"

typedef struct
{
    boolean valid;
    uint32 sample_count;
    uint16 blue_value[16u];
    uint16 white_value[16u];
    uint16 min_value[16u];
    uint16 max_value[16u];
    uint16 center_value[16u];
    uint16 span_value[16u];
    sint16 offset_value[16u];
    uint32 gain_q15[16u];
} module_vehicle_phototube_calibration_t;

typedef struct
{
    boolean valid;
    float32 line_body_x_mm;
    float32 line_body_y_mm;
    float32 confidence;
    uint32 line_sum;
    uint32 active_count;
    float32 active_width_mm;
} module_vehicle_phototube_line_observation_t;

void module_vehicle_phototube_init(void);
void module_vehicle_phototube_run(void);
void module_vehicle_phototube_frame_update(void);
boolean module_vehicle_phototube_frame_ready_get(void);
void module_vehicle_phototube_frame_ready_clear(void);
void module_vehicle_phototube_calibration_command(uint8 argc, uint8* argv[]);
void module_vehicle_phototube_line_command(uint8 argc, uint8* argv[]);
void module_vehicle_phototube_turn_command(uint8 argc, uint8* argv[]);
void module_vehicle_phototube_line_stop(void);
void module_vehicle_phototube_map_command(uint8 argc, uint8* argv[]);
void module_vehicle_phototube_adc_command(uint8 argc, uint8* argv[]);
void module_vehicle_phototube_power_command(uint8 argc, uint8* argv[]);
void module_vehicle_phototube_correction_command(uint8 argc, uint8* argv[]);
boolean module_vehicle_phototube_calibration_flash_save(void);
const module_vehicle_phototube_calibration_t* module_vehicle_phototube_calibration_get(void);
boolean module_vehicle_phototube_line_observation_get(
    module_vehicle_phototube_line_observation_t* observation);
void module_vehicle_phototube_values_get(uint16 values[16u]);
void module_vehicle_phototube_calibrated_values_get(uint16 values[16u]);
void module_vehicle_phototube_normalized_values_get(uint16 values[16u]);

#endif
