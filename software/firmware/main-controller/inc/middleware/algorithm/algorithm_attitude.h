#ifndef MAD_CIRCUITS_MIDDLEWARE_ALGORITHM_ATTITUDE_H
#define MAD_CIRCUITS_MIDDLEWARE_ALGORITHM_ATTITUDE_H

#include "Ifx_Types.h"

typedef struct
{
    float32 x;
    float32 y;
    float32 z;
} algorithm_vector3_t;

typedef struct
{
    float32 w;
    float32 x;
    float32 y;
    float32 z;
} algorithm_quaternion_t;

float32 algorithm_attitude_clamp(float32 value, float32 minimum, float32 maximum);
float32 algorithm_attitude_wrap_pi(float32 angle_rad);
boolean algorithm_vector3_linear_interpolate(const algorithm_vector3_t* value_before,
                                             const algorithm_vector3_t* value_after,
                                             float32 ratio,
                                             algorithm_vector3_t* value_out);
boolean algorithm_vector3_quadratic_interpolate(const algorithm_vector3_t value[3],
                                                const float32 time[3],
                                                float32 target_time,
                                                algorithm_vector3_t* value_out);
void algorithm_quaternion_integrate(algorithm_quaternion_t* q,
                                    const algorithm_vector3_t* gyro_rad_s,
                                    float32 dt_s);
float32 algorithm_yaw_from_quaternion(const algorithm_quaternion_t* q);

#endif
