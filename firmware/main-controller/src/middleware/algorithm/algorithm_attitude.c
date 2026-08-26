#include "../../../inc/middleware/algorithm/algorithm_attitude.h"

#include <math.h>

#define ALGORITHM_ATTITUDE_PI (3.14159265358979323846f)

float32 algorithm_attitude_clamp(float32 value, float32 minimum, float32 maximum)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}

float32 algorithm_attitude_wrap_pi(float32 angle_rad)
{
    while (angle_rad > ALGORITHM_ATTITUDE_PI)
    {
        angle_rad -= 2.0f * ALGORITHM_ATTITUDE_PI;
    }

    while (angle_rad < -ALGORITHM_ATTITUDE_PI)
    {
        angle_rad += 2.0f * ALGORITHM_ATTITUDE_PI;
    }

    return angle_rad;
}

boolean algorithm_vector3_linear_interpolate(const algorithm_vector3_t* value_before,
                                             const algorithm_vector3_t* value_after,
                                             float32 ratio,
                                             algorithm_vector3_t* value_out)
{
    if ((value_before == NULL_PTR) || (value_after == NULL_PTR) || (value_out == NULL_PTR))
    {
        return FALSE;
    }

    value_out->x = value_before->x + ((value_after->x - value_before->x) * ratio);
    value_out->y = value_before->y + ((value_after->y - value_before->y) * ratio);
    value_out->z = value_before->z + ((value_after->z - value_before->z) * ratio);
    return TRUE;
}

boolean algorithm_vector3_quadratic_interpolate(const algorithm_vector3_t value[3],
                                                const float32 time[3],
                                                float32 target_time,
                                                algorithm_vector3_t* value_out)
{
    float32 l0_den;
    float32 l1_den;
    float32 l2_den;
    float32 l0;
    float32 l1;
    float32 l2;

    if ((value == NULL_PTR) || (time == NULL_PTR) || (value_out == NULL_PTR))
    {
        return FALSE;
    }

    l0_den = (time[0] - time[1]) * (time[0] - time[2]);
    l1_den = (time[1] - time[0]) * (time[1] - time[2]);
    l2_den = (time[2] - time[0]) * (time[2] - time[1]);

    if ((l0_den == 0.0f) || (l1_den == 0.0f) || (l2_den == 0.0f))
    {
        return FALSE;
    }

    l0 = ((target_time - time[1]) * (target_time - time[2])) / l0_den;
    l1 = ((target_time - time[0]) * (target_time - time[2])) / l1_den;
    l2 = ((target_time - time[0]) * (target_time - time[1])) / l2_den;

    value_out->x = (value[0].x * l0) + (value[1].x * l1) + (value[2].x * l2);
    value_out->y = (value[0].y * l0) + (value[1].y * l1) + (value[2].y * l2);
    value_out->z = (value[0].z * l0) + (value[1].z * l1) + (value[2].z * l2);
    return TRUE;
}

void algorithm_quaternion_integrate(algorithm_quaternion_t* q,
                                    const algorithm_vector3_t* gyro_rad_s,
                                    float32 dt_s)
{
    algorithm_quaternion_t delta;
    float32 norm;

    delta.w = -0.5f * ((q->x * gyro_rad_s->x) + (q->y * gyro_rad_s->y) + (q->z * gyro_rad_s->z));
    delta.x =  0.5f * ((q->w * gyro_rad_s->x) + (q->y * gyro_rad_s->z) - (q->z * gyro_rad_s->y));
    delta.y =  0.5f * ((q->w * gyro_rad_s->y) - (q->x * gyro_rad_s->z) + (q->z * gyro_rad_s->x));
    delta.z =  0.5f * ((q->w * gyro_rad_s->z) + (q->x * gyro_rad_s->y) - (q->y * gyro_rad_s->x));

    q->w += delta.w * dt_s;
    q->x += delta.x * dt_s;
    q->y += delta.y * dt_s;
    q->z += delta.z * dt_s;

    norm = sqrtf((q->w * q->w) + (q->x * q->x) + (q->y * q->y) + (q->z * q->z));

    if (norm <= 0.0f)
    {
        q->w = 1.0f;
        q->x = 0.0f;
        q->y = 0.0f;
        q->z = 0.0f;
        return;
    }

    q->w /= norm;
    q->x /= norm;
    q->y /= norm;
    q->z /= norm;
}

float32 algorithm_yaw_from_quaternion(const algorithm_quaternion_t* q)
{
    float32 siny_cosp = 2.0f * ((q->w * q->z) + (q->x * q->y));
    float32 cosy_cosp = 1.0f - (2.0f * ((q->y * q->y) + (q->z * q->z)));

    return algorithm_attitude_wrap_pi(atan2f(siny_cosp, cosy_cosp));
}
