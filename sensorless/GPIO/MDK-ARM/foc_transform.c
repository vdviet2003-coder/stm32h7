#include "foc_transform.h"
#include <math.h>
/* Constants for Clarke and inverse Clarke transforms */
#define ONE_OVER_SQRT3   0.5773502691896258f  /* 1/v3 */
#define SQRT3_OVER_2     0.8660254037844386f  /* v3/2 */

/*----------------------------------------------------------------------------
 * Clarke transform
 *----------------------------------------------------------------------------*/
void FOC_Clarke(float a, float b, float c, float *alpha, float *beta)
{
    *alpha = (2.0f/3.0f) * a - (1.0f/3.0f) * b - (1.0f/3.0f) * c;
    *beta  = ONE_OVER_SQRT3 * (b - c);
}

/*----------------------------------------------------------------------------
 * Park transform
 *----------------------------------------------------------------------------*/
void FOC_Park(float alpha, float beta, float theta, float *d, float *q)
{
    float cos_theta = arm_cos_f32(theta);
    float sin_theta = arm_sin_f32(theta);

    *d =  cos_theta * alpha + sin_theta * beta;
    *q = -sin_theta * alpha + cos_theta * beta;
}

/*----------------------------------------------------------------------------
 * Inverse Park transform
 *----------------------------------------------------------------------------*/
void FOC_InvPark(float d, float q, float theta, float *alpha, float *beta)
{
    float cos_theta = arm_cos_f32(theta);
    float sin_theta = arm_sin_f32(theta);

    *alpha = cos_theta * d - sin_theta * q;
    *beta  = sin_theta * d + cos_theta * q;
}

/*----------------------------------------------------------------------------
 * Inverse Clarke transform
 *----------------------------------------------------------------------------*/
void FOC_InvClarke(float alpha, float beta, float *a, float *b, float *c)
{
    *a = alpha;
    *b = -0.5f * alpha + SQRT3_OVER_2 * beta;
    *c = -0.5f * alpha - SQRT3_OVER_2 * beta;
}