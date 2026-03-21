#ifndef FOC_TRANSFORM_H
#define FOC_TRANSFORM_H

#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Clarke transform (3-phase to stationary a-ß frame)
 * @param  a      Phase A component (current or voltage)
 * @param  b      Phase B component
 * @param  c      Phase C component
 * @param  alpha  Pointer to store a-axis component
 * @param  beta   Pointer to store ß-axis component
 *
 * @note   Amplitude-invariant transformation:
 *         a = (2/3)*a - (1/3)*b - (1/3)*c
 *         ß = (1/v3) * (b - c)
 */
void FOC_Clarke(float a, float b, float c, float *alpha, float *beta);

/**
 * @brief  Park transform (stationary a-ß to rotating d-q frame)
 * @param  alpha   a-axis component
 * @param  beta    ß-axis component
 * @param  theta   Electrical angle (radians)
 * @param  d       Pointer to store d-axis component
 * @param  q       Pointer to store q-axis component
 *
 * @note   d =  cos(theta)*a + sin(theta)*ß
 *         q = -sin(theta)*a + cos(theta)*ß
 */
void FOC_Park(float alpha, float beta, float theta, float *d, float *q);

/**
 * @brief  Inverse Park transform (rotating d-q to stationary a-ß frame)
 * @param  d       d-axis component
 * @param  q       q-axis component
 * @param  theta   Electrical angle (radians)
 * @param  alpha   Pointer to store a-axis component
 * @param  beta    Pointer to store ß-axis component
 *
 * @note   a = cos(theta)*d - sin(theta)*q
 *         ß = sin(theta)*d + cos(theta)*q
 */
void FOC_InvPark(float d, float q, float theta, float *alpha, float *beta);

/**
 * @brief  Inverse Clarke transform (stationary a-ß to 3-phase)
 * @param  alpha   a-axis component
 * @param  beta    ß-axis component
 * @param  a       Pointer to store phase A component
 * @param  b       Pointer to store phase B component
 * @param  c       Pointer to store phase C component
 *
 * @note   Amplitude-invariant transformation:
 *         a = a
 *         b = -0.5*a + (v3/2)*ß
 *         c = -0.5*a - (v3/2)*ß
 */
void FOC_InvClarke(float alpha, float beta, float *a, float *b, float *c);

#ifdef __cplusplus
}
#endif

#endif /* FOC_TRANSFORM_H */