/**
  * @file    foc_transform.c
  * @brief   Implementation of FOC transforms
  */

#include "foc_transform.h"

#define SQRT3  1.7320508075688772f
#define INV_SQRT3 0.5773502691896258f

void foc_clark(float i1, float i2, float i3, float *i_alpha, float *i_beta)
{
    // Amplitude invariant Clark transform
    *i_alpha = i1;
    *i_beta = (i1 + 2.0f * i2) * INV_SQRT3; // i3 = -(i1+i2)
}

void foc_park(float i_alpha, float i_beta, float theta_rad, float *i_d, float *i_q)
{
    float sin_th = arm_sin_f32(theta_rad);
    float cos_th = arm_cos_f32(theta_rad);
    *i_d =  cos_th * i_alpha + sin_th * i_beta;
    *i_q = -sin_th * i_alpha + cos_th * i_beta;
}

void foc_inv_park(float v_d, float v_q, float theta_rad, float *v_alpha, float *v_beta)
{
    float sin_th = arm_sin_f32(theta_rad);
    float cos_th = arm_cos_f32(theta_rad);
    *v_alpha = cos_th * v_d - sin_th * v_q;
    *v_beta  = sin_th * v_d + cos_th * v_q;
}