/**
  * @file    foc_transform.h
  * @brief   FOC transforms (Clark, Park, Inverse Park)
  */

#ifndef FOC_TRANSFORM_H
#define FOC_TRANSFORM_H

#include <arm_math.h>

void foc_clark(float i1, float i2, float i3, float *i_alpha, float *i_beta);
void foc_park(float i_alpha, float i_beta, float theta_rad, float *i_d, float *i_q);
void foc_inv_park(float v_d, float v_q, float theta_rad, float *v_alpha, float *v_beta);

#endif