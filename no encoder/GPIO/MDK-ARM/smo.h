#ifndef SMO_H
#define SMO_H

#include <stdint.h>
#include "main.h"

typedef struct {
    float Rs, Ls, psi_f, pole_pairs;   // Tham s? d?ng co
    float Ks;                          // H? s? tru?t (V)
    float fc_filter;                   // T?n s? c?t l?c thÙng th?p (Hz)
    float alpha_filter;                // H? s? l?c s?

    float Kp_pll, Ki_pll;              // Tham s? PI c?a PLL

    float ialpha_est, ibeta_est;       // DÚng u?c lu?ng aﬂ (A)
    float ealpha_est, ebeta_est;       // S?c ph?n di?n u?c lu?ng aﬂ (V)
    float theta_est;                   // GÛc di?n u?c lu?ng (rad)
    float omega_est;                   // T?c d? di?n u?c lu?ng (rad/s)
    float pll_integral;                // TÌch ph‚n c?a PLL

    float omega_max;                   // Gi?i h?n t?c d? di?n t?i da (rad/s)
    float omega_min;                   // Gi?i h?n t?c d? di?n t?i thi?u (rad/s)
} SMO_Handle;

void SMO_Init(SMO_Handle *smo, float Rs, float Ls, float psi_f, float pole_pairs,
              float speed_max_rpm, float speed_min_rpm, float dt);
void SMO_Update(SMO_Handle *smo, float ualpha, float ubeta,
                float ialpha, float ibeta, float dt);
float SMO_GetCompensatedTheta(SMO_Handle *smo);
float SMO_GetOmega(SMO_Handle *smo);
float SMO_GetSpeedRPM(SMO_Handle *smo);

#endif