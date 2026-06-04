#include "pi_control.h"

void PI_Init(PI_Clamping *pi, float kp, float ki, float ts, float umax, float umin) {
    pi->Kp = kp;
    pi->Ki_Ts = ki * ts;
    pi->Umax = umax;
    pi->Umin = umin;
    pi->I_prev = 0.0;
}

float PI_Update(PI_Clamping *pi, float ref, float meas) {
    float e_n = ref - meas;
    float P_part = pi->Kp * e_n;
    float v = P_part + (float)pi->I_prev;
    
    float u_n = v;
    if (u_n > pi->Umax) {
        u_n = pi->Umax;
    } else if (u_n < pi->Umin) {
        u_n = pi->Umin;
    }

    int isSaturated = (v != u_n) ? 1 : 0;
    int isSameSign = ((u_n > 0.0f && e_n > 0.0f) || (u_n < 0.0f && e_n < 0.0f)) ? 1 : 0;

    if (!(isSaturated && isSameSign)) {
        pi->I_prev = pi->I_prev + (double)(pi->Ki_Ts * e_n);
    }

    return u_n;
}