#ifndef PI_CONTROL_H
#define PI_CONTROL_H

typedef struct {
    float Kp;
    float Ki_Ts;
    float Umax;
    float Umin;
    double I_prev;
} PI_Clamping;

void PI_Init(PI_Clamping *pi, float kp, float ki, float ts, float umax, float umin);
float PI_Update(PI_Clamping *pi, float ref, float meas);

#endif