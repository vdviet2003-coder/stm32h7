#include "scurve_ramp.h"

void SCurve_Init(SCurve_Profile *ramp, float ts, float accel, float decel, float tau) {
    ramp->Ts = ts;
    ramp->Accel = accel;
    ramp->Decel = decel;
    
    if (tau <= 0.0001f) {
        ramp->Tau = 0.0001f; 
    } else {
        ramp->Tau = tau;
    }

    ramp->Linear_prev = 0.0f;
    ramp->SCurve_prev = 0.0f;
}

float SCurve_Update(SCurve_Profile *ramp, float target) {
    float lin_out = ramp->Linear_prev;
    float s_out = ramp->SCurve_prev;

    if (target > lin_out) {
        lin_out += (ramp->Accel * ramp->Ts);
        if (lin_out > target) lin_out = target;
    } 
    else if (target < lin_out) {
        lin_out -= (ramp->Decel * ramp->Ts);
        if (lin_out < target) lin_out = target;
    }

    s_out = s_out + (ramp->Ts / ramp->Tau) * (lin_out - s_out);

    ramp->Linear_prev = lin_out;
    ramp->SCurve_prev = s_out;

    return s_out;
}