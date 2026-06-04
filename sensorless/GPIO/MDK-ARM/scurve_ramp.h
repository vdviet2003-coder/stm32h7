#ifndef SCURVE_RAMP_H
#define SCURVE_RAMP_H

#include <math.h>

typedef struct {
    float Ts;
    float Accel;
    float Decel;
    float Tau;
    float Linear_prev;
    float SCurve_prev;
} SCurve_Profile;

void SCurve_Init(SCurve_Profile *ramp, float ts, float accel, float decel, float tau);
float SCurve_Update(SCurve_Profile *ramp, float target);

#endif