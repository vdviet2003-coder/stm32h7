#include "ramp_soft_start.h"

void Ramp_Init(Ramp_Struct *r, float Ts, float slope_rads2) {
    r->Ts = Ts;
    r->Slope = slope_rads2;
    r->Target_rads = 0.0f;
    r->Current_rads = 0.0f;
    
    // Step (rad/sample) = Slope (rad/s^2) * Ts (s)
    r->Step = slope_rads2 * Ts;
}

float Ramp_Update(Ramp_Struct *r, float target_rads) {
    r->Target_rads = target_rads;
    float error = r->Target_rads - r->Current_rads;

    if (error > r->Step) {
        r->Current_rads += r->Step;
    } 
    else if (error < -r->Step) {
        r->Current_rads -= r->Step;
    } 
    else {
        r->Current_rads = r->Target_rads;
    }
    return r->Current_rads;
}