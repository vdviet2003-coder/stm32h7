#include "multi_ramp.h"

void MultiRamp_Init(MultiRamp_Gen *ramp, float ts, float threshold, float up1, float up2, float down1, float down2) {
    ramp->Ts = ts;
    ramp->Threshold = threshold;
    
    ramp->AccUp_1 = up1;
    ramp->AccUp_2 = up2;
    ramp->AccDown_1 = down1;
    ramp->AccDown_2 = down2;
    
    ramp->Output_prev = 0.0f;
}

float MultiRamp_Update(MultiRamp_Gen *ramp, float target) {
    float out = ramp->Output_prev;
    float step;

    int is_Region_1 = (fabs(out) < ramp->Threshold) ? 1 : 0;

    if (target > out) {
        float rate = is_Region_1 ? ramp->AccUp_1 : ramp->AccUp_2;
        step = rate * ramp->Ts;
        out += step;
        
        if (out > target) out = target;
    } 
    else if (target < out) {
        float rate = is_Region_1 ? ramp->AccDown_1 : ramp->AccDown_2;
        step = rate * ramp->Ts;
        out -= step;
        
        if (out < target) out = target;
    }

    ramp->Output_prev = out;
    return out;
}