#ifndef MULTI_RAMP_H
#define MULTI_RAMP_H

#include <math.h>

typedef struct {
    float Ts;             
    float Threshold;      
    
    float AccUp_1;        
    float AccUp_2;        
    float AccDown_1;      
    float AccDown_2;      

    float Output_prev;
} MultiRamp_Gen;

void MultiRamp_Init(MultiRamp_Gen *ramp, float ts, float threshold, float up1, float up2, float down1, float down2);
float MultiRamp_Update(MultiRamp_Gen *ramp, float target);

#endif