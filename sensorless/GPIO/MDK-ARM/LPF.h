#ifndef LPF_H
#define LPF_H

#include <stdint.h>
#include "main.h"

typedef struct
{
    float y;          // Output filtered value
    float alpha;      // Filter coefficient
    float ts;         // Sampling time, second
    float fc;         // Cut-off frequency, Hz
    uint8_t initialized;
} LPF_FirstOrder_t;


void LPF_FirstOrder_Init(LPF_FirstOrder_t *lpf, float fc, float ts);
float LPF_FirstOrder_Update(LPF_FirstOrder_t *lpf, float x);
void LPF_FirstOrder_Reset(LPF_FirstOrder_t *lpf, float value);
#endif