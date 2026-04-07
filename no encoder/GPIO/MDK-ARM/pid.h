#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float Kp;           // Proportional gain
    float Ki;           // Integral gain
    float Kd;           // Derivative gain
    float T;            // Sampling time (seconds)
    float maxOut;       // Maximum output limit
    float minOut;       // Minimum output limit
    float E1;           // Previous error (e(k-1))
    float E2;           // Error before previous (e(k-2))
    float lastOutput;   // Previous output (u(k-1))
    float alpha;        // Precomputed coefficient alpha
    float beta;         // Precomputed coefficient beta
    float gamma;        // Precomputed coefficient gamma
    uint8_t initialized;
} PID_HandleTypeDef;

// Initialize PID controller with given parameters
void PID_Init(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd, float T, float maxOut, float minOut);

// Update PID parameters (recalculates alpha, beta, gamma)
void PID_SetParameters(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd, float T);

// Set output limits
void PID_SetOutputLimits(PID_HandleTypeDef *pid, float maxOut, float minOut);

// Compute PID output based on setpoint and current measurement
float PID_Compute(PID_HandleTypeDef *pid, float setpoint, float point);

// Reset PID state (clear error history and last output)
void PID_Reset(PID_HandleTypeDef *pid);

#endif