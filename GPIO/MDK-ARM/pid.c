#include "pid.h"
#include <math.h>

// Precompute coefficients based on current Kp, Ki, Kd, T
static void PID_UpdateCoefficients(PID_HandleTypeDef *pid) {
    float T = pid->T;
    float Kp = pid->Kp;
    float Ki = pid->Ki;
    float Kd = pid->Kd;

    pid->alpha = 2.0f * T * Kp + Ki * T * T + 2.0f * Kd;
    pid->beta  = Ki * T * T - 4.0f * Kd - 2.0f * T * Kp;
    pid->gamma = 2.0f * Kd;
}

void PID_Init(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd, float T, float maxOut, float minOut) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->T = T;
    pid->maxOut = maxOut;
    pid->minOut = minOut;
    pid->E1 = 0.0f;
    pid->E2 = 0.0f;
    pid->lastOutput = 0.0f;
    pid->initialized = 1;
    PID_UpdateCoefficients(pid);
}

void PID_SetParameters(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd, float T) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->T = T;
    PID_UpdateCoefficients(pid);
}

void PID_SetOutputLimits(PID_HandleTypeDef *pid, float maxOut, float minOut) {
    pid->maxOut = maxOut;
    pid->minOut = minOut;
}

void PID_Reset(PID_HandleTypeDef *pid) {
    pid->E1 = 0.0f;
    pid->E2 = 0.0f;
    pid->lastOutput = 0.0f;
}

float PID_Compute(PID_HandleTypeDef *pid, float setpoint, float point) {
    if (!pid->initialized) return 0.0f;

    float error = setpoint - point;
    float T = pid->T;
    if (T <= 0.0f) return 0.0f; // avoid division by zero

    // Compute output using the given formula
    float output = (pid->alpha * error +
                    pid->beta  * pid->E1 +
                    pid->gamma * pid->E2 +
                    2.0f * T * pid->lastOutput) / (2.0f * T);

    // Apply output limits
    if (output > pid->maxOut) output = pid->maxOut;
    if (output < pid->minOut) output = pid->minOut;

    // Update state variables
    pid->lastOutput = output;
    pid->E2 = pid->E1;
    pid->E1 = error;

    return output;
}