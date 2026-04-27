/**
 ******************************************************************************
 * @file    pid.c
 * @author  (Your Name)
 * @brief   Implementation of advanced PID controller
 ******************************************************************************
 */

#include "pid.h"
#include <math.h>
#include <float.h>

/* ======================== Configuration Constants ======================== */

/** Maximum allowed sampling period to prevent coefficient overflow */
#define PID_MAX_T       (1.0f)      /* 1 second */
#define PID_MIN_T       (1e-6f)     /* 1 microsecond */

/* ======================== Private Helper Functions ======================== */

/**
 * @brief Validate and clamp sampling period to safe range
 */
static float PID_ClampT(float T) {
    if (T < PID_MIN_T) return PID_MIN_T;
    if (T > PID_MAX_T) return PID_MAX_T;
    return T;
}

/**
 * @brief Precompute coefficients alpha, beta, gamma based on Tustin method.
 * 
 * Continuous PID: G(s) = Kp + Ki/s + Kd*s
 * Tustin substitution: s = (2/T) * (z-1)/(z+1)
 * Resulting difference equation:
 *   u[k] = (alpha*e[k] + beta*e[k-1] + gamma*e[k-2] + 2*T*u[k-1]) / (2*T)
 */
static void PID_UpdateCoefficients(PID_HandleTypeDef *pid) {
    float T = pid->T;
    float Kp = pid->Kp;
    float Ki = pid->Ki;
    float Kd = pid->Kd;

    /* Compute pre-multiplied coefficients to speed up PID_Compute() */
    pid->alpha = 2.0f * T * Kp + Ki * T * T + 2.0f * Kd;
    pid->beta  = Ki * T * T - 4.0f * Kd - 2.0f * T * Kp;
    pid->gamma = 2.0f * Kd;
}

/* ======================== Public API Implementation ======================== */

void PID_Init(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd,
              float T, float maxOut, float minOut) {
    /* Clamp sampling time to safe range */
    T = PID_ClampT(T);

    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->T  = T;
    pid->maxOut = maxOut;
    pid->minOut = minOut;
    pid->E1 = 0.0f;
    pid->E2 = 0.0f;
    pid->lastOutput = 0.0f;
    pid->initialized = 1;
    pid->antiWindup = true;   /* Enable by default */

    PID_UpdateCoefficients(pid);
}

void PID_SetParameters(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd, float T) {
    T = PID_ClampT(T);

    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->T  = T;
    PID_UpdateCoefficients(pid);
}

void PID_SetOutputLimits(PID_HandleTypeDef *pid, float maxOut, float minOut) {
    pid->maxOut = maxOut;
    pid->minOut = minOut;
}

void PID_SetAntiWindup(PID_HandleTypeDef *pid, bool enable) {
    pid->antiWindup = enable;
}

void PID_Reset(PID_HandleTypeDef *pid) {
    pid->E1 = 0.0f;
    pid->E2 = 0.0f;
    pid->lastOutput = 0.0f;
}

float PID_Compute(PID_HandleTypeDef *pid, float setpoint, float point) {
    if (!pid->initialized) return 0.0f;
    if (pid->T <= 0.0f) return 0.0f;

    float error = setpoint - point;

    /* Compute raw output using Tustin formula */
    float output = (pid->alpha * error +
                    pid->beta  * pid->E1 +
                    pid->gamma * pid->E2 +
                    2.0f * pid->T * pid->lastOutput) / (2.0f * pid->T);

    /* Apply output saturation */
    bool saturated = false;
    if (output > pid->maxOut) {
        output = pid->maxOut;
        saturated = true;
    } else if (output < pid->minOut) {
        output = pid->minOut;
        saturated = true;
    }

    /* Anti-windup: freeze integrator state when output saturates */
    if (!(pid->antiWindup && saturated)) {
        /* Normal update: shift error history and store current output */
        pid->E2 = pid->E1;
        pid->E1 = error;
        pid->lastOutput = output;
    }
    /* else: state unchanged ? integrator frozen */

    return output;
}