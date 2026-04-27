/**
 ******************************************************************************
 * @file    pid.h
 * @author  (Your Name)
 * @brief   Advanced PID controller with Tustin discretization and anti-windup
 ******************************************************************************
 */

#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief PID controller handle structure
 */
typedef struct {
    /* User parameters */
    float Kp;           /**< Proportional gain */
    float Ki;           /**< Integral gain */
    float Kd;           /**< Derivative gain */
    float T;            /**< Sampling period (seconds) */
    float maxOut;       /**< Maximum output limit */
    float minOut;       /**< Minimum output limit */

    /* Precomputed coefficients (Tustin discretization) */
    float alpha;        /**< Coefficient for e[k] */
    float beta;         /**< Coefficient for e[k-1] */
    float gamma;        /**< Coefficient for e[k-2] */

    /* State variables */
    float E1;           /**< Previous error e[k-1] */
    float E2;           /**< Error before previous e[k-2] */
    float lastOutput;   /**< Previous controller output u[k-1] */

    /* Flags */
    uint8_t initialized;    /**< Initialization flag */
    bool antiWindup;        /**< Enable anti-windup (default true) */
} PID_HandleTypeDef;

/* ======================== Public API ======================== */

/**
 * @brief Initialize PID controller instance
 * @param pid   Pointer to PID handle
 * @param Kp    Proportional gain
 * @param Ki    Integral gain
 * @param Kd    Derivative gain
 * @param T     Sampling period (seconds). Must be > 0 and <= PID_MAX_T.
 * @param maxOut Maximum output limit
 * @param minOut Minimum output limit
 * @note  T is clamped to safe range internally. Anti-windup is enabled by default.
 */
void PID_Init(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd,
              float T, float maxOut, float minOut);

/**
 * @brief Update PID parameters and recompute internal coefficients
 * @param pid   Pointer to PID handle
 * @param Kp    New proportional gain
 * @param Ki    New integral gain
 * @param Kd    New derivative gain
 * @param T     New sampling period (clamped to safe range)
 */
void PID_SetParameters(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd, float T);

/**
 * @brief Set output saturation limits
 * @param pid   Pointer to PID handle
 * @param maxOut Maximum output
 * @param minOut Minimum output
 */
void PID_SetOutputLimits(PID_HandleTypeDef *pid, float maxOut, float minOut);

/**
 * @brief Enable or disable anti-windup (conditional integration)
 * @param pid     Pointer to PID handle
 * @param enable  true to enable, false to disable
 */
void PID_SetAntiWindup(PID_HandleTypeDef *pid, bool enable);

/**
 * @brief Reset controller state (errors and last output set to zero)
 * @param pid Pointer to PID handle
 */
void PID_Reset(PID_HandleTypeDef *pid);

/**
 * @brief Compute PID output
 * @param pid      Pointer to PID handle
 * @param setpoint Desired value
 * @param point    Measured value
 * @return Controller output (clamped to [minOut, maxOut])
 */
float PID_Compute(PID_HandleTypeDef *pid, float setpoint, float point);

#endif /* PID_H */