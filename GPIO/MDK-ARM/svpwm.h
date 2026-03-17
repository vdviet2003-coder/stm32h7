/**
  * @file    svpwm.h
  * @brief   Space Vector PWM library (based on your original algorithm)
  */

#ifndef SVPWM_H
#define SVPWM_H

#include "main.h"
#include "arm_math.h"

/* ===================== Data Structure ===================== */
typedef struct {
    /* Inputs (set by user) */
    float vd;               /* d-axis voltage command (Volts) */
    float vq;               /* q-axis voltage command (Volts) */
    float angle_deg;        /* Electrical angle (degrees) */
    float vdc;              /* DC bus voltage (Volts) */
    float tperiod;          /* PWM period (seconds) */

    /* Outputs (read-only, for debugging) */
    float valpha;           /* a-axis voltage (V) */
    float vbeta;            /* ß-axis voltage (V) */
    float vref;             /* Reference voltage magnitude (V) */
    float angle_rad;        /* Angle of voltage vector (rad) */
    uint8_t sector;         /* Current sector (1..6) */
    float t1, t2, t0;       /* Vector times (seconds) */
    float tsw1, tsw2, tsw3; /* Switching times for phases U, V, W (seconds) */
} SVPWM_HandleTypeDef;

/* ===================== Function Prototypes ===================== */

/**
 * @brief Initialize SVPWM handle
 * @param hsvpwm  Pointer to SVPWM handle
 * @param vdc     DC bus voltage (Volts)
 * @param tperiod PWM period (seconds)
 */
void SVPWM_Init(SVPWM_HandleTypeDef *hsvpwm, float vdc, float tperiod);

/**
 * @brief Set target voltages and electrical angle
 * @param hsvpwm    Pointer to SVPWM handle
 * @param vd        d-axis voltage (Volts)
 * @param vq        q-axis voltage (Volts)
 * @param angle_deg Electrical angle (degrees)
 */
void SVPWM_SetTarget(SVPWM_HandleTypeDef *hsvpwm, float vd, float vq, float angle_deg);

/**
 * @brief Perform SVPWM calculations (call this periodically)
 * @param hsvpwm Pointer to SVPWM handle
 */
void SVPWM_Update(SVPWM_HandleTypeDef *hsvpwm);

/**
 * @brief Apply computed switching times to the PWM timer (TIM1)
 * @param hsvpwm Pointer to SVPWM handle
 * @param htim   Pointer to TIM handle (assumes TIM1 with channels 1,2,3)
 */
void SVPWM_Apply(SVPWM_HandleTypeDef *hsvpwm, TIM_HandleTypeDef *htim);

#endif /* SVPWM_H */