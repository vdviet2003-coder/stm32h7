#ifndef SVPWM_H
#define SVPWM_H

#include "tim.h"
#include "main.h"
#include "conf.h"
#include "arm_math.h"
/**
 * @brief Initialize the SVPWM library
 * @param htim: pointer to TIM_HandleTypeDef of TIM1 (PWM configured)
 * @param vdc: DC bus voltage (Volts)
 * @param tperiod: "Tperiod" value (same as MOTOR_PWM_FREQ in original code, interpreted as switching frequency in Hz)
 * @param period_count: max_count = htim1.Init.Period (timer auto-reload value)
 */
void SVPWM_Init(TIM_HandleTypeDef *htim, float vdc, float tperiod, uint32_t period_count);

/**
 * @brief Update duty cycles based on electrical angle (radians)
 * @param angle_radian: electrical angle (rad) – obtained from Sensor_GetElectricalAngle()
 * @note This function directly writes to TIM1 CCR1, CCR2, CCR3 registers
 */
void SVPWM_Update(float Valpha, float Vbeta, float iu, float iv, float iw);

/**
 * @brief Start PWM outputs (all channels and complementary channels)
 * @note If you already called HAL_TIM_PWM_Start in main, you don't need to call this
 */
void SVPWM_Start(void);

#endif