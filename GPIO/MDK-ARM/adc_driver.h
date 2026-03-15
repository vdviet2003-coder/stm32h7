/**
  * @file    adc_driver.h
  * @brief   ADC driver for LA-25-NP current sensors (dual-channel, injected mode)
  */

#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#include "main.h"
#include "conf.h"
#include "tim.h"   
#include "adc.h"
/* ===================== LA-25-NP Configuration ===================== */
#define LA25NP_TURNS_RATIO      0.003f      /* 3:1000 */
#define LA25NP_RM               120.0f      /* Measuring resistor (O) */
#define LA25NP_VOFFSET          -2.88f      /* Offset voltage (V) – from design */
#define LA25NP_AMP_GAIN         0.5f        /* Amplifier gain (absolute value) */

/* ===================== ADC Configuration ===================== */
#ifndef ADC_VREF
#define ADC_VREF                3.3f        /* ADC reference voltage (V) – adjust per board */
#endif
#define ADC_RESOLUTION           65536.0f    /* 16-bit ADC */

/* ===================== Conversion Formula ===================== */
/* From V_ADC = 1.44 - 0.18 * I_P  =>  I_P = (1.44 - V_ADC) / 0.18 */
#define LA25NP_V_TO_I(v)        ((1.44f - (v)) / 0.18f)

/* Real-world calibration (if needed) */
#define LA25NP_OFFSET_CALIB      0.0f        /* A */
#define LA25NP_GAIN_CALIB        1.0f

/* ===================== API Functions ===================== */
void ADC_Driver_Init(void);
float ADC_Driver_GetCurrents_1(void);
float ADC_Driver_GetCurrents_2(void);

#endif