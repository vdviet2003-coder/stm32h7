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
#define LA25NP_VOFFSET          -2.5f      /* Offset voltage (V) – from design */
#define LA25NP_AMP_GAIN         0.5f        /* Amplifier gain (absolute value) */
/* ===================== ADC Configuration ===================== */
#ifndef ADC_VREF
#define ADC_VREF                3.3f        /* ADC reference voltage (V) ? adjust per board */
#endif
#define ADC_RESOLUTION          4096.0f    /* 12-bit ADC */

/* ===================== Conversion Formula ===================== */
/* From V_ADC = -1/2(120*0.003*Ip + Voffset )  =>  I_P */
#define LA25NP_V_TO_I(v)        ( (2*(v)) + LA25NP_VOFFSET )/ (-LA25NP_RM * LA25NP_TURNS_RATIO)

/* ===================== Calibration ===================== */
#define ADC_CALIB_SAMPLES       100000.0f  
void ADC_Driver_CalibrateOffset(void); 
float ADC_Driver_GetOffset_IV(void);
/* ===================== API Functions ===================== */
void ADC_Driver_Init(void);
float ADC_Driver_GetCurrents_1(void);
float ADC_Driver_GetCurrents_2(void);
float ADC_Driver_GetCurrents_3(void);

#endif