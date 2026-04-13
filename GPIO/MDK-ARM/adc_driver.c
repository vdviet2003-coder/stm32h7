/**
  * @file    adc_driver.c
  * @brief   Implementation of ADC driver for LA-25-NP
  */

#include "adc_driver.h"



static volatile uint32_t raw_iu = 0;
static volatile uint32_t raw_iv = 0;
static volatile float current_iu = 0.0f;
static volatile float current_iv = 0.0f;
static volatile float current_iw = 0.0f;

/* Convert raw ADC value to current (A) */
static float raw_to_current(uint32_t raw)
{
    /* raw (0..65535) -> V_ADC (0..VREF) */
    float v_adc = (float)raw * ADC_VREF / ADC_RESOLUTION;
    /* Apply LA-25-NP formula */
    float i = LA25NP_V_TO_I(v_adc);

    return i;
}

/* Initialize and start ADC conversions */
void ADC_Driver_Init(void)
{
    /* Start the trigger timer (TIM6) ? must be initialized beforehand */
    //HAL_TIM_Base_Start(&TIM_CURRENT );
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET_LINEARITY,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_CALIB_OFFSET_LINEARITY,ADC_SINGLE_ENDED);
			HAL_ADC_Start(&hadc2);
			HAL_ADC_Start(&hadc1);
    /* Start injected conversions with interrupt */
			HAL_ADCEx_InjectedStart_IT(&hadc2);
	    HAL_ADCEx_InjectedStart_IT(&hadc1);

}



/* Injected conversion complete callback ? called by HAL from ADC_IRQHandler */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        raw_iu = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        raw_iv = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
		}
}
/* Get the latest current values  */
float ADC_Driver_GetCurrents_1(void)
{		
		current_iu = raw_to_current(raw_iu);
    return  current_iu;
}
float ADC_Driver_GetCurrents_2(void)
{
    current_iv = raw_to_current(raw_iv);
    return  current_iv;
}
float ADC_Driver_GetCurrents_3(void)
{
    current_iw = -current_iu -current_iv;
    return  current_iw ;
}