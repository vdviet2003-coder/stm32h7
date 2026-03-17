/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "sensor.h"
#include "adc_driver.h"
//#include "svpwm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
		/* Sensor variables */
float elec_angle_rad;          /* Electrical angle (rad) */
float mech_angle_rad;          /* Mechanical angle (rad) */
float velocity_rads;           /* Angular velocity (rad/s) */
uint8_t hall_step;             /* Current Hall step (1..6) */
uint16_t z_counter;             /* Z pulse counter */

/* Current measurement variables */
float i1, i2;                   /* Phase A, B currents (A) */
float i_alpha, i_beta;          /* Stationary frame currents */
float i_d, i_q;                  /* Rotating frame currents */

/* Voltage commands (from PI controllers) */
float agl ; // // degree
float agl_radian ; // rad
float Tperiod = MOTOR_PWM_FREQ ;
float Vd = 0.0f;
float Vq = 5.0f;
float Vdc = 35.0f;
float Valpha , Vbeta , Vref ;
float Ta = 0.0f, Tb = 0.0f, T0 = 0.0f, T1 = 0.0f, T2 = 0.0f;
float u = 0.0f, v = 0.0f, w = 0.0f, g = 0.0f;
float Tsw1 = 0.0f, Tsw2 = 0.0f, Tsw3 = 0.0f;
uint8_t s;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SVPWM(float Angle_radian)
{
    //Angle_radian = Angle * (Pi / 180.0f);
    Valpha = arm_cos_f32(Angle_radian) * Vd - arm_sin_f32(Angle_radian) * Vq;
    Vbeta  = arm_sin_f32(Angle_radian) * Vd + arm_cos_f32(Angle_radian) * Vq;
    arm_sqrt_f32(((Valpha * Valpha) + (Vbeta * Vbeta)), &Vref);
    agl_radian = atan2f(Vbeta, Valpha);
    agl = agl_radian * (180.0f / M_PI);

    if (agl >= 0 && agl < 60)       s = 1;
    else if (agl >= 60 && agl < 120)  s = 2;
    else if (agl >= 120 && agl < 180) s = 3;
    else if (agl >= -180 && agl < -120) s = 4;
    else if (agl >= -120 && agl < -60)  s = 5;
    else if (agl >= -60 && agl < 0)     s = 6;
    else s = 0;

    Ta = T1 = (Tperiod * sqrtf(3.0f) / Vdc) * Vref * sinf((M_PI * s / 3.0f) - agl_radian);
    Tb = T2 = (Tperiod * sqrtf(3.0f) / Vdc) * Vref * sinf(agl_radian - ((s - 1) * M_PI / 3.0f));
    T0 = Tperiod - Ta - Tb;

    u = Tperiod - T0 / 2.0f;
    v = (T0 / 2.0f) + T2;
    w = T0 / 2.0f;
    g = (T0 / 2.0f) + T1;

    switch (s)
    {
        case 0: Tsw1 = 0; Tsw2 = 0; Tsw3 = 0; break;
        case 1: Tsw1 = u; Tsw2 = v; Tsw3 = w; break;
        case 2: Tsw1 = g; Tsw2 = u; Tsw3 = w; break;
        case 3: Tsw1 = w; Tsw2 = u; Tsw3 = v; break;
        case 4: Tsw1 = w; Tsw2 = g; Tsw3 = u; break;
        case 5: Tsw1 = v; Tsw2 = w; Tsw3 = u; break;
        case 6: Tsw1 = u; Tsw2 = w; Tsw3 = g; break;
    }

    uint32_t max_count = htim1.Init.Period;
    TIM1->CCR1 = (uint32_t)((Tsw1 / Tperiod) * max_count); // U
    TIM1->CCR2 = (uint32_t)((Tsw2 / Tperiod) * max_count); // V
    TIM1->CCR3 = (uint32_t)((Tsw3 / Tperiod) * max_count); // W
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
		{
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
        elec_angle_rad = Sensor_GetElectricalAngle();     // Electrical angle (rad) for Park transform
        mech_angle_rad = Sensor_GetMechanicalAngle();     // Mechanical angle (rad)
        velocity_rads  = Sensor_GetVelocity();            // Angular velocity (rad/s)
        hall_step      = Hall_GetStep();                  // Current Hall step (1..6)
        z_counter      = ReadZ();
				i1 = ADC_Driver_GetCurrents_1();
			  i2 = ADC_Driver_GetCurrents_2();
				SVPWM(elec_angle_rad);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
		}
		else if (htim->Instance == TIM5)
		{
			EncoderSensor_Update();
		}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
			HallSensor_Update();
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM23_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_Base_Start_IT(&htim3);
		Sensor_Init();
		HallSensor_Update();
		//adc//
		ADC_Driver_Init();
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 44;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
