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
#include "dma.h"
#include "tim.h"
#include "usart.h"
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
#include "svpwm.h"
#include "uart_dma_lib.h"
#include "pid.h"
#include "foc_transform.h"
#include "calib_encoder_incremental.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*PID*/
PID_HandleTypeDef pid_speed;
PID_HandleTypeDef pid_id;
PID_HandleTypeDef pid_iq;

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
float elec_angle_rad_hall;          /* Electrical angle (rad) */
float elec_angle_rad_encoder;          
float mech_angle_rad_encoder;          /* Mechanical angle (rad) */
float mech_angle_deg;
float elec_angle_deg; 
float raw_angle;
float speed_ref_rads = 20; // setpoint speed rad/s

uint8_t hall_step;             /* Current Hall step (1..6) */

/* Current measurement variables */
float iu, iv,iw;                  								 /* Phase A, B,C currents (A) */

float i_alpha, i_beta , v_alpha , v_beta;          /* Stationary frame currents */

float i_d = 0 , i_q = 0 , i_q_ref = 0 , i_d_ref = 0;             /* Rotating frame currents */
   

float v_d = 0.0f , v_q = 0.0f;

int state = 0;

int state_Z=0;

int dir =1;

/*PID*/
float speed_mechanic_rad;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void Calib(void);
void Align_Process(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* FOC loop interrupt (e.g., 10 kHz) */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)   // FOC loop (10kHz)
    {		
        elec_angle_rad_hall = Sensor_Get_Electrical_Angle_Hall();
				
				raw_angle = Encoder_Get_Raw_Angle();
				elec_angle_rad_encoder = Encoder_Get_Electric_Angle();
				mech_angle_rad_encoder = Encoder_Get_Mechanic_Angle();
        speed_mechanic_rad = Encoder_Get_Mechanic_Speed();
        hall_step = Hall_GetStep();
        iu = ADC_Driver_GetCurrents_1();
        iv = ADC_Driver_GetCurrents_2();
        iw = ADC_Driver_GetCurrents_3();
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);

				  if (state == 1) 
					{
					FOC_InvPark(v_d, v_q, 0, &v_alpha, &v_beta);
					SVPWM_Update(v_alpha, v_beta); 
					Encoder_Reset();
					} 
					else if (state==2) {

					FOC_Clarke(iu, iv, iw, &i_alpha, &i_beta);
					FOC_Park(i_alpha, i_beta, elec_angle_rad_encoder, &i_d, &i_q);


					i_q_ref = PID_Compute(&pid_speed, speed_ref_rads, speed_mechanic_rad);
					v_q = PID_Compute(&pid_iq, i_q_ref, i_q);
					v_d = PID_Compute(&pid_id, 0,i_d);

					FOC_InvPark(v_d, v_q, elec_angle_rad_encoder, &v_alpha, &v_beta);
					SVPWM_Update(v_alpha, v_beta);
						
				
						

					}	
					else if(state ==3){
						FOC_InvPark(v_d, v_q, elec_angle_rad_encoder, &v_alpha, &v_beta);
						SVPWM_Update(v_alpha, v_beta);
					}
					else if(state ==4){
						
						v_q = PID_Compute(&pid_iq, i_q_ref, i_q);
						v_d = PID_Compute(&pid_id, i_d_ref, i_d);
						FOC_InvPark(v_d, v_q, elec_angle_rad_encoder, &v_alpha, &v_beta);
						SVPWM_Update(v_alpha, v_beta);
						FOC_Clarke(iu, iv, iw, &i_alpha, &i_beta);
						FOC_Park(i_alpha, i_beta, elec_angle_rad_encoder, &i_d, &i_q);
						
						HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);

					}
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
		else if (htim->Instance == TIM12)
		{
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  MX_TIM12_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
		//center-alinge
		SVPWM_Init(&htim1, VDC_BUS, (float)MOTOR_PWM_FREQ, htim1.Init.Period);
		SVPWM_Start();
    HAL_TIM_Base_Start_IT(&TIM_FOC_LOOP);
		HAL_TIM_Base_Start_IT(&TIM_SPEED_CALC);
		/* ======================== Hall sensor ======================== */
		Hall_Sensor_Init();
		HallSensor_Update();
		/* ======================== Encoder sensor ======================== */
		Encoder_Sensor_Init();
		/* ======================== ADC sensor ======================== */
		ADC_Driver_Init();
		/* ======================== UART sensor ======================== */
		MX_USART1_UART_Init();   
		UART_DMA_Init();   
		UART_DMA_SendString("UART DMA with IDLE+HT/TC ready\r\n");
		/* ======================== PID ======================== */

		PID_Init(&pid_iq, 0.9f, 1000.0f, 0.0f, 1/MOTOR_SPEED_CALC_FREQ , VDC_BUS/SQRT3, -VDC_BUS/SQRT3);
		PID_Init(&pid_id, 0.9f, 1000.0f, 0.0f, 1/MOTOR_SPEED_CALC_FREQ , VDC_BUS/SQRT3, -VDC_BUS/SQRT3);
		PID_Init(&pid_speed, 0.1f, 0.9f, 0.0f, 1/MOTOR_SPEED_CALC_FREQ , 10, -10);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


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
