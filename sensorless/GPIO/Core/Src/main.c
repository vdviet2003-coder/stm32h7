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
#include "pi_control.h"
#include "scurve_ramp.h"
#include "multi_ramp.h"
#include "smo.h"
#include "LPF.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*PID*/
PID_HandleTypeDef pid_speed;
PID_HandleTypeDef pid_id;
PID_HandleTypeDef pid_iq;

PI_Clamping pi_speed;
PI_Clamping pi_id;
PI_Clamping pi_iq;
SCurve_Profile s_ramp;
MultiRamp_Gen ramp;
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


uint8_t hall_step;             /* Current Hall step (1..6) */

/* Current measurement variables */
float iu, iv,iw;                  								 /* Phase A, B,C currents (A) */

float i_alpha, i_beta , v_alpha , v_beta;          /* Stationary frame currents */

float i_d = 0 , i_q = 0 , i_q_ref = 0 , i_d_ref = 0;             /* Rotating frame currents */
   

float v_d = 0.0f , v_q = 0.0f;

uint8_t state = 0;

int state_Z=0;

int dir =1;
float threshold;

/*PID*/
float speed_mechanic_rad;
float speed_ref_rpm = 1000.0f;      // T?c d? d?t (RPM)
float speed_mechanic_rpm = 0.0f;    // T?c d? h?i ti?p (RPM)
float speed_ref_rads = 100.0f; // setpoint speed rad/s

//smo
float theta_e, omega_e;
float error_theta;
float ss_openloop_timer = 0.0f;   // Bộ đếm thời gian trong pha open‑loop (giây)
float ss_openloop_speed = 0.0f;   // Tốc độ ảo tăng dần (rad/s)
//smo
typedef enum {
    SS_ALIGN = 0,
    SS_OPENLOOP,
    SS_SWITCH,
    SS_RUN
} SensorlessSubState;

SensorlessSubState state_smo = SS_ALIGN;
float ss_align_timer = 0.0f;
float ss_openloop_theta = 0.0f;
float ss_openloop_omega = 0.0f;
float ss_switch_timer = 0.0f;

SMO_Handle smo;                 // Đối tượng SMO toàn cục
float theta_smo, omega_smo;     // Góc và tốc độ từ SMO
float speed_mechanic_rad_smo;

LPF_FirstOrder_t LPF; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void Calib(void);
void Align_Process(void);
float value_offset;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* FOC loop interrupt (e.g., 10 kHz) */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)   // FOC loop (10kHz)
    {
				value_offset =ADC_Driver_GetOffset_IV();
        elec_angle_rad_hall = Sensor_Get_Electrical_Angle_Hall();
				
				raw_angle = Encoder_Get_Raw_Angle();
				elec_angle_rad_encoder = Encoder_Get_Electric_Angle();
				mech_angle_rad_encoder = Encoder_Get_Mechanic_Angle();
				hall_step = Hall_GetStep();
			
        
				speed_mechanic_rad = Encoder_Get_Mechanic_Speed();
        float omega_e = speed_mechanic_rad * POLE_PAIRS;

        // ----- D? do�n g�c di?n (b� tr?) -----
        float Ts = 1.0f / MOTOR_SPEED_CALC_FREQ;          // 25e-6 s
        float t_delay = 3.87e-6f;                         // 3.87 �s
        float theta_pred = elec_angle_rad_encoder + omega_e * (Ts + t_delay);
        theta_pred = fmodf(theta_pred, 2.0f * M_PI);
        if (theta_pred < 0.0f) theta_pred += 2.0f * M_PI;
        
        iu = ADC_Driver_GetCurrents_1();
        iv = ADC_Driver_GetCurrents_2();
        iw = ADC_Driver_GetCurrents_3();
				

				  if (state == 1) 
					{
						HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
					FOC_InvPark(0.8, 0, 0, &v_alpha, &v_beta);
					SVPWM_Update(v_alpha, v_beta, iu, iv, iw); 
					Encoder_Reset();
					v_d=0;
					v_q=0;	
					} 
					else if (state==2) {
						HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
					//SCurve_Update(&s_ramp, speed_ref_rads);
					MultiRamp_Update(&ramp,speed_ref_rads);

					FOC_Clarke(iu, iv, iw, &i_alpha, &i_beta);
					FOC_Park(i_alpha, i_beta, elec_angle_rad_encoder, &i_d, &i_q);


					i_q_ref = PID_Compute(&pid_speed, speed_ref_rads, speed_mechanic_rad);
					v_q = PID_Compute(&pid_iq, i_q_ref, i_q);
					v_d = PID_Compute(&pid_id, 0,i_d);
						
//						i_q_ref = PI_Update(&pi_speed,speed_ref_rads,speed_mechanic_rad);
//						v_q = PI_Update(&pi_iq,i_q_ref, i_q);
//						v_d = PI_Update(&pi_id,0, i_d);
					FOC_InvPark(v_d, v_q, elec_angle_rad_encoder, &v_alpha, &v_beta);
					SVPWM_Update(v_alpha, v_beta, iu, iv, iw);
						
				
					HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);	

					}	
					else if(state ==3){
						HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
						FOC_InvPark(v_d, v_q, elec_angle_rad_encoder, &v_alpha, &v_beta);
						SVPWM_Update(v_alpha, v_beta, iu, iv, iw);
					}
else if (state == 4) {
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
    float dt = 1.0f / MOTOR_SPEED_CALC_FREQ;

    // 1. Clarke Transform
    FOC_Clarke(iu, iv, iw, &i_alpha, &i_beta);
		MultiRamp_Update(&ramp,speed_ref_rads);
    float id_ref_local = 0.0f;
    float iq_ref_local = 0.0f;

    // ====================== SENSORLESS STARTUP STATE MACHINE ======================
    switch (state_smo) {
        case SS_ALIGN: {
            // Rotor alignment (Id = 0.8A, Iq = 0, theta = 0)
            theta_e = 0.0f;
            omega_e = 0.0f;
            id_ref_local = 0.8f;
            iq_ref_local = 0.0f;

            ss_align_timer += dt;
            if (ss_align_timer >= 1.0f) {          // Align trong 1 giây
                state_smo = SS_OPENLOOP;
                ss_openloop_timer = 0.0f;
                ss_openloop_speed = 0.0f;
                ss_align_timer = 0.0f;
            }
            break;
        }

        case SS_OPENLOOP: {
            // Open-loop ramp-up
            ss_openloop_speed += 100.0f * dt;       // Gia tốc ~80 rad/s² (có thể chỉnh)
            ss_openloop_timer += dt;

            theta_e = ss_openloop_speed * ss_openloop_timer;   // theta = integral(omega)
            omega_e = ss_openloop_speed;

            id_ref_local = 0.0f;
            iq_ref_local = speed_ref_rads * 0.015f;   // Iq nhỏ để kéo động

            // Chuyển sang SWITCH khi đạt omega_min
            if (ss_openloop_speed >= smo.omega_min) {
                state_smo = SS_RUN;
                ss_switch_timer = 0.0f;
            }
            break;
        }

        case SS_SWITCH: {
            // Giữ open-loop thêm một khoảng thời gian để SMO hội tụ
            theta_e = ss_openloop_speed * ss_openloop_timer;
            omega_e = ss_openloop_speed;
            id_ref_local = 0.0f;
            iq_ref_local = speed_ref_rads * 0.015f;

            if (fabsf(smo.ialpha_est - i_alpha) < 0.001f &&
                fabsf(smo.ibeta_est - i_beta) < 0.001f) {
                state_smo = SS_RUN;
            }

            break;
        }

        case SS_RUN: {
            // Full sensorless - dùng SMO
            theta_e = SMO_GetCompensatedTheta(&smo);
            omega_e = LPF_FirstOrder_Update(&LPF,SMO_GetOmega(&smo));
					
            id_ref_local = 0.0f;
            speed_mechanic_rad_smo = omega_e / POLE_PAIRS;
            iq_ref_local = PID_Compute(&pid_speed, speed_ref_rads, speed_mechanic_rad_smo);
            break;
        }
    }

    // ====================== CURRENT CONTROL ======================
    FOC_Park(i_alpha, i_beta, theta_e, &i_d, &i_q);

    v_d = PID_Compute(&pid_id, id_ref_local, i_d);
    v_q = PID_Compute(&pid_iq, iq_ref_local, i_q);

    FOC_InvPark(v_d, v_q, theta_e, &v_alpha, &v_beta);

    // ====================== UPDATE SMO & PWM ======================
    SMO_Update(&smo, v_alpha, v_beta, i_alpha, i_beta, dt);

    SVPWM_Update(v_alpha, v_beta, iu, iv, iw);

    // Optional: monitor error
    error_theta = elec_angle_rad_encoder - theta_e  ;
}

					else if(state ==0){
						

						
						HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
						v_alpha = 0.0f; v_beta = 0.0f;
						SVPWM_Update(v_alpha, v_beta, iu, iv, iw);
						// Reset sensorless state
						state_smo = SS_ALIGN;
						ss_align_timer = 0.0f;
					}
    }
		else if (htim->Instance == TIM3)
		{
			EncoderSensor_Update();
		}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
    {
        uint32_t cap_val = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_3);
        Encoder_Capture_Handler(cap_val);
    }
    else if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        HallSensor_Update();
    }
    else if (htim->Instance == TIM12)
    {
        // Z-pulse nếu cần
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
		/* ======================== ADC sensor ======================== */
		ADC_Driver_Init();
		ADC_Driver_CalibrateOffset();
		/* ======================== Hall sensor ======================== */
		Hall_Sensor_Init();
		HallSensor_Update();
		/* ======================== Encoder sensor ======================== */
		Encoder_Sensor_Init();

		/* ======================== UART sensor ======================== */
		MX_USART1_UART_Init();   
		UART_DMA_Init();   
		UART_DMA_SendString("UART DMA with IDLE+HT/TC ready\r\n");
		/* ======================== PID ======================== */
			
		PID_Init(&pid_iq, 8.0f, 1749.0f, 0.0f, 1/MOTOR_SPEED_CALC_FREQ , VDC_BUS/SQRT3, -VDC_BUS/SQRT3);
		PID_Init(&pid_id, 8.0f, 1749.0f, 0.0f, 1/MOTOR_SPEED_CALC_FREQ , VDC_BUS/SQRT3, -VDC_BUS/SQRT3);
		PID_Init(&pid_speed, 0.0053f, 2.37f, 0.0f, 1/MOTOR_SPEED_CALC_FREQ, 2.8f, -2.8f); 
		//PID_Init(&pid_speed, 0.0053f, 4.0f, 0.0f, 1/MOTOR_SPEED_CALC_FREQ, 2.8f, -2.8f); 
		
		PI_Init(&pi_speed,0.1125,0.9375,1/MOTOR_SPEED_CALC_FREQ,10.0f,-10.0f);
		PI_Init(&pi_id,237.6697,11330,1/MOTOR_SPEED_CALC_FREQ,VDC_BUS/SQRT3,-VDC_BUS/SQRT3);
		PI_Init(&pi_iq,237.6697,11330,1/MOTOR_SPEED_CALC_FREQ,VDC_BUS/SQRT3,-VDC_BUS/SQRT3);
		
		SCurve_Init(&s_ramp,1/MOTOR_SPEED_CALC_FREQ,1,1,1);
		
		MultiRamp_Init(&ramp,1/MOTOR_SPEED_CALC_FREQ,5,10,10,20,20);
		/* ======================== SMO ======================== */
		
		SMO_Init(&smo, MOTOR_RS, MOTOR_LS, MOTOR_PSI_F, POLE_PAIRS,3000.0f, 300.0f, 1.0f / MOTOR_SPEED_CALC_FREQ);
		LPF_FirstOrder_Init(&LPF,1000,1.0f / MOTOR_SPEED_CALC_FREQ);
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
