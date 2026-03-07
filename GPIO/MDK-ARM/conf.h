#pragma once
// Motor physical parameters:
#define motor_pwm_freq 40000 // Driver bridge PWM frequency, Hz
#define motor_speed_calc_freq 930 // Motor speed calculation frequency, Hz
/* Encoder (ABZ) */
#define ENCODER_PPR                 2500.0f
#define ENCODER_QUADRATURE          4.0f
#define ENCODER_COUNTS_PER_REV      (ENCODER_PPR * ENCODER_QUADRATURE)  // 10000

#define POLE_PAIRS                  4.0f
#define COUNTS_PER_ELEC_REV         ((uint32_t)(ENCODER_COUNTS_PER_REV / POLE_PAIRS))  // 2500

#define Z_PULSE_OFFSET              300.0f      // Ð? bù t? Z d?n d-axis (do th?c t?)

/* Hall (UVW) */
#define HALL_GPIO_PORT              GPIOD
#define HALL_U_PIN                  GPIO_PIN_12
#define HALL_V_PIN                  GPIO_PIN_13
#define HALL_W_PIN                  GPIO_PIN_14

/* Timer */
#define TIM_ENCODER                 htim2
#define TIM_HALL                    htim4
#define TIM_Z                       htim23
#define TIM_SENSOR_SAMPLE           htim5       // Sample rate cho encoder update
/*M_PI*/
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef M_2PI
#define M_2PI (2.0f * M_PI)
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD(deg) ((deg) * (M_PI / 180.0f))
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG(rad) ((rad) * (180.0f / M_PI))
#endif