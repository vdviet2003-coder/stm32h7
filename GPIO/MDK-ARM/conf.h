/**
  * @file    conf.h
  * @brief   Global configuration parameters for motor and sensors.
  */

#pragma once

/* ===================== Motor electrical parameters ===================== */
/**
 * Number of motor pole pairs.
 * With 8 poles total => POLE_PAIRS = 4.
 */
#define POLE_PAIRS                  4.0f

/* ===================== PWM and control loop frequencies ================ */
#define MOTOR_PWM_FREQ              40000   /*!< PWM frequency (Hz) */
#define MOTOR_SPEED_CALC_FREQ       10000   /*!< Speed calculation frequency (Hz)  */

/* ===================== Encoder (ABZ) specifications ==================== */
/**
 * OIH 48 series incremental encoder.
 * Resolution: choose from 1000 to 12000 C/T (cycles per turn).
 * Example: 2500 C/T with quadrature x4 gives 10000 counts/rev.
 */
#define ENCODER_PPR                 2500.0f     /*!< Pulses per revolution – xung/vòng */
#define ENCODER_QUADRATURE          4.0f        /*!< Decoding factor (x4)  */
#define ENCODER_COUNTS_PER_REV      (ENCODER_PPR * ENCODER_QUADRATURE)  /*!< Total counts per mechanical revolution  */

/**
 * Electrical counts per revolution = total counts / pole pairs.
 */
#define COUNTS_PER_ELEC_REV         ((uint32_t)(ENCODER_COUNTS_PER_REV / POLE_PAIRS))

/* ===================== Z pulse (index) offset ========================== */
/**
 * Offset between Z pulse (index) 
 * Based on OIH 48 datasheet: phase between U rise and Z center is ±1° mechanical.
 * Since step 6 (101) corresponds to 300° electrical, and Z occurs near U rise,
 * the mechanical angle at Z should be 300° electrical / 4 = 75° mechanical.
 * Adjust Z_PULSE_OFFSET_DEG experimentally to fine-tune.
 */
#define Z_PULSE_OFFSET_DEG          75.0f       /*Mechanical offset in degrees*/
#define Z_PULSE_OFFSET_RAD          (Z_PULSE_OFFSET_DEG * (M_PI / 180.0f)) /* Converted to radians  */

/* ===================== Hall sensor configuration ======================= */
/**
 * Hall sensor electrical offset for step 1.
 * Step 1 (usually 100) can be set to any electrical angle. Default 0° means step 1 = 0° electrical.
 */
#define HALL_ELEC_OFFSET_DEG        0.0f        /*!< Electrical offset for step 1 (degrees) */
#define HALL_ELEC_OFFSET_RAD        (HALL_ELEC_OFFSET_DEG * (M_PI / 180.0f)) /*!< Converted to radians */

/**
 * Width of one Hall step in electrical radians.
 * Each step covers 60° electrical = p/3 rad.
 */
#define HALL_STEP_ELEC_WIDTH_RAD    (M_2PI / 6.0f)   // = p/3

/* ===================== Hall sensor pin mapping ========================= */
#define HALL_GPIO_PORT              GPIOD
#define HALL_U_PIN                  GPIO_PIN_12
#define HALL_V_PIN                  GPIO_PIN_13
#define HALL_W_PIN                  GPIO_PIN_14

/* ===================== Timer handles (defined in main.h or tim.h) ===== */
/* These are assumed to be extern TIM_HandleTypeDef variables */
#define TIM_ENCODER                 htim2       /*!< Encoder counter timer */
#define TIM_HALL                    htim4       /*!< Hall capture timer */
#define TIM_Z                       htim23      /*!< Timer for Z pulse (index) */
#define TIM_SENSOR_SAMPLE           htim5       /*!< Periodic update timer (velocity) */
#define TIM_CURRENT                 htim6 

/* ===================== Math constants ================================== */
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