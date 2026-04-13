#ifndef CONF_H
#define CONF_H

#include <math.h>

/* ===================== Motor parameters ===================== */
#define POLE_PAIRS                  4.0f

/* ===================== PWM and control loops =============== */
#define MOTOR_PWM_FREQ              16000.0f   /* Hz */
#define MOTOR_SPEED_CALC_FREQ       10000.0f   /* Hz for FOC loop */
#define MOTOR_SPEED_CALC_FREQ_FOR_ENCODER 1000.0f /* Hz for speed calculation */

/* ===================== Encoder (ABZ) ======================= */
#define ENCODER_PPR                 2500.0f
#define ENCODER_QUADRATURE          4.0f
#define ENCODER_COUNTS_PER_REV      (ENCODER_PPR * ENCODER_QUADRATURE)  /* 10000 */

/* ===================== Hall sensor ========================= */
#define HALL_GPIO_PORT              GPIOD
#define HALL_U_PIN                  GPIO_PIN_12
#define HALL_V_PIN                  GPIO_PIN_13
#define HALL_W_PIN                  GPIO_PIN_14

/* ===================== Timer handles ======================= */
#define TIM_SVPWM                   htim1
#define TIM_FOC_LOOP                htim3
#define TIM_ENCODER                 htim2
#define TIM_HALL                    htim4
#define TIM_SPEED_CALC              htim5
#define TIM_Z_PULSE                 htim12   // Không dùng Z pulse

/* ===================== DC bus voltage ===================== */
#ifndef VDC_BUS
#define VDC_BUS                     35.0f
#endif

/* ===================== Math constants ===================== */
#ifndef M_PI
#define M_PI                        3.14159265358979323846f
#endif
#ifndef M_2PI
#define M_2PI                       (2.0f * M_PI)
#endif
#ifndef SQRT3
#define SQRT3                       1.7320508075688772f
#endif

/* ===================== Calibration defaults ================ */
#define CALIB_DEFAULT_VD            2.0f      // Ði?n áp lock (V)
#define CALIB_DEFAULT_VQ            0.0f
#define CALIB_DEFAULT_DURATION_MS   1500      // ms

#endif /* CONF_H */