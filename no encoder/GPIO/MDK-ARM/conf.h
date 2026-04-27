#ifndef CONF_H
#define CONF_H

#include <math.h>

/* ===================== Motor parameters ===================== */
#define POLE_PAIRS                  4.0f

/* ===================== PWM and control loops =============== */
#define MOTOR_PWM_FREQ              20000.0f   /* Hz */
#define MOTOR_SPEED_CALC_FREQ       40000.0f   /* Hz for FOC loop */
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
/* ===================== Motor electrical parameters (for SMO) ===================== */
#define MOTOR_RS                    5.83f       // Ohm
#define MOTOR_LS                    0.01223f    // Henry
#define MOTOR_PSI_F                 0.075595238095238f      // Wb
/* ===================== Timer handles ======================= */
#define TIM_SVPWM                   htim1
#define TIM_FOC_LOOP                htim6
#define TIM_ENCODER                 htim2		//32bit
#define TIM_HALL                    htim4		
#define TIM_SPEED_CALC              htim3    //16bit
#define TIM_Z_PULSE                 htim12   //16bit

/* ===================== DC bus voltage ===================== */
#ifndef VDC_BUS
#define VDC_BUS                     60.0f
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

#ifndef SQRT3_OVER_2
#define SQRT3_OVER_2                0.8660254037844386f
#endif

#ifndef ONE_OVER_SQRT3
#define ONE_OVER_SQRT3   						0.5773502691896258f
#endif


#endif /* CONF_H */