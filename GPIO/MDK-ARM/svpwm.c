#include "svpwm.h"
#include "foc_transform.h"

/* Internal static variables */
static TIM_HandleTypeDef *tim1_handle = NULL;
static float Vdc = VDC_BUS;           /* DC bus voltage */
static float Tperiod = 16000.0f;     /* Tperiod (same as MOTOR_PWM_FREQ) */
static uint32_t max_count = 0;       /* Timer period (auto-reload value) */


/* Intermediate variables (kept as in original code) */
static float agl, agl_radian;
static float Valpha, Vbeta, Vref;
static float Ta, Tb, T0, T1, T2;
static float u, v, w, g;
static float Tsw1, Tsw2, Tsw3;
static uint8_t sector;                /* Current sector 1..6 */

/*----------------------------------------------------------------------------
 * SVPWM calculation – identical to original algorithm
 *----------------------------------------------------------------------------*/
static void SVPWM_Calculate(float Valpha,float Vbeta  )
{
    arm_sqrt_f32((Valpha * Valpha) + (Vbeta * Vbeta), &Vref);
    arm_atan2_f32(Vbeta, Valpha, &agl_radian);
    agl = agl_radian * (180.0f / M_PI);

    /* Determine sector */
    if (agl >= 0 && agl < 60)       sector = 1;
    else if (agl >= 60 && agl < 120) sector = 2;
    else if (agl >= 120 && agl < 180) sector = 3;
    else if (agl >= -180 && agl < -120) sector = 4;
    else if (agl >= -120 && agl < -60) sector = 5;
    else if (agl >= -60 && agl < 0) sector = 6;
    else sector = 0;

    /* Calculate active vector times */
    Ta = T1 = (Tperiod * SQRT3 / Vdc) * Vref * sinf((M_PI * sector / 3.0f) - agl_radian);
    Tb = T2 = (Tperiod * SQRT3 / Vdc) * Vref * sinf(agl_radian - ((sector - 1) * M_PI / 3.0f));
    T0 = Tperiod - Ta - Tb;

    /* Compute switching times for center-aligned PWM */
    u = Tperiod - T0 / 2.0f;
    v = (T0 / 2.0f) + T2;
    w = T0 / 2.0f;
    g = (T0 / 2.0f) + T1;

    /* Assign phase times based on sector */
    switch (sector)
    {
        case 0: Tsw1 = 0; Tsw2 = 0; Tsw3 = 0; break;
        case 1: Tsw1 = u; Tsw2 = v; Tsw3 = w; break;
        case 2: Tsw1 = g; Tsw2 = u; Tsw3 = w; break;
        case 3: Tsw1 = w; Tsw2 = u; Tsw3 = v; break;
        case 4: Tsw1 = w; Tsw2 = g; Tsw3 = u; break;
        case 5: Tsw1 = v; Tsw2 = w; Tsw3 = u; break;
        case 6: Tsw1 = u; Tsw2 = w; Tsw3 = g; break;
    }

    /* Write compare values to TIM1 CCR registers – same formula as original */
    if (max_count != 0 && tim1_handle != NULL)
    {
        TIM1->CCR1 = (uint32_t)(Tsw1 * max_count / Tperiod);
        TIM1->CCR2 = (uint32_t)(Tsw2 * max_count / Tperiod);
        TIM1->CCR3 = (uint32_t)(Tsw3 * max_count / Tperiod);
    }
}

/*----------------------------------------------------------------------------
 * Public functions
 *----------------------------------------------------------------------------*/
void SVPWM_Init(TIM_HandleTypeDef *htim, float vdc, float tperiod, uint32_t period_count)
{
    tim1_handle = htim;
    Vdc = vdc;
    Tperiod = tperiod;
    max_count = period_count;
}


void SVPWM_Update(float Valpha,float Vbeta )
{
    if (tim1_handle == NULL) return;
    SVPWM_Calculate( Valpha, Vbeta );
}

void SVPWM_Start(void)
{
    if (tim1_handle == NULL) return;
    HAL_TIM_PWM_Start(tim1_handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(tim1_handle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(tim1_handle, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(tim1_handle, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(tim1_handle, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(tim1_handle, TIM_CHANNEL_3);
}