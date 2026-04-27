#include "svpwm.h"
#include "foc_transform.h"
#include <math.h>

/* Internal static variables */
static TIM_HandleTypeDef *tim1_handle = NULL;
static float Vdc = VDC_BUS;
static float Tperiod = 1.0f / MOTOR_PWM_FREQ;   /* PWM period (seconds) */
static uint32_t max_count = 0;                  /* Timer auto-reload value (ARR) */

/* Dead-time compensation parameters */
#define DEADTIME_US         0.484f      /* 133 / 275 MHz = 0.484 µs */
#define VDIODE_V            0.8f        /* typical body diode drop */

/* Access to global phase currents (defined in main.c) */

/*----------------------------------------------------------------------------
 * Apply dead-time compensation to three-phase duty cycles
 *----------------------------------------------------------------------------*/
static void SVPWM_ApplyDeadtimeComp(float *Ta, float *Tb, float *Tc,float iu, float iv, float iw)
{
    // Calculate base compensation (as fraction of duty cycle)
    float Tsw_us = 1.0e6f / MOTOR_PWM_FREQ;     // PWM period in µs
    float delta_duty_base = DEADTIME_US / Tsw_us;
    float delta_duty_diode = VDIODE_V / Vdc;

    // Phase A
    float sign_a = (iu > 0.0f) ? 1.0f : -1.0f;
    *Ta += delta_duty_base + sign_a * delta_duty_diode;

    // Phase B
    float sign_b = (iv > 0.0f) ? 1.0f : -1.0f;
    *Tb += delta_duty_base + sign_b * delta_duty_diode;

    // Phase C
    float sign_c = (iw > 0.0f) ? 1.0f : -1.0f;
    *Tc += delta_duty_base + sign_c * delta_duty_diode;

    // Clamp to [0, 1]
    *Ta = fmaxf(0.0f, fminf(1.0f, *Ta));
    *Tb = fmaxf(0.0f, fminf(1.0f, *Tb));
    *Tc = fmaxf(0.0f, fminf(1.0f, *Tc));
}

/*----------------------------------------------------------------------------
 * SVPWM calculation – Based on reference document (Section 5.1)
 * (Original algorithm unchanged until duty generation)
 *----------------------------------------------------------------------------*/
static void SVPWM_Calculate(float Valpha, float Vbeta,float iu, float iv, float iw)
{
    // ========================================================================
    // 1. Sector determination using the Sign Method
    // ========================================================================
    float U1 = Vbeta;
    float U2 = SQRT3_OVER_2 * Valpha - 0.5f * Vbeta;
    float U3 = -SQRT3_OVER_2 * Valpha - 0.5f * Vbeta;

    int N = 0;
    if (U1 > 0) N += 1;
    if (U2 > 0) N += 2;
    if (U3 > 0) N += 4;

    static const int sector_table[8] = {0, 2, 6, 1, 4, 5, 3, 0};
    int sector = sector_table[N];

    // ========================================================================
    // 2. Calculate intermediate variables X, Y, Z (normalized to duty cycle)
    // ========================================================================
    float X = SQRT3 * Vbeta / Vdc;
    float Y = (1.5f * Valpha + 0.5f * SQRT3 * Vbeta) / Vdc;
    float Z = (1.5f * Valpha - 0.5f * SQRT3 * Vbeta) / Vdc;

    // ========================================================================
    // 3. Determine Tx and Ty based on sector
    // ========================================================================
    float Tx, Ty;
    switch (sector) {
        case 1: Tx = Z; Ty = Y; break;
        case 2: Tx = Y; Ty = -X; break;
        case 3: Tx = -Z; Ty = X; break;
        case 4: Tx = -X; Ty = Z; break;
        case 5: Tx = X; Ty = -Y; break;
        case 6: Tx = -Y; Ty = -Z; break;
        default: Tx = 0; Ty = 0; break;
    }

    // ========================================================================
    // 4. Overmodulation handling
    // ========================================================================
    float Tsum = Tx + Ty;
    if (Tsum > 1.0f) {
        Tx = Tx / Tsum;
        Ty = Ty / Tsum;
    }

    // ========================================================================
    // 5. Calculate three-phase duty cycles (before dead-time compensation)
    // ========================================================================
    float Ta = 0.5f + Valpha / Vdc;
    float Tb = 0.5f + (-0.5f * Valpha + SQRT3_OVER_2 * Vbeta) / Vdc;
    float Tc = 0.5f + (-0.5f * Valpha - SQRT3_OVER_2 * Vbeta) / Vdc;

    // ========================================================================
    // 6. Apply dead-time compensation
    // ========================================================================
    SVPWM_ApplyDeadtimeComp(&Ta, &Tb, &Tc,iu,iv,iw);

    // ========================================================================
    // 7. Write compare values to timer registers
    // ========================================================================
    if (max_count != 0 && tim1_handle != NULL) {
        TIM1->CCR1 = (uint32_t)(Ta * max_count);
        TIM1->CCR2 = (uint32_t)(Tb * max_count);
        TIM1->CCR3 = (uint32_t)(Tc * max_count);
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

void SVPWM_Update(float Valpha, float Vbeta, float iu, float iv, float iw)
{
    if (tim1_handle == NULL) return;
    SVPWM_Calculate(Valpha, Vbeta, iu, iv, iw);
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