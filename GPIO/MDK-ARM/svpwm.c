/**
  * @file    svpwm.c
  * @brief   Implementation of SVPWM (exact algorithm from your main.c)
  */

#include "svpwm.h"
#include "conf.h"   // for M_PI, SQRT3 if needed, but we define constants here

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef SQRT3
#define SQRT3 1.7320508075688772f
#endif

/* Private function: determine sector from angle (radians) using degree conversion */
static uint8_t get_sector_from_angle_deg(float angle_deg)
{
    if (angle_deg >= 0 && angle_deg < 60)        return 1;
    else if (angle_deg >= 60 && angle_deg < 120) return 2;
    else if (angle_deg >= 120 && angle_deg < 180) return 3;
    else if (angle_deg >= -180 && angle_deg < -120) return 4;
    else if (angle_deg >= -120 && angle_deg < -60)  return 5;
    else if (angle_deg >= -60 && angle_deg < 0)     return 6;
    else return 0;
}

void SVPWM_Init(SVPWM_HandleTypeDef *hsvpwm, float vdc, float tperiod)
{
    hsvpwm->vd = 0.0f;
    hsvpwm->vq = 0.0f;
    hsvpwm->angle_deg = 0.0f;
    hsvpwm->vdc = vdc;
    hsvpwm->tperiod = tperiod;

    hsvpwm->valpha = 0.0f;
    hsvpwm->vbeta = 0.0f;
    hsvpwm->vref = 0.0f;
    hsvpwm->angle_rad = 0.0f;
    hsvpwm->sector = 0;
    hsvpwm->t1 = hsvpwm->t2 = hsvpwm->t0 = 0.0f;
    hsvpwm->tsw1 = hsvpwm->tsw2 = hsvpwm->tsw3 = 0.0f;
}

void SVPWM_SetTarget(SVPWM_HandleTypeDef *hsvpwm, float vd, float vq, float angle_deg)
{
    hsvpwm->vd = vd;
    hsvpwm->vq = vq;
    hsvpwm->angle_deg = angle_deg;
}

void SVPWM_Update(SVPWM_HandleTypeDef *hsvpwm)
{
    float angle_rad, sin_th, cos_th;
    float v_alpha, v_beta, vref;
    float agl_rad, agl_deg;
    float Tp = hsvpwm->tperiod;
    float Vdc = hsvpwm->vdc;
    float sqrt3 = SQRT3;
    float t1, t2, t0;
    float u, v, w, g;
    uint8_t sector;

    /* 1. Inverse Park transform (dq -> aß) */
    angle_rad = hsvpwm->angle_deg * (M_PI / 180.0f);
    arm_sin_cos_f32(angle_rad, &sin_th, &cos_th);
    v_alpha = cos_th * hsvpwm->vd - sin_th * hsvpwm->vq;
    v_beta  = sin_th * hsvpwm->vd + cos_th * hsvpwm->vq;

    /* 2. Compute reference magnitude and angle */
    arm_sqrt_f32(v_alpha * v_alpha + v_beta * v_beta, &vref);
    agl_rad = atan2f(v_beta, v_alpha);
    agl_deg = agl_rad * (180.0f / M_PI);

    /* 3. Determine sector */
    sector = get_sector_from_angle_deg(agl_deg);

    /* 4. Calculate T1, T2 (times in seconds) exactly as in your code */
    float k = Tp * sqrt3 / Vdc * vref;
    t1 = k * arm_sin_f32((M_PI * sector / 3.0f) - agl_rad);
    t2 = k * arm_sin_f32(agl_rad - ((sector - 1) * M_PI / 3.0f));

    /* 5. Handle overmodulation (if t1 + t2 > Tp) */
    if (t1 + t2 > Tp) {
        float scale = Tp / (t1 + t2);
        t1 *= scale;
        t2 *= scale;
    }
    t0 = Tp - t1 - t2;

    /* 6. Compute u, v, w, g */
    u = Tp - t0 * 0.5f;
    v = t0 * 0.5f + t2;
    w = t0 * 0.5f;
    g = t0 * 0.5f + t1;

    /* 7. Assign switching times according to sector */
    switch (sector)
    {
        case 0:
            hsvpwm->tsw1 = 0; hsvpwm->tsw2 = 0; hsvpwm->tsw3 = 0; break;
        case 1:
            hsvpwm->tsw1 = u; hsvpwm->tsw2 = v; hsvpwm->tsw3 = w; break;
        case 2:
            hsvpwm->tsw1 = g; hsvpwm->tsw2 = u; hsvpwm->tsw3 = w; break;
        case 3:
            hsvpwm->tsw1 = w; hsvpwm->tsw2 = u; hsvpwm->tsw3 = v; break;
        case 4:
            hsvpwm->tsw1 = w; hsvpwm->tsw2 = g; hsvpwm->tsw3 = u; break;
        case 5:
            hsvpwm->tsw1 = v; hsvpwm->tsw2 = w; hsvpwm->tsw3 = u; break;
        case 6:
            hsvpwm->tsw1 = u; hsvpwm->tsw2 = w; hsvpwm->tsw3 = g; break;
        default:
            hsvpwm->tsw1 = hsvpwm->tsw2 = hsvpwm->tsw3 = 0; break;
    }

    /* Save intermediate values for debugging */
    hsvpwm->valpha = v_alpha;
    hsvpwm->vbeta = v_beta;
    hsvpwm->vref = vref;
    hsvpwm->angle_rad = agl_rad;
    hsvpwm->sector = sector;
    hsvpwm->t1 = t1;
    hsvpwm->t2 = t2;
    hsvpwm->t0 = t0;
}

void SVPWM_Apply(SVPWM_HandleTypeDef *hsvpwm, TIM_HandleTypeDef *htim)
{
    uint32_t period = htim->Init.Period;
    float inv_Tp = 1.0f / hsvpwm->tperiod;

    /* Convert switching times to timer compare values */
    uint32_t ccr1 = (uint32_t)((hsvpwm->tsw1 * inv_Tp) * period);
    uint32_t ccr2 = (uint32_t)((hsvpwm->tsw2 * inv_Tp) * period);
    uint32_t ccr3 = (uint32_t)((hsvpwm->tsw3 * inv_Tp) * period);

    /* Update timer compare registers (assuming TIM1) */
    TIM1->CCR1 = ccr1;
    TIM1->CCR2 = ccr2;
    TIM1->CCR3 = ccr3;
}