#include "calib_encoder_incremental.h"
#include "sensor.h"
#include "conf.h"
#include "foc_transform.h"
#include "svpwm.h"
#include "gpio.h"

/* C?u hình m?c d?nh (có th? override t? conf.h) */
#ifndef CALIB_ALIGN_CURRENT
#define CALIB_ALIGN_CURRENT         2.0f    /* Dòng Iq (A) dùng d? lock rotor */
#endif

#ifndef CALIB_ALIGN_DURATION_MS
#define CALIB_ALIGN_DURATION_MS     1500    /* Th?i gian gi? dòng (ms) */
#endif

#ifndef CALIB_UPDATE_PERIOD_MS
#define CALIB_UPDATE_PERIOD_MS      1       /* Chu k? c?p nh?t SVPWM trong khi ch? (ms) */
#endif

/**
 * @brief  Hàm blocking th?c hi?n can ch?nh encoder.
 */
void Calib_Encoder_Run(void)
{
    float v_alpha, v_beta;
    uint32_t elapsed_ms = 0;

    /* B?t driver (PF9 = HIGH) */
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);

    /* Vòng l?p gi? dòng lock rotor */
    while (elapsed_ms < CALIB_ALIGN_DURATION_MS)
    {
        /* T?o vector di?n áp c? d?nh: Id = 0, Iq = CALIB_ALIGN_CURRENT, theta = 0 */
        FOC_InvPark(0.0f, CALIB_ALIGN_CURRENT, 0.0f, &v_alpha, &v_beta);
        SVPWM_Update(v_alpha, v_beta);

        /* Ch? CALIB_UPDATE_PERIOD_MS ms */
        HAL_Delay(CALIB_UPDATE_PERIOD_MS);
        elapsed_ms += CALIB_UPDATE_PERIOD_MS;
    }

    /* Ðã gi? d? th?i gian -> rotor ? v? trí di?n 0° */
    Encoder_Reset();

    /* T?t di?n áp (cho an toàn tru?c khi chuy?n sang FOC) */
    FOC_InvPark(0.0f, 0.0f, 0.0f, &v_alpha, &v_beta);
    SVPWM_Update(v_alpha, v_beta);

    /* Có th? t?t PF9 t?m th?i, ho?c d? nguyên cho FOC (tùy thi?t k?).
       ? dây ta d? nguyên HIGH d? FOC k? ti?p s? d?ng. */
    // HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
}