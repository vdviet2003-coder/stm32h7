#ifndef CALIB_ENCODER_INCREMENTAL_H
#define CALIB_ENCODER_INCREMENTAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

/**
 * @brief  Th?c hi?n quá trình can ch?nh encoder (blocking).
 * @note   Hàm này s? c?p dòng di?n Iq = CALIB_ALIGN_CURRENT vào d?ng co,
 *         gi? trong CALIB_ALIGN_DURATION_MS mili giây d? lock rotor v? v? trí di?n 0°.
 *         Sau dó t? d?ng reset b? d?m encoder.
 * @warning Hàm này ch?a vòng l?p ch? v?i HAL_Delay, ch? g?i khi chua ch?y FOC.
 */
void Calib_Encoder_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* CALIB_ENCODER_INCREMENTAL_H */