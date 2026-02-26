/**
 * @file    encoder.h
 * @brief   Header file for incremental encoder handling using STM32 Timer Encoder Mode
 * @author  Based on Steppeschool tutorial + adaptations
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "stm32f4xx_hal.h"   // Thay b?ng dòng MCU c?a b?n: stm32f1xx_hal.h, stm32f4xx_hal.h, v.v.
#include <stdint.h>

// C?u trúc luu tr?ng thái encoder (t? Steppeschool)
typedef struct {
    int16_t  velocity;           // V?n t?c (counts per sampling period, có d?u)
    int64_t  position;           // V? trí tích luy (counts, h? tr? nhi?u vòng quay)
    uint32_t last_counter_value; // Giá tr? CNT l?n tru?c d? tính delta
} encoder_instance_t;

// Extern bi?n toàn c?c (có th? truy c?p t? main.c ho?c noi khác)
extern encoder_instance_t enc_instance_mot;  // Instance cho motor encoder (d?i tên n?u c?n)
extern volatile int64_t   encoder_position;  // V? trí hi?n t?i (copy t? struct cho d? dùng)
extern volatile int16_t   encoder_velocity;  // V?n t?c hi?n t?i

// Prototypes
void     Encoder_Init(void);
void     update_encoder(encoder_instance_t *enc, TIM_HandleTypeDef *htim);
uint32_t Encoder_GetPosition(void);         // Optional: getter n?u c?n
int16_t  Encoder_GetVelocity(void);

#endif /* ENCODER_H_ */