/**
 * @file    encoder.h
 * @brief   Encoder interface (quadrature AB + index Z pulse)
 * @date    2026
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define ENCODER_PPR         2500.0f
#define ENCODER_4X          (ENCODER_PPR * 4.0f)
#define POLE_PAIRS          4.0f
#define Z_PULSE_OFFSET      300.0f      // electrical degree offset

typedef struct
{
    int16_t  velocity;          // delta counts per sampling period (signed)
    int64_t  position;          // accumulated counts (4x quadrature)
    uint32_t last_counter_value;
} encoder_instance_t;

// Public / exported variables
extern encoder_instance_t enc_instance_mot;
extern volatile int64_t   encoder_position;
extern volatile int16_t   encoder_velocity;
extern volatile bool      is_z_aligned;
extern float              Angle;                // electrical angle in degrees

// Public functions
void     Encoder_Init(TIM_HandleTypeDef *htim_enc, TIM_HandleTypeDef *htim_index);
void     Encoder_Update(TIM_HandleTypeDef *htim_enc, TIM_HandleTypeDef *htim_index);
float    Encoder_GetElectricalAngle(TIM_HandleTypeDef *htim_enc);
int64_t  Encoder_GetTotalCounts(void);
int16_t  Encoder_GetVelocity(void);
bool     Encoder_IsIndexFound(void);

#endif /* ENCODER_H_ */