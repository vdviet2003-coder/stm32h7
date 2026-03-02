/**
 * @file    encoder.c
 * @brief   Encoder processing (quadrature TI12 + Index via TIM23)
 */

#include "encoder.h"

// ------------------------------------------------
// Module global variables
encoder_instance_t enc_instance_mot = {0};
volatile int64_t   encoder_position = 0;
volatile int16_t   encoder_velocity = 0;
volatile bool      is_z_aligned      = false;
float              Angle             = 0.0f;

static uint32_t    last_z_count      = 0;
static uint8_t     first_time        = 0;

// ------------------------------------------------
static void update_encoder_raw(encoder_instance_t *enc, TIM_HandleTypeDef *htim)
{
    uint32_t temp_counter = __HAL_TIM_GET_COUNTER(htim);

    if (!first_time)
    {
        enc->velocity = 0;
        first_time = 1;
    }
    else
    {
        int32_t delta;

        if (temp_counter == enc->last_counter_value)
        {
            delta = 0;
        }
        else if (temp_counter > enc->last_counter_value)
        {
            if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
            {
                delta = -(enc->last_counter_value + (__HAL_TIM_GET_AUTORELOAD(htim) - temp_counter + 1));
            }
            else
            {
                delta = temp_counter - enc->last_counter_value;
            }
        }
        else // underflow / wrap around
        {
            if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
            {
                delta = temp_counter - enc->last_counter_value;
            }
            else
            {
                delta = temp_counter + (__HAL_TIM_GET_AUTORELOAD(htim) - enc->last_counter_value + 1);
            }
        }

        enc->velocity = (int16_t)delta;
    }

    enc->position += enc->velocity;
    enc->last_counter_value = temp_counter;
}

// ------------------------------------------------
void Encoder_Init(TIM_HandleTypeDef *htim_enc, TIM_HandleTypeDef *htim_index)
{
    enc_instance_mot.velocity = 0;
    enc_instance_mot.position = 0;
    enc_instance_mot.last_counter_value = 0;
    encoder_position = 0;
    encoder_velocity = 0;
    first_time = 0;
    is_z_aligned = false;
    last_z_count = 0;
    Angle = 0.0f;

    HAL_TIM_Encoder_Start(htim_enc, TIM_CHANNEL_ALL);
    HAL_TIM_Base_Start(htim_index);
}

// ------------------------------------------------
void Encoder_Update(TIM_HandleTypeDef *htim_enc, TIM_HandleTypeDef *htim_index)
{
    // 1. Check for Index (Z) pulse
    uint32_t counter = __HAL_TIM_GET_COUNTER(htim_index);

    if (!is_z_aligned && (counter > last_z_count))
    {
        // Z pulse detected ? reset encoder reference to zero
        __HAL_TIM_SET_COUNTER(htim_enc, 0);
        enc_instance_mot.position = 0;
        enc_instance_mot.last_counter_value = 0;
        update_encoder_raw(&enc_instance_mot, htim_enc); // immediate read
        is_z_aligned = true;
    }
    last_z_count = counter;

    // 2. Update normal quadrature encoder
    update_encoder_raw(&enc_instance_mot, htim_enc);

    encoder_position = enc_instance_mot.position;
    encoder_velocity = enc_instance_mot.velocity;
}

// ------------------------------------------------
float Encoder_GetElectricalAngle(TIM_HandleTypeDef *htim_enc)
{
    if (!is_z_aligned) return 0.0f;

    uint32_t tim_cnt = __HAL_TIM_GET_COUNTER(htim_enc);
    uint32_t counts_per_elec_rev = (uint32_t)(ENCODER_4X / POLE_PAIRS);

    uint32_t elec_cnt = tim_cnt % counts_per_elec_rev;
    float elec_angle = ((float)elec_cnt / (float)counts_per_elec_rev) * 360.0f;
    elec_angle += Z_PULSE_OFFSET;

    Angle = fmodf(elec_angle, 360.0f);
    if (Angle < 0.0f) Angle += 360.0f;

    return Angle;
}

// ------------------------------------------------
int64_t Encoder_GetTotalCounts(void)
{
    return encoder_position;
}

int16_t Encoder_GetVelocity(void)
{
    return encoder_velocity;
}

bool Encoder_IsIndexFound(void)
{
    return is_z_aligned;
}