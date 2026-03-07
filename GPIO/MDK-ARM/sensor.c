/**
  * @file    sensor.c
  * @brief   Tri?n khai sensor theo phong cách SimpleFOC (C cho STM32 HAL)
  */

#include "sensor.h"
#include "tim.h"
#include "main.h"
/* Private variables */
static uint32_t sample_timestamp_last = 0;  // Ð? tính dt

/* Exported variables */
Sensor_t sensor = {0};
HallSensor_t hall_sensor = {0};
EncoderSensor_t encoder_sensor = {0};

/* Private functions */
static uint8_t Hall_CalculateStep(uint8_t raw)
{
    switch (raw)
    {
        case 0b001: return 5;
        case 0b011: return 4;
        case 0b010: return 3;
        case 0b110: return 2;
        case 0b100: return 1;
        case 0b101: return 6;
        default:    return 0;
    }
}

static void Hall_Update_Internal(void)
{
    uint8_t u = HAL_GPIO_ReadPin(HALL_GPIO_PORT, HALL_U_PIN);
    uint8_t v = HAL_GPIO_ReadPin(HALL_GPIO_PORT, HALL_V_PIN);
    uint8_t w = HAL_GPIO_ReadPin(HALL_GPIO_PORT, HALL_W_PIN);

    hall_sensor.raw = (u << 2) | (v << 1) | w;
    hall_sensor.step = Hall_CalculateStep(hall_sensor.raw);

    // Góc co h?c u?c lu?ng t? Hall (coarse)
    if (hall_sensor.step != 0)
    {
        sensor.angle = (hall_sensor.step - 1) * (2.0f * M_PI / 6.0f) + (M_PI / 6.0f);  // rad, +30° offset
    }
}

static void Encoder_Update_Internal(void)
{
    uint32_t now = HAL_GetTick();  // ho?c dùng timer tick n?u c?n chính xác hon
    uint32_t dt_ms = now - sample_timestamp_last;
    if (dt_ms == 0) dt_ms = 1;     // tránh chia 0
    float dt = dt_ms * 0.001f;      // giây

    // Ð?c counter
    uint32_t cnt = __HAL_TIM_GET_COUNTER(&TIM_ENCODER);
    int32_t delta = (int32_t)(cnt - encoder_sensor.last_cnt);

    // X? lý wrap-around
    int32_t period_half = (int32_t)(TIM_ENCODER.Init.Period / 2);
    if (delta > period_half) delta -= (TIM_ENCODER.Init.Period + 1);
    else if (delta < -period_half) delta += (TIM_ENCODER.Init.Period + 1);

    encoder_sensor.delta = delta;
    encoder_sensor.counts += delta;
    encoder_sensor.last_cnt = cnt;

    // Tính v?n t?c góc co h?c (rad/s)
    sensor.velocity = ((float)delta / ENCODER_COUNTS_PER_REV) * (2.0f * M_PI) / dt;

    // X? lý Z pulse
    encoder_sensor.z_cnt = __HAL_TIM_GET_COUNTER(&TIM_Z);
    if (!sensor.index_found && encoder_sensor.z_cnt != encoder_sensor.last_z_cnt)
    {
        __HAL_TIM_SET_COUNTER(&TIM_ENCODER, 0);
        encoder_sensor.last_cnt = 0;
        encoder_sensor.counts = 0;
        sensor.index_found = true;
    }
    encoder_sensor.last_z_cnt = encoder_sensor.z_cnt;

    // Tính góc co h?c (rad)
    if (sensor.index_found)
    {
        int64_t pos = encoder_sensor.counts;
        int64_t mod = pos % (int64_t)COUNTS_PER_ELEC_REV;
        if (mod < 0) mod += (int64_t)COUNTS_PER_ELEC_REV;

        float mech_angle = ((float)mod / (float)COUNTS_PER_ELEC_REV) * 2.0f * M_PI;
        sensor.angle = mech_angle + DEG_TO_RAD(Z_PULSE_OFFSET);  // rad
        sensor.angle = fmodf(sensor.angle, 2.0f * M_PI);
        if (sensor.angle < 0.0f) sensor.angle += 2.0f * M_PI;
    }

    sample_timestamp_last = now;
}

/* Exported functions */
void Sensor_Init(void)
{
    HAL_TIMEx_HallSensor_Start_IT(&TIM_HALL);
    HAL_TIM_Encoder_Start(&TIM_ENCODER, TIM_CHANNEL_ALL);
    HAL_TIM_Base_Start(&TIM_Z);
    HAL_TIM_Base_Start_IT(&TIM_SENSOR_SAMPLE);

    // Ð?c Hall l?n d?u
    Hall_Update_Internal();

    // Reset encoder
    encoder_sensor.counts = 0;
    sensor.index_found = false;
    sensor.angle = 0.0f;
    sensor.velocity = 0.0f;
}

void EncoderSensor_Update(void)
{
    Encoder_Update_Internal();  // Encoder c?p nh?t chính
    // Hall ch? c?p nh?t khi có ng?t ? không c?n g?i ? dây
}
uint8_t ReadZ(void)
{
    return encoder_sensor.z_cnt;
}
void HallSensor_Update(void)
{
    Hall_Update_Internal();  // Encoder c?p nh?t chính
}

float Sensor_GetElectricalAngle(void)
{
    return sensor.angle * POLE_PAIRS;
}

float Sensor_GetMechanicalAngle(void)
{
    return sensor.angle;
}

float Sensor_GetVelocity(void)
{
    return sensor.velocity;
}

uint8_t Hall_GetStep(void)
{
    return hall_sensor.step;
}

