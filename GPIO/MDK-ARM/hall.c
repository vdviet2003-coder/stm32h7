/**
 * @file    hall.c
 * @brief   Hall sensor processing (GPIO based reading)
 */

#include "hall.h"

// ------------------------------------------------
uint8_t s_hall = 0;
static uint8_t raw_state = 0;

// ------------------------------------------------
static uint8_t read_hall_raw(void)
{
    uint8_t hall_u = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12);
    uint8_t hall_v = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13);
    uint8_t hall_w = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14);

    raw_state = (hall_u << 2) | (hall_v << 1) | hall_w;

    switch (raw_state)
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

// ------------------------------------------------
void Hall_Init(TIM_HandleTypeDef *htim)
{
    __HAL_TIM_ENABLE_IT(htim, TIM_IT_TRIGGER);
    __HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);
    HAL_TIMEx_HallSensor_Start(htim);
}

// ------------------------------------------------
uint8_t Hall_GetState(void)
{
    return read_hall_raw();
}

uint8_t Hall_GetSector(void)
{
    s_hall = read_hall_raw();
    return s_hall;
}

// ------------------------------------------------
void Hall_Commutate(uint8_t step)
{
    // Only use Hall for angle forcing before index (Z) is found
    if (is_z_aligned) return;

    extern float Angle;     // from encoder module

    switch (step)
    {
        case 1:  Angle =  30.0f; break;
        case 2:  Angle =  90.0f; break;
        case 3:  Angle = 150.0f; break;
        case 4:  Angle = 210.0f; break;
        case 5:  Angle = 270.0f; break;
        case 6:  Angle = 330.0f; break;
        default: Angle =   0.0f; break;
    }

    s_hall = step;
}