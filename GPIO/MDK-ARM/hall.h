/**
 * @file    hall.h
 * @brief   Hall sensor commutation support (6-step trapezoidal)
 * @date    2026
 */

#ifndef HALL_H_
#define HALL_H_
#include <stdbool.h>   
#include "stm32h7xx_hal.h"
#include <stdint.h>       
// Public / exported variables
extern uint8_t s_hall;                  // current Hall sector (1..6)
extern volatile bool is_z_aligned;      // from encoder module

// Public functions
void     Hall_Init(TIM_HandleTypeDef *htim);
uint8_t  Hall_GetState(void);
void     Hall_Commutate(uint8_t step);
uint8_t  Hall_GetSector(void);

#endif /* HALL_H_ */