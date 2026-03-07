/**
  * @file    sensor.h
  * @brief   Module Sensor (Hall UVW + Encoder ABZ) theo phong cách SimpleFOC
  *          Dành cho STM32 HAL + PMSM FOC
  */

#ifndef SENSOR_H_
#define SENSOR_H_

#include "main.h"
#include "conf.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* Exported types */
typedef struct {
    float angle;                // Góc co h?c hi?n t?i (rad) - t? sensor
    float velocity;             // V?n t?c góc (rad/s)
    float electrical_angle;     // Góc di?n (rad) = angle * pole_pairs
    bool  index_found;          // Ðã b?t du?c Z/index
} Sensor_t;

/* Hall-specific */
typedef struct {
    uint8_t step;               // 1..6 ho?c 0 (invalid)
    uint8_t raw;                // 0bUVW
} HallSensor_t;

/* Encoder-specific */
typedef struct {
    int64_t counts;             // Tích luy counts (có d?u)
    int32_t delta;              // Delta counts/sample
    uint32_t last_cnt;          // Giá tr? CNT tru?c
    uint32_t last_z_cnt;        // Z counter tru?c
		uint32_t z_cnt ;
} EncoderSensor_t;

/* Exported variables */
extern Sensor_t sensor;                 // Sensor chung (angle, velocity, ...)
extern HallSensor_t hall_sensor;
extern EncoderSensor_t encoder_sensor;

/* Exported functions */
void Sensor_Init(void);                         // Kh?i t?o Hall + Encoder
void EncoderSensor_Update(void);                       // G?i d?nh k? (trong TIM5 callback)
uint8_t ReadZ(void);
void HallSensor_Update(void);
float Sensor_GetElectricalAngle(void);          // Góc di?n (rad) dùng cho FOC
float Sensor_GetMechanicalAngle(void);          // Góc co h?c (rad)
float Sensor_GetVelocity(void);                 // V?n t?c (rad/s)
uint8_t Hall_GetStep(void);                     // L?y bu?c Hall (1..6)

#endif /* SENSOR_H_ */