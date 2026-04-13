#ifndef SENSOR_H
#define SENSOR_H

#include "tim.h"
#include "gpio.h"
#include "conf.h"
#include "math.h"
#include <stdint.h>

/* ======================== Hall sensor structure ======================== */
typedef struct {
    uint8_t raw;          // Raw bits (U,V,W)
    uint8_t step;         // Hall step 1..6
    float angle_elec_hall;     // Electrical angle from table (rad)
} HallSensor_t;

/* ======================== Encoder (ABZ) structure ======================== */
typedef struct {
    int64_t raw_count;    // 32-bit signed count (handles overflow)
    float mech_angle;     // Mechanical angle (rad) [0, 2pi)
    float elec_angle;     // Electrical angle (rad) [0, 2pi)
	  float mech_speed;           // Mechanical speed (rad/s)
    float elec_speed;           // Electrical speed (rad/s)
} Encoder_t;


/* ======================== Global instances ======================== */
extern HallSensor_t hall_sensor;
extern Encoder_t    encoder;

/* ======================== Hall functions ======================== */
void Hall_Sensor_Init(void);
void HallSensor_Update(void);
float Sensor_Get_Electrical_Angle_Hall(void);
uint8_t Hall_GetStep(void);

/* ======================== Encoder functions ======================== */
void Encoder_Sensor_Init(void);
void EncoderSensor_Update(void);
float Encoder_Get_Raw_Angle(void);
float Encoder_Get_Electric_Angle(void);
float Encoder_Get_Mechanic_Angle(void);
float Encoder_Get_Mechanic_Speed(void);
float Encoder_Get_Electric_Speed(void);
void Encoder_Reset(void);

#endif /* SENSOR_H */