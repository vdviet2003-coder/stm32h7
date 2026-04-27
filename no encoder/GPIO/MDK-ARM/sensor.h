#ifndef SENSOR_H
#define SENSOR_H

#include "tim.h"
#include "gpio.h"
#include "conf.h"
#include "math.h"
#include <stdint.h>

/* ======================== Hall sensor structure ======================== */
typedef struct {
    uint8_t raw;
    uint8_t step;
    float angle_elec_hall;
} HallSensor_t;

/* ======================== Encoder (ABZ) structure ======================== */
typedef struct {
    int64_t raw_count;
    float mech_angle;
    float elec_angle;
    float mech_speed;
    float elec_speed;
    float rpm;
} Encoder_t;

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
float Encoder_Get_RPM(void);

void Encoder_Reset(void);

/* ======================== Speed method selection ======================== */
typedef enum {
    SPEED_METHOD_M  = 0,
    SPEED_METHOD_T  = 1,
    SPEED_METHOD_MT = 2,
    SPEED_METHOD_AUTO = 3
} SpeedMethod_t;

void Encoder_SetSpeedMethod(SpeedMethod_t method);
SpeedMethod_t Encoder_GetSpeedMethod(void);

/* Hàm x? lý s? ki?n capture – du?c g?i t? main.c */
void Encoder_Capture_Handler(uint32_t capture_value);

#endif /* SENSOR_H */