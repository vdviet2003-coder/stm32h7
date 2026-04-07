#ifndef SENSOR_H_
#define SENSOR_H_

#include "main.h"
#include "conf.h"
#include "tim.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

typedef struct {
    float angle_mech;       // mechanical angle (rad)
} Sensor_t;

typedef struct {
    uint8_t step;
    uint8_t raw;
    float angle_elec;
} HallSensor_t;

extern volatile Sensor_t sensor;
extern volatile HallSensor_t hall_sensor;

void Sensor_Init(void);
void HallSensor_Update(void);

float Sensor_GetElectricalAngle(void);
float Sensor_GetMechanicalAngle(void);
uint8_t Hall_GetStep(void);

#endif