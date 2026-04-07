#ifndef SENSOR_H
#define SENSOR_H

#include "tim.h"
#include "gpio.h"
#include "conf.h"
#include <stdint.h>

/* Hall structures */
typedef struct {
    uint8_t raw;
    uint8_t step;
    float angle_elec;
} HallSensor_t;

/* Encoder structure */
typedef struct {
    volatile int32_t raw_count;
    volatile float angle_mech;
    volatile float angle_elec;
    volatile float velocity_rads;
    volatile uint8_t calibrated;   // 1 if absolute position known (Z seen)
    volatile float mech_offset;     // mechanical angle offset (rad)
    volatile uint8_t z_pulse_detected;
} Encoder_t;

/* Extern declarations */
extern HallSensor_t hall_sensor;
extern Encoder_t encoder;

/* Hall functions */
void Sensor_Init(void);
void HallSensor_Update(void);
float Sensor_GetElectricalAngle(void);
float Sensor_GetMechanicalAngle(void);
uint8_t Hall_GetStep(void);

/* Encoder functions */
void Encoder_Init(void);
void Encoder_Update(void);
void Encoder_SyncWithHall(void);        // coarse sync using Hall (before Z)
void Encoder_CalibrateWithZ(void);      // absolute calibration using MECH_ANGLE_AT_Z_RAD
float Encoder_GetMechanicalAngle(void);
float Encoder_GetElectricalAngle(void);
float Encoder_GetVelocity(void);
uint8_t Encoder_IsCalibrated(void);
void Encoder_CheckSyncWithHall(void);   // optional periodic check

/* Unified smooth angle (always from encoder) */
float GetSmoothElectricalAngle(void);
float GetSmoothMechanicalAngle(void);

#endif