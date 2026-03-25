/**
  * @file    sensor.h
  * @brief   Combined Hall + Encoder sensor module (SimpleFOC style)
  */

#ifndef SENSOR_H_
#define SENSOR_H_

#include "main.h"
#include "conf.h"
#include "tim.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ===================== Data structures ================================ */

typedef struct {
    float angle_mech;       /* Mechanical angle in [0, 2p) [rad]  */
    bool  index_found;      /* Z pulse detected? */
} Sensor_t;

typedef struct {
    uint8_t step;           /* Current step (1..6) */
    uint8_t raw;            /* Raw (U<<2)|(V<<1)|W */
    float angle_elec;       /* Electrical angle estimated from Hall (for debugging)  */
} HallSensor_t;

typedef struct {
    int64_t counts;         /* Total counts accumulated since start (never reset) – used for velocity */
    int64_t cnt_index;      /* Counts relative to the most recent Z pulse – used for angle */
    int64_t z_counts;       /* Value of 'counts' at the most recent Z pulse */
    int32_t delta;          /* Counts change last period  */
    uint32_t last_cnt;      /* Previous counter value  */
    uint32_t last_z_cnt;    /* Previous Z counter */
    uint32_t z_cnt;         /* Current Z counter  */
} EncoderSensor_t;

/* ===================== Exported variables ============================== */
extern volatile Sensor_t sensor;
extern volatile HallSensor_t hall_sensor;
extern volatile EncoderSensor_t encoder_sensor;

/* ===================== Function prototypes ============================= */
void Sensor_Init(void);
void EncoderSensor_Update(void);
void HallSensor_Update(void);
uint8_t ReadZ(void);

float Sensor_GetElectricalAngle(void);
float Sensor_GetMechanicalAngle(void);
uint8_t Hall_GetStep(void);

#endif /* SENSOR_H_ */