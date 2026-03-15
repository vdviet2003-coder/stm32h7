/**
  * @file    sensor.h
  * @brief   Combined Hall + Encoder sensor module (SimpleFOC style)
  */

#ifndef SENSOR_H_
#define SENSOR_H_

#include "main.h"
#include "conf.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ===================== Data structures ================================ */

/**
 * @brief  Common sensor data (mechanical angle, velocity)
 */
typedef struct {
    float angle_mech;       /* Mechanical angle in [0, 2p) [rad]  */
    float velocity;         /* Angular velocity [rad/s]  */
    bool  index_found;      /* Z pulse detected? */
} Sensor_t;

/**
 * @brief  Hall-specific data
 */
typedef struct {
    uint8_t step;           /* Current step (1..6) */
    uint8_t raw;            /* Raw (U<<2)|(V<<1)|W */
    float angle_elec;       /*!< Electrical angle estimated from Hall (for debugging)  */
} HallSensor_t;

/**
 * @brief  Encoder-specific data
 */
typedef struct {
    int64_t counts;         /* Accumulated counts (signed) – */
    int64_t z_counts;       /* Counts value at the moment of Z pulse (only used after index_found) */
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

/* [0, 2pi) – dùng cho FOC */
float Sensor_GetElectricalAngle(void);

/* Góc co trong m?t vòng [0, 2pi) */
float Sensor_GetMechanicalAngle(void);

/* V?n t?c góc (rad/s) */
float Sensor_GetVelocity(void);

/* V? trí tuy?t d?i (rad), không wrap – có th? l?n hon 2p */
float Sensor_GetAbsolutePosition(void);

/* Bu?c Hall hi?n t?i (1..6) */
uint8_t Hall_GetStep(void);

/* Debug functions – Hàm debug (có th? d?nh nghia sau) */
void Sensor_PrintHallStatus(void);
void Sensor_PrintEncoderStatus(void);

#endif /* SENSOR_H_ */