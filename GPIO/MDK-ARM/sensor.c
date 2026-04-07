#include "sensor.h"
#include "tim.h"
#include "conf.h"
#include <math.h>

/* Definitions */
HallSensor_t hall_sensor = {0};
Encoder_t encoder = {0};

/* Hall angle table for 60° electrical steps (in radians) */
static const float hall_angle_table[7] = {
    0.0f,
    30.0f * (M_PI / 180.0f),   // step 1: 30°
    90.0f * (M_PI / 180.0f),   // step 2: 90°
    150.0f * (M_PI / 180.0f),  // step 3: 150°
    210.0f * (M_PI / 180.0f),  // step 4: 210°
    270.0f * (M_PI / 180.0f),  // step 5: 270°
    330.0f * (M_PI / 180.0f)   // step 6: 330°
};

static uint8_t Hall_CalculateStep(uint8_t raw) {
    switch (raw) {
        case 0b001: return 5;
        case 0b011: return 4;
        case 0b010: return 3;
        case 0b110: return 2;
        case 0b100: return 1;
        case 0b101: return 6;
        default:    return 0;
    }
}

static void Hall_Update_Internal(void) {
    uint8_t u = HAL_GPIO_ReadPin(HALL_GPIO_PORT, HALL_U_PIN);
    uint8_t v = HAL_GPIO_ReadPin(HALL_GPIO_PORT, HALL_V_PIN);
    uint8_t w = HAL_GPIO_ReadPin(HALL_GPIO_PORT, HALL_W_PIN);
    hall_sensor.raw = (u << 2) | (v << 1) | w;
    hall_sensor.step = Hall_CalculateStep(hall_sensor.raw);
    if (hall_sensor.step != 0 && hall_sensor.step <= 6) {
        hall_sensor.angle_elec = hall_angle_table[hall_sensor.step];
        hall_sensor.angle_elec = fmodf(hall_sensor.angle_elec, M_2PI);
        if (hall_sensor.angle_elec < 0) hall_sensor.angle_elec += M_2PI;
    }
}

void Sensor_Init(void) {
    HAL_TIMEx_HallSensor_Start_IT(&TIM_HALL);
    Hall_Update_Internal();
}

void HallSensor_Update(void) {
    Hall_Update_Internal();
}

float Sensor_GetElectricalAngle(void) {
    return hall_sensor.angle_elec;
}

float Sensor_GetMechanicalAngle(void) {
    return hall_sensor.angle_elec / POLE_PAIRS;
}

uint8_t Hall_GetStep(void) {
    return hall_sensor.step;
}

/* ===================== Encoder implementation ===================== */
void Encoder_Init(void) {
    HAL_TIM_Encoder_Start(&TIM_ENCODER, TIM_CHANNEL_ALL);
    HAL_TIM_IC_Start_IT(&TIM_Z_PULSE, TIM_CHANNEL_1);
    encoder.raw_count = 0;
    encoder.angle_mech = 0.0f;
    encoder.angle_elec = 0.0f;
    encoder.velocity_rads = 0.0f;
    encoder.calibrated = 0;
    encoder.mech_offset = 0.0f;
    encoder.z_pulse_detected = 0;
}

void Encoder_Update(void) {
    int32_t cnt = (int32_t)__HAL_TIM_GET_COUNTER(&TIM_ENCODER);
    encoder.raw_count = cnt;
    float cnt_angle = (float)cnt / ENCODER_COUNTS_PER_REV * M_2PI;
    encoder.angle_mech = cnt_angle + encoder.mech_offset;
    // Normalize to [0, 2p)
    encoder.angle_mech = fmodf(encoder.angle_mech, M_2PI);
    if (encoder.angle_mech < 0) encoder.angle_mech += M_2PI;
    encoder.angle_elec = encoder.angle_mech * POLE_PAIRS;
    encoder.angle_elec = fmodf(encoder.angle_elec, M_2PI);
    if (encoder.angle_elec < 0) encoder.angle_elec += M_2PI;
}

/* Coarse sync using Hall (only before Z calibration) */
void Encoder_SyncWithHall(void) {
    if (encoder.calibrated) return;  // already absolute
    float hall_mech = Sensor_GetMechanicalAngle();
    int32_t cnt = (int32_t)__HAL_TIM_GET_COUNTER(&TIM_ENCODER);
    float cnt_angle = (float)cnt / ENCODER_COUNTS_PER_REV * M_2PI;
    encoder.mech_offset = hall_mech - cnt_angle;
    // Normalize offset
    encoder.mech_offset = fmodf(encoder.mech_offset, M_2PI);
    if (encoder.mech_offset > M_PI) encoder.mech_offset -= M_2PI;
    if (encoder.mech_offset < -M_PI) encoder.mech_offset += M_2PI;
    Encoder_Update();
}

/* Absolute calibration using known mechanical angle at Z pulse */
void Encoder_CalibrateWithZ(void) {
    // Reset encoder counter to 0
    __HAL_TIM_SET_COUNTER(&TIM_ENCODER, 0);
    // Set offset to the known mechanical angle at Z (from datasheet/measurement)
    encoder.mech_offset = MECH_ANGLE_AT_Z_RAD;
    // Normalize offset
    encoder.mech_offset = fmodf(encoder.mech_offset, M_2PI);
    if (encoder.mech_offset < 0) encoder.mech_offset += M_2PI;
    encoder.calibrated = 1;
    encoder.z_pulse_detected = 1;
    Encoder_Update();
}

/* Optional: periodic check to detect drift (uses Hall as reference) */
void Encoder_CheckSyncWithHall(void) {
    if (!encoder.calibrated) {
        // Still not calibrated, keep using Hall for coarse sync
        Encoder_SyncWithHall();
        return;
    }
    float hall_mech = Sensor_GetMechanicalAngle();
    float diff_mech = hall_mech - encoder.angle_mech;
    diff_mech = fmodf(diff_mech, M_2PI);
    if (diff_mech > M_PI) diff_mech -= M_2PI;
    if (diff_mech < -M_PI) diff_mech += M_2PI;
    float diff_elec = diff_mech * POLE_PAIRS;
    if (fabsf(diff_elec) > 30.0f * (M_PI / 180.0f)) {
        // Drift too large, re-sync using Hall (or optionally re-calibrate with Z if possible)
        Encoder_SyncWithHall();
    }
}

float Encoder_GetMechanicalAngle(void) {
    return encoder.angle_mech;
}

float Encoder_GetElectricalAngle(void) {
    return encoder.angle_elec;
}

float Encoder_GetVelocity(void) {
    return encoder.velocity_rads;
}

uint8_t Encoder_IsCalibrated(void) {
    return encoder.calibrated;
}

float GetSmoothElectricalAngle(void) {
    return Encoder_GetElectricalAngle();
}

float GetSmoothMechanicalAngle(void) {
    return Encoder_GetMechanicalAngle();
}