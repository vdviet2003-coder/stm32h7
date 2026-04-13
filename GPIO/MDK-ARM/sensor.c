#include "sensor.h"

/* ------------------- Hall implementation ------------------- */
HallSensor_t hall_sensor = {0};
/* Hall angle table for 60° electrical steps (rad) */
static const float hall_angle_table[7] = {
    0.0f,
    00.0f * (M_PI / 180.0f),   // step 1: 0?
    60.0f * (M_PI / 180.0f),   // step 2: 60?
    120.0f * (M_PI / 180.0f),  // step 3: 120?
    180.0f * (M_PI / 180.0f),  // step 4: 180?
    240.0f * (M_PI / 180.0f),  // step 5: 240?
    300.0f * (M_PI / 180.0f)   // step 6: 300?
};

/* Convert raw 3-bit value to step 1..6 */
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

/* Internal Hall update – reads GPIOs */
static void Hall_Update_Internal(void) {
    uint8_t u = HAL_GPIO_ReadPin(HALL_GPIO_PORT, HALL_U_PIN);
    uint8_t v = HAL_GPIO_ReadPin(HALL_GPIO_PORT, HALL_V_PIN);
    uint8_t w = HAL_GPIO_ReadPin(HALL_GPIO_PORT, HALL_W_PIN);
    hall_sensor.raw = (u << 2) | (v << 1) | w;
    hall_sensor.step = Hall_CalculateStep(hall_sensor.raw);
    if (hall_sensor.step != 0 && hall_sensor.step <= 6) {
        hall_sensor.angle_elec_hall = hall_angle_table[hall_sensor.step];
    }
}

/* Public Hall functions */
void  Hall_Sensor_Init(void) {
    HAL_TIMEx_HallSensor_Start_IT(&TIM_HALL);
    Hall_Update_Internal();
}

void HallSensor_Update(void) {
    Hall_Update_Internal();
}

float Sensor_Get_Electrical_Angle_Hall(void) {
    return hall_sensor.angle_elec_hall;
}

uint8_t Hall_GetStep(void) {
    return hall_sensor.step;
}


/* ------------------- Encoder implementation ------------------- */
Encoder_t encoder = {0};
static int64_t encoder_last_cnt = 0;       // Luu giá tr? CNT l?n tru?c (ép v? 64-bit)
static int64_t encoder_accumulator = 0;    // T?ng s? xung 64-bit
static float encoder_prev_mech_angle = 0.0f;
static int64_t encoder_prev_time_us = 0;


#define ENCODER_32BIT_RANGE     4294967296ULL   // 2^32
#define ENCODER_HALF_RANGE      2147483647LL    // 2^31 - 1
void Encoder_Sensor_Init(void) {
    HAL_TIM_Encoder_Start(&TIM_ENCODER, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&TIM_ENCODER, 0);
    encoder.raw_count = 0;
    encoder.mech_angle = 0.0f;
    encoder.elec_angle = 0.0f;
		encoder.mech_speed = 0.0f;
    encoder.elec_speed = 0.0f;
}
static void Encoder_Update_Internal(void){
    int64_t current_cnt = (int64_t)__HAL_TIM_GET_COUNTER(&TIM_ENCODER);
    int64_t delta = current_cnt - encoder_last_cnt;

    // X? lý tràn 32-bit
    if (delta > ENCODER_HALF_RANGE) {
        delta -= ENCODER_32BIT_RANGE;
    } else if (delta < -ENCODER_HALF_RANGE) {
        delta += ENCODER_32BIT_RANGE;
    }

    encoder_accumulator += delta;
    encoder.raw_count = encoder_accumulator;   	
    encoder_last_cnt = current_cnt;

    // mechanic angle (rad)
    encoder.mech_angle = (float)((double)encoder.raw_count * (2.0 * M_PI) / ENCODER_COUNTS_PER_REV);
    encoder.mech_angle = fmodf(encoder.mech_angle, (float)(2.0 * M_PI));
    if (encoder.mech_angle < 0) encoder.mech_angle += (float)(2.0 * M_PI);

    // electric angle
    encoder.elec_angle = encoder.mech_angle * POLE_PAIRS;
    encoder.elec_angle = fmodf(encoder.elec_angle, (float)(2.0 * M_PI));
    if (encoder.elec_angle < 0) encoder.elec_angle += (float)(2.0 * M_PI);
		
		static float prev_mech_angle = 0.0f;
    static float prev_elec_angle = 0.0f;
    static int first_run = 1;

    float dt = 1.0f / MOTOR_SPEED_CALC_FREQ_FOR_ENCODER;  // dt c? d?nh (vd: 0.001s)

    if (first_run) {
        encoder.mech_speed = 0.0f;
        encoder.elec_speed = 0.0f;
        first_run = 0;
    } else {
        float delta_mech = encoder.mech_angle - prev_mech_angle;
        float delta_elec = encoder.elec_angle - prev_elec_angle;

        // X? lý wrap-around góc t? 2p v? 0
        if (delta_mech > M_PI)   delta_mech -= (float)(2.0 * M_PI);
        if (delta_mech < -M_PI)  delta_mech += (float)(2.0 * M_PI);
        if (delta_elec > M_PI)   delta_elec -= (float)(2.0 * M_PI);
        if (delta_elec < -M_PI)  delta_elec += (float)(2.0 * M_PI);

        encoder.mech_speed = delta_mech / dt;
        encoder.elec_speed = delta_elec / dt;
    }

    prev_mech_angle = encoder.mech_angle;
    prev_elec_angle = encoder.elec_angle;
		
}
void EncoderSensor_Update(void){
	Encoder_Update_Internal();
}
float Encoder_Get_Raw_Angle(void) {
    return encoder.raw_count;
}
float Encoder_Get_Electric_Angle(void) {
    return encoder.elec_angle;
}
float Encoder_Get_Mechanic_Angle(void) {
    return encoder.mech_angle;
}
float Encoder_Get_Mechanic_Speed(void) {
    return encoder.mech_speed;
}

float Encoder_Get_Electric_Speed(void) {
    return encoder.elec_speed;
}
void Encoder_Reset(void) {
    __HAL_TIM_SET_COUNTER(&TIM_ENCODER, 0);
    encoder_accumulator = 0;
    encoder_last_cnt = 0;
    encoder.raw_count = 0;
    encoder.mech_angle = 0.0f;
    encoder.elec_angle = 0.0f;
    encoder.mech_speed = 0.0f;
    encoder.elec_speed = 0.0f;
}















