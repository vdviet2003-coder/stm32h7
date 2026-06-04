#include "sensor.h"

/* ------------------- Hall sensor (gi? nguyên) ------------------- */
HallSensor_t hall_sensor = {0};

static const float hall_angle_table[7] = {
    0.0f,
    0.0f * (M_PI / 180.0f),
    60.0f * (M_PI / 180.0f),
    120.0f * (M_PI / 180.0f),
    180.0f * (M_PI / 180.0f),
    240.0f * (M_PI / 180.0f),
    300.0f * (M_PI / 180.0f)
};

static uint8_t Hall_CalculateStep(uint8_t raw) {
    switch (raw) {
        case 0b101: return 1;
        case 0b100: return 2;
        case 0b110: return 3;
        case 0b010: return 4;
        case 0b011: return 5;
        case 0b001: return 6;
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
        hall_sensor.angle_elec_hall = hall_angle_table[hall_sensor.step];
    }
}

void Hall_Sensor_Init(void) {
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

/* ------------------- Encoder implementation (TIM2 encoder, TIM3 gate, TIM5 capture) ------------------- */
Encoder_t encoder = {0};

static int64_t encoder_last_cnt = 0;
static int64_t encoder_accumulator = 0;
static int64_t last_count_M = 0;

static float rpm_M = 0.0f;
static float rpm_T = 0.0f;

static SpeedMethod_t current_method = SPEED_METHOD_AUTO;

/* Bi?n cho T-method (TIM5 capture) */
static uint32_t last_capture = 0;
static uint32_t capture_period = 0;
static uint8_t  capture_valid = 0;

/* H? tr? d?u cho T-method: luu hu?ng g?n nh?t */
static int8_t  direction_sign = 1;   // +1 ho?c -1

#define M_THRESHOLD     100
#define T_THRESHOLD     10
#define FT_FREQ_HZ      1000000.0f

void Encoder_Sensor_Init(void) {
    HAL_TIM_Encoder_Start(&TIM_ENCODER, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&TIM_ENCODER, 0);

    HAL_TIM_Base_Start_IT(&htim3);           // gate 1ms
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_3); // capture

    encoder.raw_count = 0;
    encoder.mech_angle = 0.0f;
    encoder.elec_angle = 0.0f;
    encoder.mech_speed = 0.0f;
    encoder.elec_speed = 0.0f;
    encoder.rpm = 0.0f;

    encoder_last_cnt = 0;
    encoder_accumulator = 0;
    last_count_M = 0;
    last_capture = 0;
    capture_period = 0;
    capture_valid = 0;
    direction_sign = 1;
}
static inline float Normalize_Angle(float angle)
{
    while (angle >= TWO_PI)
        angle -= TWO_PI;

    while (angle < 0.0f)
        angle += TWO_PI;

    return angle;
}
static void Encoder_Update_Position(void) {
    int64_t current_cnt = (int64_t)__HAL_TIM_GET_COUNTER(&TIM_ENCODER);
    int64_t delta = current_cnt - encoder_last_cnt;

    if (delta > 2147483647LL) {
        delta -= 4294967296ULL;
    } else if (delta < -2147483647LL) {
        delta += 4294967296ULL;
    }

    encoder_accumulator += delta;
    encoder.raw_count = encoder_accumulator;
    encoder_last_cnt = current_cnt;

    encoder.mech_angle = (float)((double)encoder.raw_count * (2.0 * M_PI) / ENCODER_COUNTS_PER_REV);
    encoder.mech_angle = fmodf(encoder.mech_angle, (float)(2.0 * M_PI));
    if (encoder.mech_angle < 0) encoder.mech_angle += (float)(2.0 * M_PI);
		//encoder.mech_angle = Normalize_Angle(encoder.mech_angle);
		
		
    encoder.elec_angle = encoder.mech_angle * POLE_PAIRS;
    encoder.elec_angle = fmodf(encoder.elec_angle, (float)(2.0 * M_PI));
    if (encoder.elec_angle < 0) encoder.elec_angle += (float)(2.0 * M_PI);
		//encoder.elec_angle = Normalize_Angle(encoder.elec_angle);
}


static float Calculate_RPM_M(int64_t delta_count, float gate_time_s) {
    if (gate_time_s <= 0.0f) return 0.0f;
    return (delta_count * 60.0f) / (ENCODER_COUNTS_PER_REV * gate_time_s);
}

static float Calculate_RPM_T(uint32_t period_us) {
    if (period_us == 0) return 0.0f;
    return (60.0f * FT_FREQ_HZ) / (2500 * (float)period_us);
}

void EncoderSensor_Update(void) {
    Encoder_Update_Position();

    const float Ts = 1.0f / MOTOR_SPEED_CALC_FREQ_FOR_ENCODER;
    int64_t delta_M = encoder.raw_count - last_count_M;
    last_count_M = encoder.raw_count;

    // Xác d?nh hu?ng t? delta_M (dùng cho T-method)
    if (delta_M > 0) direction_sign = 1;
    else if (delta_M < 0) direction_sign = -1;

    // M-method (có d?u)
    rpm_M = Calculate_RPM_M(delta_M, Ts);

    // T-method (luôn duong, sau dó gán d?u)
    uint32_t period = capture_period;
    if (capture_valid) {
        float abs_rpm_T = Calculate_RPM_T(period);
        rpm_T = direction_sign * abs_rpm_T;   // thêm d?u
        capture_valid = 0;
    }

    // L?a ch?n phuong pháp
    float selected_rpm = 0.0f;
    int64_t abs_delta = (delta_M >= 0) ? delta_M : -delta_M;

    switch (current_method) {
        case SPEED_METHOD_M:   selected_rpm = rpm_M; break;
        case SPEED_METHOD_T:   selected_rpm = rpm_T; break;
        case SPEED_METHOD_AUTO:
        default:
            if (abs_delta >= M_THRESHOLD)      selected_rpm = rpm_M;
            else if (abs_delta <= T_THRESHOLD) selected_rpm = rpm_T;
            else                               selected_rpm = (rpm_M + rpm_T) / 2.0f;
            break;
    }

    encoder.rpm = selected_rpm;
    encoder.mech_speed = selected_rpm * (2.0f * M_PI) / 60.0f;
    encoder.elec_speed = encoder.mech_speed * POLE_PAIRS;
}

void Encoder_Capture_Handler(uint32_t capture_value) {
    if (last_capture != 0) {
        uint32_t diff;
        if (capture_value >= last_capture) {
            diff = capture_value - last_capture;
        } else {
            diff = (0xFFFFFFFF - last_capture) + capture_value + 1;
        }
        capture_period = diff;
        capture_valid = 1;
    }
    last_capture = capture_value;
}

float Encoder_Get_Raw_Angle(void) { return (float)encoder.raw_count; }
float Encoder_Get_Electric_Angle(void) { return encoder.elec_angle; }
float Encoder_Get_Mechanic_Angle(void) { return encoder.mech_angle; }
float Encoder_Get_Mechanic_Speed(void) { return encoder.mech_speed; }
float Encoder_Get_Electric_Speed(void) { return encoder.elec_speed; }
float Encoder_Get_RPM(void) { return encoder.rpm; }

void Encoder_Reset(void) {
    __HAL_TIM_SET_COUNTER(&TIM_ENCODER, 0);
    encoder_accumulator = 0;
    encoder_last_cnt = 0;
    encoder.raw_count = 0;
    encoder.mech_angle = 0.0f;
    encoder.elec_angle = 0.0f;
    encoder.mech_speed = 0.0f;
    encoder.elec_speed = 0.0f;
    encoder.rpm = 0.0f;
    last_count_M = 0;
    last_capture = 0;
    capture_period = 0;
    capture_valid = 0;
    direction_sign = 1;
}

void Encoder_SetSpeedMethod(SpeedMethod_t method) { current_method = method; }
SpeedMethod_t Encoder_GetSpeedMethod(void) { return current_method; }