#include "sensor.h"

volatile Sensor_t sensor = {0};
volatile HallSensor_t hall_sensor = {0};

static const float hall_angle_table[7] = {
    0.0f,
    30.0f * (M_PI / 180.0f),
    90.0f * (M_PI / 180.0f),
    150.0f * (M_PI / 180.0f),
    210.0f * (M_PI / 180.0f),
    270.0f * (M_PI / 180.0f),
    330.0f * (M_PI / 180.0f)
};

static uint8_t Hall_CalculateStep(uint8_t raw)
{
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

static void Hall_Update_Internal(void)
{
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

    // Mechanical angle = electrical angle / pole_pairs
    sensor.angle_mech = hall_sensor.angle_elec / POLE_PAIRS;
    sensor.angle_mech = fmodf(sensor.angle_mech, M_2PI);
    if (sensor.angle_mech < 0) sensor.angle_mech += M_2PI;
}

void Sensor_Init(void)
{
    HAL_TIMEx_HallSensor_Start_IT(&TIM_HALL);
    Hall_Update_Internal();
}

void HallSensor_Update(void)
{
    Hall_Update_Internal();
}

float Sensor_GetElectricalAngle(void)
{
    float elec = sensor.angle_mech * POLE_PAIRS;
    elec = fmodf(elec, M_2PI);
    if (elec < 0) elec += M_2PI;
    return elec;
}

float Sensor_GetMechanicalAngle(void)
{
    return sensor.angle_mech;
}

uint8_t Hall_GetStep(void)
{
    return hall_sensor.step;
}