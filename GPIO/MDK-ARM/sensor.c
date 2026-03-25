#include "sensor.h"

volatile Sensor_t sensor = {0};
volatile HallSensor_t hall_sensor = {0};
volatile EncoderSensor_t encoder_sensor = {0};

/* ===================== Private constants =============================== */

/**
 * @brief  B?ng tra góc di?n (rad) cho t?ng Hall step (1..6).
 *         Giá tr? có th? di?u ch?nh theo do d?c th?c t?.
 *         Step 1 = 0°, step 2 = 60°, step 3 = 120°, step 4 = 180°, step 5 = 270°, step 6 = 300°.
 */
static const float hall_angle_table[7] = {
    0.0f,                           // index 0 không dùng
    30.0f   * (M_PI / 180.0f),       // step 1: 
		90.0f  * (M_PI / 180.0f),       // step 2: 
    150.0f * (M_PI / 180.0f),       // step 3: 
    210.0f * (M_PI / 180.0f),       // step 4: 
    270.0f * (M_PI / 180.0f),       // step 5: 
    330.0f * (M_PI / 180.0f)        // step 6: 
};

/* ===================== Private functions =============================== */

/**
 * @brief  Convert raw UVW to step number (1..6). Adjust mapping if needed.
 */
static uint8_t Hall_CalculateStep(uint8_t raw)
{
    // Typical mapping for 120° Hall sensors. Step 1 = 100, step 2 = 110, step 3 = 010,
    // step 4 = 011, step 5 = 001, step 6 = 101.
    switch (raw)
    {
        case 0b001: return 5;   // (U=0,V=0,W=1)
        case 0b011: return 4;   // (0,1,1)
        case 0b010: return 3;   // (0,1,0)
        case 0b110: return 2;   // (1,1,0)
        case 0b100: return 1;   // (1,0,0)
        case 0b101: return 6;   // (1,0,1)
        default:    return 0;   // invalid
    }
}

/**
 * @brief  Internal Hall update: read pins, compute step, estimate electrical angle.
 *         Called from HallSensor_Update() (interrupt context).
 */
static void Hall_Update_Internal(void)
{
    // Read Hall sensor pins
    uint8_t u = HAL_GPIO_ReadPin(HALL_GPIO_PORT, HALL_U_PIN);
    uint8_t v = HAL_GPIO_ReadPin(HALL_GPIO_PORT, HALL_V_PIN);
    uint8_t w = HAL_GPIO_ReadPin(HALL_GPIO_PORT, HALL_W_PIN);

    // Combine into a 3-bit raw value
    hall_sensor.raw = (u << 2) | (v << 1) | w;

    // Convert raw to step number
    hall_sensor.step = Hall_CalculateStep(hall_sensor.raw);

    if (hall_sensor.step != 0 && hall_sensor.step <= 6)
    {
        // L?y góc di?n t? b?ng tra
        hall_sensor.angle_elec = hall_angle_table[hall_sensor.step];

        // Wrap to [0, 2p) to avoid numerical issues
        hall_sensor.angle_elec = fmodf(hall_sensor.angle_elec, M_2PI);
        if (hall_sensor.angle_elec < 0)
            hall_sensor.angle_elec += M_2PI;
    }

    // Only use Hall angle if encoder index not yet found (startup phase)
    if (!sensor.index_found && hall_sensor.step != 0)
    {
        // Convert electrical angle to mechanical angle
        sensor.angle_mech = hall_sensor.angle_elec / POLE_PAIRS;

        // Wrap mechanical angle to [0, 2p)
        sensor.angle_mech = fmodf(sensor.angle_mech, M_2PI);
        if (sensor.angle_mech < 0)
            sensor.angle_mech += M_2PI;
    }
}

/**
 * @brief  Internal encoder update: read counter, compute delta, accumulate,
 *         calculate velocity, mechanical angle, and cnt_index.
 *         Called periodically from EncoderSensor_Update().
 */
static void Encoder_Update_Internal(void)
{
    uint32_t cnt = __HAL_TIM_GET_COUNTER(&TIM_ENCODER);
    int32_t delta = (int32_t)(cnt - encoder_sensor.last_cnt);

    // Handle counter overflow/underflow (for 16-bit or 32-bit timers)
    int32_t period_half = (int32_t)(TIM_ENCODER.Init.Period / 2);
    if (delta > period_half)
        delta -= (TIM_ENCODER.Init.Period + 1);
    else if (delta < -period_half)
        delta += (TIM_ENCODER.Init.Period + 1);

    encoder_sensor.delta = delta;
    encoder_sensor.counts += delta;          // Total counts (for velocity)
    encoder_sensor.last_cnt = cnt;

    // Read Z pulse counter
    encoder_sensor.z_cnt = __HAL_TIM_GET_COUNTER(&TIM_Z);

    // Detect Z pulse (index) – update reference every time Z occurs
    if (encoder_sensor.z_cnt != encoder_sensor.last_z_cnt)
    {
        // Store the total counts at the moment of Z
        encoder_sensor.z_counts = encoder_sensor.counts;
        sensor.index_found = true;   // mark that we have a valid index
    }
    encoder_sensor.last_z_cnt = encoder_sensor.z_cnt;

    // Compute cnt_index (relative counts from most recent Z)
    encoder_sensor.cnt_index = encoder_sensor.counts - encoder_sensor.z_counts;

    // Calculate mechanical angle using cnt_index (relative position)
    if (sensor.index_found)
    {
        // Modulo to get position within one mechanical revolution
        int64_t mod = encoder_sensor.cnt_index % (int64_t)ENCODER_COUNTS_PER_REV;
        if (mod < 0) mod += (int64_t)ENCODER_COUNTS_PER_REV;

        // Mechanical angle from relative counts (0 to 2p)
        float mech_angle = ((float)mod / ENCODER_COUNTS_PER_REV) * M_2PI;

        // Add Z offset to align with d-axis
        sensor.angle_mech = mech_angle + Z_PULSE_OFFSET_RAD;

        // Wrap to [0, 2pi)
        sensor.angle_mech = fmodf(sensor.angle_mech, M_2PI);
        if (sensor.angle_mech < 0)
            sensor.angle_mech += M_2PI;
    }
}

/* ===================== Public functions ================================ */

void Sensor_Init(void)
{
    // Start peripherals
    HAL_TIMEx_HallSensor_Start_IT(&TIM_HALL);
    HAL_TIM_Encoder_Start(&TIM_ENCODER, TIM_CHANNEL_ALL);
    HAL_TIM_Base_Start(&TIM_Z);
    HAL_TIM_Base_Start_IT(&TIM_SENSOR_SAMPLE);

    // Initial Hall read
    Hall_Update_Internal();

    // Reset encoder state
    encoder_sensor.counts = 0;
    encoder_sensor.cnt_index = 0;
    encoder_sensor.z_counts = 0;
    encoder_sensor.last_cnt = __HAL_TIM_GET_COUNTER(&TIM_ENCODER);
    encoder_sensor.last_z_cnt = __HAL_TIM_GET_COUNTER(&TIM_Z);
    sensor.index_found = false;
    sensor.angle_mech = 0.0f;
}

void EncoderSensor_Update(void)
{
    Encoder_Update_Internal();
}

void HallSensor_Update(void)
{
    Hall_Update_Internal();
}

uint8_t ReadZ(void)
{
    return encoder_sensor.z_cnt;
}

float Sensor_GetElectricalAngle(void)
{
    // Electrical angle = mechanical angle * pole_pairs, then wrap to [0, 2p)
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