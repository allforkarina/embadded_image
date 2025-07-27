/**
 * @file adc_sensors.c
 * @brief ADC sensor reading module implementation for TI M0G3507
 * @author TI M0G3507 Safety System
 * @date 2024
 */

#include "adc_sensors.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Static variables
static adc_reading_t adc_readings[ADC_SENSOR_COUNT];
static adc_calibration_t adc_calibrations[ADC_SENSOR_COUNT];
static adc_filter_t adc_filters[ADC_SENSOR_COUNT];
static angle_sensor_data_t current_angle_data;
static gas_sensor_data_t current_gas_data;
static uint32_t adc_error_count = 0;
static bool adc_initialized = false;

// Hardware abstraction layer functions (to be implemented with TI drivers)
static bool hal_adc_init(void);
static void hal_adc_deinit(void);
static bool hal_adc_start_conversion(uint8_t channel);
static bool hal_adc_read_value(uint8_t channel, uint16_t* value);
static bool hal_adc_is_conversion_complete(uint8_t channel);

// Internal helper functions
static void adc_convert_to_voltage(adc_sensor_type_t sensor);
static void adc_process_angle_sensor(void);
static void adc_process_gas_sensor(void);
static float adc_calculate_gas_ppm(float sensor_voltage);
static float adc_calculate_gas_resistance(float sensor_voltage);

/**
 * @brief Initialize ADC sensors module
 * @return true if initialization successful, false otherwise
 */
bool adc_sensors_init(void)
{
    // Initialize hardware ADC
    if (!hal_adc_init()) {
        adc_error_count++;
        return false;
    }
    
    // Initialize readings structure
    memset(adc_readings, 0, sizeof(adc_readings));
    
    // Initialize calibration data with defaults
    for (int i = 0; i < ADC_SENSOR_COUNT; i++) {
        adc_calibrations[i].offset = 0.0f;
        adc_calibrations[i].gain = 1.0f;
        adc_calibrations[i].zero_point = 0;
        adc_calibrations[i].full_scale = ADC_MAX_VALUE;
        adc_calibrations[i].calibrated = false;
        
        // Initialize filters
        adc_filter_init(&adc_filters[i]);
    }
    
    // Initialize angle sensor data
    memset(&current_angle_data, 0, sizeof(current_angle_data));
    current_angle_data.scale_factor = 360.0f / ADC_MAX_VALUE; // Default scale
    
    // Initialize gas sensor data
    memset(&current_gas_data, 0, sizeof(current_gas_data));
    current_gas_data.r0_calibration = GAS_CALIBRATION_R0;
    
    adc_initialized = true;
    return true;
}

/**
 * @brief Deinitialize ADC sensors module
 */
void adc_sensors_deinit(void)
{
    hal_adc_deinit();
    adc_initialized = false;
}

/**
 * @brief Start ADC conversion for specified sensor
 * @param sensor Sensor type to convert
 * @return true if conversion started successfully, false otherwise
 */
bool adc_sensors_start_conversion(adc_sensor_type_t sensor)
{
    if (!adc_initialized || sensor >= ADC_SENSOR_COUNT) {
        return false;
    }
    
    uint8_t channel = (sensor == ADC_SENSOR_ANGLE) ? ADC_CHANNEL_ANGLE : ADC_CHANNEL_GAS;
    return hal_adc_start_conversion(channel);
}

/**
 * @brief Read raw ADC value for specified sensor
 * @param sensor Sensor type to read
 * @param raw_value Pointer to store raw ADC value
 * @return true if read successful, false otherwise
 */
bool adc_sensors_read_raw(adc_sensor_type_t sensor, uint16_t* raw_value)
{
    if (!adc_initialized || sensor >= ADC_SENSOR_COUNT || raw_value == NULL) {
        return false;
    }
    
    uint8_t channel = (sensor == ADC_SENSOR_ANGLE) ? ADC_CHANNEL_ANGLE : ADC_CHANNEL_GAS;
    
    if (hal_adc_read_value(channel, raw_value)) {
        // Apply filtering
        *raw_value = adc_filter_add_sample(&adc_filters[sensor], *raw_value);
        
        // Store in readings structure
        adc_readings[sensor].raw_value = *raw_value;
        adc_readings[sensor].timestamp = g_system_status.uptime_seconds;
        adc_readings[sensor].valid = true;
        
        // Convert to voltage
        adc_convert_to_voltage(sensor);
        
        return true;
    }
    
    adc_error_count++;
    return false;
}

/**
 * @brief Read voltage value for specified sensor
 * @param sensor Sensor type to read
 * @param voltage_mv Pointer to store voltage in millivolts
 * @return true if read successful, false otherwise
 */
bool adc_sensors_read_voltage(adc_sensor_type_t sensor, float* voltage_mv)
{
    if (!adc_initialized || sensor >= ADC_SENSOR_COUNT || voltage_mv == NULL) {
        return false;
    }
    
    if (adc_readings[sensor].valid) {
        *voltage_mv = adc_readings[sensor].voltage_mv;
        return true;
    }
    
    return false;
}

/**
 * @brief Get processed angle sensor data
 * @param angle_data Pointer to store angle data
 * @return true if data available, false otherwise
 */
bool adc_sensors_get_angle_data(angle_sensor_data_t* angle_data)
{
    if (angle_data == NULL) {
        return false;
    }
    
    memcpy(angle_data, &current_angle_data, sizeof(angle_sensor_data_t));
    return adc_readings[ADC_SENSOR_ANGLE].valid;
}

/**
 * @brief Get processed gas sensor data
 * @param gas_data Pointer to store gas data
 * @return true if data available, false otherwise
 */
bool adc_sensors_get_gas_data(gas_sensor_data_t* gas_data)
{
    if (gas_data == NULL) {
        return false;
    }
    
    memcpy(gas_data, &current_gas_data, sizeof(gas_sensor_data_t));
    return adc_readings[ADC_SENSOR_GAS].valid;
}

/**
 * @brief Process ADC sensor readings
 */
void adc_sensors_process(void)
{
    if (!adc_initialized) {
        return;
    }
    
    // Start conversions for both sensors
    adc_sensors_start_conversion(ADC_SENSOR_ANGLE);
    adc_sensors_start_conversion(ADC_SENSOR_GAS);
    
    // Process angle sensor if data available
    uint16_t raw_value;
    if (adc_sensors_read_raw(ADC_SENSOR_ANGLE, &raw_value)) {
        adc_process_angle_sensor();
    }
    
    // Process gas sensor if data available
    if (adc_sensors_read_raw(ADC_SENSOR_GAS, &raw_value)) {
        adc_process_gas_sensor();
    }
}

/**
 * @brief Calibrate angle sensor with known angle
 * @param known_angle_deg Known angle in degrees for calibration
 * @return true if calibration successful, false otherwise
 */
bool adc_sensors_calibrate_angle(float known_angle_deg)
{
    if (!adc_initialized || known_angle_deg < ANGLE_MIN_DEG || known_angle_deg > ANGLE_MAX_DEG) {
        return false;
    }
    
    uint16_t raw_value;
    if (adc_sensors_read_raw(ADC_SENSOR_ANGLE, &raw_value)) {
        // Calculate offset and scale factor
        current_angle_data.offset_deg = known_angle_deg - (raw_value * current_angle_data.scale_factor);
        current_angle_data.calibrated = true;
        adc_calibrations[ADC_SENSOR_ANGLE].calibrated = true;
        
        return true;
    }
    
    return false;
}

/**
 * @brief Calibrate gas sensor (R0 calibration in clean air)
 * @return true if calibration successful, false otherwise
 */
bool adc_sensors_calibrate_gas(void)
{
    if (!adc_initialized) {
        return false;
    }
    
    uint16_t raw_value;
    if (adc_sensors_read_raw(ADC_SENSOR_GAS, &raw_value)) {
        float voltage = adc_readings[ADC_SENSOR_GAS].voltage_mv;
        float resistance = adc_calculate_gas_resistance(voltage);
        
        // Set R0 calibration value (resistance in clean air)
        current_gas_data.r0_calibration = resistance;
        adc_calibrations[ADC_SENSOR_GAS].calibrated = true;
        
        return true;
    }
    
    return false;
}

/**
 * @brief Reset calibration for specified sensor
 * @param sensor Sensor type to reset calibration
 */
void adc_sensors_reset_calibration(adc_sensor_type_t sensor)
{
    if (sensor >= ADC_SENSOR_COUNT) {
        return;
    }
    
    adc_calibrations[sensor].calibrated = false;
    
    if (sensor == ADC_SENSOR_ANGLE) {
        current_angle_data.calibrated = false;
        current_angle_data.offset_deg = 0.0f;
        current_angle_data.scale_factor = 360.0f / ADC_MAX_VALUE;
    } else if (sensor == ADC_SENSOR_GAS) {
        current_gas_data.r0_calibration = GAS_CALIBRATION_R0;
    }
}

/**
 * @brief Check if ADC sensors are ready
 * @return true if ready, false otherwise
 */
bool adc_sensors_is_ready(void)
{
    return adc_initialized;
}

/**
 * @brief Get ADC sensor error count
 * @return Number of ADC errors
 */
uint32_t adc_sensors_get_error_count(void)
{
    return adc_error_count;
}

/**
 * @brief Reset ADC sensor error count
 */
void adc_sensors_reset_errors(void)
{
    adc_error_count = 0;
}

// Internal helper functions

/**
 * @brief Convert raw ADC value to voltage
 * @param sensor Sensor type to convert
 */
static void adc_convert_to_voltage(adc_sensor_type_t sensor)
{
    if (sensor >= ADC_SENSOR_COUNT) {
        return;
    }
    
    uint16_t raw = adc_readings[sensor].raw_value;
    float voltage = (float)raw * ADC_VREF_MV / ADC_MAX_VALUE;
    
    // Apply calibration
    voltage = voltage * adc_calibrations[sensor].gain + adc_calibrations[sensor].offset;
    
    adc_readings[sensor].voltage_mv = voltage;
}

/**
 * @brief Process angle sensor data
 */
static void adc_process_angle_sensor(void)
{
    float raw_angle = adc_readings[ADC_SENSOR_ANGLE].raw_value * current_angle_data.scale_factor;
    
    // Apply calibration offset
    current_angle_data.angle_deg = raw_angle + current_angle_data.offset_deg;
    
    // Normalize to 0-360 degrees
    while (current_angle_data.angle_deg < 0) {
        current_angle_data.angle_deg += 360.0f;
    }
    while (current_angle_data.angle_deg >= 360.0f) {
        current_angle_data.angle_deg -= 360.0f;
    }
    
    // Convert to radians
    current_angle_data.angle_rad = current_angle_data.angle_deg * M_PI / 180.0f;
}

/**
 * @brief Process gas sensor data
 */
static void adc_process_gas_sensor(void)
{
    float voltage = adc_readings[ADC_SENSOR_GAS].voltage_mv;
    
    // Calculate sensor resistance
    current_gas_data.resistance_ohm = adc_calculate_gas_resistance(voltage);
    
    // Calculate Rs/R0 ratio
    current_gas_data.ratio = current_gas_data.resistance_ohm / current_gas_data.r0_calibration;
    
    // Calculate PPM based on MQ-2 characteristic curve
    current_gas_data.ppm = (uint16_t)adc_calculate_gas_ppm(voltage);
    
    // Check alarm threshold
    current_gas_data.alarm_state = (current_gas_data.ppm > GAS_THRESHOLD_PPM);
}

/**
 * @brief Calculate gas concentration in PPM
 * @param sensor_voltage Sensor voltage in millivolts
 * @return Gas concentration in PPM
 */
static float adc_calculate_gas_ppm(float sensor_voltage)
{
    // MQ-2 sensor characteristic equation (approximation)
    // PPM = A * (Rs/R0)^B where A and B are constants derived from datasheet
    float resistance = adc_calculate_gas_resistance(sensor_voltage);
    float ratio = resistance / current_gas_data.r0_calibration;
    
    // Simplified calculation based on MQ-2 datasheet
    // These constants should be calibrated for specific sensor
    const float A = 613.9f;
    const float B = -2.074f;
    
    if (ratio > 0) {
        return A * powf(ratio, B);
    }
    
    return 0.0f;
}

/**
 * @brief Calculate gas sensor resistance
 * @param sensor_voltage Sensor voltage in millivolts
 * @return Sensor resistance in ohms
 */
static float adc_calculate_gas_resistance(float sensor_voltage)
{
    // Calculate Rs using voltage divider formula
    // Rs = (Vc - Vs) * RL / Vs
    const float supply_voltage = ADC_VREF_MV;
    const float load_resistance = 10000.0f; // 10k ohm load resistor
    
    if (sensor_voltage > 0) {
        return (supply_voltage - sensor_voltage) * load_resistance / sensor_voltage;
    }
    
    return 0.0f;
}

// Filter functions

/**
 * @brief Initialize ADC filter
 * @param filter Pointer to filter structure
 */
void adc_filter_init(adc_filter_t* filter)
{
    if (filter == NULL) {
        return;
    }
    
    memset(filter->buffer, 0, sizeof(filter->buffer));
    filter->index = 0;
    filter->sum = 0;
    filter->filled = false;
}

/**
 * @brief Add sample to ADC filter
 * @param filter Pointer to filter structure
 * @param sample Sample to add
 * @return Filtered value
 */
uint16_t adc_filter_add_sample(adc_filter_t* filter, uint16_t sample)
{
    if (filter == NULL) {
        return sample;
    }
    
    // Remove old sample from sum
    filter->sum -= filter->buffer[filter->index];
    
    // Add new sample
    filter->buffer[filter->index] = sample;
    filter->sum += sample;
    
    // Update index
    filter->index = (filter->index + 1) % ADC_FILTER_SIZE;
    
    // Check if buffer is filled
    if (!filter->filled && filter->index == 0) {
        filter->filled = true;
    }
    
    return adc_filter_get_average(filter);
}

/**
 * @brief Get filtered average value
 * @param filter Pointer to filter structure
 * @return Average filtered value
 */
uint16_t adc_filter_get_average(const adc_filter_t* filter)
{
    if (filter == NULL) {
        return 0;
    }
    
    uint8_t count = filter->filled ? ADC_FILTER_SIZE : filter->index;
    
    if (count == 0) {
        return 0;
    }
    
    return (uint16_t)(filter->sum / count);
}

// Hardware abstraction layer implementations (placeholders for TI drivers)
static bool hal_adc_init(void)
{
    // Initialize TI ADC peripheral
    // Configure channels, reference voltage, sampling time
    return true; // Placeholder
}

static void hal_adc_deinit(void)
{
    // Deinitialize TI ADC peripheral
}

static bool hal_adc_start_conversion(uint8_t channel)
{
    // Start ADC conversion for specified channel
    (void)channel; // Suppress unused parameter warning
    return true; // Placeholder
}

static bool hal_adc_read_value(uint8_t channel, uint16_t* value)
{
    // Read ADC conversion result
    (void)channel; // Suppress unused parameter warning
    if (value != NULL) {
        *value = 2048; // Placeholder value (mid-scale)
        return true;
    }
    return false;
}

static bool hal_adc_is_conversion_complete(uint8_t channel)
{
    // Check if ADC conversion is complete
    (void)channel; // Suppress unused parameter warning
    return true; // Placeholder
}