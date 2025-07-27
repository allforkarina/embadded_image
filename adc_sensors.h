/**
 * @file adc_sensors.h
 * @brief ADC sensor reading module header for TI M0G3507
 * @author TI M0G3507 Safety System
 * @date 2024
 */

#ifndef ADC_SENSORS_H
#define ADC_SENSORS_H

#include "system_config.h"

// ADC sensor types
typedef enum {
    ADC_SENSOR_ANGLE = 0,
    ADC_SENSOR_GAS,
    ADC_SENSOR_COUNT
} adc_sensor_type_t;

// ADC reading structure
typedef struct {
    uint16_t raw_value;         // Raw ADC value (0-4095 for 12-bit)
    float voltage_mv;           // Converted voltage in millivolts
    float calibrated_value;     // Calibrated sensor value
    uint32_t timestamp;         // Timestamp of reading
    bool valid;                 // Data validity flag
} adc_reading_t;

// Angle sensor data
typedef struct {
    float angle_deg;            // Angle in degrees (0-360)
    float angle_rad;            // Angle in radians
    bool calibrated;            // Calibration status
    float offset_deg;           // Calibration offset
    float scale_factor;         // Calibration scale factor
} angle_sensor_data_t;

// Gas sensor data (MQ-2)
typedef struct {
    uint16_t ppm;               // Gas concentration in PPM
    float resistance_ohm;       // Sensor resistance
    float ratio;                // Rs/R0 ratio
    bool alarm_state;           // Alarm threshold exceeded
    float r0_calibration;       // R0 calibration value
} gas_sensor_data_t;

// ADC calibration data
typedef struct {
    float offset;               // Offset calibration
    float gain;                 // Gain calibration
    uint16_t zero_point;        // Zero point ADC value
    uint16_t full_scale;        // Full scale ADC value
    bool calibrated;            // Calibration status
} adc_calibration_t;

// Filter configuration
#define ADC_FILTER_SIZE     8U
typedef struct {
    uint16_t buffer[ADC_FILTER_SIZE];
    uint8_t index;
    uint32_t sum;
    bool filled;
} adc_filter_t;

// Function prototypes
bool adc_sensors_init(void);
void adc_sensors_deinit(void);
bool adc_sensors_start_conversion(adc_sensor_type_t sensor);
bool adc_sensors_read_raw(adc_sensor_type_t sensor, uint16_t* raw_value);
bool adc_sensors_read_voltage(adc_sensor_type_t sensor, float* voltage_mv);
bool adc_sensors_get_angle_data(angle_sensor_data_t* angle_data);
bool adc_sensors_get_gas_data(gas_sensor_data_t* gas_data);
void adc_sensors_process(void);
bool adc_sensors_calibrate_angle(float known_angle_deg);
bool adc_sensors_calibrate_gas(void);
void adc_sensors_reset_calibration(adc_sensor_type_t sensor);
bool adc_sensors_is_ready(void);
uint32_t adc_sensors_get_error_count(void);
void adc_sensors_reset_errors(void);

// Filter functions
void adc_filter_init(adc_filter_t* filter);
uint16_t adc_filter_add_sample(adc_filter_t* filter, uint16_t sample);
uint16_t adc_filter_get_average(const adc_filter_t* filter);

#endif // ADC_SENSORS_H