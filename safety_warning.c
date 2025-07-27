/**
 * @file safety_warning.c
 * @brief Safety warning logic module implementation for TI M0G3507
 * @author TI M0G3507 Safety System
 * @date 2024
 */

#include "safety_warning.h"
#include <string.h>
#include <math.h>

// Static variables
static safety_system_t safety_system;
static bool safety_initialized = false;
static uint32_t safety_error_count = 0;

// Internal helper functions
static void safety_update_system_state(void);
static void safety_process_angle_sensor(void);
static void safety_process_gas_sensor(void);
static void safety_process_vibration_sensor(void);
static warning_level_t safety_calculate_warning_level(warning_event_t event, float value, float threshold);
static uint32_t safety_get_system_time_ms(void);
static void safety_log_warning(warning_event_t event, warning_level_t level, float value);

/**
 * @brief Initialize safety warning system
 * @return true if initialization successful, false otherwise
 */
bool safety_warning_init(void)
{
    // Initialize safety system structure
    memset(&safety_system, 0, sizeof(safety_system_t));
    
    // Set default thresholds
    safety_system.thresholds.angle_warning_deg = ANGLE_THRESHOLD_DEG;
    safety_system.thresholds.angle_critical_deg = ANGLE_THRESHOLD_DEG * 1.5f;
    safety_system.thresholds.gas_warning_ppm = GAS_THRESHOLD_PPM;
    safety_system.thresholds.gas_critical_ppm = GAS_THRESHOLD_PPM * 2;
    safety_system.thresholds.vibration_warning_count = 5;
    safety_system.thresholds.vibration_critical_count = 10;
    safety_system.thresholds.response_time_ms = VOICE_RESPONSE_TIME_MS;
    
    // Initialize system state
    safety_system.state = SAFETY_STATE_NORMAL;
    safety_system.monitoring.current_state = SAFETY_STATE_NORMAL;
    safety_system.monitoring.highest_warning = WARNING_LEVEL_NONE;
    safety_system.monitoring.system_healthy = true;
    
    safety_initialized = true;
    return true;
}

/**
 * @brief Deinitialize safety warning system
 */
void safety_warning_deinit(void)
{
    safety_initialized = false;
    memset(&safety_system, 0, sizeof(safety_system_t));
}

/**
 * @brief Process safety warning system
 */
void safety_warning_process(void)
{
    if (!safety_initialized) {
        return;
    }
    
    uint32_t current_time = safety_get_system_time_ms();
    safety_system.last_update = current_time;
    
    // Update sensor health monitoring
    safety_warning_update_sensor_health();
    
    // Process individual sensor warnings
    safety_process_angle_sensor();
    safety_process_gas_sensor();
    safety_process_vibration_sensor();
    
    // Check overall system conditions
    safety_warning_check_conditions();
    
    // Update system state based on active warnings
    safety_update_system_state();
    
    // Update global system status
    g_system_status.current_warning = safety_system.monitoring.highest_warning;
}

/**
 * @brief Check all safety conditions
 * @return true if any warnings are active, false otherwise
 */
bool safety_warning_check_conditions(void)
{
    bool warnings_active = false;
    warning_level_t highest_level = WARNING_LEVEL_NONE;
    uint32_t active_count = 0;
    
    // Count active warnings and find highest level
    for (int i = 0; i < safety_system.active_count; i++) {
        if (safety_system.conditions[i].active) {
            active_count++;
            warnings_active = true;
            
            if (safety_system.conditions[i].level > highest_level) {
                highest_level = safety_system.conditions[i].level;
            }
        }
    }
    
    // Update monitoring data
    safety_system.monitoring.active_warnings = active_count;
    safety_system.monitoring.highest_warning = highest_level;
    
    if (warnings_active) {
        safety_system.monitoring.last_warning_time = safety_get_system_time_ms();
    }
    
    return warnings_active;
}

/**
 * @brief Get current warning level
 * @return Current highest warning level
 */
warning_level_t safety_warning_get_level(void)
{
    return safety_system.monitoring.highest_warning;
}

/**
 * @brief Get current safety state
 * @return Current safety system state
 */
safety_state_t safety_warning_get_state(void)
{
    return safety_system.monitoring.current_state;
}

/**
 * @brief Check if specific warning event is active
 * @param event_type Warning event type to check
 * @return true if warning is active, false otherwise
 */
bool safety_warning_is_active(warning_event_t event_type)
{
    for (int i = 0; i < safety_system.active_count; i++) {
        if (safety_system.conditions[i].event_type == event_type && 
            safety_system.conditions[i].active) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Get number of active warnings
 * @return Number of active warning conditions
 */
uint32_t safety_warning_get_active_count(void)
{
    return safety_system.monitoring.active_warnings;
}

/**
 * @brief Acknowledge specific warning
 * @param event_type Warning event type to acknowledge
 * @return true if warning found and acknowledged, false otherwise
 */
bool safety_warning_acknowledge(warning_event_t event_type)
{
    for (int i = 0; i < safety_system.active_count; i++) {
        if (safety_system.conditions[i].event_type == event_type) {
            safety_system.conditions[i].acknowledged = true;
            return true;
        }
    }
    return false;
}

/**
 * @brief Acknowledge all active warnings
 */
void safety_warning_acknowledge_all(void)
{
    for (int i = 0; i < safety_system.active_count; i++) {
        safety_system.conditions[i].acknowledged = true;
    }
}

/**
 * @brief Set safety thresholds
 * @param thresholds Pointer to threshold configuration
 * @return true if thresholds set successfully, false otherwise
 */
bool safety_warning_set_thresholds(const safety_thresholds_t* thresholds)
{
    if (thresholds == NULL) {
        return false;
    }
    
    memcpy(&safety_system.thresholds, thresholds, sizeof(safety_thresholds_t));
    return true;
}

/**
 * @brief Get current safety thresholds
 * @param thresholds Pointer to store threshold configuration
 * @return true if thresholds retrieved successfully, false otherwise
 */
bool safety_warning_get_thresholds(safety_thresholds_t* thresholds)
{
    if (thresholds == NULL) {
        return false;
    }
    
    memcpy(thresholds, &safety_system.thresholds, sizeof(safety_thresholds_t));
    return true;
}

/**
 * @brief Get monitoring data
 * @param data Pointer to store monitoring data
 * @return true if data retrieved successfully, false otherwise
 */
bool safety_warning_get_monitoring_data(safety_monitoring_t* data)
{
    if (data == NULL) {
        return false;
    }
    
    memcpy(data, &safety_system.monitoring, sizeof(safety_monitoring_t));
    return true;
}

/**
 * @brief Reset warning counters
 * @return true if reset successful, false otherwise
 */
bool safety_warning_reset_counters(void)
{
    safety_system.monitoring.total_warnings = 0;
    safety_system.total_count = 0;
    
    // Clear all non-active conditions
    for (int i = 0; i < safety_system.active_count; i++) {
        if (!safety_system.conditions[i].active) {
            memset(&safety_system.conditions[i], 0, sizeof(warning_condition_t));
        }
    }
    
    return true;
}

/**
 * @brief Enter standby mode
 */
void safety_warning_enter_standby(void)
{
    // In standby mode, continue monitoring but reduce processing
    g_system_status.power_mode = POWER_MODE_STANDBY;
}

/**
 * @brief Exit standby mode
 */
void safety_warning_exit_standby(void)
{
    g_system_status.power_mode = POWER_MODE_ACTIVE;
}

/**
 * @brief Check if response time requirement is met
 * @return true if response time is within limits, false otherwise
 */
bool safety_warning_is_response_time_met(void)
{
    uint32_t current_time = safety_get_system_time_ms();
    uint32_t response_time = current_time - safety_system.monitoring.last_warning_time;
    
    return response_time <= safety_system.thresholds.response_time_ms;
}

// Warning condition management functions

/**
 * @brief Add new warning condition
 * @param event Warning event type
 * @param level Warning level
 * @param value Current sensor value
 * @param threshold Threshold value
 * @return true if condition added successfully, false otherwise
 */
bool safety_warning_add_condition(warning_event_t event, warning_level_t level, 
                                 float value, float threshold)
{
    if (safety_system.active_count >= MAX_WARNING_CONDITIONS) {
        safety_error_count++;
        return false;
    }
    
    // Check if condition already exists
    for (int i = 0; i < safety_system.active_count; i++) {
        if (safety_system.conditions[i].event_type == event) {
            // Update existing condition
            safety_system.conditions[i].level = level;
            safety_system.conditions[i].sensor_value = value;
            safety_system.conditions[i].threshold_value = threshold;
            safety_system.conditions[i].active = true;
            safety_system.conditions[i].timestamp = safety_get_system_time_ms();
            return true;
        }
    }
    
    // Add new condition
    warning_condition_t* condition = &safety_system.conditions[safety_system.active_count];
    condition->event_type = event;
    condition->level = level;
    condition->sensor_value = value;
    condition->threshold_value = threshold;
    condition->timestamp = safety_get_system_time_ms();
    condition->duration_ms = 0;
    condition->active = true;
    condition->acknowledged = false;
    
    safety_system.active_count++;
    safety_system.total_count++;
    safety_system.monitoring.total_warnings++;
    
    safety_log_warning(event, level, value);
    return true;
}

/**
 * @brief Remove warning condition
 * @param event Warning event type to remove
 * @return true if condition removed, false otherwise
 */
bool safety_warning_remove_condition(warning_event_t event)
{
    for (int i = 0; i < safety_system.active_count; i++) {
        if (safety_system.conditions[i].event_type == event) {
            // Calculate duration
            uint32_t current_time = safety_get_system_time_ms();
            safety_system.conditions[i].duration_ms = 
                current_time - safety_system.conditions[i].timestamp;
            
            // Mark as inactive
            safety_system.conditions[i].active = false;
            return true;
        }
    }
    return false;
}

/**
 * @brief Update warning condition value
 * @param event Warning event type to update
 * @param value New sensor value
 * @return true if condition updated, false otherwise
 */
bool safety_warning_update_condition(warning_event_t event, float value)
{
    for (int i = 0; i < safety_system.active_count; i++) {
        if (safety_system.conditions[i].event_type == event && 
            safety_system.conditions[i].active) {
            safety_system.conditions[i].sensor_value = value;
            return true;
        }
    }
    return false;
}

/**
 * @brief Get warning condition by event type
 * @param event Warning event type
 * @return Pointer to warning condition or NULL if not found
 */
warning_condition_t* safety_warning_get_condition(warning_event_t event)
{
    for (int i = 0; i < safety_system.active_count; i++) {
        if (safety_system.conditions[i].event_type == event) {
            return &safety_system.conditions[i];
        }
    }
    return NULL;
}

// System health monitoring functions

/**
 * @brief Check overall system health
 * @return true if system is healthy, false otherwise
 */
bool safety_warning_check_system_health(void)
{
    bool healthy = true;
    
    // Check if sensors are responding
    if (!adc_sensors_is_ready() || !gpio_sensors_is_ready()) {
        healthy = false;
        safety_warning_add_condition(WARNING_EVENT_SENSOR_FAILURE, WARNING_LEVEL_HIGH, 0, 0);
    }
    
    // Check error counts
    if (adc_sensors_get_error_count() > MAX_ERROR_COUNT ||
        gpio_sensors_get_error_count() > MAX_ERROR_COUNT) {
        healthy = false;
        safety_warning_add_condition(WARNING_EVENT_SYSTEM_ERROR, WARNING_LEVEL_MEDIUM, 0, 0);
    }
    
    safety_system.monitoring.system_healthy = healthy;
    return healthy;
}

/**
 * @brief Update sensor health status
 */
void safety_warning_update_sensor_health(void)
{
    // Update monitoring data with current sensor values
    angle_sensor_data_t angle_data;
    if (adc_sensors_get_angle_data(&angle_data)) {
        safety_system.monitoring.angle_value = angle_data.angle_deg;
    }
    
    gas_sensor_data_t gas_data;
    if (adc_sensors_get_gas_data(&gas_data)) {
        safety_system.monitoring.gas_value = gas_data.ppm;
    }
    
    vibration_sensor_data_t vibration_data;
    if (gpio_sensors_read_vibration(&vibration_data)) {
        safety_system.monitoring.vibration_active = vibration_data.vibration_detected;
    }
    
    // Check overall system health
    safety_warning_check_system_health();
}

/**
 * @brief Check if sensors are healthy
 * @return true if all sensors are healthy, false otherwise
 */
bool safety_warning_is_sensor_healthy(void)
{
    return safety_system.monitoring.system_healthy;
}

// Internal helper functions

/**
 * @brief Update overall system state based on warnings
 */
static void safety_update_system_state(void)
{
    warning_level_t highest = safety_system.monitoring.highest_warning;
    
    switch (highest) {
        case WARNING_LEVEL_NONE:
            safety_system.monitoring.current_state = SAFETY_STATE_NORMAL;
            break;
            
        case WARNING_LEVEL_LOW:
            safety_system.monitoring.current_state = SAFETY_STATE_WARNING;
            break;
            
        case WARNING_LEVEL_MEDIUM:
            safety_system.monitoring.current_state = SAFETY_STATE_ALARM;
            break;
            
        case WARNING_LEVEL_HIGH:
        case WARNING_LEVEL_CRITICAL:
            safety_system.monitoring.current_state = SAFETY_STATE_EMERGENCY;
            break;
            
        default:
            safety_system.monitoring.current_state = SAFETY_STATE_FAULT;
            break;
    }
    
    safety_system.state = safety_system.monitoring.current_state;
}

/**
 * @brief Process angle sensor warnings
 */
static void safety_process_angle_sensor(void)
{
    angle_sensor_data_t angle_data;
    if (!adc_sensors_get_angle_data(&angle_data)) {
        return;
    }
    
    float angle = angle_data.angle_deg;
    
    // Check warning threshold
    if (fabs(angle) > safety_system.thresholds.angle_warning_deg) {
        warning_level_t level = safety_calculate_warning_level(
            WARNING_EVENT_ANGLE_THRESHOLD, angle, safety_system.thresholds.angle_warning_deg);
        
        safety_warning_add_condition(WARNING_EVENT_ANGLE_THRESHOLD, level, 
                                   angle, safety_system.thresholds.angle_warning_deg);
    } else {
        safety_warning_remove_condition(WARNING_EVENT_ANGLE_THRESHOLD);
    }
}

/**
 * @brief Process gas sensor warnings
 */
static void safety_process_gas_sensor(void)
{
    gas_sensor_data_t gas_data;
    if (!adc_sensors_get_gas_data(&gas_data)) {
        return;
    }
    
    uint16_t gas_ppm = gas_data.ppm;
    
    // Check warning threshold
    if (gas_ppm > safety_system.thresholds.gas_warning_ppm) {
        warning_level_t level = safety_calculate_warning_level(
            WARNING_EVENT_GAS_THRESHOLD, gas_ppm, safety_system.thresholds.gas_warning_ppm);
        
        safety_warning_add_condition(WARNING_EVENT_GAS_THRESHOLD, level, 
                                   gas_ppm, safety_system.thresholds.gas_warning_ppm);
    } else {
        safety_warning_remove_condition(WARNING_EVENT_GAS_THRESHOLD);
    }
}

/**
 * @brief Process vibration sensor warnings
 */
static void safety_process_vibration_sensor(void)
{
    vibration_sensor_data_t vibration_data;
    if (!gpio_sensors_read_vibration(&vibration_data)) {
        return;
    }
    
    // Check if vibration is detected
    if (vibration_data.vibration_detected) {
        uint32_t count = vibration_data.detection_count;
        warning_level_t level = safety_calculate_warning_level(
            WARNING_EVENT_VIBRATION_DETECTED, count, safety_system.thresholds.vibration_warning_count);
        
        safety_warning_add_condition(WARNING_EVENT_VIBRATION_DETECTED, level, 
                                   count, safety_system.thresholds.vibration_warning_count);
    } else {
        safety_warning_remove_condition(WARNING_EVENT_VIBRATION_DETECTED);
    }
}

/**
 * @brief Calculate warning level based on sensor value and threshold
 * @param event Warning event type
 * @param value Current sensor value
 * @param threshold Warning threshold value
 * @return Calculated warning level
 */
static warning_level_t safety_calculate_warning_level(warning_event_t event, float value, float threshold)
{
    (void)event; // Suppress unused parameter warning
    float ratio = value / threshold;
    
    if (ratio >= 3.0f) {
        return WARNING_LEVEL_CRITICAL;
    } else if (ratio >= 2.0f) {
        return WARNING_LEVEL_HIGH;
    } else if (ratio >= 1.5f) {
        return WARNING_LEVEL_MEDIUM;
    } else if (ratio >= 1.0f) {
        return WARNING_LEVEL_LOW;
    }
    
    return WARNING_LEVEL_NONE;
}

/**
 * @brief Get system time in milliseconds
 * @return Current system time in milliseconds
 */
static uint32_t safety_get_system_time_ms(void)
{
    return g_system_status.uptime_seconds * 1000;
}

/**
 * @brief Log warning event
 * @param event Warning event type
 * @param level Warning level
 * @param value Sensor value that triggered warning
 */
static void safety_log_warning(warning_event_t event, warning_level_t level, float value)
{
    // This would typically log to a system log or EEPROM
    // For now, just increment global error count for critical warnings
    (void)event; (void)value; // Suppress unused parameter warnings
    if (level >= WARNING_LEVEL_HIGH) {
        g_system_status.error_count++;
    }
}