/**
 * @file safety_warning.h
 * @brief Safety warning logic module header for TI M0G3507
 * @author TI M0G3507 Safety System
 * @date 2024
 */

#ifndef SAFETY_WARNING_H
#define SAFETY_WARNING_H

#include "system_config.h"
#include "adc_sensors.h"
#include "gpio_sensors.h"

// Warning event types
typedef enum {
    WARNING_EVENT_NONE = 0,
    WARNING_EVENT_ANGLE_THRESHOLD,
    WARNING_EVENT_GAS_THRESHOLD,
    WARNING_EVENT_VIBRATION_DETECTED,
    WARNING_EVENT_SYSTEM_ERROR,
    WARNING_EVENT_SENSOR_FAILURE,
    WARNING_EVENT_COMMUNICATION_LOSS
} warning_event_t;

// Warning condition structure
typedef struct {
    warning_event_t event_type;
    warning_level_t level;
    float sensor_value;
    float threshold_value;
    uint32_t timestamp;
    uint32_t duration_ms;
    bool active;
    bool acknowledged;
} warning_condition_t;

// Safety system state
typedef enum {
    SAFETY_STATE_NORMAL = 0,
    SAFETY_STATE_WARNING,
    SAFETY_STATE_ALARM,
    SAFETY_STATE_EMERGENCY,
    SAFETY_STATE_FAULT
} safety_state_t;

// Threshold configuration
typedef struct {
    float angle_warning_deg;
    float angle_critical_deg;
    uint16_t gas_warning_ppm;
    uint16_t gas_critical_ppm;
    uint32_t vibration_warning_count;
    uint32_t vibration_critical_count;
    uint32_t response_time_ms;
} safety_thresholds_t;

// Safety monitoring data
typedef struct {
    safety_state_t current_state;
    warning_level_t highest_warning;
    uint32_t active_warnings;
    uint32_t total_warnings;
    uint32_t last_warning_time;
    float angle_value;
    uint16_t gas_value;
    bool vibration_active;
    bool system_healthy;
} safety_monitoring_t;

#define MAX_WARNING_CONDITIONS  16
typedef struct {
    warning_condition_t conditions[MAX_WARNING_CONDITIONS];
    uint8_t active_count;
    uint8_t total_count;
    safety_state_t state;
    safety_monitoring_t monitoring;
    safety_thresholds_t thresholds;
    uint32_t last_update;
} safety_system_t;

// Function prototypes
bool safety_warning_init(void);
void safety_warning_deinit(void);
void safety_warning_process(void);
bool safety_warning_check_conditions(void);
warning_level_t safety_warning_get_level(void);
safety_state_t safety_warning_get_state(void);
bool safety_warning_is_active(warning_event_t event_type);
uint32_t safety_warning_get_active_count(void);
bool safety_warning_acknowledge(warning_event_t event_type);
void safety_warning_acknowledge_all(void);
bool safety_warning_set_thresholds(const safety_thresholds_t* thresholds);
bool safety_warning_get_thresholds(safety_thresholds_t* thresholds);
bool safety_warning_get_monitoring_data(safety_monitoring_t* data);
bool safety_warning_reset_counters(void);
void safety_warning_enter_standby(void);
void safety_warning_exit_standby(void);
bool safety_warning_is_response_time_met(void);

// Warning condition management
bool safety_warning_add_condition(warning_event_t event, warning_level_t level, 
                                 float value, float threshold);
bool safety_warning_remove_condition(warning_event_t event);
bool safety_warning_update_condition(warning_event_t event, float value);
warning_condition_t* safety_warning_get_condition(warning_event_t event);

// System health monitoring
bool safety_warning_check_system_health(void);
void safety_warning_update_sensor_health(void);
bool safety_warning_is_sensor_healthy(void);

#endif // SAFETY_WARNING_H