/**
 * @file gpio_sensors.h
 * @brief GPIO sensor reading module header for TI M0G3507
 * @author TI M0G3507 Safety System
 * @date 2024
 */

#ifndef GPIO_SENSORS_H
#define GPIO_SENSORS_H

#include "system_config.h"

// GPIO sensor states
typedef enum {
    GPIO_STATE_LOW = 0,
    GPIO_STATE_HIGH = 1,
    GPIO_STATE_UNKNOWN = 2
} gpio_state_t;

// Vibration sensor data
typedef struct {
    bool vibration_detected;    // Current vibration state
    uint32_t detection_count;   // Total detection count
    uint32_t last_detection;    // Timestamp of last detection
    uint32_t duration_ms;       // Duration of current vibration
    bool alarm_active;          // Alarm state
} vibration_sensor_data_t;

// GPIO debounce structure
typedef struct {
    gpio_state_t current_state;
    gpio_state_t last_state;
    uint32_t last_change_time;
    uint32_t debounce_time_ms;
    bool stable;
} gpio_debounce_t;

// GPIO sensor configuration
typedef struct {
    void* port;                 // GPIO port
    uint8_t pin;               // GPIO pin number
    bool pull_up_enabled;      // Pull-up resistor enabled
    bool interrupt_enabled;    // Interrupt enabled
    uint32_t debounce_time_ms; // Debounce time
} gpio_sensor_config_t;

// Function prototypes
bool gpio_sensors_init(void);
void gpio_sensors_deinit(void);
bool gpio_sensors_read_vibration(vibration_sensor_data_t* data);
bool gpio_sensors_is_vibration_detected(void);
void gpio_sensors_process(void);
void gpio_sensors_reset_counters(void);
uint32_t gpio_sensors_get_vibration_count(void);
bool gpio_sensors_configure_debounce(uint32_t debounce_ms);
void gpio_sensors_interrupt_handler(void);
bool gpio_sensors_is_ready(void);
uint32_t gpio_sensors_get_error_count(void);
void gpio_sensors_reset_errors(void);

// Debounce helper functions
void gpio_debounce_init(gpio_debounce_t* debounce, uint32_t debounce_time_ms);
gpio_state_t gpio_debounce_update(gpio_debounce_t* debounce, gpio_state_t input_state, uint32_t current_time);
bool gpio_debounce_is_stable(const gpio_debounce_t* debounce);

#endif // GPIO_SENSORS_H