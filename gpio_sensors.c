/**
 * @file gpio_sensors.c
 * @brief GPIO sensor reading module implementation for TI M0G3507
 * @author TI M0G3507 Safety System
 * @date 2024
 */

#include "gpio_sensors.h"
#include <string.h>

// Static variables
static vibration_sensor_data_t vibration_data;
static gpio_debounce_t vibration_debounce;
static gpio_sensor_config_t vibration_config;
static uint32_t gpio_error_count = 0;
static bool gpio_initialized = false;

// Hardware abstraction layer functions (to be implemented with TI drivers)
static bool hal_gpio_init(void);
static void hal_gpio_deinit(void);
static bool hal_gpio_configure_pin(void* port, uint8_t pin, bool input, bool pull_up);
static gpio_state_t hal_gpio_read_pin(void* port, uint8_t pin);
static bool hal_gpio_enable_interrupt(void* port, uint8_t pin, bool rising_edge, bool falling_edge);
static void hal_gpio_disable_interrupt(void* port, uint8_t pin);

// Internal helper functions
static void gpio_process_vibration_sensor(void);
static uint32_t gpio_get_system_time_ms(void);

/**
 * @brief Initialize GPIO sensors module
 * @return true if initialization successful, false otherwise
 */
bool gpio_sensors_init(void)
{
    // Initialize hardware GPIO
    if (!hal_gpio_init()) {
        gpio_error_count++;
        return false;
    }
    
    // Configure vibration sensor GPIO
    vibration_config.port = GPIO_VIBRATION_PORT;
    vibration_config.pin = GPIO_VIBRATION_PIN;
    vibration_config.pull_up_enabled = true;    // Enable pull-up for vibration sensor
    vibration_config.interrupt_enabled = true;
    vibration_config.debounce_time_ms = VIBRATION_DEBOUNCE_MS;
    
    // Configure GPIO pin
    if (!hal_gpio_configure_pin(vibration_config.port, vibration_config.pin, 
                               true, vibration_config.pull_up_enabled)) {
        gpio_error_count++;
        return false;
    }
    
    // Enable GPIO interrupt
    if (!hal_gpio_enable_interrupt(vibration_config.port, vibration_config.pin, 
                                  true, true)) { // Both edges
        gpio_error_count++;
        return false;
    }
    
    // Initialize vibration sensor data
    memset(&vibration_data, 0, sizeof(vibration_data));
    vibration_data.last_detection = 0;
    
    // Initialize debounce filter
    gpio_debounce_init(&vibration_debounce, vibration_config.debounce_time_ms);
    
    gpio_initialized = true;
    return true;
}

/**
 * @brief Deinitialize GPIO sensors module
 */
void gpio_sensors_deinit(void)
{
    if (gpio_initialized) {
        // Disable interrupts
        hal_gpio_disable_interrupt(vibration_config.port, vibration_config.pin);
        
        // Deinitialize hardware
        hal_gpio_deinit();
        
        gpio_initialized = false;
    }
}

/**
 * @brief Read vibration sensor data
 * @param data Pointer to store vibration sensor data
 * @return true if data available, false otherwise
 */
bool gpio_sensors_read_vibration(vibration_sensor_data_t* data)
{
    if (data == NULL || !gpio_initialized) {
        return false;
    }
    
    memcpy(data, &vibration_data, sizeof(vibration_sensor_data_t));
    return true;
}

/**
 * @brief Check if vibration is currently detected
 * @return true if vibration detected, false otherwise
 */
bool gpio_sensors_is_vibration_detected(void)
{
    return vibration_data.vibration_detected;
}

/**
 * @brief Process GPIO sensor readings
 */
void gpio_sensors_process(void)
{
    if (!gpio_initialized) {
        return;
    }
    
    // Process vibration sensor
    gpio_process_vibration_sensor();
    
    // Update alarm state based on detection
    uint32_t current_time = gpio_get_system_time_ms();
    
    // Check if vibration alarm should be active
    if (vibration_data.vibration_detected) {
        vibration_data.alarm_active = true;
        vibration_data.duration_ms = current_time - vibration_data.last_detection;
    } else {
        // Clear alarm after some time without detection
        const uint32_t ALARM_CLEAR_TIME_MS = 5000; // 5 seconds
        if (current_time - vibration_data.last_detection > ALARM_CLEAR_TIME_MS) {
            vibration_data.alarm_active = false;
            vibration_data.duration_ms = 0;
        }
    }
}

/**
 * @brief Reset vibration detection counters
 */
void gpio_sensors_reset_counters(void)
{
    vibration_data.detection_count = 0;
    vibration_data.last_detection = 0;
    vibration_data.duration_ms = 0;
    vibration_data.alarm_active = false;
}

/**
 * @brief Get total vibration detection count
 * @return Number of vibration detections
 */
uint32_t gpio_sensors_get_vibration_count(void)
{
    return vibration_data.detection_count;
}

/**
 * @brief Configure vibration sensor debounce time
 * @param debounce_ms Debounce time in milliseconds
 * @return true if configuration successful, false otherwise
 */
bool gpio_sensors_configure_debounce(uint32_t debounce_ms)
{
    if (debounce_ms > 1000) { // Maximum 1 second debounce
        return false;
    }
    
    vibration_config.debounce_time_ms = debounce_ms;
    vibration_debounce.debounce_time_ms = debounce_ms;
    
    return true;
}

/**
 * @brief GPIO interrupt handler
 */
void gpio_sensors_interrupt_handler(void)
{
    if (!gpio_initialized) {
        return;
    }
    
    uint32_t current_time = gpio_get_system_time_ms();
    
    // Read current GPIO state
    gpio_state_t current_state = hal_gpio_read_pin(vibration_config.port, vibration_config.pin);
    
    // Update debounce filter
    gpio_state_t debounced_state = gpio_debounce_update(&vibration_debounce, current_state, current_time);
    
    // Check for state change after debouncing
    if (gpio_debounce_is_stable(&vibration_debounce)) {
        if (debounced_state == GPIO_STATE_HIGH && !vibration_data.vibration_detected) {
            // Vibration detected (rising edge)
            vibration_data.vibration_detected = true;
            vibration_data.detection_count++;
            vibration_data.last_detection = current_time;
            vibration_data.alarm_active = true;
        } else if (debounced_state == GPIO_STATE_LOW && vibration_data.vibration_detected) {
            // Vibration ended (falling edge)
            vibration_data.vibration_detected = false;
            vibration_data.duration_ms = current_time - vibration_data.last_detection;
        }
    }
}

/**
 * @brief Check if GPIO sensors are ready
 * @return true if ready, false otherwise
 */
bool gpio_sensors_is_ready(void)
{
    return gpio_initialized;
}

/**
 * @brief Get GPIO sensor error count
 * @return Number of GPIO errors
 */
uint32_t gpio_sensors_get_error_count(void)
{
    return gpio_error_count;
}

/**
 * @brief Reset GPIO sensor error count
 */
void gpio_sensors_reset_errors(void)
{
    gpio_error_count = 0;
}

// Internal helper functions

/**
 * @brief Process vibration sensor with debouncing
 */
static void gpio_process_vibration_sensor(void)
{
    uint32_t current_time = gpio_get_system_time_ms();
    
    // Read GPIO state
    gpio_state_t current_state = hal_gpio_read_pin(vibration_config.port, vibration_config.pin);
    
    // Update debounce filter
    gpio_state_t debounced_state = gpio_debounce_update(&vibration_debounce, current_state, current_time);
    
    // Update vibration detection state if stable
    if (gpio_debounce_is_stable(&vibration_debounce)) {
        bool previous_state = vibration_data.vibration_detected;
        vibration_data.vibration_detected = (debounced_state == GPIO_STATE_HIGH);
        
        // Update counters on state change
        if (vibration_data.vibration_detected && !previous_state) {
            vibration_data.detection_count++;
            vibration_data.last_detection = current_time;
        }
    }
}

/**
 * @brief Get system time in milliseconds
 * @return Current system time in milliseconds
 */
static uint32_t gpio_get_system_time_ms(void)
{
    // Convert system uptime to milliseconds
    return g_system_status.uptime_seconds * 1000;
}

// Debounce helper functions

/**
 * @brief Initialize GPIO debounce filter
 * @param debounce Pointer to debounce structure
 * @param debounce_time_ms Debounce time in milliseconds
 */
void gpio_debounce_init(gpio_debounce_t* debounce, uint32_t debounce_time_ms)
{
    if (debounce == NULL) {
        return;
    }
    
    debounce->current_state = GPIO_STATE_UNKNOWN;
    debounce->last_state = GPIO_STATE_UNKNOWN;
    debounce->last_change_time = 0;
    debounce->debounce_time_ms = debounce_time_ms;
    debounce->stable = false;
}

/**
 * @brief Update GPIO debounce filter
 * @param debounce Pointer to debounce structure
 * @param input_state Current input state
 * @param current_time Current time in milliseconds
 * @return Debounced state
 */
gpio_state_t gpio_debounce_update(gpio_debounce_t* debounce, gpio_state_t input_state, uint32_t current_time)
{
    if (debounce == NULL) {
        return input_state;
    }
    
    // Check if state has changed
    if (input_state != debounce->last_state) {
        // State changed, reset debounce timer
        debounce->last_state = input_state;
        debounce->last_change_time = current_time;
        debounce->stable = false;
    } else {
        // State hasn't changed, check if debounce time has elapsed
        if (!debounce->stable && 
            (current_time - debounce->last_change_time) >= debounce->debounce_time_ms) {
            // Debounce time elapsed, state is stable
            debounce->current_state = input_state;
            debounce->stable = true;
        }
    }
    
    return debounce->current_state;
}

/**
 * @brief Check if debounced state is stable
 * @param debounce Pointer to debounce structure
 * @return true if state is stable, false otherwise
 */
bool gpio_debounce_is_stable(const gpio_debounce_t* debounce)
{
    if (debounce == NULL) {
        return false;
    }
    
    return debounce->stable;
}

// Hardware abstraction layer implementations (placeholders for TI drivers)
static bool hal_gpio_init(void)
{
    // Initialize TI GPIO peripheral
    // Enable GPIO clocks, configure ports
    return true; // Placeholder
}

static void hal_gpio_deinit(void)
{
    // Deinitialize TI GPIO peripheral
}

static bool hal_gpio_configure_pin(void* port, uint8_t pin, bool input, bool pull_up)
{
    // Configure GPIO pin as input/output with pull-up/pull-down
    (void)port; (void)pin; (void)input; (void)pull_up; // Suppress unused parameter warnings
    return true; // Placeholder
}

static gpio_state_t hal_gpio_read_pin(void* port, uint8_t pin)
{
    // Read GPIO pin state
    (void)port; (void)pin; // Suppress unused parameter warnings
    return GPIO_STATE_LOW; // Placeholder
}

static bool hal_gpio_enable_interrupt(void* port, uint8_t pin, bool rising_edge, bool falling_edge)
{
    // Enable GPIO interrupt with edge configuration
    (void)port; (void)pin; (void)rising_edge; (void)falling_edge; // Suppress unused parameter warnings
    return true; // Placeholder
}

static void hal_gpio_disable_interrupt(void* port, uint8_t pin)
{
    // Disable GPIO interrupt
    (void)port; (void)pin; // Suppress unused parameter warnings
}