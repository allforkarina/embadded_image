/**
 * @file main.c
 * @brief Main application for TI M0G3507 multi-sensor safety warning system
 * @author TI M0G3507 Safety System
 * @date 2024
 */

#include "system_config.h"
#include "i2c_slave.h"
#include "adc_sensors.h"
#include "gpio_sensors.h"
#include "safety_warning.h"
#include "voice_module.h"
#include <string.h>

// Global system status
system_status_t g_system_status;

// Static variables
static uint32_t system_tick_counter = 0;
static uint32_t last_sensor_update = 0;
static uint32_t last_safety_check = 0;
static uint32_t last_voice_update = 0;
static uint32_t last_i2c_update = 0;

// Hardware abstraction layer functions (to be implemented with TI drivers)
static void hal_system_init(void);
static void hal_watchdog_init(void);
static void hal_watchdog_refresh(void);
static void hal_systick_init(void);
static void hal_power_management_init(void);
static void hal_enter_standby_mode(void);
static void hal_exit_standby_mode(void);
static uint32_t hal_get_system_tick(void);

// Internal function prototypes
static void system_initialize(void);
static void system_main_loop(void);
static void system_process_sensors(void);
static void system_process_safety(void);
static void system_process_communication(void);
static void system_process_voice(void);
static void system_update_status(void);
static void system_handle_errors(void);
static void system_enter_standby(void);
static void system_exit_standby(void);
static bool system_check_wakeup_conditions(void);
static void system_reset_if_needed(void);

// Interrupt handlers
void SysTick_Handler(void);
void I2C_IRQHandler(void);
void GPIO_IRQHandler(void);

/**
 * @brief Main application entry point
 * @return Should never return
 */
int main(void)
{
    // Initialize system
    system_initialize();
    
    // Main application loop
    system_main_loop();
    
    // Should never reach here
    return 0;
}

/**
 * @brief Initialize system components
 */
static void system_initialize(void)
{
    // Initialize global system status
    memset(&g_system_status, 0, sizeof(system_status_t));
    g_system_status.power_mode = POWER_MODE_ACTIVE;
    
    // Initialize hardware abstraction layer
    hal_system_init();
    hal_systick_init();
    hal_watchdog_init();
    hal_power_management_init();
    
    // Initialize system configuration
    system_config_init();
    system_clock_config();
    system_gpio_config();
    
    // Initialize peripheral modules
    bool init_success = true;
    
    // Initialize ADC sensors
    if (!adc_sensors_init()) {
        system_error_handler("ADC sensors initialization failed", 0x01);
        init_success = false;
    }
    
    // Initialize GPIO sensors
    if (!gpio_sensors_init()) {
        system_error_handler("GPIO sensors initialization failed", 0x02);
        init_success = false;
    }
    
    // Initialize I2C slave communication
    if (!i2c_slave_init()) {
        system_error_handler("I2C slave initialization failed", 0x03);
        init_success = false;
    }
    
    // Initialize safety warning system
    if (!safety_warning_init()) {
        system_error_handler("Safety warning initialization failed", 0x04);
        init_success = false;
    }
    
    // Initialize voice module
    if (!voice_module_init()) {
        system_error_handler("Voice module initialization failed", 0x05);
        init_success = false;
    }
    
    // Update system status
    g_system_status.system_initialized = init_success;
    g_system_status.sensors_active = true;
    g_system_status.uptime_seconds = 0;
    
    // Refresh watchdog
    hal_watchdog_refresh();
    
    if (init_success) {
        // Play system ready message
        voice_module_play_message(VOICE_MSG_SYSTEM_READY, WARNING_LEVEL_LOW);
    }
}

/**
 * @brief Main system loop
 */
static void system_main_loop(void)
{
    while (1) {
        // Get current system tick
        system_tick_counter = hal_get_system_tick();
        
        // Process system components based on timing
        if (system_tick_counter - last_sensor_update >= SENSOR_SAMPLING_PERIOD_MS) {
            system_process_sensors();
            last_sensor_update = system_tick_counter;
        }
        
        if (system_tick_counter - last_safety_check >= (MAIN_LOOP_PERIOD_MS / 2)) {
            system_process_safety();
            last_safety_check = system_tick_counter;
        }
        
        if (system_tick_counter - last_i2c_update >= (MAIN_LOOP_PERIOD_MS / 4)) {
            system_process_communication();
            last_i2c_update = system_tick_counter;
        }
        
        if (system_tick_counter - last_voice_update >= (MAIN_LOOP_PERIOD_MS / 8)) {
            system_process_voice();
            last_voice_update = system_tick_counter;
        }
        
        // Update system status
        system_update_status();
        
        // Handle any system errors
        system_handle_errors();
        
        // Check if system reset is needed
        system_reset_if_needed();
        
        // Refresh watchdog
        hal_watchdog_refresh();
        
        // Power management - enter standby if conditions are met
        if (g_system_status.power_mode == POWER_MODE_STANDBY) {
            if (!system_check_wakeup_conditions()) {
                system_enter_standby();
            }
        }
        
        // Small delay to prevent excessive CPU usage
        // In a real implementation, this would use a proper delay function
        for (volatile int i = 0; i < 1000; i++);
    }
}

/**
 * @brief Process sensor readings
 */
static void system_process_sensors(void)
{
    if (!g_system_status.sensors_active) {
        return;
    }
    
    // Process ADC sensors (angle and gas)
    adc_sensors_process();
    
    // Process GPIO sensors (vibration)
    gpio_sensors_process();
    
    // Update I2C slave with current sensor data
    i2c_sensor_data_t sensor_data;
    
    // Get angle sensor data
    angle_sensor_data_t angle_data;
    if (adc_sensors_get_angle_data(&angle_data)) {
        sensor_data.angle_deg = angle_data.angle_deg;
    } else {
        sensor_data.angle_deg = 0.0f;
    }
    
    // Get gas sensor data
    gas_sensor_data_t gas_data;
    if (adc_sensors_get_gas_data(&gas_data)) {
        sensor_data.gas_ppm = gas_data.ppm;
    } else {
        sensor_data.gas_ppm = 0;
    }
    
    // Get vibration sensor data
    vibration_sensor_data_t vibration_data;
    if (gpio_sensors_read_vibration(&vibration_data)) {
        sensor_data.vibration_detected = vibration_data.vibration_detected;
    } else {
        sensor_data.vibration_detected = false;
    }
    
    // Set warning level and timestamp
    sensor_data.warning_level = g_system_status.current_warning;
    sensor_data.timestamp = g_system_status.uptime_seconds;
    
    // Update I2C slave data
    i2c_slave_set_sensor_data(&sensor_data);
}

/**
 * @brief Process safety warning system
 */
static void system_process_safety(void)
{
    // Process safety warning logic
    safety_warning_process();
    
    // Check if any new warnings need voice alerts
    if (safety_warning_get_level() > WARNING_LEVEL_NONE) {
        // Get active warning conditions and trigger voice alerts
        warning_level_t current_level = safety_warning_get_level();
        
        // Check for specific warning types
        if (safety_warning_is_active(WARNING_EVENT_ANGLE_THRESHOLD)) {
            voice_module_play_warning(WARNING_EVENT_ANGLE_THRESHOLD, current_level);
        }
        
        if (safety_warning_is_active(WARNING_EVENT_GAS_THRESHOLD)) {
            voice_module_play_warning(WARNING_EVENT_GAS_THRESHOLD, current_level);
        }
        
        if (safety_warning_is_active(WARNING_EVENT_VIBRATION_DETECTED)) {
            voice_module_play_warning(WARNING_EVENT_VIBRATION_DETECTED, current_level);
        }
        
        if (safety_warning_is_active(WARNING_EVENT_SYSTEM_ERROR)) {
            voice_module_play_warning(WARNING_EVENT_SYSTEM_ERROR, current_level);
        }
        
        if (safety_warning_is_active(WARNING_EVENT_SENSOR_FAILURE)) {
            voice_module_play_warning(WARNING_EVENT_SENSOR_FAILURE, current_level);
        }
    }
    
    // Check if system should enter standby mode
    if (safety_warning_get_level() == WARNING_LEVEL_NONE && 
        g_system_status.power_mode == POWER_MODE_ACTIVE) {
        // No active warnings - can consider standby mode
        static uint32_t no_warning_time = 0;
        if (no_warning_time == 0) {
            no_warning_time = system_tick_counter;
        } else if (system_tick_counter - no_warning_time > 30000) { // 30 seconds
            g_system_status.power_mode = POWER_MODE_STANDBY;
            safety_warning_enter_standby();
        }
    } else {
        // Active warnings - stay in active mode
        if (g_system_status.power_mode != POWER_MODE_ACTIVE) {
            g_system_status.power_mode = POWER_MODE_ACTIVE;
            safety_warning_exit_standby();
        }
    }
}

/**
 * @brief Process I2C communication
 */
static void system_process_communication(void)
{
    // Process I2C slave communication
    i2c_slave_process();
    
    // Check for threshold updates from master
    i2c_threshold_data_t thresholds;
    if (i2c_slave_get_thresholds(&thresholds)) {
        // Update safety system thresholds
        safety_thresholds_t safety_thresholds;
        safety_thresholds.angle_warning_deg = thresholds.angle_threshold;
        safety_thresholds.angle_critical_deg = thresholds.angle_threshold * 1.5f;
        safety_thresholds.gas_warning_ppm = thresholds.gas_threshold;
        safety_thresholds.gas_critical_ppm = thresholds.gas_threshold * 2;
        safety_thresholds.vibration_warning_count = 5;
        safety_thresholds.vibration_critical_count = 10;
        safety_thresholds.response_time_ms = VOICE_RESPONSE_TIME_MS;
        
        safety_warning_set_thresholds(&safety_thresholds);
        
        // Update sensor debounce if provided
        if (thresholds.vibration_debounce_ms > 0) {
            gpio_sensors_configure_debounce(thresholds.vibration_debounce_ms);
        }
    }
}

/**
 * @brief Process voice module
 */
static void system_process_voice(void)
{
    // Process voice module
    voice_module_process();
    
    // Check voice module response time
    if (!safety_warning_is_response_time_met() && 
        safety_warning_get_level() > WARNING_LEVEL_NONE) {
        // Response time exceeded - log error
        g_system_status.error_count++;
    }
}

/**
 * @brief Update system status
 */
static void system_update_status(void)
{
    // Update uptime (assuming system tick is in milliseconds)
    g_system_status.uptime_seconds = system_tick_counter / 1000;
    
    // Update sensor status
    g_system_status.sensors_active = adc_sensors_is_ready() && gpio_sensors_is_ready();
    
    // Update communication status
    g_system_status.i2c_ready = i2c_slave_is_ready();
    
    // Update voice status
    g_system_status.voice_ready = voice_module_is_ready();
    
    // Check overall system health
    bool system_healthy = g_system_status.sensors_active && 
                         g_system_status.i2c_ready && 
                         g_system_status.voice_ready;
    
    if (!system_healthy && g_system_status.system_initialized) {
        g_system_status.error_count++;
    }
}

/**
 * @brief Handle system errors
 */
static void system_handle_errors(void)
{
    // Check for excessive errors
    if (g_system_status.error_count > MAX_ERROR_COUNT) {
        system_error_handler("Maximum error count exceeded", g_system_status.error_count);
        
        // Try to recover by reinitializing modules
        if (adc_sensors_get_error_count() > MAX_ERROR_COUNT) {
            adc_sensors_deinit();
            adc_sensors_init();
            adc_sensors_reset_errors();
        }
        
        if (gpio_sensors_get_error_count() > MAX_ERROR_COUNT) {
            gpio_sensors_deinit();
            gpio_sensors_init();
            gpio_sensors_reset_errors();
        }
        
        if (i2c_slave_get_error_count() > MAX_ERROR_COUNT) {
            i2c_slave_deinit();
            i2c_slave_init();
            i2c_slave_reset_errors();
        }
        
        if (voice_module_get_error_count() > MAX_ERROR_COUNT) {
            voice_module_deinit();
            voice_module_init();
            voice_module_reset_counters();
        }
        
        // Reset system error count
        g_system_status.error_count = 0;
    }
}

/**
 * @brief Enter standby mode
 */
static void system_enter_standby(void)
{
    // Reduce sensor sampling rate
    // Disable non-critical peripherals
    // Configure wake-up sources
    
    hal_enter_standby_mode();
}

/**
 * @brief Exit standby mode
 */
static void system_exit_standby(void)
{
    hal_exit_standby_mode();
    
    // Re-enable full sensor sampling
    // Restore normal operation
}

/**
 * @brief Check conditions that should wake up from standby
 * @return true if should wake up, false otherwise
 */
static bool system_check_wakeup_conditions(void)
{
    // Wake up if any warnings are active
    if (safety_warning_get_level() > WARNING_LEVEL_NONE) {
        return true;
    }
    
    // Wake up if vibration detected
    if (gpio_sensors_is_vibration_detected()) {
        return true;
    }
    
    // Wake up if I2C communication active
    if (i2c_slave_get_state() != I2C_STATE_IDLE) {
        return true;
    }
    
    return false;
}

/**
 * @brief Reset system if needed
 */
static void system_reset_if_needed(void)
{
    // Check if system reset is requested
    if (g_system_status.error_count >= MAX_ERROR_COUNT * 2) {
        // Critical system failure - reset
        voice_module_play_message(VOICE_MSG_SYSTEM_RESET, WARNING_LEVEL_CRITICAL);
        
        // Wait for voice message to complete
        while (voice_module_is_playing()) {
            voice_module_process();
            hal_watchdog_refresh();
        }
        
        // Reset system (implementation dependent)
        // NVIC_SystemReset(); // Example for ARM Cortex-M
    }
}

// Interrupt handlers

/**
 * @brief SysTick interrupt handler
 */
void SysTick_Handler(void)
{
    system_tick_counter++;
    
    // Update uptime every second
    if (system_tick_counter % 1000 == 0) {
        g_system_status.uptime_seconds++;
    }
}

/**
 * @brief I2C interrupt handler
 */
void I2C_IRQHandler(void)
{
    i2c_slave_interrupt_handler();
}

/**
 * @brief GPIO interrupt handler
 */
void GPIO_IRQHandler(void)
{
    gpio_sensors_interrupt_handler();
}

// System configuration functions

/**
 * @brief Initialize system configuration
 */
void system_config_init(void)
{
    // Initialize global system status
    g_system_status.system_initialized = false;
    g_system_status.sensors_active = false;
    g_system_status.i2c_ready = false;
    g_system_status.voice_ready = false;
    g_system_status.current_warning = WARNING_LEVEL_NONE;
    g_system_status.power_mode = POWER_MODE_ACTIVE;
    g_system_status.error_count = 0;
    g_system_status.uptime_seconds = 0;
}

/**
 * @brief Configure system clock
 */
void system_clock_config(void)
{
    // Configure system clock to 48MHz
    // Configure peripheral clocks
    // This would be implemented with TI-specific clock configuration
}

/**
 * @brief Configure system GPIO
 */
void system_gpio_config(void)
{
    // Configure GPIO ports and pins
    // Set up alternate functions for I2C, UART, ADC
    // This would be implemented with TI-specific GPIO configuration
}

/**
 * @brief System error handler
 * @param error_msg Error message string
 * @param error_code Error code
 */
void system_error_handler(const char* error_msg, uint32_t error_code)
{
    // Log error (could be to EEPROM, UART, etc.)
    (void)error_msg; // Suppress unused parameter warning
    g_system_status.error_count++;
    
    // Add system error warning
    safety_warning_add_condition(WARNING_EVENT_SYSTEM_ERROR, WARNING_LEVEL_HIGH, 
                                error_code, 0);
    
    // Play error message if voice module is available
    if (g_system_status.voice_ready) {
        voice_module_play_message(VOICE_MSG_SYSTEM_ERROR, WARNING_LEVEL_HIGH);
    }
}

// Hardware abstraction layer implementations (placeholders for TI drivers)

static void hal_system_init(void)
{
    // Initialize TI M0G3507 system
    // Configure system clocks, power, etc.
}

static void hal_watchdog_init(void)
{
    // Initialize watchdog timer
}

static void hal_watchdog_refresh(void)
{
    // Refresh watchdog timer
}

static void hal_systick_init(void)
{
    // Initialize SysTick timer for 1ms interrupts
}

static void hal_power_management_init(void)
{
    // Initialize power management
}

static void hal_enter_standby_mode(void)
{
    // Enter low-power standby mode
}

static void hal_exit_standby_mode(void)
{
    // Exit standby mode
}

static uint32_t hal_get_system_tick(void)
{
    // Get current system tick in milliseconds
    return system_tick_counter;
}