/**
 * @file system_config.h
 * @brief System configuration for TI M0G3507 multi-sensor safety warning system
 * @author TI M0G3507 Safety System
 * @date 2024
 */

#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

// System clock configuration
#define SYSTEM_CLOCK_HZ         48000000U   // 48MHz system clock
#define PERIPHERAL_CLOCK_HZ     24000000U   // 24MHz peripheral clock

// Sensor sampling configuration
#define SENSOR_SAMPLING_FREQ_HZ     10U     // 10Hz sampling frequency
#define SENSOR_SAMPLING_PERIOD_MS   (1000U / SENSOR_SAMPLING_FREQ_HZ)

// I2C configuration
#define I2C_SLAVE_ADDRESS       0x48        // 7-bit slave address
#define I2C_CLOCK_FREQ_HZ       100000U     // 100kHz I2C clock
#define I2C_TIMEOUT_MS          1000U       // I2C timeout

// ADC configuration
#define ADC_RESOLUTION_BITS     12U         // 12-bit ADC resolution
#define ADC_MAX_VALUE           ((1U << ADC_RESOLUTION_BITS) - 1U)
#define ADC_VREF_MV             3300U       // 3.3V reference voltage

// ADC channel assignments
#define ADC_CHANNEL_ANGLE       0U          // Angle sensor ADC channel
#define ADC_CHANNEL_GAS         1U          // MQ-2 gas sensor ADC channel

// GPIO pin assignments
#define GPIO_VIBRATION_PORT     ((void*)0x40010000)  // GPIO port A base address
#define GPIO_VIBRATION_PIN      5U          // Vibration sensor GPIO pin

// Sensor thresholds and calibration
#define ANGLE_MIN_DEG           0.0f        // Minimum angle in degrees
#define ANGLE_MAX_DEG           360.0f      // Maximum angle in degrees
#define ANGLE_THRESHOLD_DEG     45.0f       // Angle warning threshold

#define GAS_THRESHOLD_PPM       300U        // Gas concentration threshold (ppm)
#define GAS_CALIBRATION_R0      10000U      // MQ-2 sensor R0 resistance

#define VIBRATION_DEBOUNCE_MS   50U         // Vibration sensor debounce time

// Safety warning levels
typedef enum {
    WARNING_LEVEL_NONE = 0,
    WARNING_LEVEL_LOW,
    WARNING_LEVEL_MEDIUM,
    WARNING_LEVEL_HIGH,
    WARNING_LEVEL_CRITICAL
} warning_level_t;

// System power modes
typedef enum {
    POWER_MODE_ACTIVE = 0,
    POWER_MODE_STANDBY,
    POWER_MODE_SLEEP
} power_mode_t;

// Voice module configuration
#define VOICE_RESPONSE_TIME_MS  100U        // Maximum response time for voice alerts
#define VOICE_UART_BAUDRATE     9600U       // Voice module UART baud rate

// System timing configuration
#define MAIN_LOOP_PERIOD_MS     100U        // Main loop period (100ms)
#define WATCHDOG_TIMEOUT_MS     5000U       // Watchdog timeout

// Power management
#define STANDBY_CURRENT_TARGET_UA   1000U   // Target standby current (1mA)
#define SLEEP_WAKEUP_SOURCES        (RTC_WAKEUP | GPIO_WAKEUP)

// Error handling
#define MAX_ERROR_COUNT         10U         // Maximum consecutive errors before reset
#define ERROR_LOG_SIZE          32U         // Error log buffer size

// System status flags
typedef struct {
    bool system_initialized;
    bool sensors_active;
    bool i2c_ready;
    bool voice_ready;
    warning_level_t current_warning;
    power_mode_t power_mode;
    uint32_t error_count;
    uint32_t uptime_seconds;
} system_status_t;

// Global system status
extern system_status_t g_system_status;

// Function prototypes for system utilities
void system_config_init(void);
void system_clock_config(void);
void system_gpio_config(void);
void system_error_handler(const char* error_msg, uint32_t error_code);

#endif // SYSTEM_CONFIG_H