/**
 * @file i2c_slave.c
 * @brief I2C slave communication module implementation for TI M0G3507
 * @author TI M0G3507 Safety System
 * @date 2024
 */

#include "i2c_slave.h"
#include <string.h>

// Forward declarations for internal functions
static void i2c_handle_received_command(void);
static void i2c_prepare_sensor_data_response(void);
static void i2c_prepare_warning_status_response(void);
static void i2c_handle_set_thresholds(void);
static void i2c_handle_system_reset(void);
static void i2c_prepare_system_status_response(void);

// Static variables
static i2c_comm_buffer_t i2c_buffer;
static i2c_sensor_data_t current_sensor_data;
static i2c_threshold_data_t current_thresholds;
static uint32_t i2c_error_count = 0;

// Hardware abstraction layer functions (to be implemented with TI drivers)
static bool hal_i2c_init(uint8_t slave_address, uint32_t clock_freq);
static void hal_i2c_deinit(void);
static void hal_i2c_enable_interrupts(void);
static void hal_i2c_disable_interrupts(void);
static bool hal_i2c_send_data(const uint8_t* data, uint8_t length);
static uint8_t hal_i2c_receive_data(uint8_t* data, uint8_t max_length);

/**
 * @brief Initialize I2C slave module
 * @return true if initialization successful, false otherwise
 */
bool i2c_slave_init(void)
{
    // Initialize communication buffer
    memset(&i2c_buffer, 0, sizeof(i2c_buffer));
    i2c_buffer.state = I2C_STATE_IDLE;
    
    // Initialize sensor data structure
    memset(&current_sensor_data, 0, sizeof(current_sensor_data));
    
    // Initialize default thresholds
    current_thresholds.angle_threshold = ANGLE_THRESHOLD_DEG;
    current_thresholds.gas_threshold = GAS_THRESHOLD_PPM;
    current_thresholds.vibration_debounce_ms = VIBRATION_DEBOUNCE_MS;
    
    // Initialize hardware I2C peripheral
    if (!hal_i2c_init(I2C_SLAVE_ADDRESS, I2C_CLOCK_FREQ_HZ)) {
        i2c_error_count++;
        return false;
    }
    
    // Enable I2C interrupts
    hal_i2c_enable_interrupts();
    
    g_system_status.i2c_ready = true;
    return true;
}

/**
 * @brief Deinitialize I2C slave module
 */
void i2c_slave_deinit(void)
{
    hal_i2c_disable_interrupts();
    hal_i2c_deinit();
    
    i2c_buffer.state = I2C_STATE_IDLE;
    g_system_status.i2c_ready = false;
}

/**
 * @brief Check if I2C slave is ready for communication
 * @return true if ready, false otherwise
 */
bool i2c_slave_is_ready(void)
{
    return (i2c_buffer.state != I2C_STATE_ERROR) && g_system_status.i2c_ready;
}

/**
 * @brief Process I2C communication
 */
void i2c_slave_process(void)
{
    if (!g_system_status.i2c_ready) {
        return;
    }
    
    switch (i2c_buffer.state) {
        case I2C_STATE_IDLE:
            // Nothing to do in idle state
            break;
            
        case I2C_STATE_RECEIVING:
            // Handle received command
            if (i2c_buffer.rx_length > 0) {
                i2c_handle_received_command();
                i2c_buffer.state = I2C_STATE_IDLE;
            }
            break;
            
        case I2C_STATE_TRANSMITTING:
            // Check if transmission is complete
            if (i2c_buffer.tx_index >= i2c_buffer.tx_length) {
                i2c_buffer.state = I2C_STATE_IDLE;
            }
            break;
            
        case I2C_STATE_ERROR:
            // Handle error state
            i2c_error_count++;
            if (i2c_error_count < MAX_ERROR_COUNT) {
                // Try to recover
                i2c_buffer.state = I2C_STATE_IDLE;
            } else {
                // Reset I2C module
                i2c_slave_deinit();
                i2c_slave_init();
            }
            break;
    }
}

/**
 * @brief Set current sensor data for transmission
 * @param data Pointer to sensor data structure
 */
void i2c_slave_set_sensor_data(const i2c_sensor_data_t* data)
{
    if (data != NULL) {
        memcpy(&current_sensor_data, data, sizeof(i2c_sensor_data_t));
    }
}

/**
 * @brief Get current threshold settings
 * @param thresholds Pointer to threshold data structure to fill
 * @return true if successful, false otherwise
 */
bool i2c_slave_get_thresholds(i2c_threshold_data_t* thresholds)
{
    if (thresholds == NULL) {
        return false;
    }
    
    memcpy(thresholds, &current_thresholds, sizeof(i2c_threshold_data_t));
    return true;
}

/**
 * @brief Handle received I2C command
 */
static void i2c_handle_received_command(void)
{
    if (i2c_buffer.rx_length == 0) {
        return;
    }
    
    uint8_t command = i2c_buffer.rx_buffer[0];
    
    switch (command) {
        case I2C_CMD_GET_SENSOR_DATA:
            i2c_prepare_sensor_data_response();
            break;
            
        case I2C_CMD_GET_WARNING_STATUS:
            i2c_prepare_warning_status_response();
            break;
            
        case I2C_CMD_SET_THRESHOLDS:
            i2c_handle_set_thresholds();
            break;
            
        case I2C_CMD_SYSTEM_RESET:
            i2c_handle_system_reset();
            break;
            
        case I2C_CMD_GET_SYSTEM_STATUS:
            i2c_prepare_system_status_response();
            break;
            
        default:
            // Unknown command - send error response
            i2c_buffer.tx_buffer[0] = 0xFF;
            i2c_buffer.tx_length = 1;
            i2c_error_count++;
            break;
    }
}

/**
 * @brief Prepare sensor data response
 */
static void i2c_prepare_sensor_data_response(void)
{
    uint8_t* buf = i2c_buffer.tx_buffer;
    uint8_t index = 0;
    
    // Pack sensor data into transmission buffer
    memcpy(&buf[index], &current_sensor_data.angle_deg, sizeof(float));
    index += sizeof(float);
    
    memcpy(&buf[index], &current_sensor_data.gas_ppm, sizeof(uint16_t));
    index += sizeof(uint16_t);
    
    buf[index++] = current_sensor_data.vibration_detected ? 1 : 0;
    buf[index++] = (uint8_t)current_sensor_data.warning_level;
    
    memcpy(&buf[index], &current_sensor_data.timestamp, sizeof(uint32_t));
    index += sizeof(uint32_t);
    
    i2c_buffer.tx_length = index;
    i2c_buffer.tx_index = 0;
    i2c_buffer.state = I2C_STATE_TRANSMITTING;
}

/**
 * @brief Prepare warning status response
 */
static void i2c_prepare_warning_status_response(void)
{
    i2c_buffer.tx_buffer[0] = (uint8_t)current_sensor_data.warning_level;
    i2c_buffer.tx_buffer[1] = current_sensor_data.vibration_detected ? 1 : 0;
    i2c_buffer.tx_length = 2;
    i2c_buffer.tx_index = 0;
    i2c_buffer.state = I2C_STATE_TRANSMITTING;
}

/**
 * @brief Handle set thresholds command
 */
static void i2c_handle_set_thresholds(void)
{
    if (i2c_buffer.rx_length < (1 + sizeof(i2c_threshold_data_t))) {
        i2c_error_count++;
        return;
    }
    
    // Extract threshold data from receive buffer
    memcpy(&current_thresholds, &i2c_buffer.rx_buffer[1], sizeof(i2c_threshold_data_t));
    
    // Send acknowledgment
    i2c_buffer.tx_buffer[0] = 0x00; // Success
    i2c_buffer.tx_length = 1;
    i2c_buffer.tx_index = 0;
    i2c_buffer.state = I2C_STATE_TRANSMITTING;
}

/**
 * @brief Handle system reset command
 */
static void i2c_handle_system_reset(void)
{
    // Send acknowledgment first
    i2c_buffer.tx_buffer[0] = 0x00; // Success
    i2c_buffer.tx_length = 1;
    i2c_buffer.tx_index = 0;
    i2c_buffer.state = I2C_STATE_TRANSMITTING;
    
    // Schedule system reset (to be implemented in main loop)
    g_system_status.error_count = MAX_ERROR_COUNT;
}

/**
 * @brief Prepare system status response
 */
static void i2c_prepare_system_status_response(void)
{
    i2c_system_status_t status;
    status.system_version = 1;
    status.system_ready = g_system_status.system_initialized;
    status.uptime_seconds = g_system_status.uptime_seconds;
    status.error_count = g_system_status.error_count;
    status.power_mode = g_system_status.power_mode;
    
    memcpy(i2c_buffer.tx_buffer, &status, sizeof(i2c_system_status_t));
    i2c_buffer.tx_length = sizeof(i2c_system_status_t);
    i2c_buffer.tx_index = 0;
    i2c_buffer.state = I2C_STATE_TRANSMITTING;
}

/**
 * @brief I2C interrupt handler
 */
void i2c_slave_interrupt_handler(void)
{
    // This will be implemented with TI-specific interrupt handling
    // For now, it's a placeholder that would handle:
    // - Address match
    // - Data received
    // - Data transmitted
    // - Stop condition
    // - Error conditions
    
    i2c_buffer.last_activity = g_system_status.uptime_seconds;
}

/**
 * @brief Get current I2C state
 * @return Current I2C communication state
 */
i2c_state_t i2c_slave_get_state(void)
{
    return i2c_buffer.state;
}

/**
 * @brief Get I2C error count
 * @return Number of I2C errors
 */
uint32_t i2c_slave_get_error_count(void)
{
    return i2c_error_count;
}

/**
 * @brief Reset I2C error count
 */
void i2c_slave_reset_errors(void)
{
    i2c_error_count = 0;
}

// Hardware abstraction layer implementations (placeholders for TI drivers)
static bool hal_i2c_init(uint8_t slave_address, uint32_t clock_freq)
{
    // Initialize TI I2C peripheral
    // Configure pins, clock, slave address
    (void)slave_address; (void)clock_freq; // Suppress unused parameter warnings
    return true; // Placeholder
}

static void hal_i2c_deinit(void)
{
    // Deinitialize TI I2C peripheral
}

static void hal_i2c_enable_interrupts(void)
{
    // Enable I2C interrupts in NVIC
}

static void hal_i2c_disable_interrupts(void)
{
    // Disable I2C interrupts in NVIC
}

static bool hal_i2c_send_data(const uint8_t* data, uint8_t length)
{
    // Send data via I2C
    (void)data; (void)length; // Suppress unused parameter warnings
    return true; // Placeholder
}

static uint8_t hal_i2c_receive_data(uint8_t* data, uint8_t max_length)
{
    // Receive data via I2C
    (void)data; (void)max_length; // Suppress unused parameter warnings
    return 0; // Placeholder
}