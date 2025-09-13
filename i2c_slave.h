/**
 * @file i2c_slave.h
 * @brief I2C slave communication module header for TI M0G3507
 * @author TI M0G3507 Safety System
 * @date 2024
 */

#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H

#include "system_config.h"

// I2C communication states
typedef enum {
    I2C_STATE_IDLE = 0,
    I2C_STATE_RECEIVING,
    I2C_STATE_TRANSMITTING,
    I2C_STATE_ERROR
} i2c_state_t;

// I2C command codes
#define I2C_CMD_GET_SENSOR_DATA     0x01
#define I2C_CMD_GET_WARNING_STATUS  0x02
#define I2C_CMD_SET_THRESHOLDS      0x03
#define I2C_CMD_SYSTEM_RESET        0x04
#define I2C_CMD_GET_SYSTEM_STATUS   0x05

// I2C data structures
typedef struct {
    float angle_deg;
    uint16_t gas_ppm;
    bool vibration_detected;
    warning_level_t warning_level;
    uint32_t timestamp;
} i2c_sensor_data_t;

typedef struct {
    float angle_threshold;
    uint16_t gas_threshold;
    uint16_t vibration_debounce_ms;
} i2c_threshold_data_t;

typedef struct {
    uint8_t system_version;
    bool system_ready;
    uint32_t uptime_seconds;
    uint32_t error_count;
    power_mode_t power_mode;
} i2c_system_status_t;

// I2C communication buffer
#define I2C_BUFFER_SIZE     32U
typedef struct {
    uint8_t tx_buffer[I2C_BUFFER_SIZE];
    uint8_t rx_buffer[I2C_BUFFER_SIZE];
    uint8_t tx_index;
    uint8_t rx_index;
    uint8_t tx_length;
    uint8_t rx_length;
    i2c_state_t state;
    uint32_t last_activity;
} i2c_comm_buffer_t;

// Function prototypes
bool i2c_slave_init(void);
void i2c_slave_deinit(void);
bool i2c_slave_is_ready(void);
void i2c_slave_process(void);
void i2c_slave_set_sensor_data(const i2c_sensor_data_t* data);
bool i2c_slave_get_thresholds(i2c_threshold_data_t* thresholds);
void i2c_slave_interrupt_handler(void);
i2c_state_t i2c_slave_get_state(void);
uint32_t i2c_slave_get_error_count(void);
void i2c_slave_reset_errors(void);

#endif // I2C_SLAVE_H