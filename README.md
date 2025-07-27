# TI M0G3507 Multi-Sensor Safety Warning System

A comprehensive safety warning system implementation for the TI M0G3507 microcontroller, featuring multi-sensor monitoring, real-time warning detection, and voice alerts.

## Features

### Core Functionality
- **I2C Slave Communication**: Interface with EC800MLE module for data exchange
- **ADC Sensor Reading**: Angle sensor and MQ-2 gas sensor monitoring
- **GPIO Sensor Reading**: Vibration sensor with debouncing
- **Safety Warning Logic**: Real-time threshold monitoring and alert generation
- **Voice Module**: Audio alerts with priority-based message queuing
- **Power Management**: Low-power standby mode (<1mA current consumption)

### Technical Specifications
- **Sensor Sampling Rate**: 10Hz
- **I2C Communication**: 100kHz, slave mode
- **Voice Response Time**: <100ms from warning trigger
- **Power Consumption**: <1mA in standby mode
- **ADC Resolution**: 12-bit
- **System Clock**: 48MHz

## File Structure

```
├── main.c              # Main application entry point
├── system_config.h     # System configuration and constants
├── i2c_slave.h/.c      # I2C slave communication module
├── adc_sensors.h/.c    # ADC sensor reading (angle + gas)
├── gpio_sensors.h/.c   # GPIO sensor reading (vibration)
├── safety_warning.h/.c # Safety warning logic and monitoring
├── voice_module.h/.c   # Voice alert system
├── Makefile           # Build configuration
└── README.md          # This file
```

## Hardware Requirements

### TI M0G3507 Microcontroller
- ARM Cortex-M0+ core
- 48MHz system clock
- I2C, ADC, GPIO, UART peripherals

### Sensors
- **Angle Sensor**: Connected to ADC channel 0
- **MQ-2 Gas Sensor**: Connected to ADC channel 1  
- **Vibration Sensor**: Connected to GPIO pin PA5

### Communication
- **I2C Interface**: Slave address 0x48, connects to EC800MLE
- **Voice Module**: UART interface at 9600 baud

## Building the Project

### Prerequisites
- ARM GCC toolchain (`arm-none-eabi-gcc`)
- Make utility
- TI M0G3507 linker script (`M0G3507.ld`)

### Build Commands
```bash
# Build all targets
make all

# Clean build files
make clean

# Display size information
make size

# Run static analysis (requires cppcheck)
make check

# Format code (requires clang-format)
make format

# Generate documentation (requires doxygen)
make docs
```

## System Architecture

### Modular Design
The system is designed with clear separation of concerns:

1. **Hardware Abstraction Layer (HAL)**: Low-level hardware interface
2. **Sensor Modules**: Individual sensor reading and processing
3. **Safety Logic**: Warning condition monitoring and state management
4. **Communication**: I2C slave interface for external control
5. **Voice Alerts**: Priority-based audio warning system
6. **Main Application**: System coordination and timing

### Safety Warning Levels
- **NONE**: Normal operation
- **LOW**: Minor threshold exceeded
- **MEDIUM**: Significant warning condition
- **HIGH**: Serious safety concern
- **CRITICAL**: Emergency condition requiring immediate attention

### Power Management
- **ACTIVE**: Full operation mode
- **STANDBY**: Reduced power consumption, continued monitoring
- **SLEEP**: Minimal power, wake on interrupt

## Configuration

### Sensor Thresholds
```c
#define ANGLE_THRESHOLD_DEG     45.0f    // Angle warning threshold
#define GAS_THRESHOLD_PPM       300U     // Gas concentration threshold
#define VIBRATION_DEBOUNCE_MS   50U      // Vibration debounce time
```

### I2C Communication
```c
#define I2C_SLAVE_ADDRESS       0x48     // 7-bit slave address
#define I2C_CLOCK_FREQ_HZ       100000U  // 100kHz I2C clock
```

### Voice Module
```c
#define VOICE_RESPONSE_TIME_MS  100U     // Maximum response time
#define VOICE_UART_BAUDRATE     9600U    // UART baud rate
```

## API Reference

### Sensor Reading
```c
// Read angle sensor data
bool adc_sensors_get_angle_data(angle_sensor_data_t* data);

// Read gas sensor data  
bool adc_sensors_get_gas_data(gas_sensor_data_t* data);

// Read vibration sensor data
bool gpio_sensors_read_vibration(vibration_sensor_data_t* data);
```

### Safety Monitoring
```c
// Get current warning level
warning_level_t safety_warning_get_level(void);

// Check if specific warning is active
bool safety_warning_is_active(warning_event_t event_type);

// Set safety thresholds
bool safety_warning_set_thresholds(const safety_thresholds_t* thresholds);
```

### Voice Alerts
```c
// Play warning message
bool voice_module_play_warning(warning_event_t event, warning_level_t level);

// Set volume level
bool voice_module_set_volume(uint8_t volume_percent);
```

### I2C Communication
```c
// Commands supported by I2C slave interface
#define I2C_CMD_GET_SENSOR_DATA     0x01
#define I2C_CMD_GET_WARNING_STATUS  0x02
#define I2C_CMD_SET_THRESHOLDS      0x03
#define I2C_CMD_SYSTEM_RESET        0x04
#define I2C_CMD_GET_SYSTEM_STATUS   0x05
```

## Testing and Validation

### Unit Testing
Each module includes comprehensive error handling and validation:
- Sensor reading validation
- Communication timeout handling
- Warning condition verification
- Voice module queue management

### System Integration
- Sensor sampling frequency verification (10Hz)
- I2C communication timing (100kHz)
- Voice response time measurement (<100ms)
- Power consumption validation (<1mA standby)

## Error Handling

### Error Recovery
- Automatic module reinitialization on excessive errors
- Watchdog timer protection
- System reset on critical failures
- Error logging and reporting

### Safety Mechanisms
- Sensor health monitoring
- Communication link verification
- Warning acknowledgment system
- Emergency override capabilities

## Deployment

### Programming
```bash
# Flash to target device (example with uniflash)
make install
```

### Debugging
```bash
# Start debug session
make debug
```

### Production Testing
- Sensor calibration verification
- Communication interface testing
- Voice module functionality check
- Power consumption measurement

## License

This project is developed for educational/commercial use with TI M0G3507 microcontroller applications.

## Contributors

- TI M0G3507 Safety System Team
- Embedded Systems Development

## Support

For technical support and questions:
- Review the source code documentation
- Check the build logs for compilation issues
- Verify hardware connections and power supply
- Ensure proper sensor calibration
