/**
 * @file voice_module.h
 * @brief Voice module header for TI M0G3507 safety warning system
 * @author TI M0G3507 Safety System
 * @date 2024
 */

#ifndef VOICE_MODULE_H
#define VOICE_MODULE_H

#include "system_config.h"
#include "safety_warning.h"

// Voice message types
typedef enum {
    VOICE_MSG_NONE = 0,
    VOICE_MSG_SYSTEM_READY,
    VOICE_MSG_ANGLE_WARNING,
    VOICE_MSG_GAS_WARNING,
    VOICE_MSG_VIBRATION_WARNING,
    VOICE_MSG_SYSTEM_ERROR,
    VOICE_MSG_SENSOR_FAILURE,
    VOICE_MSG_EMERGENCY_ALARM,
    VOICE_MSG_SYSTEM_RESET,
    VOICE_MSG_CALIBRATION_COMPLETE,
    VOICE_MSG_COUNT
} voice_message_t;

// Voice module state
typedef enum {
    VOICE_STATE_IDLE = 0,
    VOICE_STATE_PLAYING,
    VOICE_STATE_BUSY,
    VOICE_STATE_ERROR,
    VOICE_STATE_INIT
} voice_state_t;

// Voice command structure
typedef struct {
    voice_message_t message_type;
    warning_level_t priority;
    uint32_t timestamp;
    uint32_t duration_ms;
    bool repeat_enabled;
    uint8_t repeat_count;
    uint8_t current_repeats;
} voice_command_t;

// Voice queue configuration
#define VOICE_QUEUE_SIZE    8
typedef struct {
    voice_command_t commands[VOICE_QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
    bool full;
} voice_queue_t;

// Voice module configuration
typedef struct {
    uint32_t uart_baudrate;
    uint8_t volume_level;          // 0-100%
    bool enabled;
    uint32_t max_response_time_ms;
    uint32_t repeat_interval_ms;
    bool priority_override;
} voice_config_t;

// Voice module status
typedef struct {
    voice_state_t state;
    voice_message_t current_message;
    uint32_t messages_played;
    uint32_t queue_overflows;
    uint32_t errors;
    uint32_t last_activity;
    bool hardware_ready;
} voice_status_t;

// Function prototypes
bool voice_module_init(void);
void voice_module_deinit(void);
bool voice_module_play_message(voice_message_t message, warning_level_t priority);
bool voice_module_play_warning(warning_event_t event, warning_level_t level);
void voice_module_process(void);
bool voice_module_is_ready(void);
bool voice_module_is_playing(void);
voice_state_t voice_module_get_state(void);
void voice_module_stop_current(void);
void voice_module_clear_queue(void);
uint8_t voice_module_get_queue_count(void);
bool voice_module_set_volume(uint8_t volume_percent);
uint8_t voice_module_get_volume(void);
bool voice_module_set_config(const voice_config_t* config);
bool voice_module_get_config(voice_config_t* config);
bool voice_module_get_status(voice_status_t* status);
void voice_module_reset_counters(void);
uint32_t voice_module_get_error_count(void);

// Queue management
bool voice_queue_add(voice_message_t message, warning_level_t priority, bool repeat);
bool voice_queue_get_next(voice_command_t* command);
bool voice_queue_is_empty(void);
bool voice_queue_is_full(void);
void voice_queue_clear(void);
uint8_t voice_queue_get_count(void);

// Message management
const char* voice_get_message_text(voice_message_t message);
uint32_t voice_get_message_duration(voice_message_t message);
bool voice_should_repeat_message(voice_message_t message, warning_level_t level);

#endif // VOICE_MODULE_H