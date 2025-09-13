/**
 * @file voice_module.c
 * @brief Voice module implementation for TI M0G3507 safety warning system
 * @author TI M0G3507 Safety System
 * @date 2024
 */

#include "voice_module.h"
#include <string.h>

// Static variables
static voice_queue_t voice_queue;
static voice_config_t voice_config;
static voice_status_t voice_status;
static voice_command_t current_command;
static uint32_t voice_error_count = 0;
static bool voice_initialized = false;

// Voice message texts (for reference/debugging)
static const char* voice_message_texts[VOICE_MSG_COUNT] = {
    "",                                    // VOICE_MSG_NONE
    "System ready",                        // VOICE_MSG_SYSTEM_READY
    "Angle warning detected",              // VOICE_MSG_ANGLE_WARNING
    "Gas concentration warning",           // VOICE_MSG_GAS_WARNING
    "Vibration detected",                  // VOICE_MSG_VIBRATION_WARNING
    "System error occurred",               // VOICE_MSG_SYSTEM_ERROR
    "Sensor failure detected",             // VOICE_MSG_SENSOR_FAILURE
    "Emergency alarm activated",           // VOICE_MSG_EMERGENCY_ALARM
    "System reset initiated",              // VOICE_MSG_SYSTEM_RESET
    "Calibration completed"                // VOICE_MSG_CALIBRATION_COMPLETE
};

// Voice message durations in milliseconds
static const uint32_t voice_message_durations[VOICE_MSG_COUNT] = {
    0,      // VOICE_MSG_NONE
    2000,   // VOICE_MSG_SYSTEM_READY
    3000,   // VOICE_MSG_ANGLE_WARNING
    3000,   // VOICE_MSG_GAS_WARNING
    2500,   // VOICE_MSG_VIBRATION_WARNING
    2500,   // VOICE_MSG_SYSTEM_ERROR
    3000,   // VOICE_MSG_SENSOR_FAILURE
    3500,   // VOICE_MSG_EMERGENCY_ALARM
    2500,   // VOICE_MSG_SYSTEM_RESET
    2500    // VOICE_MSG_CALIBRATION_COMPLETE
};

// Hardware abstraction layer functions (to be implemented with TI drivers)
static bool hal_voice_uart_init(uint32_t baudrate);
static void hal_voice_uart_deinit(void);
static bool hal_voice_send_command(uint8_t command, uint8_t* data, uint8_t length);
static bool hal_voice_is_busy(void);
static void hal_voice_set_volume(uint8_t volume);
static bool hal_voice_play_message(uint8_t message_id);
static void hal_voice_stop_playback(void);

// Internal helper functions
static void voice_process_queue(void);
static void voice_process_current_command(void);
static uint32_t voice_get_system_time_ms(void);
static bool voice_start_playback(const voice_command_t* command);
static void voice_finish_current_command(void);

/**
 * @brief Initialize voice module
 * @return true if initialization successful, false otherwise
 */
bool voice_module_init(void)
{
    // Initialize voice queue
    memset(&voice_queue, 0, sizeof(voice_queue_t));
    
    // Initialize default configuration
    voice_config.uart_baudrate = VOICE_UART_BAUDRATE;
    voice_config.volume_level = 80;  // 80% volume
    voice_config.enabled = true;
    voice_config.max_response_time_ms = VOICE_RESPONSE_TIME_MS;
    voice_config.repeat_interval_ms = 5000;  // 5 seconds between repeats
    voice_config.priority_override = true;
    
    // Initialize status
    memset(&voice_status, 0, sizeof(voice_status_t));
    voice_status.state = VOICE_STATE_INIT;
    
    // Initialize current command
    memset(&current_command, 0, sizeof(voice_command_t));
    
    // Initialize hardware UART
    if (!hal_voice_uart_init(voice_config.uart_baudrate)) {
        voice_error_count++;
        return false;
    }
    
    // Set initial volume
    hal_voice_set_volume(voice_config.volume_level);
    
    // Mark as initialized
    voice_initialized = true;
    voice_status.state = VOICE_STATE_IDLE;
    voice_status.hardware_ready = true;
    g_system_status.voice_ready = true;
    
    // Play system ready message
    voice_module_play_message(VOICE_MSG_SYSTEM_READY, WARNING_LEVEL_LOW);
    
    return true;
}

/**
 * @brief Deinitialize voice module
 */
void voice_module_deinit(void)
{
    if (voice_initialized) {
        // Stop current playback
        voice_module_stop_current();
        
        // Clear queue
        voice_module_clear_queue();
        
        // Deinitialize hardware
        hal_voice_uart_deinit();
        
        voice_initialized = false;
        voice_status.hardware_ready = false;
        g_system_status.voice_ready = false;
    }
}

/**
 * @brief Play voice message with priority
 * @param message Message type to play
 * @param priority Message priority level
 * @return true if message queued successfully, false otherwise
 */
bool voice_module_play_message(voice_message_t message, warning_level_t priority)
{
    if (!voice_initialized || !voice_config.enabled || message >= VOICE_MSG_COUNT) {
        return false;
    }
    
    // Determine if message should repeat based on priority
    bool repeat = voice_should_repeat_message(message, priority);
    
    return voice_queue_add(message, priority, repeat);
}

/**
 * @brief Play warning message based on event and level
 * @param event Warning event type
 * @param level Warning level
 * @return true if message played successfully, false otherwise
 */
bool voice_module_play_warning(warning_event_t event, warning_level_t level)
{
    if (!voice_initialized) {
        return false;
    }
    
    voice_message_t message = VOICE_MSG_NONE;
    
    // Map warning events to voice messages
    switch (event) {
        case WARNING_EVENT_ANGLE_THRESHOLD:
            message = VOICE_MSG_ANGLE_WARNING;
            break;
            
        case WARNING_EVENT_GAS_THRESHOLD:
            message = VOICE_MSG_GAS_WARNING;
            break;
            
        case WARNING_EVENT_VIBRATION_DETECTED:
            message = VOICE_MSG_VIBRATION_WARNING;
            break;
            
        case WARNING_EVENT_SYSTEM_ERROR:
            message = VOICE_MSG_SYSTEM_ERROR;
            break;
            
        case WARNING_EVENT_SENSOR_FAILURE:
            message = VOICE_MSG_SENSOR_FAILURE;
            break;
            
        default:
            // For high-level warnings, use emergency alarm
            if (level >= WARNING_LEVEL_HIGH) {
                message = VOICE_MSG_EMERGENCY_ALARM;
            }
            break;
    }
    
    if (message != VOICE_MSG_NONE) {
        return voice_module_play_message(message, level);
    }
    
    return false;
}

/**
 * @brief Process voice module operations
 */
void voice_module_process(void)
{
    if (!voice_initialized) {
        return;
    }
    
    uint32_t current_time = voice_get_system_time_ms();
    voice_status.last_activity = current_time;
    
    // Process current command if playing
    if (voice_status.state == VOICE_STATE_PLAYING) {
        voice_process_current_command();
    }
    
    // Process queue if idle
    if (voice_status.state == VOICE_STATE_IDLE) {
        voice_process_queue();
    }
    
    // Check for hardware errors
    if (!voice_status.hardware_ready) {
        voice_status.state = VOICE_STATE_ERROR;
        voice_error_count++;
    }
}

/**
 * @brief Check if voice module is ready
 * @return true if ready, false otherwise
 */
bool voice_module_is_ready(void)
{
    return voice_initialized && voice_status.hardware_ready && 
           (voice_status.state == VOICE_STATE_IDLE);
}

/**
 * @brief Check if voice module is currently playing
 * @return true if playing, false otherwise
 */
bool voice_module_is_playing(void)
{
    return voice_status.state == VOICE_STATE_PLAYING;
}

/**
 * @brief Get current voice module state
 * @return Current voice module state
 */
voice_state_t voice_module_get_state(void)
{
    return voice_status.state;
}

/**
 * @brief Stop current voice playback
 */
void voice_module_stop_current(void)
{
    if (voice_status.state == VOICE_STATE_PLAYING) {
        hal_voice_stop_playback();
        voice_finish_current_command();
        voice_status.state = VOICE_STATE_IDLE;
    }
}

/**
 * @brief Clear voice message queue
 */
void voice_module_clear_queue(void)
{
    voice_queue_clear();
}

/**
 * @brief Get number of messages in queue
 * @return Number of queued messages
 */
uint8_t voice_module_get_queue_count(void)
{
    return voice_queue_get_count();
}

/**
 * @brief Set voice volume
 * @param volume_percent Volume level (0-100%)
 * @return true if volume set successfully, false otherwise
 */
bool voice_module_set_volume(uint8_t volume_percent)
{
    if (volume_percent > 100) {
        return false;
    }
    
    voice_config.volume_level = volume_percent;
    hal_voice_set_volume(volume_percent);
    
    return true;
}

/**
 * @brief Get current voice volume
 * @return Current volume level (0-100%)
 */
uint8_t voice_module_get_volume(void)
{
    return voice_config.volume_level;
}

/**
 * @brief Set voice module configuration
 * @param config Pointer to configuration structure
 * @return true if configuration set successfully, false otherwise
 */
bool voice_module_set_config(const voice_config_t* config)
{
    if (config == NULL) {
        return false;
    }
    
    memcpy(&voice_config, config, sizeof(voice_config_t));
    
    // Apply new configuration
    hal_voice_set_volume(voice_config.volume_level);
    
    return true;
}

/**
 * @brief Get voice module configuration
 * @param config Pointer to store configuration
 * @return true if configuration retrieved successfully, false otherwise
 */
bool voice_module_get_config(voice_config_t* config)
{
    if (config == NULL) {
        return false;
    }
    
    memcpy(config, &voice_config, sizeof(voice_config_t));
    return true;
}

/**
 * @brief Get voice module status
 * @param status Pointer to store status
 * @return true if status retrieved successfully, false otherwise
 */
bool voice_module_get_status(voice_status_t* status)
{
    if (status == NULL) {
        return false;
    }
    
    memcpy(status, &voice_status, sizeof(voice_status_t));
    return true;
}

/**
 * @brief Reset voice module counters
 */
void voice_module_reset_counters(void)
{
    voice_status.messages_played = 0;
    voice_status.queue_overflows = 0;
    voice_status.errors = 0;
    voice_error_count = 0;
}

/**
 * @brief Get voice module error count
 * @return Number of voice module errors
 */
uint32_t voice_module_get_error_count(void)
{
    return voice_error_count;
}

// Queue management functions

/**
 * @brief Add message to voice queue
 * @param message Message type to add
 * @param priority Message priority
 * @param repeat Enable message repeat
 * @return true if message added successfully, false otherwise
 */
bool voice_queue_add(voice_message_t message, warning_level_t priority, bool repeat)
{
    if (voice_queue_is_full()) {
        voice_status.queue_overflows++;
        
        // If priority override is enabled, replace lower priority message
        if (voice_config.priority_override) {
            // Find lowest priority message to replace
            uint8_t lowest_index = voice_queue.tail;
            warning_level_t lowest_priority = voice_queue.commands[lowest_index].priority;
            
            for (uint8_t i = 0; i < voice_queue.count; i++) {
                uint8_t index = (voice_queue.tail + i) % VOICE_QUEUE_SIZE;
                if (voice_queue.commands[index].priority < lowest_priority) {
                    lowest_index = index;
                    lowest_priority = voice_queue.commands[index].priority;
                }
            }
            
            // Replace if new message has higher priority
            if (priority > lowest_priority) {
                voice_command_t* cmd = &voice_queue.commands[lowest_index];
                cmd->message_type = message;
                cmd->priority = priority;
                cmd->timestamp = voice_get_system_time_ms();
                cmd->repeat_enabled = repeat;
                cmd->repeat_count = repeat ? 3 : 0;
                cmd->current_repeats = 0;
                return true;
            }
        }
        return false;
    }
    
    // Add to queue
    voice_command_t* cmd = &voice_queue.commands[voice_queue.head];
    cmd->message_type = message;
    cmd->priority = priority;
    cmd->timestamp = voice_get_system_time_ms();
    cmd->duration_ms = voice_get_message_duration(message);
    cmd->repeat_enabled = repeat;
    cmd->repeat_count = repeat ? 3 : 0;
    cmd->current_repeats = 0;
    
    voice_queue.head = (voice_queue.head + 1) % VOICE_QUEUE_SIZE;
    voice_queue.count++;
    
    if (voice_queue.count >= VOICE_QUEUE_SIZE) {
        voice_queue.full = true;
    }
    
    return true;
}

/**
 * @brief Get next message from queue
 * @param command Pointer to store command
 * @return true if command retrieved, false if queue empty
 */
bool voice_queue_get_next(voice_command_t* command)
{
    if (voice_queue_is_empty() || command == NULL) {
        return false;
    }
    
    memcpy(command, &voice_queue.commands[voice_queue.tail], sizeof(voice_command_t));
    
    voice_queue.tail = (voice_queue.tail + 1) % VOICE_QUEUE_SIZE;
    voice_queue.count--;
    voice_queue.full = false;
    
    return true;
}

/**
 * @brief Check if queue is empty
 * @return true if empty, false otherwise
 */
bool voice_queue_is_empty(void)
{
    return voice_queue.count == 0;
}

/**
 * @brief Check if queue is full
 * @return true if full, false otherwise
 */
bool voice_queue_is_full(void)
{
    return voice_queue.full;
}

/**
 * @brief Clear voice queue
 */
void voice_queue_clear(void)
{
    memset(&voice_queue, 0, sizeof(voice_queue_t));
}

/**
 * @brief Get queue count
 * @return Number of items in queue
 */
uint8_t voice_queue_get_count(void)
{
    return voice_queue.count;
}

// Message management functions

/**
 * @brief Get message text for debugging
 * @param message Message type
 * @return Pointer to message text
 */
const char* voice_get_message_text(voice_message_t message)
{
    if (message < VOICE_MSG_COUNT) {
        return voice_message_texts[message];
    }
    return "";
}

/**
 * @brief Get message duration
 * @param message Message type
 * @return Message duration in milliseconds
 */
uint32_t voice_get_message_duration(voice_message_t message)
{
    if (message < VOICE_MSG_COUNT) {
        return voice_message_durations[message];
    }
    return 0;
}

/**
 * @brief Check if message should repeat based on priority
 * @param message Message type
 * @param level Warning level
 * @return true if message should repeat, false otherwise
 */
bool voice_should_repeat_message(voice_message_t message, warning_level_t level)
{
    // High priority warnings should repeat
    if (level >= WARNING_LEVEL_HIGH) {
        return true;
    }
    
    // Emergency messages should always repeat
    if (message == VOICE_MSG_EMERGENCY_ALARM || 
        message == VOICE_MSG_SENSOR_FAILURE) {
        return true;
    }
    
    return false;
}

// Internal helper functions

/**
 * @brief Process voice queue
 */
static void voice_process_queue(void)
{
    if (!voice_queue_is_empty() && voice_status.state == VOICE_STATE_IDLE) {
        voice_command_t next_command;
        if (voice_queue_get_next(&next_command)) {
            memcpy(&current_command, &next_command, sizeof(voice_command_t));
            voice_start_playback(&current_command);
        }
    }
}

/**
 * @brief Process current playing command
 */
static void voice_process_current_command(void)
{
    uint32_t current_time = voice_get_system_time_ms();
    
    // Check if hardware is still busy
    if (hal_voice_is_busy()) {
        return;
    }
    
    // Check if playback duration has elapsed
    uint32_t elapsed = current_time - current_command.timestamp;
    if (elapsed >= current_command.duration_ms) {
        // Playback finished
        voice_status.messages_played++;
        
        // Check if message should repeat
        if (current_command.repeat_enabled && 
            current_command.current_repeats < current_command.repeat_count) {
            
            // Check repeat interval
            if (elapsed >= voice_config.repeat_interval_ms) {
                current_command.current_repeats++;
                current_command.timestamp = current_time;
                voice_start_playback(&current_command);
                return;
            }
        } else {
            // Finished playing (including repeats)
            voice_finish_current_command();
        }
    }
}

/**
 * @brief Get system time in milliseconds
 * @return Current system time in milliseconds
 */
static uint32_t voice_get_system_time_ms(void)
{
    return g_system_status.uptime_seconds * 1000;
}

/**
 * @brief Start playback of command
 * @param command Command to play
 * @return true if playback started, false otherwise
 */
static bool voice_start_playback(const voice_command_t* command)
{
    if (command == NULL || command->message_type >= VOICE_MSG_COUNT) {
        return false;
    }
    
    // Map message type to hardware message ID
    uint8_t message_id = (uint8_t)command->message_type;
    
    if (hal_voice_play_message(message_id)) {
        voice_status.state = VOICE_STATE_PLAYING;
        voice_status.current_message = command->message_type;
        return true;
    }
    
    voice_error_count++;
    return false;
}

/**
 * @brief Finish current command playback
 */
static void voice_finish_current_command(void)
{
    memset(&current_command, 0, sizeof(voice_command_t));
    voice_status.state = VOICE_STATE_IDLE;
    voice_status.current_message = VOICE_MSG_NONE;
}

// Hardware abstraction layer implementations (placeholders for TI drivers)
static bool hal_voice_uart_init(uint32_t baudrate)
{
    // Initialize UART for voice module communication
    (void)baudrate; // Suppress unused parameter warning
    return true; // Placeholder
}

static void hal_voice_uart_deinit(void)
{
    // Deinitialize UART
}

static bool hal_voice_send_command(uint8_t command, uint8_t* data, uint8_t length)
{
    // Send command to voice module via UART
    (void)command; (void)data; (void)length; // Suppress unused parameter warnings
    return true; // Placeholder
}

static bool hal_voice_is_busy(void)
{
    // Check if voice module is busy playing
    return false; // Placeholder
}

static void hal_voice_set_volume(uint8_t volume)
{
    // Set voice module volume (0-100%)
    (void)volume; // Suppress unused parameter warning
}

static bool hal_voice_play_message(uint8_t message_id)
{
    // Play specific message by ID
    (void)message_id; // Suppress unused parameter warning
    return true; // Placeholder
}

static void hal_voice_stop_playback(void)
{
    // Stop current voice playback
}