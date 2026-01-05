/**
 * @file motor_control.h
 * @brief Motor control application layer - Pure business logic only
 * @author MultiAxesControl
 * @date 2025
 *
 * This layer contains ONLY application-level motor control logic.
 * No HAL details, no RTOS details, no hardware-specific code.
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Application status codes (platform-agnostic)
 */
typedef enum {
    APP_OK = 0,
    APP_ERROR,
    APP_BUSY,
    APP_TIMEOUT,
    APP_INVALID_PARAM
} app_status_t;

/**
 * @brief Motor type definitions (shared with HAL)
 */
#ifndef MOTOR_TYPES_DEFINED
#define MOTOR_TYPES_DEFINED

typedef enum {
    MOTOR_LINEAR = 0,
    MOTOR_ROTARY = 1,
    MOTOR_MAX = 2
} motor_id_t;

typedef enum {
    MOTOR_DIR_FORWARD = 0,
    MOTOR_DIR_REVERSE = 1
} motor_direction_t;

typedef enum {
    MOTOR_MODE_STOP = 0,
    MOTOR_MODE_JOG,
    MOTOR_MODE_INCREMENTAL,
    MOTOR_MODE_CYCLIC
} motor_mode_t;

#endif /* MOTOR_TYPES_DEFINED */

/**
 * @brief Command types
 */
typedef enum {
    CMD_MOVE_FORWARD = 'C',
    CMD_MOVE_REVERSE = 'D',
    CMD_STOP = 'S',
    CMD_CYCLIC = 'E',
    CMD_STEP_FORWARD = 'F',
    CMD_STEP_REVERSE = 'G',
    CMD_RESET = 'R'
} command_type_t;

/**
 * @brief Motor command structure
 */
typedef struct {
    command_type_t type;
    motor_id_t motor_id;
    uint32_t param1;    // Arc length for cyclic mode
    uint32_t param2;    // Number of cycles for cyclic mode
} motor_command_t;

/**
 * @brief Motor state
 */
typedef struct {
    motor_id_t id;
    motor_mode_t mode;
    motor_direction_t direction;
    uint32_t step_count;
    uint32_t target_steps;
    bool is_running;
} motor_state_t;

/**
 * @brief System configuration
 */
typedef struct {
    uint32_t max_frequency_linear;    // Hz
    uint32_t max_frequency_rotary;    // Hz
    uint32_t microsteps_linear;       // steps per mm
    uint32_t microsteps_rotary;       // steps per revolution
    float disc_radius;                // mm
    float circumference;              // mm
    uint32_t cyclic_delay_ms;         // ms
} motor_system_config_t;

/**
 * @brief Application initialization
 */
app_status_t motor_control_init(void);
app_status_t motor_control_deinit(void);

/**
 * @brief Command processing
 */
app_status_t motor_control_parse_command(const char *cmd_str, motor_command_t *command);
app_status_t motor_control_process_command(const motor_command_t *command);

/**
 * @brief Motor control operations
 */
app_status_t motor_control_start_jog(motor_id_t motor_id, motor_direction_t direction);
app_status_t motor_control_stop(motor_id_t motor_id);
app_status_t motor_control_start_incremental(motor_id_t motor_id, motor_direction_t direction);
app_status_t motor_control_start_cyclic(motor_id_t motor_id, uint32_t arc_length, uint32_t cycles);
app_status_t motor_control_update(void); // Called periodically to update motor states

/**
 * @brief State management
 */
app_status_t motor_control_get_state(motor_id_t motor_id, motor_state_t *state);
app_status_t motor_control_set_config(const motor_system_config_t *config);
app_status_t motor_control_get_config(motor_system_config_t *config);

/**
 * @brief Step notification (called by HAL when step pulse generated)
 */
void motor_control_step_notification(motor_id_t motor_id);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H */
