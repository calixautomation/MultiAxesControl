/**
 * @file motor_control.c
 * @brief Motor control application implementation - Pure business logic
 * @author MultiAxesControl
 * @date 2025
 *
 * This layer is completely platform-agnostic and RTOS-agnostic.
 * All hardware and RTOS interactions go through the HAL interface.
 */

#include "motor_control.h"
#include "../hal/hal_interface.h"
#include <string.h>
#include <stdlib.h>

// Motor state tracking
static motor_state_t g_motors[MOTOR_MAX];
static motor_system_config_t g_system_config;
static bool g_system_initialized = false;

// Default configuration
static const motor_system_config_t default_config = {
    .max_frequency_linear = 200,      // 200Hz
    .max_frequency_rotary = 36,       // 36Hz
    .microsteps_linear = 1600,        // 1600 microsteps per mm
    .microsteps_rotary = 6400,        // 6400 microsteps per revolution
    .disc_radius = 15.0f,             // 15mm radius
    .circumference = 94.247779f,      // 2 * PI * 15
    .cyclic_delay_ms = 500            // 500ms delay
};

//=============================================================================
// Initialization
//=============================================================================

app_status_t motor_control_init(void) {
    // Initialize motor states
    memset(g_motors, 0, sizeof(g_motors));
    for (int i = 0; i < MOTOR_MAX; i++) {
        g_motors[i].id = (motor_id_t)i;
        g_motors[i].mode = MOTOR_MODE_STOP;
        g_motors[i].is_running = false;
        g_motors[i].step_count = 0;
        g_motors[i].target_steps = 0;
    }

    // Set default configuration
    g_system_config = default_config;

    g_system_initialized = true;
    return APP_OK;
}

app_status_t motor_control_deinit(void) {
    // Stop all motors
    for (int i = 0; i < MOTOR_MAX; i++) {
        motor_control_stop((motor_id_t)i);
    }

    // Deinitialize HAL
    hal_deinit();

    g_system_initialized = false;
    return APP_OK;
}

//=============================================================================
// Command Parsing
//=============================================================================

app_status_t motor_control_parse_command(const char *cmd_str, motor_command_t *command) {
    if (!cmd_str || !command) {
        return APP_INVALID_PARAM;
    }

    // Clear command structure
    memset(command, 0, sizeof(motor_command_t));

    // Minimum command length check
    if (strlen(cmd_str) < 2) {
        return APP_INVALID_PARAM;
    }

    // Parse command type (first character)
    command->type = (command_type_t)cmd_str[0];

    // Parse motor ID (second character: '1' = linear, '2' = rotary)
    // Exception: 'R' (reset) command has no motor ID
    if (command->type != CMD_RESET) {
        if (cmd_str[1] >= '1' && cmd_str[1] <= '2') {
            command->motor_id = (motor_id_t)(cmd_str[1] - '1');
        } else {
            return APP_INVALID_PARAM;
        }
    }

    // Parse parameters for cyclic mode (E2:10:5 format)
    if (command->type == CMD_CYCLIC && strlen(cmd_str) > 3) {
        // Make a copy since strtok modifies the string
        char temp[32];
        strncpy(temp, &cmd_str[3], sizeof(temp) - 1);
        temp[sizeof(temp) - 1] = '\0';

        char *token = strtok(temp, ":");
        if (token) {
            command->param1 = atoi(token); // Arc length in mm
        }

        token = strtok(NULL, ":");
        if (token) {
            command->param2 = atoi(token); // Number of cycles
        }

        if (command->param1 == 0 || command->param2 == 0) {
            return APP_INVALID_PARAM;
        }
    }

    return APP_OK;
}

//=============================================================================
// Command Processing
//=============================================================================

app_status_t motor_control_process_command(const motor_command_t *command) {
    if (!command) {
        return APP_INVALID_PARAM;
    }

    if (!g_system_initialized) {
        return APP_ERROR;
    }

    // Validate motor ID (except for reset command)
    if (command->type != CMD_RESET && command->motor_id >= MOTOR_MAX) {
        return APP_INVALID_PARAM;
    }

    app_status_t status = APP_OK;

    switch (command->type) {
        case CMD_MOVE_FORWARD:
            status = motor_control_start_jog(command->motor_id, MOTOR_DIR_FORWARD);
            break;

        case CMD_MOVE_REVERSE:
            status = motor_control_start_jog(command->motor_id, MOTOR_DIR_REVERSE);
            break;

        case CMD_STOP:
            status = motor_control_stop(command->motor_id);
            break;

        case CMD_STEP_FORWARD:
            status = motor_control_start_incremental(command->motor_id, MOTOR_DIR_FORWARD);
            break;

        case CMD_STEP_REVERSE:
            status = motor_control_start_incremental(command->motor_id, MOTOR_DIR_REVERSE);
            break;

        case CMD_CYCLIC:
            // Only rotary motor supports cyclic mode
            if (command->motor_id == MOTOR_ROTARY) {
                status = motor_control_start_cyclic(command->motor_id,
                                                     command->param1,
                                                     command->param2);
            } else {
                status = APP_INVALID_PARAM;
            }
            break;

        case CMD_RESET:
            hal_system_reset();
            // Never returns
            break;

        default:
            status = APP_INVALID_PARAM;
            break;
    }

    return status;
}

//=============================================================================
// Motor Control Operations
//=============================================================================

app_status_t motor_control_start_jog(motor_id_t motor_id, motor_direction_t direction) {
    if (motor_id >= MOTOR_MAX) {
        return APP_INVALID_PARAM;
    }

    // Update state
    g_motors[motor_id].mode = MOTOR_MODE_JOG;
    g_motors[motor_id].direction = direction;
    g_motors[motor_id].is_running = true;
    g_motors[motor_id].step_count = 0;

    // Set direction in HAL
    hal_status_t hal_status = hal_motor_set_direction(motor_id, direction);
    if (hal_status != CUSTOM_HAL_OK) {
        return APP_ERROR;
    }

    // Set frequency
    uint32_t frequency = (motor_id == MOTOR_LINEAR) ?
                         g_system_config.max_frequency_linear :
                         g_system_config.max_frequency_rotary;

    hal_status = hal_timer_set_frequency(motor_id, frequency);
    if (hal_status != CUSTOM_HAL_OK) {
        return APP_ERROR;
    }

    // Start motor
    hal_status = hal_motor_start(motor_id);
    if (hal_status != CUSTOM_HAL_OK) {
        return APP_ERROR;
    }

    return APP_OK;
}

app_status_t motor_control_stop(motor_id_t motor_id) {
    if (motor_id >= MOTOR_MAX) {
        return APP_INVALID_PARAM;
    }

    // Update state
    g_motors[motor_id].mode = MOTOR_MODE_STOP;
    g_motors[motor_id].is_running = false;

    // Stop motor in HAL
    hal_status_t hal_status = hal_motor_stop(motor_id);
    if (hal_status != CUSTOM_HAL_OK) {
        return APP_ERROR;
    }

    return APP_OK;
}

app_status_t motor_control_start_incremental(motor_id_t motor_id, motor_direction_t direction) {
    if (motor_id >= MOTOR_MAX) {
        return APP_INVALID_PARAM;
    }

    // Update state
    g_motors[motor_id].mode = MOTOR_MODE_INCREMENTAL;
    g_motors[motor_id].direction = direction;
    g_motors[motor_id].step_count = 0;
    g_motors[motor_id].target_steps = 1; // Single step
    g_motors[motor_id].is_running = true;

    // Set direction in HAL
    hal_status_t hal_status = hal_motor_set_direction(motor_id, direction);
    if (hal_status != CUSTOM_HAL_OK) {
        return APP_ERROR;
    }

    // Set frequency
    uint32_t frequency = (motor_id == MOTOR_LINEAR) ?
                         g_system_config.max_frequency_linear :
                         g_system_config.max_frequency_rotary;

    hal_status = hal_timer_set_frequency(motor_id, frequency);
    if (hal_status != CUSTOM_HAL_OK) {
        return APP_ERROR;
    }

    // Start motor (will be stopped after one step by step notification)
    hal_status = hal_motor_start(motor_id);
    if (hal_status != CUSTOM_HAL_OK) {
        return APP_ERROR;
    }

    return APP_OK;
}

app_status_t motor_control_start_cyclic(motor_id_t motor_id, uint32_t arc_length, uint32_t cycles) {
    if (motor_id >= MOTOR_MAX || motor_id != MOTOR_ROTARY) {
        return APP_INVALID_PARAM;
    }

    if (arc_length == 0 || cycles == 0) {
        return APP_INVALID_PARAM;
    }

    // Calculate steps for the arc length
    uint32_t steps = (uint32_t)((float)arc_length / g_system_config.circumference *
                                 (float)g_system_config.microsteps_rotary);

    if (steps == 0) {
        return APP_INVALID_PARAM;
    }

    // Update state
    g_motors[motor_id].mode = MOTOR_MODE_CYCLIC;
    g_motors[motor_id].target_steps = steps;
    g_motors[motor_id].step_count = 0;
    g_motors[motor_id].is_running = true;

    // Set frequency
    hal_status_t hal_status = hal_timer_set_frequency(motor_id, g_system_config.max_frequency_rotary);
    if (hal_status != CUSTOM_HAL_OK) {
        return APP_ERROR;
    }

    // Perform cyclic movement
    for (uint32_t cycle = 0; cycle < cycles; cycle++) {
        // Forward movement
        g_motors[motor_id].direction = MOTOR_DIR_FORWARD;
        g_motors[motor_id].step_count = 0;
        hal_motor_set_direction(motor_id, MOTOR_DIR_FORWARD);
        hal_motor_start(motor_id);

        // Wait for completion (checked by motor_control_update or step notification)
        while (g_motors[motor_id].step_count < steps) {
            hal_delay_ms(10);
        }
        hal_motor_stop(motor_id);
        hal_delay_ms(g_system_config.cyclic_delay_ms);

        // Reverse movement
        g_motors[motor_id].direction = MOTOR_DIR_REVERSE;
        g_motors[motor_id].step_count = 0;
        hal_motor_set_direction(motor_id, MOTOR_DIR_REVERSE);
        hal_motor_start(motor_id);

        // Wait for completion
        while (g_motors[motor_id].step_count < steps) {
            hal_delay_ms(10);
        }
        hal_motor_stop(motor_id);
        hal_delay_ms(g_system_config.cyclic_delay_ms);
    }

    // Stop and reset state
    g_motors[motor_id].mode = MOTOR_MODE_STOP;
    g_motors[motor_id].is_running = false;

    return APP_OK;
}

app_status_t motor_control_update(void) {
    // Update motor states based on current step counts
    for (int i = 0; i < MOTOR_MAX; i++) {
        if (!g_motors[i].is_running) {
            continue;
        }

        // Check if incremental step is complete
        if (g_motors[i].mode == MOTOR_MODE_INCREMENTAL) {
            if (g_motors[i].step_count >= g_motors[i].target_steps) {
                motor_control_stop((motor_id_t)i);
            }
        }

        // Cyclic mode is handled in motor_control_start_cyclic
        // JOG mode runs continuously until stopped
    }

    return APP_OK;
}

//=============================================================================
// State Management
//=============================================================================

app_status_t motor_control_get_state(motor_id_t motor_id, motor_state_t *state) {
    if (motor_id >= MOTOR_MAX || !state) {
        return APP_INVALID_PARAM;
    }

    *state = g_motors[motor_id];
    return APP_OK;
}

app_status_t motor_control_set_config(const motor_system_config_t *config) {
    if (!config) {
        return APP_INVALID_PARAM;
    }

    g_system_config = *config;
    return APP_OK;
}

app_status_t motor_control_get_config(motor_system_config_t *config) {
    if (!config) {
        return APP_INVALID_PARAM;
    }

    *config = g_system_config;
    return APP_OK;
}

//=============================================================================
// Step Notification (called by HAL when step pulse generated)
//=============================================================================

void motor_control_step_notification(motor_id_t motor_id) {
    if (motor_id >= MOTOR_MAX) {
        return;
    }

    // Increment step count
    g_motors[motor_id].step_count++;

    // For incremental mode, stop after one step
    if (g_motors[motor_id].mode == MOTOR_MODE_INCREMENTAL) {
        if (g_motors[motor_id].step_count >= g_motors[motor_id].target_steps) {
            motor_control_stop(motor_id);
        }
    }
}
