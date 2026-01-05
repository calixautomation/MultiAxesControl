# Application Refactoring Summary

## Overview

The application folder has been completely refactored to remove all HAL-related code and eliminate redundancy. The new architecture provides clear separation of concerns across three distinct layers.

---

## New Architecture

```
┌─────────────────────────────────────────────────────────┐
│                     main.c (Firmware)                    │
│  - System initialization                                 │
│  - Clock configuration                                   │
│  - Peripheral setup (UART, GPIO, Watchdog)              │
└────────────┬────────────────────────────────────────────┘
             │
             ├──────────────┬──────────────┬──────────────┐
             │              │              │              │
             ▼              ▼              ▼              ▼
┌────────────────┐ ┌──────────────┐ ┌─────────────┐ ┌─────────┐
│   Application  │ │   OS/Task    │ │     HAL     │ │ Config  │
│     Layer      │ │   Manager    │ │   Layer     │ │  Layer  │
└────────────────┘ └──────────────┘ └─────────────┘ └─────────┘
```

---

## Layer 1: Application Layer (Pure Business Logic)

**Location**: `application/`

### Files
- `motor_control.h` - Application API
- `motor_control.c` - Implementation

### Responsibilities
- ✅ Command parsing
- ✅ Motor control state machine
- ✅ Configuration management
- ✅ Step counting and mode handling

### What Was Removed
- ❌ FreeRTOS task implementations
- ❌ Queue and semaphore handling
- ❌ UART ISR handlers
- ❌ HAL-specific callbacks
- ❌ Watchdog and LED tasks

### Key Functions
```c
// Initialization
app_status_t motor_control_init(void);
app_status_t motor_control_deinit(void);

// Command processing
app_status_t motor_control_parse_command(const char *cmd_str, motor_command_t *command);
app_status_t motor_control_process_command(const motor_command_t *command);

// Motor operations
app_status_t motor_control_start_jog(motor_id_t motor_id, motor_direction_t direction);
app_status_t motor_control_stop(motor_id_t motor_id);
app_status_t motor_control_start_incremental(motor_id_t motor_id, motor_direction_t direction);
app_status_t motor_control_start_cyclic(motor_id_t motor_id, uint32_t arc_length, uint32_t cycles);

// State management
app_status_t motor_control_get_state(motor_id_t motor_id, motor_state_t *state);
app_status_t motor_control_set_config(const motor_system_config_t *config);

// HAL notification callback
void motor_control_step_notification(motor_id_t motor_id);
```

### Status Codes
```c
typedef enum {
    APP_OK = 0,
    APP_ERROR,
    APP_BUSY,
    APP_TIMEOUT,
    APP_INVALID_PARAM
} app_status_t;
```

---

## Layer 2: OS/Task Management Layer

**Location**: `os/`

### Files
- `task_manager.h` - Task management API
- `task_manager.c` - FreeRTOS task implementations

### Responsibilities
- ✅ FreeRTOS task creation
- ✅ Queue management
- ✅ UART ISR handling
- ✅ Watchdog refresh task
- ✅ Status LED task
- ✅ Motor update task

### Key Functions
```c
// Initialization
task_status_t task_manager_init(void);

// Communication
task_status_t task_manager_send_command(const char *command);
task_status_t task_manager_send_response(const char *response);

// ISR handler
void task_manager_uart_rx_isr(void);

// Task functions (created automatically)
void task_uart_handler(void *pvParameters);
void task_motor_update(void *pvParameters);
void task_watchdog_refresh(void *pvParameters);
void task_status_led(void *pvParameters);
```

### Tasks Created

| Task Name | Priority | Stack | Function | Purpose |
|-----------|----------|-------|----------|---------|
| UART | 2 (High) | 256 words | `task_uart_handler` | Process commands from queue |
| MOTOR | 1 (Normal) | 256 words | `task_motor_update` | Update motor states |
| WATCHDOG | 3 (Highest) | 128 words | `task_watchdog_refresh` | Refresh hardware watchdog |
| LED | 1 (Normal) | 128 words | `task_status_led` | Blink status LED |

---

## Layer 3: HAL Layer

**Location**: `hal/`

### Files
- `hal_interface.h` - HAL API definition
- `hal/platforms/stm32/hal_stm32.c` - STM32 implementation

### Responsibilities
- ✅ Hardware abstraction
- ✅ Timer interrupts (TIM2, TIM3)
- ✅ UART interrupts (USART1, USART2)
- ✅ GPIO control
- ✅ Motor step pulse generation

### Key Changes
- Timer ISRs now call `motor_control_step_notification()`
- UART ISR calls `task_manager_uart_rx_isr()`
- No application logic in HAL

### ISR Flow

**Timer ISR (TIM2 for Linear, TIM3 for Rotary)**:
```c
void TIM2_IRQHandler(void) {
    // 1. Clear interrupt flag
    LL_TIM_ClearFlag_UPDATE(TIM2);

    // 2. Generate step pulse
    hal_motor_step(MOTOR_LINEAR);

    // 3. Notify application
    motor_control_step_notification(MOTOR_LINEAR);

    // 4. Call registered callback (if any)
    if (g_timer_callbacks[0]) {
        g_timer_callbacks[0]();
    }
}
```

**UART ISR**:
```c
void USART2_IRQHandler(void) {
    // Delegate to task manager
    task_manager_uart_rx_isr();
}
```

---

## Type Definitions (Shared)

To avoid duplication, motor types are defined once with include guards:

```c
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

#endif
```

These are included in both:
- `application/motor_control.h`
- `hal/hal_interface.h`

---

## Initialization Flow (main.c)

```c
int main(void) {
    // 1. Initialize STM32 HAL
    HAL_Init();

    // 2. Configure clocks (170MHz)
    SystemClock_Config();

    // 3. Initialize peripherals
    GPIO_Init();
    UART_Init();

    // 4. Initialize application layer
    motor_control_init();  // → Initializes HAL, sets default config

    // 5. Initialize task manager (creates FreeRTOS tasks)
    task_manager_init();

    // 6. Initialize watchdog
    IWDG_Init();

    // 7. Start FreeRTOS scheduler
    vTaskStartScheduler();
}
```

---

## Command Execution Flow

### Example: `C1Z` (Move Linear Motor Forward)

```
1. USART2_IRQHandler (HAL)
   └─> task_manager_uart_rx_isr() (OS Layer)
       └─> Accumulates bytes until 'Z'
           └─> xQueueSendFromISR(g_command_queue, "C1")

2. task_uart_handler() wakes up (OS Layer)
   └─> xQueueReceive(g_command_queue, "C1")
   └─> motor_control_parse_command("C1", &command)  (Application)
       └─> command.type = CMD_MOVE_FORWARD
       └─> command.motor_id = MOTOR_LINEAR
   └─> motor_control_process_command(&command)  (Application)
       └─> motor_control_start_jog(MOTOR_LINEAR, MOTOR_DIR_FORWARD)
           └─> hal_motor_set_direction(MOTOR_LINEAR, MOTOR_DIR_FORWARD)  (HAL)
           └─> hal_timer_set_frequency(MOTOR_LINEAR, 200)  (HAL)
           └─> hal_motor_start(MOTOR_LINEAR)  (HAL)

3. TIM2_IRQHandler fires at 200Hz (HAL)
   └─> hal_motor_step(MOTOR_LINEAR)  (HAL) - Toggle PA0
   └─> motor_control_step_notification(MOTOR_LINEAR)  (Application)
       └─> g_motors[MOTOR_LINEAR].step_count++
```

---

## Benefits of Refactoring

### 1. Clear Separation of Concerns
- **Application**: Business logic only, no hardware details
- **OS Layer**: RTOS integration, task management
- **HAL**: Hardware-specific code

### 2. Improved Portability
- Application code is now 100% platform-agnostic
- Easy to port to different MCUs (just change HAL)
- Easy to change RTOS (just change OS layer)

### 3. Better Testability
- Application layer can be unit tested without hardware
- Mock HAL for testing
- No RTOS dependencies in application

### 4. Reduced Redundancy
- Motor types defined once with include guards
- No duplicate callback implementations
- Single source of truth for each responsibility

### 5. Maintainability
- Each layer has a clear purpose
- Changes to hardware don't affect application logic
- Changes to RTOS don't affect application logic

---

## File Structure

```
MultiAxesControl/
├── application/
│   ├── motor_control.h      ✅ Pure business logic API
│   └── motor_control.c      ✅ Platform-agnostic implementation
│
├── os/
│   ├── task_manager.h       ✅ FreeRTOS task management API
│   └── task_manager.c       ✅ Task implementations
│
├── hal/
│   ├── hal_interface.h      ✅ Hardware abstraction API
│   └── platforms/
│       └── stm32/
│           ├── hal_stm32.c  ✅ STM32-specific implementation
│           └── hal_stm32.h
│
├── config/
│   └── register_config.h    ✅ Generic register definitions
│
└── firmware/
    └── src/
        └── main.c           ✅ System initialization only
```

---

## What Was Removed from Application Layer

### Before Refactoring
```c
// ❌ These were in motor_control.c
void motor_control_uart_task(void *pvParameters);
void motor_control_motor_task(void *pvParameters);
void motor_control_watchdog_task(void *pvParameters);
void motor_control_status_led_task(void *pvParameters);
hal_status_t motor_control_setup_tasks(void);
void motor_control_uart_rx_isr(void);
void uart_rx_callback(uint8_t data);
void motor_step_callback(motor_id_t motor_id);
```

### After Refactoring
```c
// ✅ Now in os/task_manager.c
task_status_t task_manager_init(void);
void task_uart_handler(void *pvParameters);
void task_motor_update(void *pvParameters);
void task_watchdog_refresh(void *pvParameters);
void task_status_led(void *pvParameters);
void task_manager_uart_rx_isr(void);
```

---

## Migration Notes

If you have existing code that used the old API:

### Old Code
```c
#include "motor_control.h"

// In main.c
motor_control_init();
motor_control_setup_tasks();
xTaskCreate(motor_control_uart_task, ...);
xTaskCreate(motor_control_motor_task, ...);
```

### New Code
```c
#include "motor_control.h"
#include "task_manager.h"

// In main.c
motor_control_init();   // Initialize application
task_manager_init();    // Creates all tasks automatically
```

### Old ISR Handler
```c
void USART2_IRQHandler(void) {
    motor_control_uart_rx_isr();
}
```

### New ISR Handler
```c
void USART2_IRQHandler(void) {
    task_manager_uart_rx_isr();
}
```

---

## Summary

The application folder is now **pure business logic** with:
- ✅ No FreeRTOS dependencies
- ✅ No HAL-specific code
- ✅ No hardware details
- ✅ No redundant type definitions
- ✅ Clear, testable API
- ✅ Complete separation of concerns

All RTOS integration is in `os/task_manager.*` and all hardware details are in `hal/platforms/stm32/`.
