#ifndef BATTERY_H
#define BATTERY_H

#include "stm32h5xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize battery module with UART handles (battery + optional logging)
// and an optional FDCAN handle for telemetry broadcasting.
void battery_init(UART_HandleTypeDef *huart_batt,
                  UART_HandleTypeDef *huart_log,
                  FDCAN_HandleTypeDef *hfdcan);

// Call frequently (e.g., in the main loop). Non-blocking state machine.
void battery_task(void);

#ifdef __cplusplus
}
#endif

#endif // BATTERY_H
