#pragma once

#include <cstdint>

namespace board_config {

constexpr const char *CAN_INTERFACE = "can0";
constexpr uint32_t CAN_BITRATE = 1'000'000;
constexpr uint8_t CAN_NODE_ID = 2;

// Steering position report period in milliseconds (0 = disable).
constexpr uint32_t CAN_POS_TX_MS = 50;

// RX timeout in milliseconds before HOLD-CURRENT policy applies.
constexpr uint32_t CAN_TIMEOUT_MS = 100;

// RX recovery hysteresis in milliseconds.
constexpr uint32_t RX_RECOVER_MS = 50;

// Mechanical output clamp in centi-degrees.
constexpr int16_t STEERING_MIN_CENTI_DEG = -32768;
constexpr int16_t STEERING_MAX_CENTI_DEG = 32767;

} // namespace board_config
