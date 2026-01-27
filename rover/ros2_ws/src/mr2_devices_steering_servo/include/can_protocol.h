#pragma once

#include <cstdint>
#include <linux/can.h>

class CanProtocol {
public:
  static uint16_t steering_command_id(uint8_t node_id);
  static uint16_t steering_position_id(uint8_t node_id);

  static can_frame encode_steering_command(uint8_t node_id,
                                           int16_t centi_deg);
  static can_frame encode_steering_position(uint8_t node_id,
                                            int16_t centi_deg);

  static bool decode_steering_command(const can_frame &frame, uint8_t node_id,
                                      int16_t &centi_deg);
  static bool decode_steering_position(const can_frame &frame, uint8_t node_id,
                                       int16_t &centi_deg);
};
