#pragma once

#include <cstdint>
#include <linux/can.h>

class CanProtocol {
public:
  static uint16_t gripper_command_id(uint8_t node_id);
  static uint16_t gripper_position_id(uint8_t node_id);

  static can_frame encode_gripper_command(uint8_t node_id, int16_t centi_deg);
  static can_frame encode_gripper_position(uint8_t node_id, int16_t centi_deg);

  static bool decode_gripper_command(const can_frame &frame, uint8_t node_id,
                                     int16_t &centi_deg);
  static bool decode_gripper_position(const can_frame &frame, uint8_t node_id,
                                      int16_t &centi_deg);
};
