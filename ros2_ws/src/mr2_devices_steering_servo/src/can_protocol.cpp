#include "can_protocol.h"

namespace {
constexpr uint16_t kSteeringCommandBase = 0x200;
constexpr uint16_t kSteeringPositionBase = 0x400;
constexpr uint8_t kPayloadLen = 2;
}

uint16_t CanProtocol::steering_command_id(uint8_t node_id) {
  return static_cast<uint16_t>(kSteeringCommandBase + node_id);
}

uint16_t CanProtocol::steering_position_id(uint8_t node_id) {
  return static_cast<uint16_t>(kSteeringPositionBase + node_id);
}

can_frame CanProtocol::encode_steering_command(uint8_t node_id,
                                               int16_t centi_deg) {
  can_frame frame{};
  frame.can_id = steering_command_id(node_id);
  frame.can_dlc = kPayloadLen;
  frame.data[0] = static_cast<uint8_t>(centi_deg & 0xFF);
  frame.data[1] = static_cast<uint8_t>((centi_deg >> 8) & 0xFF);
  return frame;
}

can_frame CanProtocol::encode_steering_position(uint8_t node_id,
                                                int16_t centi_deg) {
  can_frame frame{};
  frame.can_id = steering_position_id(node_id);
  frame.can_dlc = kPayloadLen;
  frame.data[0] = static_cast<uint8_t>(centi_deg & 0xFF);
  frame.data[1] = static_cast<uint8_t>((centi_deg >> 8) & 0xFF);
  return frame;
}

static bool decode_payload(const can_frame &frame, int16_t &centi_deg) {
  if (frame.can_dlc < kPayloadLen) {
    return false;
  }

  const uint16_t lo = static_cast<uint16_t>(frame.data[0]);
  const uint16_t hi = static_cast<uint16_t>(frame.data[1]) << 8;
  centi_deg = static_cast<int16_t>(hi | lo);
  return true;
}

bool CanProtocol::decode_steering_command(const can_frame &frame,
                                          uint8_t node_id,
                                          int16_t &centi_deg) {
  if (frame.can_id & CAN_EFF_FLAG) {
    return false;
  }
  if (frame.can_id != steering_command_id(node_id)) {
    return false;
  }
  return decode_payload(frame, centi_deg);
}

bool CanProtocol::decode_steering_position(const can_frame &frame,
                                           uint8_t node_id,
                                           int16_t &centi_deg) {
  if (frame.can_id & CAN_EFF_FLAG) {
    return false;
  }
  if (frame.can_id != steering_position_id(node_id)) {
    return false;
  }
  return decode_payload(frame, centi_deg);
}
