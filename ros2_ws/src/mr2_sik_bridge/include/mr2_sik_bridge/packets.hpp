#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace mr2_sik_bridge {

constexpr uint8_t kMagic = 0xA5;

enum class MsgId : uint8_t {
  kCmdDrive = 0x01,
  kCmdArmTwist = 0x02,
  kHeartbeat = 0x03,
  kTelemBattery = 0x10,
};

struct Header {
  uint8_t magic{0};
  MsgId msg_id{MsgId::kHeartbeat};
  uint8_t length{0};
  uint8_t seq{0};
};

struct CmdDrive {
  uint32_t timestamp_ms{0};
  float linear_x_m_s{0.0f};
  float linear_y_m_s{0.0f};
  float angular_z_rad_s{0.0f};
};

struct CmdArmTwist {
  uint32_t timestamp_ms{0};
  float lin_x_m_s{0.0f};
  float lin_y_m_s{0.0f};
  float lin_z_m_s{0.0f};
  float ang_x_rad_s{0.0f};
  float ang_y_rad_s{0.0f};
  float ang_z_rad_s{0.0f};
};

struct Heartbeat {
  uint32_t timestamp_ms{0};
};

struct TelemBattery {
  float total_capacity_mah{0.0f};
  float available_capacity_mah{0.0f};
  float temperature_c{0.0f};
};

struct Frame {
  Header header{};
  std::vector<uint8_t> payload;
};

uint16_t crc16_ccitt_false(const uint8_t *data, size_t length);

std::vector<uint8_t> encode_cmd_drive(uint8_t seq, const CmdDrive &cmd);
std::vector<uint8_t> encode_cmd_arm_twist(uint8_t seq, const CmdArmTwist &cmd);
std::vector<uint8_t> encode_heartbeat(uint8_t seq, const Heartbeat &hb);
std::vector<uint8_t> encode_telem_battery(uint8_t seq, const TelemBattery &telem);

std::optional<Frame> decode_frame(const uint8_t *data, size_t length);

std::optional<CmdDrive> decode_cmd_drive(const Frame &frame);
std::optional<CmdArmTwist> decode_cmd_arm_twist(const Frame &frame);
std::optional<Heartbeat> decode_heartbeat(const Frame &frame);
std::optional<TelemBattery> decode_telem_battery(const Frame &frame);

}  // namespace mr2_sik_bridge
