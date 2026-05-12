#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace mr2_xbee_bridge {

constexpr uint8_t kMagic = 0xA5;

enum class MsgId : uint8_t {
  kCmdDrive = 0x01,
  kCmdArmTwist = 0x02,
  kHeartbeat = 0x03,
  kMissionControl = 0x04,
  kCmdArmGripper = 0x05,
  kCmdArmJoint = 0x06,
  kCmdCameraTurret = 0x07,
  kTelemBattery1 = 0x10,
  kTelemBattery2 = 0x11,
  kTelemNav = 0x20,
  kBaseSvin = 0x30,
  kBaseRtcm = 0x31,
  kBaseRtcmFrag = 0x32,
  kTelemBattery = kTelemBattery1,
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

struct MissionControl {
  uint8_t command{0};
  bool clear_costmap{false};
  uint32_t mission_id{0};
};

struct CmdArmGripper {
  uint32_t timestamp_ms{0};
  float position_norm{0.0f};
};

struct CmdArmJoint {
  uint32_t timestamp_ms{0};
  std::array<float, 6> velocities_rad_s{};
};

struct CmdCameraTurret {
  uint32_t timestamp_ms{0};
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
};

struct TelemBattery {
  float total_capacity_mah{0.0f};
  float available_capacity_mah{0.0f};
  float temperature_c{0.0f};
  float pack_voltage_v{0.0f};
};

struct TelemNav {
  uint32_t timestamp_ms{0};
  float latitude_deg{0.0f};
  float longitude_deg{0.0f};
  float altitude_m{0.0f};
  float heading_deg{0.0f};
  float cov_x_var{0.0f};
  float cov_y_var{0.0f};
  float cov_yaw_var{0.0f};
};

struct BaseSvin {
  int32_t mean_x_cm{0};
  int32_t mean_y_cm{0};
  int32_t mean_z_cm{0};
  int8_t mean_x_hp{0};
  int8_t mean_y_hp{0};
  int8_t mean_z_hp{0};
  bool valid{false};
  bool active{false};
  uint32_t mean_acc_0p1mm{0};
  uint32_t obs{0};
};

struct BaseRtcm {
  std::vector<uint8_t> message;
};

struct BaseRtcmFrag {
  uint16_t msg_len{0};
  uint8_t frag_count{0};
  uint8_t frag_index{0};
  std::vector<uint8_t> data;
};

struct Frame {
  Header header{};
  std::vector<uint8_t> payload;
};

uint16_t crc16_ccitt_false(const uint8_t *data, size_t length);

std::vector<uint8_t> encode_cmd_drive(uint8_t seq, const CmdDrive &cmd);
std::vector<uint8_t> encode_cmd_arm_twist(uint8_t seq, const CmdArmTwist &cmd);
std::vector<uint8_t> encode_heartbeat(uint8_t seq, const Heartbeat &hb);
std::vector<uint8_t> encode_mission_control(uint8_t seq,
                                            const MissionControl &ctrl);
std::vector<uint8_t> encode_cmd_arm_gripper(uint8_t seq,
                                            const CmdArmGripper &cmd);
std::vector<uint8_t> encode_cmd_arm_joint(uint8_t seq,
                                          const CmdArmJoint &cmd);
std::vector<uint8_t> encode_cmd_camera_turret(uint8_t seq,
                                              const CmdCameraTurret &cmd);
std::vector<uint8_t> encode_telem_battery(uint8_t seq, const TelemBattery &telem);
std::vector<uint8_t> encode_telem_battery(uint8_t seq, const TelemBattery &telem,
                                          uint8_t battery_id);
std::vector<uint8_t> encode_telem_nav(uint8_t seq, const TelemNav &nav);
std::vector<uint8_t> encode_base_svin(uint8_t seq, const BaseSvin &svin);
std::vector<uint8_t> encode_base_rtcm(uint8_t seq, const BaseRtcm &rtcm);

std::optional<Frame> decode_frame(const uint8_t *data, size_t length);

std::optional<CmdDrive> decode_cmd_drive(const Frame &frame);
std::optional<CmdArmTwist> decode_cmd_arm_twist(const Frame &frame);
std::optional<Heartbeat> decode_heartbeat(const Frame &frame);
std::optional<MissionControl> decode_mission_control(const Frame &frame);
std::optional<CmdArmGripper> decode_cmd_arm_gripper(const Frame &frame);
std::optional<CmdArmJoint> decode_cmd_arm_joint(const Frame &frame);
std::optional<CmdCameraTurret> decode_cmd_camera_turret(const Frame &frame);
std::optional<TelemBattery> decode_telem_battery(const Frame &frame);
std::optional<TelemNav> decode_telem_nav(const Frame &frame);
std::optional<BaseSvin> decode_base_svin(const Frame &frame);
std::optional<BaseRtcm> decode_base_rtcm(const Frame &frame);
std::optional<BaseRtcmFrag> decode_base_rtcm_frag(const Frame &frame);

}  // namespace mr2_xbee_bridge
