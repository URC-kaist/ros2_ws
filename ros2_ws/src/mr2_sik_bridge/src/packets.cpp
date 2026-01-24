#include "mr2_sik_bridge/packets.hpp"

#include <cstring>

namespace mr2_sik_bridge {
namespace {

constexpr size_t kHeaderSize = 4;
constexpr size_t kCrcSize = 2;

class ByteWriter {
 public:
  explicit ByteWriter(std::vector<uint8_t> *buffer) : buffer_(buffer) {}

  void write_u8(uint8_t value) { buffer_->push_back(value); }

  void write_u16(uint16_t value) {
    buffer_->push_back(static_cast<uint8_t>(value & 0xFF));
    buffer_->push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
  }

  void write_u32(uint32_t value) {
    buffer_->push_back(static_cast<uint8_t>(value & 0xFF));
    buffer_->push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    buffer_->push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
    buffer_->push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
  }

  void write_i32(int32_t value) { write_u32(static_cast<uint32_t>(value)); }

  void write_f32(float value) {
    static_assert(sizeof(float) == 4, "float must be 32-bit IEEE-754");
    uint32_t raw = 0;
    std::memcpy(&raw, &value, sizeof(raw));
    write_u32(raw);
  }

 private:
  std::vector<uint8_t> *buffer_;
};

class ByteReader {
 public:
  ByteReader(const uint8_t *data, size_t length)
      : data_(data), length_(length), offset_(0) {}

  bool read_u8(uint8_t *value) {
    if (!require(1)) {
      return false;
    }
    *value = data_[offset_++];
    return true;
  }

  bool read_u16(uint16_t *value) {
    if (!require(2)) {
      return false;
    }
    const uint16_t lo = data_[offset_++];
    const uint16_t hi = data_[offset_++];
    *value = static_cast<uint16_t>(lo | (hi << 8));
    return true;
  }

  bool read_u32(uint32_t *value) {
    if (!require(4)) {
      return false;
    }
    uint32_t out = 0;
    out |= static_cast<uint32_t>(data_[offset_++]);
    out |= static_cast<uint32_t>(data_[offset_++]) << 8;
    out |= static_cast<uint32_t>(data_[offset_++]) << 16;
    out |= static_cast<uint32_t>(data_[offset_++]) << 24;
    *value = out;
    return true;
  }

  bool read_i32(int32_t *value) {
    uint32_t raw = 0;
    if (!read_u32(&raw)) {
      return false;
    }
    *value = static_cast<int32_t>(raw);
    return true;
  }

  bool read_i8(int8_t *value) {
    uint8_t raw = 0;
    if (!read_u8(&raw)) {
      return false;
    }
    *value = static_cast<int8_t>(raw);
    return true;
  }

  bool read_f32(float *value) {
    uint32_t raw = 0;
    if (!read_u32(&raw)) {
      return false;
    }
    std::memcpy(value, &raw, sizeof(raw));
    return true;
  }

 private:
  bool require(size_t bytes) const { return offset_ + bytes <= length_; }

  const uint8_t *data_;
  size_t length_;
  size_t offset_;
};

std::vector<uint8_t> finalize_frame(const Header &header,
                                   const std::vector<uint8_t> &payload) {
  std::vector<uint8_t> buffer;
  buffer.reserve(kHeaderSize + payload.size() + kCrcSize);

  ByteWriter writer(&buffer);
  writer.write_u8(header.magic);
  writer.write_u8(static_cast<uint8_t>(header.msg_id));
  writer.write_u8(header.length);
  writer.write_u8(header.seq);
  buffer.insert(buffer.end(), payload.begin(), payload.end());

  const uint16_t crc = crc16_ccitt_false(buffer.data(), buffer.size());
  writer.write_u16(crc);
  return buffer;
}

}  // namespace

uint16_t crc16_ccitt_false(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

std::vector<uint8_t> encode_cmd_drive(uint8_t seq, const CmdDrive &cmd) {
  std::vector<uint8_t> payload;
  payload.reserve(16);
  ByteWriter writer(&payload);
  writer.write_u32(cmd.timestamp_ms);
  writer.write_f32(cmd.linear_x_m_s);
  writer.write_f32(cmd.linear_y_m_s);
  writer.write_f32(cmd.angular_z_rad_s);

  Header header;
  header.magic = kMagic;
  header.msg_id = MsgId::kCmdDrive;
  header.length = static_cast<uint8_t>(payload.size());
  header.seq = seq;

  return finalize_frame(header, payload);
}

std::vector<uint8_t> encode_cmd_arm_twist(uint8_t seq, const CmdArmTwist &cmd) {
  std::vector<uint8_t> payload;
  payload.reserve(28);
  ByteWriter writer(&payload);
  writer.write_u32(cmd.timestamp_ms);
  writer.write_f32(cmd.lin_x_m_s);
  writer.write_f32(cmd.lin_y_m_s);
  writer.write_f32(cmd.lin_z_m_s);
  writer.write_f32(cmd.ang_x_rad_s);
  writer.write_f32(cmd.ang_y_rad_s);
  writer.write_f32(cmd.ang_z_rad_s);

  Header header;
  header.magic = kMagic;
  header.msg_id = MsgId::kCmdArmTwist;
  header.length = static_cast<uint8_t>(payload.size());
  header.seq = seq;

  return finalize_frame(header, payload);
}

std::vector<uint8_t> encode_heartbeat(uint8_t seq, const Heartbeat &hb) {
  std::vector<uint8_t> payload;
  payload.reserve(4);
  ByteWriter writer(&payload);
  writer.write_u32(hb.timestamp_ms);

  Header header;
  header.magic = kMagic;
  header.msg_id = MsgId::kHeartbeat;
  header.length = static_cast<uint8_t>(payload.size());
  header.seq = seq;

  return finalize_frame(header, payload);
}

std::vector<uint8_t> encode_telem_battery(uint8_t seq,
                                          const TelemBattery &telem) {
  return encode_telem_battery(seq, telem, 1);
}

std::vector<uint8_t> encode_telem_battery(uint8_t seq,
                                          const TelemBattery &telem,
                                          uint8_t battery_id) {
  std::vector<uint8_t> payload;
  payload.reserve(16);
  ByteWriter writer(&payload);
  writer.write_f32(telem.total_capacity_mah);
  writer.write_f32(telem.available_capacity_mah);
  writer.write_f32(telem.temperature_c);
  writer.write_f32(telem.pack_voltage_v);

  Header header;
  header.magic = kMagic;
  header.msg_id =
      (battery_id == 2) ? MsgId::kTelemBattery2 : MsgId::kTelemBattery1;
  header.length = static_cast<uint8_t>(payload.size());
  header.seq = seq;

  return finalize_frame(header, payload);
}

std::vector<uint8_t> encode_telem_nav(uint8_t seq, const TelemNav &nav) {
  std::vector<uint8_t> payload;
  payload.reserve(32);
  ByteWriter writer(&payload);
  writer.write_u32(nav.timestamp_ms);
  writer.write_f32(nav.latitude_deg);
  writer.write_f32(nav.longitude_deg);
  writer.write_f32(nav.altitude_m);
  writer.write_f32(nav.heading_deg);
  writer.write_f32(nav.cov_x_var);
  writer.write_f32(nav.cov_y_var);
  writer.write_f32(nav.cov_yaw_var);

  Header header;
  header.magic = kMagic;
  header.msg_id = MsgId::kTelemNav;
  header.length = static_cast<uint8_t>(payload.size());
  header.seq = seq;

  return finalize_frame(header, payload);
}

std::vector<uint8_t> encode_base_svin(uint8_t seq, const BaseSvin &svin) {
  std::vector<uint8_t> payload;
  payload.reserve(25);
  ByteWriter writer(&payload);
  writer.write_i32(svin.mean_x_cm);
  writer.write_i32(svin.mean_y_cm);
  writer.write_i32(svin.mean_z_cm);
  writer.write_u8(static_cast<uint8_t>(svin.mean_x_hp));
  writer.write_u8(static_cast<uint8_t>(svin.mean_y_hp));
  writer.write_u8(static_cast<uint8_t>(svin.mean_z_hp));
  writer.write_u8(static_cast<uint8_t>(svin.valid ? 1 : 0));
  writer.write_u8(static_cast<uint8_t>(svin.active ? 1 : 0));
  writer.write_u32(svin.mean_acc_0p1mm);
  writer.write_u32(svin.obs);

  Header header;
  header.magic = kMagic;
  header.msg_id = MsgId::kBaseSvin;
  header.length = static_cast<uint8_t>(payload.size());
  header.seq = seq;

  return finalize_frame(header, payload);
}

std::vector<uint8_t> encode_base_rtcm(uint8_t seq, const BaseRtcm &rtcm) {
  std::vector<uint8_t> payload;
  payload.reserve(1 + rtcm.message.size());
  const size_t max_payload = 255;  // length fits in uint8_t
  const size_t max_rtcm_len = (max_payload >= 1) ? max_payload - 1 : 0;
  const size_t len = std::min(rtcm.message.size(), max_rtcm_len);
  payload.push_back(static_cast<uint8_t>(len));
  payload.insert(payload.end(), rtcm.message.begin(),
                 rtcm.message.begin() + static_cast<std::ptrdiff_t>(len));

  Header header;
  header.magic = kMagic;
  header.msg_id = MsgId::kBaseRtcm;
  header.length = static_cast<uint8_t>(payload.size());
  header.seq = seq;

  return finalize_frame(header, payload);
}

std::optional<Frame> decode_frame(const uint8_t *data, size_t length) {
  if (!data || length < kHeaderSize + kCrcSize) {
    return std::nullopt;
  }

  ByteReader reader(data, length);
  Header header;
  uint8_t msg_id_raw = 0;
  if (!reader.read_u8(&header.magic) || !reader.read_u8(&msg_id_raw) ||
      !reader.read_u8(&header.length) || !reader.read_u8(&header.seq)) {
    return std::nullopt;
  }

  header.msg_id = static_cast<MsgId>(msg_id_raw);
  if (header.magic != kMagic) {
    return std::nullopt;
  }

  const size_t expected_size = kHeaderSize + header.length + kCrcSize;
  if (length < expected_size) {
    return std::nullopt;
  }

  const uint16_t crc_expected = static_cast<uint16_t>(
      data[expected_size - 2] | (data[expected_size - 1] << 8));
  const uint16_t crc_actual =
      crc16_ccitt_false(data, expected_size - kCrcSize);
  if (crc_expected != crc_actual) {
    return std::nullopt;
  }

  Frame frame;
  frame.header = header;
  frame.payload.assign(data + kHeaderSize,
                       data + kHeaderSize + header.length);
  return frame;
}

std::optional<CmdDrive> decode_cmd_drive(const Frame &frame) {
  if (frame.header.msg_id != MsgId::kCmdDrive) {
    return std::nullopt;
  }
  if (frame.payload.size() != 12 && frame.payload.size() != 16) {
    return std::nullopt;
  }

  ByteReader reader(frame.payload.data(), frame.payload.size());
  CmdDrive cmd;
  if (!reader.read_u32(&cmd.timestamp_ms) ||
      !reader.read_f32(&cmd.linear_x_m_s)) {
    return std::nullopt;
  }
  if (frame.payload.size() == 16) {
    if (!reader.read_f32(&cmd.linear_y_m_s)) {
      return std::nullopt;
    }
  } else {
    cmd.linear_y_m_s = 0.0f;
  }
  if (!reader.read_f32(&cmd.angular_z_rad_s)) {
    return std::nullopt;
  }
  return cmd;
}

std::optional<CmdArmTwist> decode_cmd_arm_twist(const Frame &frame) {
  if (frame.header.msg_id != MsgId::kCmdArmTwist ||
      frame.payload.size() != 28) {
    return std::nullopt;
  }

  ByteReader reader(frame.payload.data(), frame.payload.size());
  CmdArmTwist cmd;
  if (!reader.read_u32(&cmd.timestamp_ms) ||
      !reader.read_f32(&cmd.lin_x_m_s) ||
      !reader.read_f32(&cmd.lin_y_m_s) ||
      !reader.read_f32(&cmd.lin_z_m_s) ||
      !reader.read_f32(&cmd.ang_x_rad_s) ||
      !reader.read_f32(&cmd.ang_y_rad_s) ||
      !reader.read_f32(&cmd.ang_z_rad_s)) {
    return std::nullopt;
  }
  return cmd;
}

std::optional<Heartbeat> decode_heartbeat(const Frame &frame) {
  if (frame.header.msg_id != MsgId::kHeartbeat || frame.payload.size() != 4) {
    return std::nullopt;
  }

  ByteReader reader(frame.payload.data(), frame.payload.size());
  Heartbeat hb;
  if (!reader.read_u32(&hb.timestamp_ms)) {
    return std::nullopt;
  }
  return hb;
}

std::optional<TelemBattery> decode_telem_battery(const Frame &frame) {
  if ((frame.header.msg_id != MsgId::kTelemBattery1 &&
       frame.header.msg_id != MsgId::kTelemBattery2) ||
      frame.payload.size() != 16) {
    return std::nullopt;
  }

  ByteReader reader(frame.payload.data(), frame.payload.size());
  TelemBattery telem;
  if (!reader.read_f32(&telem.total_capacity_mah) ||
      !reader.read_f32(&telem.available_capacity_mah) ||
      !reader.read_f32(&telem.temperature_c) ||
      !reader.read_f32(&telem.pack_voltage_v)) {
    return std::nullopt;
  }
  return telem;
}

std::optional<TelemNav> decode_telem_nav(const Frame &frame) {
  if (frame.header.msg_id != MsgId::kTelemNav || frame.payload.size() != 32) {
    return std::nullopt;
  }

  ByteReader reader(frame.payload.data(), frame.payload.size());
  TelemNav nav;
  if (!reader.read_u32(&nav.timestamp_ms) ||
      !reader.read_f32(&nav.latitude_deg) ||
      !reader.read_f32(&nav.longitude_deg) ||
      !reader.read_f32(&nav.altitude_m) ||
      !reader.read_f32(&nav.heading_deg) ||
      !reader.read_f32(&nav.cov_x_var) ||
      !reader.read_f32(&nav.cov_y_var) ||
      !reader.read_f32(&nav.cov_yaw_var)) {
    return std::nullopt;
  }
  return nav;
}

std::optional<BaseSvin> decode_base_svin(const Frame &frame) {
  if (frame.header.msg_id != MsgId::kBaseSvin || frame.payload.size() != 25) {
    return std::nullopt;
  }

  ByteReader reader(frame.payload.data(), frame.payload.size());
  BaseSvin svin;
  uint8_t valid = 0;
  uint8_t active = 0;
  if (!reader.read_i32(&svin.mean_x_cm) || !reader.read_i32(&svin.mean_y_cm) ||
      !reader.read_i32(&svin.mean_z_cm) || !reader.read_i8(&svin.mean_x_hp) ||
      !reader.read_i8(&svin.mean_y_hp) || !reader.read_i8(&svin.mean_z_hp) ||
      !reader.read_u8(&valid) || !reader.read_u8(&active) ||
      !reader.read_u32(&svin.mean_acc_0p1mm) || !reader.read_u32(&svin.obs)) {
    return std::nullopt;
  }
  svin.valid = (valid != 0);
  svin.active = (active != 0);
  return svin;
}

std::optional<BaseRtcm> decode_base_rtcm(const Frame &frame) {
  if (frame.header.msg_id != MsgId::kBaseRtcm) {
    return std::nullopt;
  }
  if (frame.payload.empty()) {
    return std::nullopt;
  }
  const uint8_t len = frame.payload[0];
  if (frame.payload.size() != static_cast<size_t>(len) + 1) {
    return std::nullopt;
  }
  BaseRtcm rtcm;
  rtcm.message.assign(frame.payload.begin() + 1, frame.payload.end());
  return rtcm;
}

}  // namespace mr2_sik_bridge
