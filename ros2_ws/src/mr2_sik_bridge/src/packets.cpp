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
  payload.reserve(12);
  ByteWriter writer(&payload);
  writer.write_u32(cmd.timestamp_ms);
  writer.write_f32(cmd.linear_x_m_s);
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
  std::vector<uint8_t> payload;
  payload.reserve(12);
  ByteWriter writer(&payload);
  writer.write_f32(telem.total_capacity_mah);
  writer.write_f32(telem.available_capacity_mah);
  writer.write_f32(telem.temperature_c);

  Header header;
  header.magic = kMagic;
  header.msg_id = MsgId::kTelemBattery;
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
  if (frame.header.msg_id != MsgId::kCmdDrive || frame.payload.size() != 12) {
    return std::nullopt;
  }

  ByteReader reader(frame.payload.data(), frame.payload.size());
  CmdDrive cmd;
  if (!reader.read_u32(&cmd.timestamp_ms) ||
      !reader.read_f32(&cmd.linear_x_m_s) ||
      !reader.read_f32(&cmd.angular_z_rad_s)) {
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
  if (frame.header.msg_id != MsgId::kTelemBattery ||
      frame.payload.size() != 12) {
    return std::nullopt;
  }

  ByteReader reader(frame.payload.data(), frame.payload.size());
  TelemBattery telem;
  if (!reader.read_f32(&telem.total_capacity_mah) ||
      !reader.read_f32(&telem.available_capacity_mah) ||
      !reader.read_f32(&telem.temperature_c)) {
    return std::nullopt;
  }
  return telem;
}

}  // namespace mr2_sik_bridge
