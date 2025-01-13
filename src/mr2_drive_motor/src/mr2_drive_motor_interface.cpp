#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <mr2_drive_motor/msg/pid.hpp>

// Boost.Asio
#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Constants and Packet Definitions
// ---------------------------------------------------------------------------
static constexpr uint8_t PACKET_HEADER_0 = 0xAA;
static constexpr uint8_t PACKET_HEADER_1 = 0x55;

static constexpr uint8_t MODE_INIT = 0;
static constexpr uint8_t MODE_SET_SPEED = 1;
static constexpr uint8_t MODE_GET_SPEED = 2;
static constexpr uint8_t MODE_SET_BRAKE = 3;
static constexpr uint8_t MODE_SET_PID = 4;
static constexpr uint8_t MODE_GET_PID = 5;

static constexpr size_t SEND_PACKET_SIZE =
    24; // 2B hdr + 1B mode + 1B ID + 16B data + 4B CRC
static constexpr size_t RECEIVE_PACKET_SIZE = 20; // 16B data + 4B CRC

// ---------------------------------------------------------------------------
// Compute CRC32 (non-reflected). Adjust to match your STM32 firmware exactly.
// ---------------------------------------------------------------------------
uint32_t computeCRC32(const uint8_t *data, size_t length) {
  // Example polynomial 0x04C11DB7.
  // Real table has 256 entries. Shown short for brevity.
  static const uint32_t crc_table[256] = {
      0x00000000, 0x04C11DB7, 0x09823B6E, 0x0D4326D9, 0x130476DC, 0x17C56B6B,
      0x1A864DB2, 0x1E475005, 0x2608EDB8, 0x22C9F00F, 0x2F8AD6D6, 0x2B4BCB61,
      0x350C9B64, 0x31CD86D3, 0x3C8EA00A, 0x384FBDBD, 0x4C11DB70, 0x48D0C6C7,
      0x4593E01E, 0x4152FDA9, 0x5F15ADAC, 0x5BD4B01B, 0x569796C2, 0x52568B75,
      0x6A1936C8, 0x6ED82B7F, 0x639B0DA6, 0x675A1011, 0x791D4014, 0x7DDC5DA3,
      0x709F7B7A, 0x745E66CD, 0x9823B6E0, 0x9CE2AB57, 0x91A18D8E, 0x95609039,
      0x8B27C03C, 0x8FE6DD8B, 0x82A5FB52, 0x8664E6E5, 0xBE2B5B58, 0xBAEA46EF,
      0xB7A96036, 0xB3687D81, 0xAD2F2D84, 0xA9EE3033, 0xA4AD16EA, 0xA06C0B5D,
      0xD4326D90, 0xD0F37027, 0xDDB056FE, 0xD9714B49, 0xC7361B4C, 0xC3F706FB,
      0xCEB42022, 0xCA753D95, 0xF23A8028, 0xF6FB9D9F, 0xFBB8BB46, 0xFF79A6F1,
      0xE13EF6F4, 0xE5FFEB43, 0xE8BCCD9A, 0xEC7DD02D, 0x34867077, 0x30476DC0,
      0x3D044B19, 0x39C556AE, 0x278206AB, 0x23431B1C, 0x2E003DC5, 0x2AC12072,
      0x128E9DCF, 0x164F8078, 0x1B0CA6A1, 0x1FCDBB16, 0x018AEB13, 0x054BF6A4,
      0x0808D07D, 0x0CC9CDCA, 0x7897AB07, 0x7C56B6B0, 0x71159069, 0x75D48DDE,
      0x6B93DDDB, 0x6F52C06C, 0x6211E6B5, 0x66D0FB02, 0x5E9F46BF, 0x5A5E5B08,
      0x571D7DD1, 0x53DC6066, 0x4D9B3063, 0x495A2DD4, 0x44190B0D, 0x40D816BA,
      0xACA5C697, 0xA864DB20, 0xA527FDF9, 0xA1E6E04E, 0xBFA1B04B, 0xBB60ADFC,
      0xB6238B25, 0xB2E29692, 0x8AAD2B2F, 0x8E6C3698, 0x832F1041, 0x87EE0DF6,
      0x99A95DF3, 0x9D684044, 0x902B669D, 0x94EA7B2A, 0xE0B41DE7, 0xE4750050,
      0xE9362689, 0xEDF73B3E, 0xF3B06B3B, 0xF771768C, 0xFA325055, 0xFEF34DE2,
      0xC6BCF05F, 0xC27DEDE8, 0xCF3ECB31, 0xCBFFD686, 0xD5B88683, 0xD1799B34,
      0xDC3ABDED, 0xD8FBA05A, 0x690CE0EE, 0x6DCDFD59, 0x608EDB80, 0x644FC637,
      0x7A089632, 0x7EC98B85, 0x738AAD5C, 0x774BB0EB, 0x4F040D56, 0x4BC510E1,
      0x46863638, 0x42472B8F, 0x5C007B8A, 0x58C1663D, 0x558240E4, 0x51435D53,
      0x251D3B9E, 0x21DC2629, 0x2C9F00F0, 0x285E1D47, 0x36194D42, 0x32D850F5,
      0x3F9B762C, 0x3B5A6B9B, 0x0315D626, 0x07D4CB91, 0x0A97ED48, 0x0E56F0FF,
      0x1011A0FA, 0x14D0BD4D, 0x19939B94, 0x1D528623, 0xF12F560E, 0xF5EE4BB9,
      0xF8AD6D60, 0xFC6C70D7, 0xE22B20D2, 0xE6EA3D65, 0xEBA91BBC, 0xEF68060B,
      0xD727BBB6, 0xD3E6A601, 0xDEA580D8, 0xDA649D6F, 0xC423CD6A, 0xC0E2D0DD,
      0xCDA1F604, 0xC960EBB3, 0xBD3E8D7E, 0xB9FF90C9, 0xB4BCB610, 0xB07DABA7,
      0xAE3AFBA2, 0xAAFBE615, 0xA7B8C0CC, 0xA379DD7B, 0x9B3660C6, 0x9FF77D71,
      0x92B45BA8, 0x9675461F, 0x8832161A, 0x8CF30BAD, 0x81B02D74, 0x857130C3,
      0x5D8A9099, 0x594B8D2E, 0x5408ABF7, 0x50C9B640, 0x4E8EE645, 0x4A4FFBF2,
      0x470CDD2B, 0x43CDC09C, 0x7B827D21, 0x7F436096, 0x7200464F, 0x76C15BF8,
      0x68860BFD, 0x6C47164A, 0x61043093, 0x65C52D24, 0x119B4BE9, 0x155A565E,
      0x18197087, 0x1CD86D30, 0x029F3D35, 0x065E2082, 0x0B1D065B, 0x0FDC1BEC,
      0x3793A651, 0x3352BBE6, 0x3E119D3F, 0x3AD08088, 0x2497D08D, 0x2056CD3A,
      0x2D15EBE3, 0x29D4F654, 0xC5A92679, 0xC1683BCE, 0xCC2B1D17, 0xC8EA00A0,
      0xD6AD50A5, 0xD26C4D12, 0xDF2F6BCB, 0xDBEE767C, 0xE3A1CBC1, 0xE760D676,
      0xEA23F0AF, 0xEEE2ED18, 0xF0A5BD1D, 0xF464A0AA, 0xF9278673, 0xFDE69BC4,
      0x89B8FD09, 0x8D79E0BE, 0x803AC667, 0x84FBDBD0, 0x9ABC8BD5, 0x9E7D9662,
      0x933EB0BB, 0x97FFAD0C, 0xAFB010B1, 0xAB710D06, 0xA6322BDF, 0xA2F33668,
      0xBCB4666D, 0xB8757BDA, 0xB5365D03, 0xB1F740B4,
  };

  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < length; i++) {
    uint8_t byte = data[i];
    // If the STM32 uses reflection, you'd reflect 'byte' here.
    uint32_t idx = (crc ^ byte) & 0xFF;
    crc = (crc >> 8) ^ crc_table[idx];
  }
  crc ^= 0x00000000; // final XOR
  return crc;
}

// ---------------------------------------------------------------------------
// Packet builder: 2B hdr + 1B mode + 1B id + 16B data + 4B CRC
// ---------------------------------------------------------------------------
std::vector<uint8_t> createPacket(uint8_t mode, uint8_t id,
                                  const float data[4]) {
  std::vector<uint8_t> packet(SEND_PACKET_SIZE, 0);

  // Header
  packet[0] = PACKET_HEADER_0;
  packet[1] = PACKET_HEADER_1;

  // Mode, ID
  packet[2] = mode;
  packet[3] = id;

  // Copy 4 floats (16 bytes)
  for (int i = 0; i < 4; i++) {
    const uint8_t *fptr = reinterpret_cast<const uint8_t *>(&data[i]);
    packet[4 + 4 * i + 0] = fptr[0];
    packet[4 + 4 * i + 1] = fptr[1];
    packet[4 + 4 * i + 2] = fptr[2];
    packet[4 + 4 * i + 3] = fptr[3];
  }

  // CRC over first 20 bytes
  uint32_t crc = computeCRC32(packet.data(), 20);
  packet[20] = static_cast<uint8_t>(crc & 0xFF);
  packet[21] = static_cast<uint8_t>((crc >> 8) & 0xFF);
  packet[22] = static_cast<uint8_t>((crc >> 16) & 0xFF);
  packet[23] = static_cast<uint8_t>((crc >> 24) & 0xFF);

  return packet;
}

// ---------------------------------------------------------------------------
// Single Class: Stm32ControlNode
// ---------------------------------------------------------------------------
class Stm32ControlNode : public rclcpp::Node {
public:
  Stm32ControlNode(const std::string &port_name, unsigned int baud_rate)
      : Node("mr2_drive_motor_interface"), io_context_(),
        serial_port_(io_context_) {
    // 1. Open serial port
    boost::system::error_code ec;
    serial_port_.open(port_name, ec);
    if (ec) {
      RCLCPP_ERROR(get_logger(), "Failed to open port %s: %s",
                   port_name.c_str(), ec.message().c_str());
      return;
    }

    // 2. Configure baud & 8N1
    serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate),
                            ec);
    if (ec) {
      RCLCPP_ERROR(get_logger(), "Failed to set baud rate: %s",
                   ec.message().c_str());
      return;
    }
    serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_.set_option(boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one));
    serial_port_.set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));

    RCLCPP_INFO(get_logger(), "Opened %s at %u baud.", port_name.c_str(),
                baud_rate);

    // 3. Create ROS Interfaces
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/steering_node/joint_states", 10,
        std::bind(&Stm32ControlNode::onJointState, this,
                  std::placeholders::_1));

    pid_service_ = create_service<std_srvs::srv::Trigger>(
        "/set_pid_params",
        std::bind(&Stm32ControlNode::onPidService, this, std::placeholders::_1,
                  std::placeholders::_2));

    speed_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        "/mr2_drive_motor_interface/current_speed", 10);
    pid_pub_ = create_publisher<mr2_drive_motor::msg::Pid>(
        "/mr2_drive_motor_interface/current_pid", 10);

    // Poll every 500 ms
    // poll_timer_ = create_wall_timer(500ms, [this]() { this->pollStm32(); });

    RCLCPP_INFO(get_logger(), "Stm32ControlNode initialized.");
  }

  ~Stm32ControlNode() {
    if (serial_port_.is_open()) {
      boost::system::error_code ec;
      serial_port_.close(ec);
    }
  }

private:
  // -------------------------------------------------------------------------
  // Subscriber callback
  // -------------------------------------------------------------------------
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->velocity.size() < 4) {
      RCLCPP_WARN(get_logger(),
                  "JointState has <4 velocities (got %zu). Not sending speeds.",
                  msg->velocity.size());
      return;
    }
    float v1 = static_cast<float>(msg->velocity[0]);
    float v2 = static_cast<float>(msg->velocity[1]);
    float v3 = static_cast<float>(msg->velocity[2]);
    float v4 = static_cast<float>(msg->velocity[3]);
    sendWheelSpeeds(v1, v2, v3, v4);
  }

  // -------------------------------------------------------------------------
  // Service callback (dummy: sets some fixed PID)
  // -------------------------------------------------------------------------
  void
  onPidService(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    float p = 1.0f, i = 0.1f, d = 0.01f, a = 0.0f;
    sendPidParams(p, i, d, a);
    response->success = true;
    response->message = "PID set successfully.";
  }

  // -------------------------------------------------------------------------
  // Timer: poll STM32 for speed & PID
  // -------------------------------------------------------------------------
  void pollStm32() {
    requestCurrentSpeed();
    {
      std::vector<float> data;
      if (readFloatsFromSTM32(data)) {
        // Expect 4 floats = speeds
        if (data.size() == 4) {
          RCLCPP_INFO(get_logger(), "Speeds: %f %f %f %f", data[0], data[1],
                      data[2], data[3]);
          auto msg = sensor_msgs::msg::JointState();
          msg.header.stamp = now();
          msg.name = {"wheel_1", "wheel_2", "wheel_3", "wheel_4"};
          msg.velocity = {data[0], data[1], data[2], data[3]};
          speed_pub_->publish(msg);
        }
      }
    }

    requestCurrentPid();
    {
      std::vector<float> data;
      if (readFloatsFromSTM32(data)) {
        // Expect 4 floats = P, I, D, A
        if (data.size() == 4) {
          auto pid_msg = mr2_drive_motor::msg::Pid();
          pid_msg.p = data[0];
          pid_msg.i = data[1];
          pid_msg.d = data[2];
          pid_msg.a = data[3];
          pid_pub_->publish(pid_msg);
        }
      }
    }
  }

  // -------------------------------------------------------------------------
  // Low-level write
  // -------------------------------------------------------------------------
  bool writePacket(const std::vector<uint8_t> &packet) {
    RCLCPP_INFO(get_logger(), "Writing packet of size %zu", packet.size());
    RCLCPP_INFO(get_logger(), "Packet: %02x %02x %02x %02x %02x %02x",
                packet[0], packet[1], packet[2], packet[3], packet[4],
                packet[5]);
    if (!serial_port_.is_open()) {
      RCLCPP_ERROR(get_logger(), "Serial port not open!");
      return false;
    }
    boost::system::error_code ec;
    size_t n =
        boost::asio::write(serial_port_, boost::asio::buffer(packet), ec);
    if (ec) {
      RCLCPP_ERROR(get_logger(), "Write error: %s", ec.message().c_str());
      return false;
    }
    if (n != packet.size()) {
      RCLCPP_ERROR(get_logger(), "Wrote %zu of %zu bytes.", n, packet.size());
      return false;
    }
    return true;
  }

  // -------------------------------------------------------------------------
  // Synchronous read for 20 bytes, parse floats
  // -------------------------------------------------------------------------
  bool readFloatsFromSTM32(std::vector<float> &floats_out) {
    floats_out.clear();
    if (!serial_port_.is_open())
      return false;

    // Attempt reading exactly 20 bytes
    std::vector<uint8_t> buffer(RECEIVE_PACKET_SIZE);
    boost::system::error_code ec;

    RCLCPP_INFO(get_logger(), "Reading packet of size %zu",
                RECEIVE_PACKET_SIZE);

    // This does a blocking read. If STM32 doesn't respond, it'll block or
    // error.
    size_t n = boost::asio::read(
        serial_port_, boost::asio::buffer(buffer),
        boost::asio::transfer_exactly(RECEIVE_PACKET_SIZE), ec);

    RCLCPP_INFO(get_logger(), "Read %zu bytes", n);

    if (ec) {
      RCLCPP_WARN(get_logger(), "Read error: %s", ec.message().c_str());
      return false;
    }
    if (n != RECEIVE_PACKET_SIZE) {
      RCLCPP_WARN(get_logger(), "Partial read: %zu of %zu", n,
                  RECEIVE_PACKET_SIZE);
      return false;
    }

    // Check CRC
    uint32_t rx_crc = 0;
    rx_crc |= static_cast<uint32_t>(buffer[16]);
    rx_crc |= (static_cast<uint32_t>(buffer[17]) << 8);
    rx_crc |= (static_cast<uint32_t>(buffer[18]) << 16);
    rx_crc |= (static_cast<uint32_t>(buffer[19]) << 24);

    uint32_t calc_crc = computeCRC32(buffer.data(), 16);
    if (rx_crc != calc_crc) {
      RCLCPP_WARN(get_logger(), "CRC mismatch: got 0x%08X, expected 0x%08X",
                  rx_crc, calc_crc);
      return false;
    }

    // Unpack floats
    floats_out.resize(4);
    for (int i = 0; i < 4; i++) {
      float val;
      std::memcpy(&val, &buffer[i * 4], sizeof(float));
      floats_out[i] = val;
    }

    return true;
  }

  // -------------------------------------------------------------------------
  // Request current speed / PID
  // -------------------------------------------------------------------------
  void requestCurrentSpeed() {
    float dummy[4] = {0, 0, 0, 0};
    auto pkt = createPacket(MODE_GET_SPEED, 0, dummy);
    if (!writePacket(pkt)) {
      RCLCPP_ERROR(get_logger(), "Failed to write GET_SPEED request");
    }
  }

  void requestCurrentPid() {
    float dummy[4] = {0, 0, 0, 0};
    auto pkt = createPacket(MODE_GET_PID, 0, dummy);
    if (!writePacket(pkt)) {
      RCLCPP_ERROR(get_logger(), "Failed to write GET_PID request");
    }
  }

  // -------------------------------------------------------------------------
  // Send speed / PID
  // -------------------------------------------------------------------------
  void sendWheelSpeeds(float v1, float v2, float v3, float v4) {
    float arr[4] = {v1, v2, v3, v4};
    auto pkt = createPacket(MODE_SET_SPEED, 0 /*all*/, arr);
    writePacket(pkt);
  }

  void sendPidParams(float p, float i, float d, float a) {
    float arr[4] = {p, i, d, a};
    auto pkt = createPacket(MODE_SET_PID, 0, arr);
    writePacket(pkt);
  }

private:
  // ROS
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pid_service_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr speed_pub_;
  rclcpp::Publisher<mr2_drive_motor::msg::Pid>::SharedPtr pid_pub_;
  rclcpp::TimerBase::SharedPtr poll_timer_;

  // Boost.Asio
  boost::asio::io_context io_context_;
  boost::asio::serial_port serial_port_;
};

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::string port = (argc > 1) ? argv[1] : "/dev/ttyACM0";
  unsigned int baud = (argc > 2) ? std::atoi(argv[2]) : 2000000;

  auto node = std::make_shared<Stm32ControlNode>(port, baud);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
