#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

// We'll define a custom message for PID parameters:
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
      0x00000000, 0x77073096, 0xEE0E612C, /* ... */ 0x2D02EF8D
      // fill in the rest if needed
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
      : Node("stm32_control_node"), io_context_(), serial_port_(io_context_) {
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
        "/stm32/current_speed", 10);
    pid_pub_ =
        create_publisher<my_stm32_ros2_pkg::msg::Pid>("/stm32/current_pid", 10);

    // Poll every 500 ms
    poll_timer_ = create_wall_timer(500ms, [this]() { this->pollStm32(); });

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
          auto pid_msg = my_stm32_ros2_pkg::msg::Pid();
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

    // This does a blocking read. If STM32 doesn't respond, it'll block or
    // error.
    size_t n = boost::asio::read(
        serial_port_, boost::asio::buffer(buffer),
        boost::asio::transfer_exactly(RECEIVE_PACKET_SIZE), ec);

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
  rclcpp::Publisher<my_stm32_ros2_pkg::msg::Pid>::SharedPtr pid_pub_;
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

  std::string port = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
  unsigned int baud = (argc > 2) ? std::atoi(argv[2]) : 2000000;

  auto node = std::make_shared<Stm32ControlNode>(port, baud);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
