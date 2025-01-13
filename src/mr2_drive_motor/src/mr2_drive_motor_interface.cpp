#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// Example custom PID service message or you can use a std_srvs or any other
// message. This is just a placeholder. Adjust as needed.
#include <std_srvs/srv/trigger.hpp>

// Boost.Asio headers
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <boost/system/error_code.hpp>

#include <memory>
#include <string>
#include <vector>

// -------------------------------------------------------------------------
// Constants for packet building
// -------------------------------------------------------------------------
static constexpr uint8_t PACKET_HEADER_0 = 0xAA;
static constexpr uint8_t PACKET_HEADER_1 = 0x55;

// Example modes (must match your STM32 firmware)
static constexpr uint8_t MODE_INIT = 0;
static constexpr uint8_t MODE_SET_SPEED = 1;
static constexpr uint8_t MODE_GET_SPEED = 2;
static constexpr uint8_t MODE_SET_BRAKE = 3;
static constexpr uint8_t MODE_SET_PID = 4;
static constexpr uint8_t MODE_GET_PID = 5;

// Packet sizes
static constexpr size_t SEND_PACKET_SIZE =
    24; // header(2) + mode(1) + id(1) + data(16) + CRC(4)
static constexpr size_t RECEIVE_PACKET_SIZE = 20; // data(16) + CRC(4)

// -------------------------------------------------------------------------
// A simple CRC32 placeholder. Adjust to match your exact STM32 requirements.
// -------------------------------------------------------------------------
static uint32_t computeCRC32(const uint8_t *data, size_t length) {
  // Example polynomial 0x04C11DB7 (standard CRC-32), no reflection in this
  // snippet.
  static const uint32_t crc_table[256] = {
      /* You would fill in a full 256-entry lookup table here. */
      0x00000000, 0x77073096, 0xEE0E612C, /* ... */ 0x2D02EF8D
      // etc.
  };

  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < length; i++) {
    uint8_t byte = data[i];
    // If reflection is needed (rev=True in Python), you'd reverse bits here.
    uint32_t idx = (crc ^ byte) & 0xFF;
    crc = (crc >> 8) ^ crc_table[idx];
  }

  // Final XOR (if required)
  crc ^= 0x00000000;
  return crc;
}

// -------------------------------------------------------------------------
// Helper to create a packet (2B header + 1B mode + 1B ID + 16B floats + 4B
// CRC). data[4] is an array of four floats to send.
// -------------------------------------------------------------------------
static std::vector<uint8_t> createPacket(uint8_t mode, uint8_t id,
                                         const float data[4]) {
  std::vector<uint8_t> packet(SEND_PACKET_SIZE, 0);

  // Header
  packet[0] = PACKET_HEADER_0;
  packet[1] = PACKET_HEADER_1;

  // Mode and ID
  packet[2] = mode;
  packet[3] = id;

  // Copy 4 floats (16 bytes) in little-endian
  for (int i = 0; i < 4; i++) {
    const uint8_t *float_ptr = reinterpret_cast<const uint8_t *>(&data[i]);
    packet[4 + i * 4 + 0] = float_ptr[0];
    packet[4 + i * 4 + 1] = float_ptr[1];
    packet[4 + i * 4 + 2] = float_ptr[2];
    packet[4 + i * 4 + 3] = float_ptr[3];
  }

  // Compute CRC over first 20 bytes
  uint32_t crc = computeCRC32(packet.data(), 20);

  // Store CRC in last 4 bytes (little-endian)
  packet[20] = static_cast<uint8_t>(crc & 0xFF);
  packet[21] = static_cast<uint8_t>((crc >> 8) & 0xFF);
  packet[22] = static_cast<uint8_t>((crc >> 16) & 0xFF);
  packet[23] = static_cast<uint8_t>((crc >> 24) & 0xFF);

  return packet;
}

// ==============================================================================
// Class: Stm32ControlNode
// ==============================================================================
class Stm32ControlNode : public rclcpp::Node {
public:
  Stm32ControlNode(const std::string &port_name, unsigned int baud_rate)
      : Node("stm32_control_node"), io_context_(), serial_port_(io_context_) {
    // 1. Open the serial port using Boost.Asio
    boost::system::error_code ec;

    serial_port_.open(port_name, ec);
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s: %s",
                   port_name.c_str(), ec.message().c_str());
      return;
    }

    // 2. Set serial port options
    serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate),
                            ec);
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set baud rate: %s",
                   ec.message().c_str());
      return;
    }
    serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_.set_option(boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one));
    serial_port_.set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));

    RCLCPP_INFO(this->get_logger(), "Opened serial port %s at %u baud.",
                port_name.c_str(), baud_rate);

    // 3. Create subscriber to JointState
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/steering_node/joint_states", 10,
        std::bind(&Stm32ControlNode::jointStateCallback, this,
                  std::placeholders::_1));

    // 4. Create a service for PID (example uses std_srvs::srv::Trigger, but
    //    you'd typically have your own custom service with P, I, D, etc.)
    pid_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/set_pid_params",
        std::bind(&Stm32ControlNode::pidServiceCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Optional: If you plan to do asynchronous I/O, you might start a
    // separate thread for io_context_.run(). For synchronous usage, you
    // can just do blocking reads/writes inside your callbacks.
    //
    // io_thread_ = std::thread([this]() {
    //   io_context_.run();
    // });
  }

  ~Stm32ControlNode() {
    // Cleanly close the serial port
    if (serial_port_.is_open()) {
      boost::system::error_code ec;
      serial_port_.close(ec);
    }
    // If running an io_thread_, join it
    // if (io_thread_.joinable()) {
    //   io_thread_.join();
    // }
  }

private:
  // ---------------------------------------------------------------------------
  // Subscriber callback
  // We'll assume the first four velocities in JointState map to 4 wheels.
  // ---------------------------------------------------------------------------
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->velocity.size() < 4) {
      RCLCPP_WARN(
          this->get_logger(),
          "Received JointState with not enough velocity elements (need 4).");
      return;
    }
    float v1 = static_cast<float>(msg->velocity[0]);
    float v2 = static_cast<float>(msg->velocity[1]);
    float v3 = static_cast<float>(msg->velocity[2]);
    float v4 = static_cast<float>(msg->velocity[3]);

    // Build & send "SET_SPEED" packet
    this->sendWheelSpeeds(v1, v2, v3, v4);
  }

  // ---------------------------------------------------------------------------
  // Example service callback
  // For demonstration, we assume a Trigger service. A real PID service would
  // have float parameters P, I, D, etc.
  // ---------------------------------------------------------------------------
  void pidServiceCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request; // Not used in this placeholder

    // Suppose you have some fixed PID to set, or you read from a param. For
    // now:
    float p = 1.0f;
    float i = 0.1f;
    float d = 0.01f;
    float a = 0.0f;

    this->sendPidParams(p, i, d, a);
    response->success = true;
    response->message = "PID params set successfully.";
  }

  // ---------------------------------------------------------------------------
  // Helper: sendWheelSpeeds
  // ---------------------------------------------------------------------------
  void sendWheelSpeeds(float v1, float v2, float v3, float v4) {
    float data[4] = {v1, v2, v3, v4};

    // "packet_id = 0" to set all motors, per your example
    uint8_t packet_id = 0;
    auto packet = createPacket(MODE_SET_SPEED, packet_id, data);

    writePacket(packet, "SetWheelSpeeds");
  }

  // ---------------------------------------------------------------------------
  // Helper: sendPidParams
  // ---------------------------------------------------------------------------
  void sendPidParams(float p, float i, float d, float a) {
    float data[4] = {p, i, d, a};
    uint8_t packet_id = 0; // or specify a motor ID if needed

    auto packet = createPacket(MODE_SET_PID, packet_id, data);
    writePacket(packet, "SetPidParams");
  }

  // ---------------------------------------------------------------------------
  // Low-level write function using synchronous Boost.Asio
  // ---------------------------------------------------------------------------
  void writePacket(const std::vector<uint8_t> &packet, const std::string &tag) {
    if (!serial_port_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "[%s] Serial port is not open!",
                   tag.c_str());
      return;
    }

    boost::system::error_code ec;
    size_t bytes_written =
        boost::asio::write(serial_port_, boost::asio::buffer(packet), ec);
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "[%s] Write error: %s", tag.c_str(),
                   ec.message().c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "[%s] Wrote %zu bytes to STM32.",
                tag.c_str(), bytes_written);
  }

private:
  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pid_service_;

  // Boost.Asio objects
  boost::asio::io_context io_context_;
  boost::asio::serial_port serial_port_;

  // Optional thread for async usage:
  // std::thread io_thread_;
};

// -----------------------------------------------------------------------------
// main
// -----------------------------------------------------------------------------
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Example usage: pass serial device name and baud rate as arguments:
  //   ros2 run my_package stm32_control_boost_asio /dev/ttyUSB0 2250000
  std::string port = (argc > 1) ? std::string(argv[1]) : "/dev/ttyUSB0";
  unsigned int baud = (argc > 2) ? std::atoi(argv[2]) : 2250000;

  auto node = std::make_shared<Stm32ControlNode>(port, baud);

  // If you are not using asynchronous read, you can simply spin the node:
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
