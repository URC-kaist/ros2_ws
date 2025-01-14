#include <mr2_drive_motor/msg/pid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>

// Include Boost.CRC
#include <boost/crc.hpp>

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

// Headers (per your new spec)
static constexpr uint16_t TX_HEADER = 0xAF40; // PC → STM32
static constexpr uint16_t RX_HEADER = 0xAA55; // STM32 → PC

// Packet modes/commands
static constexpr uint8_t MODE_INIT = 0;
static constexpr uint8_t MODE_SET_SPEED = 1;
static constexpr uint8_t MODE_GET_SPEED = 2;
static constexpr uint8_t MODE_SET_BRAKE = 3;
static constexpr uint8_t MODE_SET_PID = 4;
static constexpr uint8_t MODE_GET_PID = 5;

// Error codes
static constexpr uint32_t ERROR_HARDWARE = 0xFF000000;
static constexpr uint32_t ERROR_COMMUNICATION_TX = 0xFF010000;
static constexpr uint32_t ERROR_COMMUNICATION_RX = 0xFF010100;
static constexpr uint32_t ERROR_CRC = 0xFF020000;
// etc.

#pragma pack(push, 1)
struct TxPacket {
  uint16_t header; // e.g. 0xAA55
  uint8_t mode;
  uint8_t id;
  float data[4];
  uint32_t crc;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct RxPacket {
  uint16_t header; // e.g. 0xAF40
  uint16_t state;  // 0 = OK, non-zero = error

  union {
    float f[4];    // if state == 0
    uint32_t u[4]; // if state != 0 (error code, etc.)
  };

  uint32_t crc;
};
#pragma pack(pop)

static constexpr size_t TX_PACKET_SIZE = sizeof(TxPacket);
static constexpr size_t RX_PACKET_SIZE = sizeof(RxPacket);

using MyCrc32 =
    boost::crc_optimal<32,
                       0x04C11DB7, // polynomial
                       0xFFFFFFFF, // initial remainder
                       0x00000000, // final XOR
                       true,       // reflect input (true = with reflection)
                       true        // reflect output (true = with reflection)
                       >;

uint32_t computeCRC32(const uint8_t *data, size_t length) {
  MyCrc32 crc;
  crc.process_bytes(data, length);
  return crc.checksum();
}

TxPacket createTxPacket(uint16_t header, uint8_t mode, uint8_t id,
                        const float data[4]) {
  TxPacket pkt{};
  pkt.header = header;
  pkt.mode = mode;
  pkt.id = id;

  // Copy data
  for (int i = 0; i < 4; i++) {
    pkt.data[i] = data[i];
  }

  // Compute CRC over the first (24 - 4) = 20 bytes
  const size_t crc_length = TX_PACKET_SIZE - sizeof(pkt.crc);
  pkt.crc = computeCRC32(reinterpret_cast<const uint8_t *>(&pkt), crc_length);

  return pkt;
}

class Stm32ControlNode : public rclcpp::Node {
public:
  Stm32ControlNode(const std::string &port_name, unsigned int baud_rate)
      : Node("mr2_drive_motor_interface"), io_context_(),
        serial_port_(io_context_) {
    boost::system::error_code ec;
    serial_port_.open(port_name, ec);
    if (ec) {
      RCLCPP_ERROR(get_logger(), "Failed to open port %s: %s",
                   port_name.c_str(), ec.message().c_str());
      return;
    }

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

    poll_timer_ = create_wall_timer(100ms, [this]() { this->pollStm32(); });

    RCLCPP_INFO(get_logger(), "Stm32ControlNode initialized.");
  }

  ~Stm32ControlNode() {
    if (serial_port_.is_open()) {
      boost::system::error_code ec;
      serial_port_.close(ec);
    }
  }

private:
  bool writeTxPacket(const TxPacket &packet) {
    if (!serial_port_.is_open()) {
      RCLCPP_ERROR(get_logger(), "Serial port not open for writing!");
      return false;
    }
    boost::system::error_code ec;

    // cast packet to raw uint8_t array
    uint8_t packet_raw[TX_PACKET_SIZE];
    memcpy(packet_raw, &packet, TX_PACKET_SIZE);

    size_t n = boost::asio::write(
        serial_port_, boost::asio::buffer(&packet, TX_PACKET_SIZE), ec);
    if (ec) {
      RCLCPP_ERROR(get_logger(), "Write error: %s", ec.message().c_str());
      return false;
    }
    if (n != TX_PACKET_SIZE) {
      RCLCPP_ERROR(get_logger(), "Wrote %zu of %zu bytes.", n, TX_PACKET_SIZE);
      return false;
    }
    return true;
  }

  bool readRxPacket(RxPacket &rx_out) {
    if (!serial_port_.is_open()) {
      RCLCPP_WARN(get_logger(), "Serial port not open for reading!");
      return false;
    }

    boost::system::error_code ec;
    size_t n = boost::asio::read(
        serial_port_, boost::asio::buffer(&rx_out, RX_PACKET_SIZE),
        boost::asio::transfer_exactly(RX_PACKET_SIZE), ec);

    if (ec) {
      RCLCPP_WARN(get_logger(), "Read error: %s", ec.message().c_str());
      return false;
    }
    if (n != RX_PACKET_SIZE) {
      RCLCPP_WARN(get_logger(), "Partial read: %zu of %zu", n, RX_PACKET_SIZE);
      return false;
    }

    // Verify header
    if (rx_out.header != RX_HEADER) {
      RCLCPP_WARN(get_logger(), "Invalid RX header: 0x%04X (expected 0x%04X)",
                  rx_out.header, RX_HEADER);
      return false;
    }

    // Verify CRC (first 20 bytes, ignoring rx_out.crc)
    const size_t crc_len = RX_PACKET_SIZE - sizeof(rx_out.crc);
    uint32_t calc_crc =
        computeCRC32(reinterpret_cast<const uint8_t *>(&rx_out), crc_len);
    if (rx_out.crc != calc_crc) {
      RCLCPP_WARN(get_logger(), "CRC mismatch: got=0x%08X, expected=0x%08X",
                  rx_out.crc, calc_crc);
      return false;
    }

    return true;
  }

  // -------------------------------------------------------------------------
  // 1) onJointState: send speeds to STM32
  // -------------------------------------------------------------------------
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // Expect 4 velocities
    if (msg->velocity.size() < 4) {
      RCLCPP_WARN(get_logger(),
                  "JointState has <4 velocities (got %zu). Not sending speeds.",
                  msg->velocity.size());
      return;
    }

    // Convert rad/s -> your desired scaling (for example, * 50)
    float v1 = static_cast<float>(msg->velocity[0] * 50.0f);
    float v2 = static_cast<float>(msg->velocity[1] * 50.0f);
    float v3 = static_cast<float>(msg->velocity[2] * 50.0f);
    float v4 = static_cast<float>(msg->velocity[3] * 50.0f);

    sendWheelSpeeds(v1, v2, v3, v4);
  }

  // -------------------------------------------------------------------------
  // 2) onPidService: set some example PID params
  // -------------------------------------------------------------------------
  void
  onPidService(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    // Arbitrary example
    float p = 1.0f;
    float i = 0.1f;
    float d = 0.01f;
    float a = 0.0f;

    sendPidParams(p, i, d, a);

    response->success = true;
    response->message = "PID set successfully.";
  }

  // -------------------------------------------------------------------------
  // 3) pollStm32: periodically request data from STM32
  // -------------------------------------------------------------------------
  void pollStm32() {
    // Request current speed
    requestCurrentSpeed();

    {
      // Read back the RxPacket
      RxPacket rx;
      if (!readRxPacket(rx)) {
        RCLCPP_WARN(get_logger(), "Failed to read speed packet from STM32");
        return;
      }

      // If rx.state != 0, there's an error
      if (rx.state != 0) {
        // interpret rx.u[0] as error code
        RCLCPP_ERROR(get_logger(),
                     "STM32 reported error state=%u, code=0x%08X, "
                     "data_1=0x%08X, data_2=0x%08X",
                     rx.state, rx.u[0], rx.u[1], rx.u[2]);
        return;
      }

      // Publish
      auto msg = sensor_msgs::msg::JointState();
      msg.header.stamp = now();
      msg.name = {"wheel_1", "wheel_2", "wheel_3", "wheel_4"};
      msg.velocity = {rx.f[0], rx.f[1], rx.f[2], rx.f[3]};
      speed_pub_->publish(msg);
    }

    // You could also request PID, parse it, etc., in a similar fashion
    // ...
  }

  // -------------------------------------------------------------------------
  // 4) Send commands
  // -------------------------------------------------------------------------
  void sendWheelSpeeds(float v1, float v2, float v3, float v4) {
    float arr[4] = {v1, v2, v3, v4};
    TxPacket pkt = createTxPacket(TX_HEADER, MODE_SET_SPEED, 0 /*id=all*/, arr);

    if (!writeTxPacket(pkt)) {
      RCLCPP_ERROR(get_logger(), "Failed to write SET_SPEED packet!");
    }
  }

  void sendPidParams(float p, float i, float d, float a) {
    float arr[4] = {p, i, d, a};
    TxPacket pkt = createTxPacket(TX_HEADER, MODE_SET_PID, 0, arr);

    if (!writeTxPacket(pkt)) {
      RCLCPP_ERROR(get_logger(), "Failed to write SET_PID packet!");
    }
  }

  void requestCurrentSpeed() {
    float dummy[4] = {0, 0, 0, 0};
    TxPacket pkt = createTxPacket(TX_HEADER, MODE_GET_SPEED, 0, dummy);
    if (!writeTxPacket(pkt)) {
      RCLCPP_ERROR(get_logger(), "Failed to write GET_SPEED packet!");
    }
  }

private:
  // ROS interfaces
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
// main()
// ---------------------------------------------------------------------------
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Default to /dev/ttyACM0, 921600 if not specified
  std::string port = (argc > 1) ? argv[1] : "/dev/ttyACM0";
  unsigned int baud = (argc > 2) ? std::atoi(argv[2]) : 921600;

  auto node = std::make_shared<Stm32ControlNode>(port, baud);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
