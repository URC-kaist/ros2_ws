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

    io_thread_ = std::thread([this]() { this->io_context_.run(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    startAsyncRead();

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

    poll_timer_ = create_wall_timer(100ms, [this]() { this->poll(); });

    RCLCPP_INFO(get_logger(), "Stm32ControlNode initialized.");
  }

  ~Stm32ControlNode() {
    if (serial_port_.is_open()) {
      boost::system::error_code ec;
      serial_port_.close(ec);
    }

    this->io_context_.stop();
    io_thread_.join();
  }

private:
  void startAsyncRead() {
    // Keep reading “some” data until the buffer is full or we get an error.
    // We read into the region [bytes_in_buffer_ .. MAX_BUFFER_SIZE].
    serial_port_.async_read_some(
        boost::asio::buffer(read_buffer_ + bytes_in_buffer_,
                            MAX_BUFFER_SIZE - bytes_in_buffer_),
        [this](boost::system::error_code ec, std::size_t bytes_transferred) {
          RCLCPP_INFO(this->get_logger(), "Bytes transferred: %lu",
                      bytes_transferred);
          if (!ec && bytes_transferred > 0) {
            bytes_in_buffer_ += bytes_transferred;
            parseIncomingData(); // see if we can extract packets

            // If we still have space left, read again:
            if (bytes_in_buffer_ < MAX_BUFFER_SIZE) {
              startAsyncRead();
            } else {
              RCLCPP_WARN(this->get_logger(),
                          "Read buffer overflow! Consider increasing size or "
                          "draining more quickly.");
              // In real code, either clear buffer or handle the overflow
              // somehow
              bytes_in_buffer_ = 0;
              startAsyncRead();
            }
          } else {
            // Some error or 0 bytes read
            RCLCPP_WARN(this->get_logger(), "Async read error: %s",
                        ec.message().c_str());
            // Optionally, try to restart read or handle the error
            // e.g. close/reopen the port or log & stop
          }
        });
  }

  void parseIncomingData() {
    // We may have multiple packets in our buffer, or maybe partial packets.
    // We will keep scanning while there's enough data to form a full packet.

    // We'll do a `while` loop that hunts for the 2-byte header 0xAA55,
    // and if found, tries to parse the rest of the RxPacket.

    size_t offset = 0;
    while (true) {
      // 1) We need at least 2 bytes to check the header:
      if ((bytes_in_buffer_ - offset) < 2) {
        break; // not enough data to find a header
      }

      // Check for the 2-byte header 0xAA55:
      const uint8_t header_low = 0x55;
      const uint8_t header_high = 0xAA;

      // We read raw bytes from read_buffer_:
      uint8_t b0 = read_buffer_[offset];
      uint8_t b1 = read_buffer_[offset + 1];

      if (b0 == header_low && b1 == header_high) {
        // Found the 2-byte header at position `offset`.
        // Check if we have enough for the entire RxPacket:

        if ((bytes_in_buffer_ - offset) < RX_PACKET_SIZE) {
          // We haven't yet read the entire packet, need more data
          break;
        }

        // We do have enough bytes for a full RxPacket:
        RxPacket rx;
        std::memcpy(&rx, &read_buffer_[offset], RX_PACKET_SIZE);

        // Verify CRC:
        uint32_t calc_crc = computeRxCrc(rx);
        if (rx.crc == calc_crc) {
          // Good packet
          handleRxPacket(rx);
          // Move offset forward by RX_PACKET_SIZE
          offset += RX_PACKET_SIZE;
        } else {
          // Bad CRC.
          RCLCPP_WARN(get_logger(), "Bad CRC, discarding bytes up to header+2");
          // Typically, you might discard this “header” and move on,
          // hoping to find another valid header further on.
          offset += 2;
        }
      } else {
        // Not a valid header at `offset`.
        // Move forward by 1 and keep looking
        offset += 1;
      }

      // Keep scanning in a loop
    }

    // If we exit the loop, `offset` tells us how many bytes we have
    // successfully consumed (and/or discarded).
    if (offset > 0) {
      // Shift everything in read_buffer_ down by `offset`
      // e.g. read_buffer_[0..(bytes_in_buffer_-offset-1)] =
      // read_buffer_[offset..(bytes_in_buffer_-1)]
      size_t remaining = bytes_in_buffer_ - offset;
      if (remaining > 0) {
        std::memmove(read_buffer_, read_buffer_ + offset, remaining);
      }
      bytes_in_buffer_ = remaining;
    }
  }

  void handleRxPacket(const RxPacket &rx) {
    // Check rx.state, for example:
    if (rx.state != 0) {
      // handle error
      RCLCPP_ERROR(
          get_logger(),
          "STM32 error state=%u, code=0x%08X, data_1=0x%08X, data_2=0x%08X",
          rx.state, rx.u[0], rx.u[1], rx.u[2]);
      return;
    }

    // If it’s a speed packet, publish it:
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = now();
    msg.name = {"wheel_1", "wheel_2", "wheel_3", "wheel_4"};
    msg.velocity = {rx.f[0], rx.f[1], rx.f[2], rx.f[3]};
    speed_pub_->publish(msg);

    // If it’s a PID response, parse rx.f as your P, I, D, etc.
    // ...
  }

  uint32_t computeRxCrc(const RxPacket &rx) {
    // The CRC is computed over the entire RxPacket except the last 4 bytes
    // (crc).
    const size_t length = RX_PACKET_SIZE - sizeof(rx.crc);
    return computeCRC32(reinterpret_cast<const uint8_t *>(&rx), length);
  }

  void writeTxPacket(const TxPacket &packet) {
    if (!serial_port_.is_open()) {
      RCLCPP_ERROR(get_logger(), "Serial port not open for writing!");
      return;
    }

    // cast packet to raw uint8_t array
    uint8_t packet_raw[TX_PACKET_SIZE];
    memcpy(packet_raw, &packet, TX_PACKET_SIZE);

    std::vector<uint8_t> buffer(sizeof(packet));
    std::memcpy(buffer.data(), &packet, sizeof(packet));

    RCLCPP_INFO(get_logger(), "Async write started.");

    auto self = shared_from_this();
    boost::asio::async_write(
        serial_port_, boost::asio::buffer(buffer),
        [this, self, buffer](boost::system::error_code ec,
                             std::size_t size_written) {
          std::cout << "Async write complete." << std::endl;
          if (!ec && size_written == sizeof(TxPacket)) {
          } else {
            RCLCPP_ERROR(get_logger(), "Async write error: %s",
                         ec.message().c_str());
          }
        });
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
    float v1 = static_cast<float>(msg->velocity[3] * 10.0f);
    float v2 = static_cast<float>(msg->velocity[1] * 10.0f);
    float v3 = static_cast<float>(msg->velocity[0] * 10.0f);
    float v4 = static_cast<float>(msg->velocity[2] * 10.0f);

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
  void poll() { requestCurrentSpeed(); }

  // -------------------------------------------------------------------------
  // 4) Send commands
  // -------------------------------------------------------------------------
  void sendWheelSpeeds(float v1, float v2, float v3, float v4) {
    float arr[4] = {v1, v2, v3, v4};
    TxPacket pkt = createTxPacket(TX_HEADER, MODE_SET_SPEED, 0 /*id=all*/, arr);
    writeTxPacket(pkt);
  }

  void sendPidParams(float p, float i, float d, float a) {
    float arr[4] = {p, i, d, a};
    TxPacket pkt = createTxPacket(TX_HEADER, MODE_SET_PID, 0, arr);
    writeTxPacket(pkt);
  }

  void requestCurrentSpeed() {
    float dummy[4] = {0, 0, 0, 0};
    TxPacket pkt = createTxPacket(TX_HEADER, MODE_GET_SPEED, 0, dummy);
    writeTxPacket(pkt);
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
  static constexpr size_t MAX_BUFFER_SIZE = 1024;
  uint8_t read_buffer_[MAX_BUFFER_SIZE];
  size_t bytes_in_buffer_ = 0; // how many bytes are currently stored
  std::thread io_thread_;
};

// ---------------------------------------------------------------------------
// main()
// ---------------------------------------------------------------------------
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Default to /dev/ttyACM0, 9221600 if not specified
  // std::string port = (argc > 1) ? argv[1] : "/dev/ttyACM0";
  // unsigned int baud = (argc > 2) ? std::atoi(argv[2]) : 921600;

  std::string port = "/dev/ttyACM0";
  unsigned int baud = 921600;

  auto node = std::make_shared<Stm32ControlNode>(port, baud);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
