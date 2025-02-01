#include <mr2_drive_motor/msg/pid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <boost/asio.hpp>
#include <boost/crc.hpp>
#include <boost/system/error_code.hpp>

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

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

// Packet Structures
#pragma pack(push, 1)
struct TxPacket {
  uint16_t header;
  uint8_t mode;
  uint8_t id;
  float data[4];
  uint32_t crc;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct RxPacket {
  uint16_t header;
  uint16_t state; // 0 = OK, non-zero = error

  union {
    float f[4];    // if state == 0
    uint32_t u[4]; // if state != 0 (error codes etc.)
  };

  uint32_t crc;
};
#pragma pack(pop)

static constexpr size_t TX_PACKET_SIZE = sizeof(TxPacket);
static constexpr size_t RX_PACKET_SIZE = sizeof(RxPacket);

// ---------------------------------------------------------------------------
// CRC Implementation
// ---------------------------------------------------------------------------
using MyCrc32 = boost::crc_optimal<32,
                                   0x04C11DB7, // polynomial
                                   0xFFFFFFFF, // initial remainder
                                   0x00000000, // final XOR value
                                   true,       // reflect input
                                   true        // reflect output
                                   >;

uint32_t computeCRC32(const uint8_t *data, size_t length) {
  MyCrc32 crc;
  crc.process_bytes(data, length);
  return crc.checksum();
}

uint32_t computeRxCrc(const RxPacket &rx) {
  constexpr size_t length = RX_PACKET_SIZE - sizeof(rx.crc);
  return computeCRC32(reinterpret_cast<const uint8_t *>(&rx), length);
}

TxPacket createTxPacket(uint16_t header, uint8_t mode, uint8_t id,
                        const float data[4]) {
  TxPacket pkt{};
  pkt.header = header;
  pkt.mode = mode;
  pkt.id = id;

  for (int i = 0; i < 4; i++) {
    pkt.data[i] = data[i];
  }

  // Compute CRC over everything except the final 4 bytes
  constexpr size_t crc_length = TX_PACKET_SIZE - sizeof(pkt.crc);
  pkt.crc = computeCRC32(reinterpret_cast<const uint8_t *>(&pkt), crc_length);
  return pkt;
}

class Stm32ControlNode : public rclcpp::Node,
                         public std::enable_shared_from_this<Stm32ControlNode> {
public:
  Stm32ControlNode(const std::string &port_name, unsigned int baud_rate)
      : Node("mr2_drive_motor_interface"), io_context_(),
        serial_port_(io_context_),
        write_strand_(boost::asio::make_strand(io_context_)) {
    // Attempt to open the serial port
    boost::system::error_code ec;
    serial_port_.open(port_name, ec);
    if (ec) {
      RCLCPP_ERROR(get_logger(), "Failed to open port %s: %s",
                   port_name.c_str(), ec.message().c_str());
      // Throw so we don't proceed with a half-initialized node
      throw std::runtime_error("Serial port open failed");
    }

    // Attempt to set the baud rate
    serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate),
                            ec);
    if (ec) {
      RCLCPP_ERROR(get_logger(), "Failed to set baud rate %u: %s", baud_rate,
                   ec.message().c_str());
      throw std::runtime_error("Setting baud rate failed");
    }

    // Other serial port settings
    serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_.set_option(boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one));
    serial_port_.set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));

    RCLCPP_INFO(get_logger(), "Opened %s at %u baud.", port_name.c_str(),
                baud_rate);

    // Start the I/O thread
    io_thread_ = std::thread([this]() {
      RCLCPP_INFO(get_logger(), "Starting I/O");
      io_context_.run();
    });

    // Begin asynchronous reads
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

    // Poll the STM32 at 10 Hz for speed data, etc.
    poll_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                    [this]() { this->poll(); });

    RCLCPP_INFO(get_logger(), "Stm32ControlNode initialized.");
  }

  ~Stm32ControlNode() override {
    if (serial_port_.is_open()) {
      boost::system::error_code ec;
      serial_port_.close(ec);
      if (ec) {
        RCLCPP_WARN(get_logger(), "Error closing serial port: %s",
                    ec.message().c_str());
      }
    }
    // Stop the I/O context and join the thread
    io_context_.stop();
    if (io_thread_.joinable()) {
      io_thread_.join();
    }
  }

private:
  // -------------------------------------------------------------------------
  // Start an async read into the read_buffer_
  // -------------------------------------------------------------------------
  void startAsyncRead() {
    serial_port_.async_read_some(
        boost::asio::buffer(read_buffer_ + bytes_in_buffer_,
                            MAX_BUFFER_SIZE - bytes_in_buffer_),
        [this](boost::system::error_code ec, std::size_t bytes_transferred) {
          if (!ec && bytes_transferred > 0) {
            RCLCPP_DEBUG(this->get_logger(), "Bytes transferred: %zu",
                         bytes_transferred);

            bytes_in_buffer_ += bytes_transferred;
            parseIncomingData();

            // If there's still room, read again:
            if (bytes_in_buffer_ < MAX_BUFFER_SIZE) {
              startAsyncRead();
            } else {
              RCLCPP_WARN(this->get_logger(),
                          "Read buffer overflow! Clearing buffer...");
              bytes_in_buffer_ = 0;
              startAsyncRead();
            }
          } else {
            RCLCPP_WARN(this->get_logger(), "Async read error: %s",
                        ec.message().c_str());
            // Possibly retry or handle the error
          }
        });
  }

  // -------------------------------------------------------------------------
  // Attempt to parse all complete RxPackets from read_buffer_
  // -------------------------------------------------------------------------
  void parseIncomingData() {
    // We may have multiple packets or partial packets in the buffer.
    // Keep scanning as long as there's enough data for at least a header.

    size_t offset = 0;
    while (true) {
      // Need at least 2 bytes to check for the header
      if ((bytes_in_buffer_ - offset) < 2) {
        break;
      }

      // Check for the 2-byte header 0xAA55 in little-endian form:
      const uint8_t header_low = 0x55;
      const uint8_t header_high = 0xAA;

      uint8_t b0 = read_buffer_[offset];
      uint8_t b1 = read_buffer_[offset + 1];

      if (b0 == header_low && b1 == header_high) {
        // We found the potential start of a packet
        if ((bytes_in_buffer_ - offset) < RX_PACKET_SIZE) {
          // Not enough bytes for a full packet yet
          break;
        }

        // We have enough data for a full RxPacket
        RxPacket rx;
        std::memcpy(&rx, &read_buffer_[offset], RX_PACKET_SIZE);

        // Double-check the full 16-bit header just to be safe
        if (rx.header != RX_HEADER) {
          RCLCPP_WARN(get_logger(),
                      "Mismatched header in bytes; discarding 2 bytes...");
          offset += 2;
          continue;
        }

        // Check CRC
        uint32_t calc_crc = computeRxCrc(rx);
        if (rx.crc == calc_crc) {
          // Valid packet
          handleRxPacket(rx);
          offset += RX_PACKET_SIZE;
        } else {
          RCLCPP_WARN(get_logger(),
                      "Bad CRC. Discarding 2 bytes and continuing search...");
          offset += 2;
        }
      } else {
        // Not a valid header at offset, move on by 1
        offset += 1;
      }
    }

    // Shift down any leftover bytes
    if (offset > 0) {
      size_t remaining = bytes_in_buffer_ - offset;
      if (remaining > 0) {
        std::memmove(read_buffer_, read_buffer_ + offset, remaining);
      }
      bytes_in_buffer_ = remaining;
    }
  }

  // -------------------------------------------------------------------------
  // Process a valid RxPacket
  // -------------------------------------------------------------------------
  void handleRxPacket(const RxPacket &rx) {
    // If there's an error (rx.state != 0), parse the union as 'u'
    if (rx.state != 0) {
      RCLCPP_ERROR(get_logger(),
                   "STM32 error state=%u, code=0x%08X, data_1=0x%08X, "
                   "data_2=0x%08X, data_3=0x%08X",
                   rx.state, rx.u[0], rx.u[1], rx.u[2], rx.u[3]);
      return;
    }
  }

  // -------------------------------------------------------------------------
  // Send a TxPacket asynchronously
  // -------------------------------------------------------------------------
  void writeTxPacket(const TxPacket &packet) {
    if (!serial_port_.is_open()) {
      RCLCPP_ERROR(get_logger(), "Serial port is not open. Cannot write.");
      return;
    }

    // Copy packet data into a shared buffer so it lives through async_write
    auto buffer_ptr = std::make_shared<std::vector<uint8_t>>(TX_PACKET_SIZE);
    std::memcpy(buffer_ptr->data(), &packet, TX_PACKET_SIZE);

    // Post onto the strand. The lambda is then executed in our io_context_
    // thread *in sequence* with respect to other strand-posted writes.
    boost::asio::post(write_strand_, [this, buffer_ptr]() {
      // Double-check the port
      if (!serial_port_.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Serial port closed before write.");
        return;
      }

      // Start async_write
      boost::asio::async_write(
          serial_port_, boost::asio::buffer(*buffer_ptr),
          // Bind handler into the strand again (good practice).
          // Or you can rely on the fact we called async_write from within
          // the strand. We'll explicitly bind here for clarity.
          boost::asio::bind_executor(
              write_strand_,
              [this, buffer_ptr](const boost::system::error_code &ec,
                                 std::size_t size_written) {
                if (ec) {
                  RCLCPP_ERROR(this->get_logger(), "Async write error: %s",
                               ec.message().c_str());
                } else if (size_written != TX_PACKET_SIZE) {
                  RCLCPP_WARN(this->get_logger(),
                              "Wrote partial packet (%zu/%zu bytes).",
                              size_written, TX_PACKET_SIZE);
                } else {
                  RCLCPP_DEBUG(this->get_logger(),
                               "Async write complete (%zu bytes).",
                               size_written);
                }
              }));
    });
  }

  // -------------------------------------------------------------------------
  // ROS Callbacks
  // -------------------------------------------------------------------------

  // (1) Called when we get new JointState from /steering_node/joint_states
  //     => Send speeds to STM32
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->velocity.size() < 4) {
      RCLCPP_WARN(get_logger(),
                  "JointState has <4 velocities (got %zu). Not sending speeds.",
                  msg->velocity.size());
      return;
    }

    // Example scale from rad/s to something else
    float v1 = static_cast<float>(msg->velocity[3] * 10.0f);
    float v2 = static_cast<float>(msg->velocity[1] * 10.0f);
    float v3 = static_cast<float>(msg->velocity[0] * 10.0f);
    float v4 = static_cast<float>(msg->velocity[2] * 10.0f);

    sendWheelSpeeds(v1, v2, v3, v4);
  }

  // (2) Called when /set_pid_params service is invoked
  void
  onPidService(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    // Example
    float p = 1.0f;
    float i = 0.1f;
    float d = 0.01f;
    float a = 0.0f;

    sendPidParams(p, i, d, a);

    response->success = true;
    response->message = "PID parameters sent successfully.";
  }

  // (3) Timer callback to poll STM32
  void poll() {
    requestCurrentSpeed();
    // If you also want to poll PID, etc., you can do that here or less often.
  }

  // -------------------------------------------------------------------------
  // Helper functions to send commands
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
  boost::asio::strand<boost::asio::io_context::executor_type> write_strand_;

  static constexpr size_t MAX_BUFFER_SIZE = 1024;
  uint8_t read_buffer_[MAX_BUFFER_SIZE];
  size_t bytes_in_buffer_ = 0;

  // Thread that runs io_context_.run()
  std::thread io_thread_;
};

// ---------------------------------------------------------------------------
// main()
// ---------------------------------------------------------------------------
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Default to /dev/ttyACM0 at 921600 unless overridden
  // std::string port = (argc > 1) ? argv[1] : "/dev/ttyACM0";
  // unsigned int baud = (argc > 2) ? std::atoi(argv[2]) : 921600;
  std::string port = "/dev/ttyACM0";
  unsigned int baud = 921600;

  try {
    auto node = std::make_shared<Stm32ControlNode>(port, baud);
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("mr2_drive_motor_interface"),
                 "Failed to initialize Stm32ControlNode: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}
