#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// Dynamixel SDK
#include "dynamixel_sdk/dynamixel_sdk.h"

//------------------ Control Table Addresses (Protocol 1.0, e.g. MX series)
//------------------
#define ADDR_MX_TORQUE_ENABLE 24
#define ADDR_MX_GOAL_POSITION 30
#define ADDR_MX_PRESENT_POSITION 36

//------------------ Default Setting ------------------
#define DXL_BAUDRATE 1000000          // 1 Mbps
#define DXL_DEVICENAME "/dev/ttyUSB0" // or e.g. "COM3" on Windows
#define DXL_PROTOCOL_VERSION 1.0

#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0

class DynamixelJointController : public rclcpp::Node {
public:
  DynamixelJointController() : Node("dynamixel_joint_controller") {
    // Declare parameters (optionally, could be set via launch or YAML)
    this->declare_parameter<std::string>("device_name", DXL_DEVICENAME);
    this->declare_parameter<int>("baud_rate", DXL_BAUDRATE);

    // Get parameter values
    device_name_ = this->get_parameter("device_name").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();

    // The set of Dynamixel IDs we want to control.
    dxl_ids_ = {3, 2, 4, 1};

    // Initialize the Dynamixel drivers
    initDynamixels();

    // Subscribe to JointState commands (in ticks)
    sub_command_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/steering_node/joint_states", // or another topic name
        10,
        std::bind(&DynamixelJointController::commandCallback, this,
                  std::placeholders::_1));

    // Publisher for the feedback (present position) as JointState
    pub_feedback_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states_feedback", 10);

    // Create a timer to update (read/write) the servos ~10 Hz
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&DynamixelJointController::updateCallback, this));
  }

  ~DynamixelJointController() {
    // Disable torque on all motors and close port on shutdown
    disableAllMotors();
    closePort();
  }

private:
  //------------------------------------------------------------------------------
  // 1) Setup the Dynamixel environment: open port, set baud rate, enable torque
  //------------------------------------------------------------------------------
  void initDynamixels() {
    // Create PortHandler
    port_handler_ =
        dynamixel::PortHandler::getPortHandler(device_name_.c_str());
    // Create PacketHandler
    packet_handler_ =
        dynamixel::PacketHandler::getPacketHandler(DXL_PROTOCOL_VERSION);

    if (!port_handler_ || !packet_handler_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create port/packet handler!");
      return;
    }

    // Open port
    if (port_handler_->openPort()) {
      RCLCPP_INFO(this->get_logger(), "Succeeded to open port: %s",
                  device_name_.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open port: %s",
                   device_name_.c_str());
      return;
    }

    // Set baud rate
    if (port_handler_->setBaudRate(baud_rate_)) {
      RCLCPP_INFO(this->get_logger(), "Succeeded to set baud rate: %d",
                  baud_rate_);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to set baud rate: %d",
                   baud_rate_);
      return;
    }

    // Enable torque on all the motors
    enableAllMotors();
  }

  //------------------------------------------------------------------------------
  // 2) Enable torque on all configured motors
  //------------------------------------------------------------------------------
  void enableAllMotors() {
    for (int id : dxl_ids_) {
      uint8_t dxl_error = 0;
      int dxl_comm_result = packet_handler_->write1ByteTxRx(
          port_handler_, id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

      if (dxl_comm_result != COMM_SUCCESS) {
        packet_handler_->getTxRxResult(dxl_comm_result);
        RCLCPP_ERROR(this->get_logger(), "Failed to enable torque for ID: %d",
                     id);
      } else if (dxl_error != 0) {
        packet_handler_->getRxPacketError(dxl_error);
        RCLCPP_ERROR(this->get_logger(),
                     "Torque enable returned error for ID: %d", id);
      } else {
        RCLCPP_INFO(this->get_logger(), "Torque enabled for ID: %d", id);
      }
    }
  }

  //------------------------------------------------------------------------------
  // 3) Disable torque on all configured motors
  //------------------------------------------------------------------------------
  void disableAllMotors() {
    for (int id : dxl_ids_) {
      uint8_t dxl_error = 0;
      int dxl_comm_result = packet_handler_->write1ByteTxRx(
          port_handler_, id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

      if (dxl_comm_result != COMM_SUCCESS) {
        packet_handler_->getTxRxResult(dxl_comm_result);
      } else if (dxl_error != 0) {
        packet_handler_->getRxPacketError(dxl_error);
      } else {
        RCLCPP_INFO(this->get_logger(), "Torque disabled for ID: %d", id);
      }
    }
  }

  //------------------------------------------------------------------------------
  // 4) Close the serial port
  //------------------------------------------------------------------------------
  void closePort() {
    if (port_handler_) {
      port_handler_->closePort();
      RCLCPP_INFO(this->get_logger(), "Port closed.");
    }
  }

  //------------------------------------------------------------------------------
  // 5) Callback when a new JointState command arrives
  //   We'll store these positions (assuming 4 motors) in a local array
  //------------------------------------------------------------------------------
  void commandCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // We assume the message has 4 positions in the same order as dxl_ids_
    // In a more robust application, you'd check msg->name[] or ensure the order
    // is correct.
    if (msg->position.size() < dxl_ids_.size()) {
      RCLCPP_WARN(this->get_logger(),
                  "Received JointState with insufficient positions.");
      return;
    }

    for (size_t i = 0; i < dxl_ids_.size(); i++) {
      // Convert the position to an integer tick.
      // If you're *already* sending ticks, you can directly cast. If you're
      // sending angles (radians/degs), you'd need to convert (e.g., [0..4095] ~
      // [0..2π]).

      // Offset: 75 degrees, angle range: 0 to 150
      double position = -msg->position[i] * 180.0 / M_PI;
      double offset = 75.0;
      double multiplier = 1023.0 / 150.0;
      int command = std::round(position + offset) * multiplier;
      goal_positions_[i] = static_cast<uint16_t>(std::clamp(command, 0, 4095));
    }
  }

  //------------------------------------------------------------------------------
  // 6) The timer callback that runs at a fixed rate
  //   - Writes the stored goal positions to the motors
  //   - Reads back the present positions
  //   - Publishes them as a JointState
  //------------------------------------------------------------------------------
  void updateCallback() {
    // Skip if no port or packet handler
    if (!port_handler_ || !packet_handler_)
      return;

    // 1) --- SYNC WRITE SETUP ---
    // Create a group sync write instance
    // (For 2-byte data length, e.g. GOAL_POSITION for MX on Protocol 1.0)
    dynamixel::GroupSyncWrite groupSyncWrite(
        port_handler_, packet_handler_, ADDR_MX_GOAL_POSITION,
        2 /* size of goal position in bytes */
    );

    // 2) --- ADD PARAMETERS ---
    for (size_t i = 0; i < dxl_ids_.size(); i++) {
      // Convert goal_positions_[i] (uint16_t) to 2 bytes
      uint8_t param_goal_position[2];
      param_goal_position[0] = DXL_LOBYTE(goal_positions_[i]);
      param_goal_position[1] = DXL_HIBYTE(goal_positions_[i]);

      // Add param for this servo ID
      bool add_param_success =
          groupSyncWrite.addParam(dxl_ids_[i], param_goal_position);
      if (!add_param_success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to add param for ID:%d",
                     dxl_ids_[i]);
      }
    }

    // 3) --- TX PACKET (ACTUAL WRITE) ---
    int dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
      packet_handler_->getTxRxResult(dxl_comm_result);
      RCLCPP_ERROR(this->get_logger(), "Sync Write Tx Packet failed");
    }

    // 4) --- CLEAR PARAMS AFTERWARDS ---
    groupSyncWrite.clearParam();

    // 5) --- INDIVIDUAL READS (Protocol 1.0) ---
    //   (You still need to read each motor one-by-one or switch to Bulk Read if
    //   you'd like.)

    // sensor_msgs::msg::JointState feedback_msg;
    // feedback_msg.header.stamp = now();
    // feedback_msg.position.resize(dxl_ids_.size());
    //
    // for (size_t i = 0; i < dxl_ids_.size(); i++) {
    //   uint16_t present_position = 0;
    //   uint8_t dxl_error = 0;
    //   int read_result = packet_handler_->read2ByteTxRx(
    //       port_handler_, dxl_ids_[i], ADDR_MX_PRESENT_POSITION,
    //       &present_position, &dxl_error);
    //
    //   if (read_result != COMM_SUCCESS) {
    //     packet_handler_->getTxRxResult(read_result);
    //     feedback_msg.position[i] = 0.0;
    //   } else if (dxl_error != 0) {
    //     packet_handler_->getRxPacketError(dxl_error);
    //     feedback_msg.position[i] = 0.0;
    //   } else {
    //     feedback_msg.position[i] = static_cast<double>(present_position);
    //   }
    // }
    //
    // // Publish
    // pub_feedback_->publish(feedback_msg);
  }

  //------------------------------------------------------------------------------
  // Member variables
  //------------------------------------------------------------------------------

  // ROS 2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_command_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_feedback_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Dynamixel SDK
  dynamixel::PortHandler *port_handler_{nullptr};
  dynamixel::PacketHandler *packet_handler_{nullptr};

  // Port & Baud
  std::string device_name_;
  int baud_rate_;

  // IDs of all our Dynamixels
  std::vector<int> dxl_ids_;

  // Goal positions (in ticks) for each Dynamixel
  // Initialize to zero or some safe default
  std::array<uint16_t, 4> goal_positions_ = {0, 0, 0, 0};
};

//------------------------------------------------------------------------------

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DynamixelJointController>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
