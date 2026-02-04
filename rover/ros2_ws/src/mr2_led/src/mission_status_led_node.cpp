#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "mr2_action_interface/msg/mission_status.hpp"
#include "mr2_led/srv/set_led_mode.hpp"
#include "rcl/time.h"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MissionStatusLedNode : public rclcpp::Node {
public:
  MissionStatusLedNode() : rclcpp::Node("mr2_mission_status_led") {
    mission_status_topic_ = declare_parameter<std::string>(
        "mission_status_topic", "/mission_status");
    led_service_name_ = declare_parameter<std::string>(
        "led_service_name", "set_led_mode");
    const double timeout_sec = declare_parameter<double>(
        "status_timeout_sec", 1.0);
    status_timeout_ = rclcpp::Duration::from_seconds(timeout_sec);

    client_ = create_client<mr2_led::srv::SetLedMode>(led_service_name_);
    sub_ = create_subscription<mr2_action_interface::msg::MissionStatus>(
        mission_status_topic_, rclcpp::QoS(10),
        std::bind(&MissionStatusLedNode::on_status_, this,
                  std::placeholders::_1));

    timer_ = create_wall_timer(200ms,
                               std::bind(&MissionStatusLedNode::on_timer_,
                                         this));

    RCLCPP_INFO(get_logger(),
                "mr2_mission_status_led listening on %s, service %s",
                mission_status_topic_.c_str(), led_service_name_.c_str());
  }

private:
  void on_status_(
      const mr2_action_interface::msg::MissionStatus::SharedPtr msg) {
    last_status_time_ = now();
    have_status_ = true;

    uint8_t desired = mr2_led::srv::SetLedMode::Request::MODE_OFF;
    if (msg->arrival) {
      desired = mr2_led::srv::SetLedMode::Request::MODE_SUCCESS;
    } else {
      switch (msg->state) {
        case 0:  // IDLE
        case 3:  // COMPLETED
          desired = mr2_led::srv::SetLedMode::Request::MODE_OFF;
          break;
        case 1:  // RUNNING
          desired = mr2_led::srv::SetLedMode::Request::MODE_AUTONOMOUS;
          break;
        case 2:  // PAUSED
          desired = mr2_led::srv::SetLedMode::Request::MODE_MANUAL;
          break;
        default:
          desired = mr2_led::srv::SetLedMode::Request::MODE_OFF;
          break;
      }
    }

    send_mode_if_changed_(desired);
  }

  void on_timer_() {
    const auto now_time = now();
    if (!have_status_ || (now_time - last_status_time_) > status_timeout_) {
      send_mode_if_changed_(mr2_led::srv::SetLedMode::Request::MODE_OFF);
    }
  }

  void send_mode_if_changed_(uint8_t mode) {
    if (last_mode_sent_ && *last_mode_sent_ == mode) {
      return;
    }
    if (!client_->wait_for_service(0s)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "set_led_mode service not available");
      return;
    }

    auto request = std::make_shared<mr2_led::srv::SetLedMode::Request>();
    request->mode = mode;
    last_mode_sent_ = mode;
    client_->async_send_request(
        request,
        [this, mode](rclcpp::Client<mr2_led::srv::SetLedMode>::SharedFuture
                         future) {
          const auto response = future.get();
          if (!response->success) {
            RCLCPP_WARN(get_logger(), "LED mode %u rejected: %s", mode,
                        response->message.c_str());
          }
        });
  }

  std::string mission_status_topic_;
  std::string led_service_name_;
  rclcpp::Duration status_timeout_{0, 0};
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};
  bool have_status_{false};
  std::optional<uint8_t> last_mode_sent_;

  rclcpp::Subscription<mr2_action_interface::msg::MissionStatus>::SharedPtr sub_;
  rclcpp::Client<mr2_led::srv::SetLedMode>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionStatusLedNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
