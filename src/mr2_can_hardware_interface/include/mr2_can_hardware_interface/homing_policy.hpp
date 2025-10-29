#pragma once

#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>

#include <string>
#include <unordered_map>

namespace mr2_can_hardware_interface {

class HomingPolicy {
public:
  struct JointHandle {
    std::string name;
    double *command{nullptr};
    double *state{nullptr};
    double *velocity{nullptr};
    double *offset{nullptr};
  };

  using NamedStateMap = std::unordered_map<std::string, const double *>;
  using ParamMap = std::unordered_map<std::string, std::string>;

  virtual ~HomingPolicy() = default;

  virtual void configure(const rclcpp::Node::SharedPtr &node,
                         const JointHandle &joint,
                         const NamedStateMap &named_states,
                         const ParamMap &params) = 0;

  virtual void begin(const rclcpp::Time &now) = 0;
  virtual void update(const rclcpp::Time &now,
                      const rclcpp::Duration &period) = 0;

  virtual bool is_finished() const = 0;
  virtual bool has_error() const = 0;
  virtual std::string error_message() const = 0;

  virtual void finalize(const rclcpp::Time &now) = 0;
  virtual void reset() = 0;
};

} // namespace mr2_can_hardware_interface
