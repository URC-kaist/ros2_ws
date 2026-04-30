#include "mr2_rover_control/twist_to_commands_controller.hpp"

#include <algorithm>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"

namespace mr2_rover_control {

controller_interface::CallbackReturn TwistToCommandsController::on_init() {
  auto_declare<std::vector<std::string>>("wheel_joints", {});
  auto_declare<std::vector<std::string>>("steering_joints", {});
  auto_declare<double>("wheel_base", 0.95386);
  auto_declare<double>("track_width", 0.6504);
  auto_declare<double>("wheel_radius", 0.17);
  auto_declare<double>("max_wheel_linear_speed", 0.4);
  auto_declare<double>("max_steer", 2.35619); // +/- 135 degrees
  auto_declare<double>("twist_timeout", 0.5);
  auto_declare<double>("odom_publish_rate_hz", 30.0);
  auto_declare<double>("rate_limit_linear_x", 1.5); // m/s^2
  auto_declare<double>("rate_limit_linear_y", 1.5); // m/s^2
  auto_declare<double>("rate_limit_angular_z",
                       1.0); // rad/s^2 (tighter yaw slew)
  auto_declare<double>("solver_error_alpha", 0.1); // EMA factor for steering cmd
  auto_declare<double>("solver_gain_k", 4.0);      // EMA weight sharpness
  auto_declare<double>("solver_max_steer_deg", 90.0); // solver steering clamp
  auto_declare<double>("solver_speed_no_atten_deg", 15.0); // speed=1 below this
  auto_declare<double>("solver_cmd_deadzone_lin", 1e-3); // m/s
  auto_declare<double>("solver_cmd_deadzone_ang", 1e-3); // rad/s
  auto_declare<double>("solver_vel_eps", 1e-4);          // m/s
  auto_declare<double>("steering_error_ratio_deg",
                       40.0); // drive scale->0 around 30 deg
  auto_declare<bool>("mission_smooth", true);
  auto_declare<std::vector<bool>>("odom_wheel_drive_enabled", {});
  auto_declare<std::vector<bool>>("odom_wheel_steer_enabled", {});
  auto_declare<std::string>("wheel_odom_topic", "/wheel_encoder/odometry");
  auto_declare<std::string>("odom_frame_id", "odom");
  auto_declare<std::string>("base_frame_id", "base_link");
  auto_declare<std::string>("cmd_vel_topic_nominal", "/base/cmd_vel");
  auto_declare<std::string>("cmd_vel_topic_running", "/cmd_vel");
  auto_declare<std::string>("mission_status_topic", "/mission_status");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
TwistToCommandsController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // Pull parameter defaults so interfaces are declared even before
  // on_configure.
  auto wheels = get_node()
                    ->get_parameter("wheel_joints")
                    .as_string_array(); // size may be 0 pre-configure
  auto steer = get_node()->get_parameter("steering_joints").as_string_array();
  for (const auto &joint : wheels) {
    conf.names.push_back(joint + "/velocity");
  }
  for (const auto &joint : steer) {
    conf.names.push_back(joint + "/position");
  }
  return conf;
}

controller_interface::InterfaceConfiguration
TwistToCommandsController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  auto wheels = get_node()->get_parameter("wheel_joints").as_string_array();
  auto steer = get_node()->get_parameter("steering_joints").as_string_array();
  for (const auto &joint : wheels) {
    conf.names.push_back(joint + "/velocity");
  }
  for (const auto &joint : steer) {
    conf.names.push_back(joint + "/position");
  }
  return conf;
}

controller_interface::CallbackReturn
TwistToCommandsController::on_configure(const rclcpp_lifecycle::State &) {
  wheel_joints_ = get_node()->get_parameter("wheel_joints").as_string_array();
  steering_joints_ =
      get_node()->get_parameter("steering_joints").as_string_array();
  if (wheel_joints_.size() != 4 || steering_joints_.size() != 4) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected 4 wheel joints and 4 steering joints");
    return controller_interface::CallbackReturn::ERROR;
  }
  wheel_base_ = get_node()->get_parameter("wheel_base").as_double();
  track_width_ = get_node()->get_parameter("track_width").as_double();
  wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
  max_wheel_linear_speed_ =
      get_node()->get_parameter("max_wheel_linear_speed").as_double();
  max_wheel_linear_speed_ = std::max(0.0, max_wheel_linear_speed_);
  max_steer_ = get_node()->get_parameter("max_steer").as_double();
  timeout_ = get_node()->get_parameter("twist_timeout").as_double();
  odom_publish_rate_ =
      get_node()->get_parameter("odom_publish_rate_hz").as_double();
  rate_limit_vx_ = get_node()->get_parameter("rate_limit_linear_x").as_double();
  rate_limit_vy_ = get_node()->get_parameter("rate_limit_linear_y").as_double();
  rate_limit_wz_ =
      get_node()->get_parameter("rate_limit_angular_z").as_double();
  const double steer_err_ratio_deg =
      get_node()->get_parameter("steering_error_ratio_deg").as_double();
  steering_error_ratio_rad_ = (steer_err_ratio_deg > 0.0)
                                  ? steer_err_ratio_deg * M_PI / 180.0
                                  : max_steer_;
  mission_smooth_ = get_node()->get_parameter("mission_smooth").as_bool();
  solver_error_alpha_ =
      get_node()->get_parameter("solver_error_alpha").as_double();
  solver_error_alpha_ = std::clamp(solver_error_alpha_, 0.0, 1.0);
  solver_gain_k_ = get_node()->get_parameter("solver_gain_k").as_double();
  solver_gain_k_ = std::max(0.0, solver_gain_k_);
  const double solver_max_steer_deg =
      get_node()->get_parameter("solver_max_steer_deg").as_double();
  const double solver_speed_no_atten_deg =
      get_node()->get_parameter("solver_speed_no_atten_deg").as_double();
  const double solver_cmd_deadzone_lin =
      get_node()->get_parameter("solver_cmd_deadzone_lin").as_double();
  const double solver_cmd_deadzone_ang =
      get_node()->get_parameter("solver_cmd_deadzone_ang").as_double();
  const double solver_vel_eps =
      get_node()->get_parameter("solver_vel_eps").as_double();

  solver_cfg_.track_width = track_width_;
  solver_cfg_.wheel_base = wheel_base_;
  solver_cfg_.error_alpha = solver_error_alpha_;
  solver_cfg_.gain_k = solver_gain_k_;
  const double solver_max_steer_rad =
      std::abs(solver_max_steer_deg) * M_PI / 180.0;
  solver_cfg_.max_steer_angle =
      (max_steer_ > 0.0) ? std::min(max_steer_, solver_max_steer_rad)
                         : solver_max_steer_rad;
  solver_cfg_.steer_speed_no_atten =
      std::max(0.0, solver_speed_no_atten_deg) * M_PI / 180.0;
  solver_cfg_.steer_speed_full_atten =
      std::max(0.0, steering_error_ratio_rad_);
  solver_cfg_.cmd_deadzone_lin = std::max(0.0, solver_cmd_deadzone_lin);
  solver_cfg_.cmd_deadzone_ang = std::max(0.0, solver_cmd_deadzone_ang);
  solver_cfg_.vel_eps = std::max(0.0, solver_vel_eps);
  solver_.emplace(solver_cfg_);

  const auto odom_wheel_drive_enabled =
      get_node()->get_parameter("odom_wheel_drive_enabled").as_bool_array();
  if (odom_wheel_drive_enabled.empty()) {
    odom_wheel_drive_enabled_.fill(true);
  } else if (odom_wheel_drive_enabled.size() != 4) {
    odom_wheel_drive_enabled_.fill(true);
    RCLCPP_WARN(get_node()->get_logger(),
                "odom_wheel_drive_enabled must have 4 entries (FL, FR, RL, RR); "
                "falling back to all true");
  } else {
    for (size_t i = 0; i < odom_wheel_drive_enabled_.size(); ++i) {
      odom_wheel_drive_enabled_[i] = odom_wheel_drive_enabled[i];
    }
  }

  const auto odom_wheel_steer_enabled =
      get_node()->get_parameter("odom_wheel_steer_enabled").as_bool_array();
  if (odom_wheel_steer_enabled.empty()) {
    odom_wheel_steer_enabled_.fill(true);
  } else if (odom_wheel_steer_enabled.size() != 4) {
    odom_wheel_steer_enabled_.fill(true);
    RCLCPP_WARN(get_node()->get_logger(),
                "odom_wheel_steer_enabled must have 4 entries (FL, FR, RL, RR); "
                "falling back to all true");
  } else {
    for (size_t i = 0; i < odom_wheel_steer_enabled_.size(); ++i) {
      odom_wheel_steer_enabled_[i] = odom_wheel_steer_enabled[i];
    }
  }

  odom_frame_id_ = get_node()->get_parameter("odom_frame_id").as_string();
  base_frame_id_ = get_node()->get_parameter("base_frame_id").as_string();
  const auto cmd_topic_nominal =
      get_node()->get_parameter("cmd_vel_topic_nominal").as_string();
  const auto cmd_topic_running =
      get_node()->get_parameter("cmd_vel_topic_running").as_string();
  const auto mission_status_topic =
      get_node()->get_parameter("mission_status_topic").as_string();
  const auto odom_topic =
      get_node()->get_parameter("wheel_odom_topic").as_string();

  sub_twist_nominal_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic_nominal, rclcpp::SystemDefaultsQoS(),
      std::bind(&TwistToCommandsController::twistNominalCb, this,
                std::placeholders::_1));

  sub_twist_running_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic_running, rclcpp::SystemDefaultsQoS(),
      std::bind(&TwistToCommandsController::twistRunningCb, this,
                std::placeholders::_1));

  sub_mission_status_ =
      get_node()->create_subscription<mr2_action_interface::msg::MissionStatus>(
          mission_status_topic, 10,
          std::bind(&TwistToCommandsController::missionStatusCb, this,
                    std::placeholders::_1));

  odom_pub_ =
      get_node()->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);

  last_twist_.linear.x = 0.0;
  last_twist_.angular.z = 0.0;
  limited_twist_.linear.x = 0.0;
  limited_twist_.linear.y = 0.0;
  limited_twist_.angular.z = 0.0;
  last_twist_time_ = get_node()->now();
  last_odom_pub_time_ =
      rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type());
  mission_state_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
TwistToCommandsController::on_activate(const rclcpp_lifecycle::State &) {
  limited_twist_.linear.x = 0.0;
  limited_twist_.linear.y = 0.0;
  limited_twist_.angular.z = 0.0;
  publishZeros();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
TwistToCommandsController::on_deactivate(const rclcpp_lifecycle::State &) {
  limited_twist_.linear.x = 0.0;
  limited_twist_.linear.y = 0.0;
  limited_twist_.angular.z = 0.0;
  publishZeros();
  return controller_interface::CallbackReturn::SUCCESS;
}

void TwistToCommandsController::twistNominalCb(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  const bool running =
      (mission_state_.has_value() && mission_state_.value() == kMissionStateRunning);
  if (running) {
    return;
  }
  last_twist_ = *msg;
  last_twist_time_ = get_node()->now();
}

void TwistToCommandsController::twistRunningCb(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  const bool running =
      (mission_state_.has_value() && mission_state_.value() == kMissionStateRunning);
  if (!running) {
    return;
  }
  last_twist_ = *msg;
  last_twist_time_ = get_node()->now();
}

void TwistToCommandsController::missionStatusCb(
    const mr2_action_interface::msg::MissionStatus::SharedPtr msg) {
  mission_state_ = msg->state;
}

void TwistToCommandsController::publishZeros() {
  for (size_t i = 0; i < 4; ++i) {
    command_interfaces_[i].set_value(0.0);
    command_interfaces_[i + 4].set_value(0.0);
  }
}

bool TwistToCommandsController::fillWheelStates(
    std::array<double, 4> &wheel_ang_vel,
    std::array<double, 4> &steer_angle) const {
  if (state_interfaces_.size() < 8) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                         2000, "State interfaces not available yet");
    return false;
  }

  // Order follows state_interface_configuration: wheel velocities then steering
  // positions
  for (size_t i = 0; i < 4; ++i) {
    const double vel = state_interfaces_[i].get_value();
    const double steer = state_interfaces_[i + 4].get_value();
    wheel_ang_vel[i] = std::isfinite(vel) ? vel : 0.0;
    steer_angle[i] = std::isfinite(steer) ? steer : 0.0;
    if (!std::isfinite(vel)) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                           2000,
                           "Wheel joint %s velocity is NaN/inf; treating as 0",
                           wheel_joints_[i].c_str());
    }
    if (!std::isfinite(steer)) {
      RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "Steering joint %s position is NaN/inf; treating as 0",
          steering_joints_[i].c_str());
    }
  }
  return true;
}

std::array<std::array<double, 2>, 4>
TwistToCommandsController::wheelPositions() const {
  const double hx = wheel_base_ * 0.5;
  const double hy = track_width_ * 0.5;
  // Order: FL, FR, RL, RR
  return {{{+hx, +hy}, {+hx, -hy}, {-hx, +hy}, {-hx, -hy}}};
}

void TwistToCommandsController::publishOdom(
    const std::array<double, 4> &wheel_ang_vel,
    const std::array<double, 4> &steer_angle, const rclcpp::Time &stamp) {
  const auto wheel_pos = wheelPositions();

  // Average-based estimation: project each wheel's measured speed into linear
  // body components, then estimate yaw from residual per-wheel lever arm.
  double vx_sum = 0.0;
  double vy_sum = 0.0;
  size_t lin_count = 0;

  for (size_t i = 0; i < 4; ++i) {
    if (!odom_wheel_drive_enabled_[i] || !odom_wheel_steer_enabled_[i]) {
      continue;
    }
    const double theta = steer_angle[i];
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    const double v = wheel_ang_vel[i] * wheel_radius_;
    vx_sum += v * c;
    vy_sum += v * s;
    ++lin_count;
  }

  const double vx = lin_count ? vx_sum / static_cast<double>(lin_count) : 0.0;
  const double vy = lin_count ? vy_sum / static_cast<double>(lin_count) : 0.0;

  double wz_sum = 0.0;
  size_t wz_count = 0;
  for (size_t i = 0; i < 4; ++i) {
    if (!odom_wheel_drive_enabled_[i] || !odom_wheel_steer_enabled_[i]) {
      continue;
    }
    const double theta = steer_angle[i];
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    const double v = wheel_ang_vel[i] * wheel_radius_;
    const double lever = -c * wheel_pos[i][1] + s * wheel_pos[i][0];
    if (std::abs(lever) < 1e-4) {
      continue; // avoid divide-by-near-zero when lever arm vanishes
    }
    const double wz_i = (v - c * vx - s * vy) / lever;
    wz_sum += wz_i;
    ++wz_count;
  }
  const double wz = wz_count ? wz_sum / static_cast<double>(wz_count) : 0.0;

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame_id_;
  odom.child_frame_id = base_frame_id_;

  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.w = 1.0;
  constexpr double big_cov = 1e6;
  odom.pose.covariance = {big_cov, 0, 0, 0, 0, 0, 0, big_cov, 0, 0, 0, 0, 0, 0,
                          big_cov, 0, 0, 0, 0, 0, 0, big_cov, 0, 0, 0, 0, 0, 0,
                          big_cov, 0, 0, 0, 0, 0, 0, big_cov};

  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = wz;
  constexpr double twist_cov = 0.05;
  odom.twist.covariance = {
      twist_cov, 0, 0, 0, 0, 0, 0, twist_cov, 0, 0, 0, 0, 0, 0,
      twist_cov, 0, 0, 0, 0, 0, 0, twist_cov, 0, 0, 0, 0, 0, 0,
      twist_cov, 0, 0, 0, 0, 0, 0, twist_cov};

  odom_pub_->publish(odom);
}

double TwistToCommandsController::applyRateLimit(double target, double prev,
                                                 double rate_limit,
                                                 double dt) const {
  if (rate_limit <= 0.0 || dt <= 0.0) {
    return target;
  }
  const double delta = target - prev;
  const double max_delta = rate_limit * dt;
  return prev + std::clamp(delta, -max_delta, max_delta);
}

controller_interface::return_type
TwistToCommandsController::update(const rclcpp::Time &,
                                  const rclcpp::Duration &period) {
  const auto now = get_node()->now();
  const bool timed_out = (now - last_twist_time_).seconds() > timeout_;

  const double dt = std::max(0.0, period.seconds());

  const double target_vx = timed_out ? 0.0 : last_twist_.linear.x;
  const double target_vy = timed_out ? 0.0 : last_twist_.linear.y;
  const double target_wz = timed_out ? 0.0 : last_twist_.angular.z;

  limited_twist_.linear.x =
      applyRateLimit(target_vx, limited_twist_.linear.x, rate_limit_vx_, dt);
  limited_twist_.linear.y =
      applyRateLimit(target_vy, limited_twist_.linear.y, rate_limit_vy_, dt);
  limited_twist_.angular.z =
      applyRateLimit(target_wz, limited_twist_.angular.z, rate_limit_wz_, dt);

  const double vx = limited_twist_.linear.x;
  const double vy = limited_twist_.linear.y;
  const double wz = limited_twist_.angular.z;

  const bool running =
      (mission_state_.has_value() && mission_state_.value() == kMissionStateRunning);
  const bool use_solver = running && mission_smooth_;

  if (!use_solver) {
    if (wheel_radius_ <= 0.0) {
      RCLCPP_ERROR_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "wheel_radius must be > 0 (got %.3f); zeroing commands", wheel_radius_);
      publishZeros();
    } else if (state_interfaces_.size() < 8) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                           2000,
                           "State interfaces not available yet; zeroing outputs");
      publishZeros();
    } else {
      std::array<double, 4> current_steer{};
      bool steer_feedback_valid = true;
      for (size_t i = 0; i < 4; ++i) {
        const double val = state_interfaces_[i + 4].get_value();
        if (std::isfinite(val)) {
          current_steer[i] = val;
        } else {
          steer_feedback_valid = false;
          RCLCPP_WARN_THROTTLE(
              get_node()->get_logger(), *get_node()->get_clock(), 2000,
              "Steering joint %s position missing/invalid; zeroing wheel commands",
              steering_joints_[i].c_str());
        }
      }

      if (!steer_feedback_valid) {
        publishZeros();
      } else {
        const auto wheels = wheelPositions();
        std::array<double, 4> speed{};
        std::array<double, 4> steer{};

        auto ang_distance = [](double a, double b) {
          // Smallest absolute difference between two angles (wrap at 2*pi)
          const double diff = std::remainder(a - b, 2.0 * M_PI);
          return std::abs(diff);
        };

        for (size_t i = 0; i < wheels.size(); ++i) {
          const auto &w = wheels[i];
          const double vx_i = vx - wz * w[1];
          const double vy_i = vy + wz * w[0];
          const double v_lin = std::hypot(vx_i, vy_i);

          constexpr double kVelStop = 1e-4;
          if (v_lin < kVelStop) { // Preserve steer even if the rover is stop.
            speed[i] = 0.0;
            continue;
          }

          const double v_lin_limited = (max_wheel_linear_speed_ > 0.0)
                                           ? std::min(v_lin, max_wheel_linear_speed_)
                                           : v_lin;

          double ang = std::atan2(vy_i, vx_i);
          double w_ang = v_lin_limited / wheel_radius_;

          // Two equivalent steering solutions: (ang, w_ang) or (ang +/- pi, -w_ang).
          double ang_alt = ang;
          double w_ang_alt = w_ang;
          if (ang > 0) {
            ang_alt = ang - M_PI;
          } else {
            ang_alt = ang + M_PI;
          }
          w_ang_alt *= -1.0;

          const bool reachable1 = std::abs(ang) <= max_steer_;
          const bool reachable2 = std::abs(ang_alt) <= max_steer_;

          // Choose the solution closest to current steering angle for that wheel.
          if (reachable1 && !reachable2) {
            ang = std::clamp(ang, -max_steer_, max_steer_);
          } else if (!reachable1 && reachable2) {
            ang = std::clamp(ang_alt, -max_steer_, max_steer_);
            w_ang = w_ang_alt;
          } else if (!reachable1 && !reachable2) {
            const double unclamped = ang;
            ang = std::clamp(unclamped, -max_steer_, max_steer_);
            w_ang = 0.0; // cannot realize kinematics at this angle
            RCLCPP_WARN_THROTTLE(
                get_node()->get_logger(), *get_node()->get_clock(), 2000,
                "Steering solution %.2f rad exceeds limit (±%.2f); "
                "zeroing wheel speed for wheel %zu",
                unclamped, max_steer_, i);
          } else { // both reachable
            const double cand1 = std::clamp(ang, -max_steer_, max_steer_);
            const double cand2 = std::clamp(ang_alt, -max_steer_, max_steer_);
            if (ang_distance(cand2, current_steer[i]) <
                ang_distance(cand1, current_steer[i])) {
              ang = cand2;
              w_ang = w_ang_alt;
            } else {
              ang = cand1;
            }
          }

          steer[i] = std::clamp(ang, -max_steer_, max_steer_);
          speed[i] = w_ang;
        }

        // Apply a global speed scale based on the worst steering error.
        double max_steer_err = 0.0;
        for (size_t i = 0; i < wheels.size(); ++i) {
          const double steer_err = ang_distance(steer[i], current_steer[i]);
          if (steer_err > max_steer_err) {
            max_steer_err = steer_err;
          }
        }
        if (max_steer_ > 0.0) {
          const double denom = steering_error_ratio_rad_ > 0.0
                                   ? steering_error_ratio_rad_
                                   : max_steer_;
          const double norm = denom > 0.0 ? (max_steer_err / denom) : 0.0;
          constexpr double power = 2.0; // square the normalized error
          double scale = 1.0 - std::pow(norm, power);
          scale = std::clamp(scale, 0.0, 1.0);
          for (double &v : speed) {
            v *= scale;
          }
        }

        for (size_t i = 0; i < 4; ++i) {
          command_interfaces_[i].set_value(speed[i]);
          command_interfaces_[i + 4].set_value(steer[i]);
        }
      }
    }

    // Publish odom at configured rate using available state interfaces
    if (odom_pub_ && odom_publish_rate_ > 0.0) {
      const double dt = (now - last_odom_pub_time_).seconds();
      if (dt >= (1.0 / odom_publish_rate_)) {
        std::array<double, 4> wheel_ang_vel{};
        std::array<double, 4> steer_angle{};
        if (fillWheelStates(wheel_ang_vel, steer_angle)) {
          publishOdom(wheel_ang_vel, steer_angle, now);
          last_odom_pub_time_ = now;
        }
      }
    }

    if (timed_out) {
      // Ensure outputs stay zeroed when commands stale
      publishZeros();
    }

    return controller_interface::return_type::OK;
  }

  if (!solver_) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                          2000,
                          "FourWheelSteeringSolver is not configured; zeroing");
    publishZeros();
    return controller_interface::return_type::OK;
  }

  const double effective_error_alpha = solver_error_alpha_;
  solver_->setErrorAlpha(effective_error_alpha);

  if (state_interfaces_.size() < 8) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                         2000,
                         "State interfaces not available yet; zeroing outputs");
    publishZeros();
    return controller_interface::return_type::OK;
  }

  // Current steering joint positions (state_interfaces_: wheel vels first,
  // then steering positions)
  std::array<double, 4> current_steer{};
  bool steer_feedback_valid = true;
  for (size_t i = 0; i < 4; ++i) {
    const double val = state_interfaces_[i + 4].get_value();
    if (std::isfinite(val)) {
      current_steer[i] = val;
    } else {
      steer_feedback_valid = false;
      RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "Steering joint %s position missing/invalid; zeroing commands",
          steering_joints_[i].c_str());
    }
  }

  if (!steer_feedback_valid) {
    publishZeros();
    return controller_interface::return_type::OK;
  }

  FourWheelSteeringSolver::Cmd cmd{vx, vy, wz};
  const auto targets = solver_->solve(cmd, current_steer);

  if (wheel_radius_ <= 0.0) {
    RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "wheel_radius must be > 0 (got %.3f); zeroing commands", wheel_radius_);
    publishZeros();
  } else {
    for (size_t i = 0; i < 4; ++i) {
      double wheel_linear = targets[i].speed;
      if (max_wheel_linear_speed_ > 0.0) {
        wheel_linear = std::clamp(
            wheel_linear, -max_wheel_linear_speed_, max_wheel_linear_speed_);
      }
      const double wheel_speed = wheel_linear / wheel_radius_;
      double steer_cmd = targets[i].angle;
      if (max_steer_ > 0.0) {
        steer_cmd = std::clamp(steer_cmd, -max_steer_, max_steer_);
      }
      command_interfaces_[i].set_value(wheel_speed);
      command_interfaces_[i + 4].set_value(steer_cmd);
    }
  }

  // Publish odom at configured rate using available state interfaces
  if (odom_pub_ && odom_publish_rate_ > 0.0) {
    const double dt = (now - last_odom_pub_time_).seconds();
    if (dt >= (1.0 / odom_publish_rate_)) {
      std::array<double, 4> wheel_ang_vel{};
      std::array<double, 4> steer_angle{};
      if (fillWheelStates(wheel_ang_vel, steer_angle)) {
        publishOdom(wheel_ang_vel, steer_angle, now);
        last_odom_pub_time_ = now;
      }
    }
  }

  if (timed_out) {
    // Ensure outputs stay zeroed when commands stale
    publishZeros();
  }

  return controller_interface::return_type::OK;
}

} // namespace mr2_rover_control

PLUGINLIB_EXPORT_CLASS(mr2_rover_control::TwistToCommandsController,
                       controller_interface::ControllerInterface)
