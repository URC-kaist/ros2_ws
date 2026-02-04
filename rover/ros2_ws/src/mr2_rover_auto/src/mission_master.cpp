#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/srv/clear_entire_costmap.hpp>

#include <mr2_action_interface/action/gnss_only.hpp>
#include <mr2_action_interface/action/cover_vision.hpp>
#include <mr2_action_interface/msg/mission_list.hpp>
#include <mr2_action_interface/msg/mission_control.hpp>
#include <mr2_action_interface/msg/mission_status.hpp>

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

class MissionMaster : public rclcpp::Node
{
public:
  MissionMaster()
  : Node("mission_master")
  {
    mission_list_topic_ = this->declare_parameter<std::string>("mission_list_topic", "mission_list");
    mission_control_topic_ = this->declare_parameter<std::string>("mission_control_topic", "mission_control");
    status_topic_ = this->declare_parameter<std::string>("status_topic", "mission_status");

    gnss_action_name_ = this->declare_parameter<std::string>("gnss_action_name", "gnss_only");
    cover_action_name_ = this->declare_parameter<std::string>("cover_action_name", "cover_vision");

    clear_global_costmap_service_ = this->declare_parameter<std::string>(
      "clear_global_costmap_service",
      "global_costmap/clear_entirely_global_costmap");
    clear_local_costmap_service_ = this->declare_parameter<std::string>(
      "clear_local_costmap_service",
      "local_costmap/clear_entirely_local_costmap");

    gnss_client_ = rclcpp_action::create_client<GnssOnly>(this, gnss_action_name_);
    cover_client_ = rclcpp_action::create_client<CoverVision>(this, cover_action_name_);

    clear_global_client_ =
      this->create_client<nav2_msgs::srv::ClearEntireCostmap>(clear_global_costmap_service_);
    clear_local_client_ =
      this->create_client<nav2_msgs::srv::ClearEntireCostmap>(clear_local_costmap_service_);

    mission_list_sub_ = this->create_subscription<MissionList>(
      mission_list_topic_, 10,
      std::bind(&MissionMaster::on_mission_list, this, std::placeholders::_1));

    mission_control_sub_ = this->create_subscription<MissionControl>(
      mission_control_topic_, 10,
      std::bind(&MissionMaster::on_mission_control, this, std::placeholders::_1));

    status_pub_ = this->create_publisher<MissionStatus>(status_topic_, 10);
    status_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&MissionMaster::publish_status, this));

    RCLCPP_INFO(get_logger(), "Mission Master ready");
  }

private:
  using GnssOnly = mr2_action_interface::action::GnssOnly;
  using CoverVision = mr2_action_interface::action::CoverVision;
  using MissionList = mr2_action_interface::msg::MissionList;
  using MissionControl = mr2_action_interface::msg::MissionControl;
  using MissionStatus = mr2_action_interface::msg::MissionStatus;
  using MissionSpec = mr2_action_interface::msg::MissionSpec;

  enum MissionType : uint8_t {
    MISSION_UNKNOWN = 0,
    MISSION_GNSS_ONLY = 1,
    MISSION_COVER_VISION = 2
  };

  enum MissionState : uint8_t {
    STATE_IDLE = 0,
    STATE_RUNNING = 1,
    STATE_PAUSED = 2,
    STATE_COMPLETED = 3,
    STATE_FAILED = 4
  };

  enum ControlCommand : uint8_t {
    CMD_NOOP = 0,
    CMD_PAUSE = 1,
    CMD_RESUME = 2,
    CMD_ABORT = 3
  };

  enum class ActiveAction {
    NONE,
    GNSS,
    COVER
  };

  void on_mission_list(const MissionList::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    missions_ = msg->missions;
    current_index_ = 0;
    active_mission_ = MissionSpec{};
    last_detail_.clear();
    abort_requested_ = false;
    pending_restart_ = true;

    if (active_action_ != ActiveAction::NONE) {
      cancel_active_goal_locked();
      return;
    }

    if (pause_requested_ || state_ == STATE_PAUSED) {
      state_ = STATE_PAUSED;
      return;
    }

    pending_restart_ = false;
    start_current_mission_locked();
  }

  void on_mission_control(const MissionControl::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (msg->command == CMD_PAUSE) {
      pause_requested_ = true;
      if (state_ == STATE_RUNNING) {
        state_ = STATE_PAUSED;
        cancel_active_goal_locked();
      }
      if (msg->clear_costmap) {
        clear_costmaps_locked();
      }
      return;
    }

    if (msg->command == CMD_RESUME) {
      if (state_ == STATE_PAUSED) {
        pause_requested_ = false;
        last_detail_.clear();
        start_current_mission_locked();
      }
      return;
    }

    if (msg->command == CMD_ABORT) {
      abort_requested_ = true;
      pending_restart_ = false;
      cancel_active_goal_locked();
      state_ = STATE_FAILED;
      missions_.clear();
      current_index_ = 0;
      last_detail_ = "Mission abort requested";
      return;
    }
  }

  void start_current_mission_locked()
  {
    if (missions_.empty()) {
      state_ = STATE_IDLE;
      active_mission_ = MissionSpec{};
      return;
    }

    if (current_index_ >= missions_.size()) {
      state_ = STATE_COMPLETED;
      active_mission_ = MissionSpec{};
      return;
    }

    if (pause_requested_) {
      state_ = STATE_PAUSED;
      return;
    }

    active_mission_ = missions_[current_index_];
    current_waypoint_index_ = 0;
    total_waypoints_ = 0;
    distance_remaining_ = -1.0f;
    state_ = STATE_RUNNING;

    switch (active_mission_.mission_type) {
      case MISSION_GNSS_ONLY:
        send_gnss_goal_locked(active_mission_);
        break;
      case MISSION_COVER_VISION:
        send_cover_goal_locked(active_mission_);
        break;
      default:
        state_ = STATE_FAILED;
        last_detail_ = "Unknown mission type";
        break;
    }
  }

  void send_gnss_goal_locked(const MissionSpec & spec)
  {
    if (!gnss_client_->wait_for_action_server(std::chrono::seconds(2))) {
      state_ = STATE_FAILED;
      last_detail_ = "GnssOnly action server unavailable";
      return;
    }

    GnssOnly::Goal goal;
    goal.target_latitude = spec.target_latitude;
    goal.target_longitude = spec.target_longitude;

    auto opts = rclcpp_action::Client<GnssOnly>::SendGoalOptions();
    opts.goal_response_callback =
      [this](rclcpp_action::ClientGoalHandle<GnssOnly>::SharedPtr gh)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!gh) {
          state_ = STATE_FAILED;
          last_detail_ = "GnssOnly goal rejected";
          return;
        }
        active_action_ = ActiveAction::GNSS;
        gnss_goal_handle_ = gh;
      };

    opts.feedback_callback =
      [this](rclcpp_action::ClientGoalHandle<GnssOnly>::SharedPtr,
             const std::shared_ptr<const GnssOnly::Feedback> &)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        // GnssOnly feedback currently only reports bt_status.
      };

    opts.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<GnssOnly>::WrappedResult & result)
      {
        bool start_next = false;
        bool restart_now = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          active_action_ = ActiveAction::NONE;
          if (pending_restart_ && !pause_requested_) {
            pending_restart_ = false;
            restart_now = true;
          }
          if (!restart_now) {
            if (pause_requested_ && result.code == rclcpp_action::ResultCode::CANCELED) {
              state_ = STATE_PAUSED;
              return;
            }
            if (abort_requested_) {
              return;
            }
            pause_requested_ = false;

            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
              current_index_++;
              start_next = true;
            } else {
              state_ = STATE_FAILED;
              last_detail_ = "GnssOnly mission failed";
            }
          }
        }
        if (restart_now) {
          std::lock_guard<std::mutex> lock(mutex_);
          start_current_mission_locked();
          return;
        }
        if (start_next) {
          std::lock_guard<std::mutex> lock(mutex_);
          start_current_mission_locked();
        }
      };

    gnss_client_->async_send_goal(goal, opts);
  }

  void send_cover_goal_locked(const MissionSpec & spec)
  {
    if (!cover_client_->wait_for_action_server(std::chrono::seconds(2))) {
      state_ = STATE_FAILED;
      last_detail_ = "CoverVision action server unavailable";
      return;
    }

    CoverVision::Goal goal;
    goal.target_latitude = spec.target_latitude;
    goal.target_longitude = spec.target_longitude;
    goal.target_radius = spec.target_radius;
    goal.detection_method = spec.detection_method;
    goal.object_type = spec.object_type;

    auto opts = rclcpp_action::Client<CoverVision>::SendGoalOptions();
    opts.goal_response_callback =
      [this](rclcpp_action::ClientGoalHandle<CoverVision>::SharedPtr gh)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!gh) {
          state_ = STATE_FAILED;
          last_detail_ = "CoverVision goal rejected";
          return;
        }
        active_action_ = ActiveAction::COVER;
        cover_goal_handle_ = gh;
      };

    opts.feedback_callback =
      [this](rclcpp_action::ClientGoalHandle<CoverVision>::SharedPtr,
             const std::shared_ptr<const CoverVision::Feedback> & fb)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        total_waypoints_ = static_cast<uint32_t>(fb->total_waypoints);
        current_waypoint_index_ = static_cast<uint32_t>(fb->current_waypoint_index);
      };

    opts.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<CoverVision>::WrappedResult & result)
      {
        bool start_next = false;
        bool restart_now = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          active_action_ = ActiveAction::NONE;
          if (pending_restart_ && !pause_requested_) {
            pending_restart_ = false;
            restart_now = true;
          }
          if (!restart_now) {
            if (pause_requested_ && result.code == rclcpp_action::ResultCode::CANCELED) {
              state_ = STATE_PAUSED;
              return;
            }
            if (abort_requested_) {
              return;
            }
            pause_requested_ = false;

            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
              current_index_++;
              start_next = true;
            } else {
              state_ = STATE_FAILED;
              last_detail_ = "CoverVision mission failed";
            }
          }
        }
        if (restart_now) {
          std::lock_guard<std::mutex> lock(mutex_);
          start_current_mission_locked();
          return;
        }
        if (start_next) {
          std::lock_guard<std::mutex> lock(mutex_);
          start_current_mission_locked();
        }
      };

    cover_client_->async_send_goal(goal, opts);
  }

  void cancel_active_goal_locked()
  {
    if (active_action_ == ActiveAction::GNSS && gnss_goal_handle_) {
      gnss_client_->async_cancel_goal(gnss_goal_handle_);
    } else if (active_action_ == ActiveAction::COVER && cover_goal_handle_) {
      cover_client_->async_cancel_goal(cover_goal_handle_);
    }
  }

  void clear_costmaps_locked()
  {
    auto req = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
    if (clear_global_client_->service_is_ready()) {
      clear_global_client_->async_send_request(req);
    } else {
      RCLCPP_WARN(get_logger(), "Global costmap clear service not ready");
    }
    if (clear_local_client_->service_is_ready()) {
      clear_local_client_->async_send_request(req);
    } else {
      RCLCPP_WARN(get_logger(), "Local costmap clear service not ready");
    }
  }

  void publish_status()
  {
    MissionStatus status;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      status.stamp = this->now();
      status.active_mission = active_mission_;
      status.state = state_;
      status.current_waypoint_index = current_waypoint_index_;
      status.total_waypoints = total_waypoints_;
      status.distance_remaining = distance_remaining_;
      status.detail = last_detail_;
    }
    status_pub_->publish(status);
  }

  std::mutex mutex_;

  std::vector<MissionSpec> missions_;
  size_t current_index_{0};

  MissionSpec active_mission_{};
  uint8_t state_{STATE_IDLE};
  bool pause_requested_{false};
  bool abort_requested_{false};
  bool pending_restart_{false};

  uint32_t current_waypoint_index_{0};
  uint32_t total_waypoints_{0};
  float distance_remaining_{-1.0f};
  std::string last_detail_;

  ActiveAction active_action_{ActiveAction::NONE};
  rclcpp_action::ClientGoalHandle<GnssOnly>::SharedPtr gnss_goal_handle_;
  rclcpp_action::ClientGoalHandle<CoverVision>::SharedPtr cover_goal_handle_;

  rclcpp_action::Client<GnssOnly>::SharedPtr gnss_client_;
  rclcpp_action::Client<CoverVision>::SharedPtr cover_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_global_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_local_client_;

  rclcpp::Subscription<MissionList>::SharedPtr mission_list_sub_;
  rclcpp::Subscription<MissionControl>::SharedPtr mission_control_sub_;
  rclcpp::Publisher<MissionStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::string mission_list_topic_;
  std::string mission_control_topic_;
  std::string status_topic_;
  std::string gnss_action_name_;
  std::string cover_action_name_;
  std::string clear_global_costmap_service_;
  std::string clear_local_costmap_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionMaster>());
  rclcpp::shutdown();
  return 0;
}
