#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <mr2_action_interface/action/gnss_only.hpp>
using GnssOnly = mr2_action_interface::action::GnssOnly;

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("gnss_only_client");
  auto client = rclcpp_action::create_client<GnssOnly>(node, "gnss_only");

  if (!client->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node->get_logger(), "gnss_only server not available"); return 1;
  }

  GnssOnly::Goal goal;
  goal.target_latitude  = std::stod(std::string(argv[1]));
  goal.target_longitude = std::stod(std::string(argv[2]));

  auto send_opts = rclcpp_action::Client<GnssOnly>::SendGoalOptions();
  send_opts.feedback_callback = [node](auto, const std::shared_ptr<const GnssOnly::Feedback> &fb){
    RCLCPP_INFO(node->get_logger(), "BT status: %d", fb->bt_status);
  };
auto gh = client->async_send_goal(goal, send_opts);

// Wait for the goal handle to be returned
auto gh_result = gh.get();  

// Ask for the result future
auto res_future = client->async_get_result(gh_result);

// Wait until the result future is ready
rclcpp::spin_until_future_complete(node, res_future);

// Get the actual WrappedResult
auto res = res_future.get();  

if (res.result->navigation_result == 1) {
    RCLCPP_INFO(node->get_logger(), "SUCCESS");
} else {
    RCLCPP_INFO(node->get_logger(), "FAIL");
}
  rclcpp::shutdown();
  return 0;
}
