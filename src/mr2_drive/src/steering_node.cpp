#include <Eigen/Dense>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

const std::string JOINT_STATE_TOPIC = "/steering_node/joint_states";
const std::string TWIST_TOPIC = "/driving_node/twist";

class SteeringNode : public rclcpp::Node {
public:
  SteeringNode() : Node("steering_node") {

    target_twist_subscriber_ =
        this->create_subscription<geometry_msgs::msg::Twist>(
            TWIST_TOPIC, 10,
            std::bind(&SteeringNode::twist_callback, this,
                      std::placeholders::_1));
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      target_twist_subscriber_;
  geometry_msgs::msg::Twist twist_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SteeringNode>());
  rclcpp::shutdown();
  return 0;
}
