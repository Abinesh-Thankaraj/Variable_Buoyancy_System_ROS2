#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>

class BallastControl : public rclcpp::Node {
public:
  BallastControl() : Node("ballast_control") {
    // ROS 2 Publisher
    ros_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/ballast_command", 10);
    
    // Ignition Transport
    ign_node_.Subscribe("/world/default/model/blueROV2/link/base_link/wrench",
      &BallastControl::WrenchCallback, this);
    
    // Control timer
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = {0.5, 0.5}; // Default neutral
        ros_pub_->publish(msg);
      });
  }

private:
  void WrenchCallback(const ignition::msgs::Wrench &_msg) {
    RCLCPP_INFO(get_logger(), "Received wrench: fx=%.2f, fy=%.2f", 
                _msg.force().x(), _msg.force().y());
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ros_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  ignition::transport::Node ign_node_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallastControl>());
  rclcpp::shutdown();
  return 0;
}
