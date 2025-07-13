#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class BallastControl : public rclcpp::Node
{
public:
  BallastControl() : Node("ballast_control")
  {
    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/ballast_command", 10);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&BallastControl::control_callback, this));
    
    gz_node_.Subscribe("/ballast_status", &BallastControl::status_callback, this);
  }

private:
  void control_callback()
  {
    auto msg = std_msgs::msg::Float64MultiArray();
    // [front_volume, rear_volume] (0.0-1.0)
    msg.data = {0.5, 0.5};  // Neutral position
    pub_->publish(msg);
  }

  void status_callback(const gz::msgs::Float_V& msg)
  {
    RCLCPP_INFO(get_logger(), "Ballast status: Front=%.2f, Rear=%.2f", 
                msg.data(0), msg.data(1));
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  gz::transport::Node gz_node_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallastControl>());
  rclcpp::shutdown();
  return 0;
}
