#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class MinimalTwistPub : public rclcpp::Node
{
public:
  MinimalTwistPub()
  : Node("minimal_twist_pub")
  {
    // Publisher for TwistStamped commands
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", 10);

    // Timer to publish periodically (10 Hz)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&MinimalTwistPub::publishTwist, this));

    RCLCPP_INFO(this->get_logger(), "Minimal Twist Publisher started.");
  }

private:
  void publishTwist()
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link"; // or "link_7" depending on your setup

    // Predefined twist values (change as needed)
    msg.twist.linear.x = 0.1;
    msg.twist.linear.y = 0.0;
    msg.twist.linear.z = 0.0;

    msg.twist.angular.x = 0.0;
    msg.twist.angular.y = 0.0;
    msg.twist.angular.z = 0.1;

    twist_pub_->publish(msg);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Publishing Twist: linear(%.2f, %.2f, %.2f), angular(%.2f, %.2f, %.2f)",
                         msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
                         msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z);
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// Standard main function
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalTwistPub>());
  rclcpp::shutdown();
  return 0;
}
