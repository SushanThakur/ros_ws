#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>

using namespace moveit_servo;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("minimal_servo_twist");

  const std::string param_namespace = "moveit_servo";

  // Load Servo parameters
  auto servo_param_listener = std::make_shared<const servo::ParamListener>(node, param_namespace);
  auto servo_params = servo_param_listener->get_params();

  // Create a PlanningSceneMonitor
  auto planning_scene_monitor = createPlanningSceneMonitor(node, servo_params);

  // Initialize the Servo object
  Servo servo(node, servo_param_listener, planning_scene_monitor);

  // Publisher to the twist command topic
  auto twist_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>(
      servo_params.command_out_topic, 10);

  rclcpp::Rate rate(50);
  RCLCPP_INFO(node->get_logger(), "Minimal Servo Twist Publisher running...");

  while (rclcpp::ok())
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = node->now();
    cmd.header.frame_id = "base_link";  // or "link_7"
    cmd.twist.linear.x = 0.1;
    cmd.twist.linear.y = 0.0;
    cmd.twist.linear.z = 0.0;
    cmd.twist.angular.x = 0.0;
    cmd.twist.angular.y = 0.0;
    cmd.twist.angular.z = 0.0;

    twist_pub->publish(cmd);

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
