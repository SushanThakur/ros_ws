#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char * argv[])
{
  // Initialized ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGropu Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "robotic_arm");

  // Set a target Pose
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;

    msg.orientation.w = -0.00000183;
    msg.orientation.x = -5.5;
    msg.orientation.y = 1;
    msg.orientation.z = 0.0;

    msg.position.x = -0.1;
    msg.position.y = 0.00;
    msg.position.z = 0.53;
    return msg;
  }();
  ;
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [sucess, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(sucess){
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning Failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}