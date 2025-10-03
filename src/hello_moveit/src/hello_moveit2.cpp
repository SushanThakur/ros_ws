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
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Get Current Pose
  geometry_msgs::msg::PoseStamped current_stamped = move_group_interface.getCurrentPose();
  geometry_msgs::msg::Pose current_pose = current_stamped.pose;
  RCLCPP_INFO(node->get_logger(),
  "Current Pose: Position (x: %.3f, y: %.3f, z: %.3f), "
  "Orientation: (x: %.3f, y: %.3f, z: %.3f, w: %.3f)",
  current_pose.position.x, current_pose.position.y, current_pose.position.z, 
  current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z,
  current_pose.orientation.w
);

  // Set a target Pose
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    //test
    msg.orientation.w = 0.00079;

    msg.orientation.x = 1;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;

    msg.position.x = -0.1;
    msg.position.y = -0.18;
    msg.position.z = 0.36;
    return msg;
  }();
  ;
//   move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
//   auto const [sucess, plan] = [&move_group_interface]{
//     moveit::planning_interface::MoveGroupInterface::Plan msg;
//     auto const ok = static_cast<bool>(move_group_interface.plan(msg));
//     return std::make_pair(ok, msg);
//   }();

  // Execute the plan
//   if(sucess){
//     move_group_interface.execute(plan);
//   } else {
//     RCLCPP_ERROR(logger, "Planning Failed!");
//   }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}