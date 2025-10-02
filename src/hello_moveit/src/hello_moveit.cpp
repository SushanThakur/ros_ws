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

  // Set a target Pose
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    // msg.orientation.w = -0.35853302478790283;
    // msg.orientation.x = 0.3583613336086273;
    // msg.orientation.y = 0.60956871509552;
    // msg.orientation.z = 0.6094728708267212;
    // msg.position.x = -0.08079582452774048;
    // msg.position.y = -0.1770092397928238;
    // msg.position.z = 0.43801742792129517;

    // Home
    // msg.orientation.w = -0.000025124352760030888;
    // msg.orientation.x = -0.0001710644137347117;
    // msg.orientation.y = 0.7071045637130737;
    // msg.orientation.z = 0.7071089744567871;
    // msg.position.x = -0.10730043798685074;
    // msg.position.y = -0.17701663076877594;
    // msg.position.z = 0.44518014788627625;

    //test
    msg.orientation.w = 0.000;

    msg.orientation.x = 0.000;
    msg.orientation.y = 0.7;
    msg.orientation.z = 0.7;

    msg.position.x = -0.1;
    msg.position.y = -0.18;
    msg.position.z = 0.36;
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