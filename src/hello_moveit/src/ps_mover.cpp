#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>

using std::placeholders::_1;

class psSub : public rclcpp::Node
{
  public:
    psSub()
    : Node("ps_subscriber")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/ee_pose", 10, std::bind(&psSub::topic_callback, this, _1)
      );
    }

    private:
      void topic_callback(const geometry_msgs::msg::Pose & pose) const
      {
        RCLCPP_INFO(this->get_logger(), 
        "Pose: position(x=%.3f, y=%.3f, z=%.3f), Orientation(w=%.3f)",
        pose.position.x, pose.position.y, pose.position.z, pose.orientation.w
        );
      }
      rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<psSub>());
  rclcpp::shutdown();

  return 0;
}