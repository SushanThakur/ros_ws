#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class PoseToVelocityNode : public rclcpp::Node
{
public:
  PoseToVelocityNode() : Node("pose_to_velocity_node")
  {
    this->declare_parameter("gain_linear", 1.0);
    this->declare_parameter("gain_angular", 1.0);
    this->declare_parameter("publish_rate_hz", 50.0);
    this->declare_parameter("frame_id", "base_link");

    gain_linear_ = this->get_parameter("gain_linear").as_double();
    gain_angular_ = this->get_parameter("gain_angular").as_double();
    double publish_rate_hz = this->get_parameter("publish_rate_hz").as_double();
    frame_id_ = this->get_parameter("frame_id").as_string();

    if (gain_linear_ <= 0.0 || gain_angular_ <= 0.0)
    {
      RCLCPP_WARN(this->get_logger(), "Gains must be positive. Setting to default 1.0.");
      gain_linear_ = 1.0;
      gain_angular_ = 1.0;
    }
    if (publish_rate_hz <= 0.0)
    {
      RCLCPP_WARN(this->get_logger(), "Publish rate must be positive. Setting to 50 Hz.");
      publish_rate_hz = 50.0;
    }

    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/ee_pose", 10, std::bind(&PoseToVelocityNode::poseCallback, this, std::placeholders::_1));
    vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/servo_node/delta_twist_cmds", 10);

    double period_ms = 1000.0 / publish_rate_hz;
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
        std::bind(&PoseToVelocityNode::publishVelocity, this));

    RCLCPP_INFO(this->get_logger(), "PoseToVelocityNode started, listening to /ee_pose (geometry_msgs/Pose), publishing to /servo_node/delta_twist_cmds");
    RCLCPP_INFO(this->get_logger(), "Parameters: gain_linear=%.2f, gain_angular=%.2f, publish_rate=%.2f Hz, frame_id=%s",
                gain_linear_, gain_angular_, publish_rate_hz, frame_id_.c_str());
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Pose last_pose_;
  bool first_msg_ = true;
  double gain_linear_;
  double gain_angular_;
  std::string frame_id_;
  geometry_msgs::msg::Pose latest_pose_;
  rclcpp::Time last_time_;

  void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    latest_pose_ = *msg;
    if (first_msg_)
    {
      last_pose_ = *msg;
      last_time_ = this->now();
      first_msg_ = false;
      return;
    }
    last_time_ = this->now();
  }

  void publishVelocity()
  {
    if (first_msg_)
    {
      return;
    }

    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time_).seconds();

    if (dt <= 0.0)
    {
      RCLCPP_WARN(this->get_logger(), "Invalid time delta (%.6f s). Skipping velocity computation.", dt);
      return;
    }

    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = this->now();
    twist.header.frame_id = frame_id_;

    double dx = latest_pose_.position.x - last_pose_.position.x;
    double dy = latest_pose_.position.y - last_pose_.position.y;
    double dz = latest_pose_.position.z - last_pose_.position.z;

    twist.twist.linear.x = gain_linear_ * dx / dt;
    twist.twist.linear.y = gain_linear_ * dy / dt;
    twist.twist.linear.z = gain_linear_ * dz / dt;

    tf2::Quaternion q_current, q_last, q_diff;
    tf2::fromMsg(latest_pose_.orientation, q_current);
    tf2::fromMsg(last_pose_.orientation, q_last);
    q_diff = q_current * q_last.inverse();
    tf2::Matrix3x3 m(q_diff);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    twist.twist.angular.x = gain_angular_ * roll / dt;
    twist.twist.angular.y = gain_angular_ * pitch / dt;
    twist.twist.angular.z = gain_angular_ * yaw / dt;

    vel_pub_->publish(twist);
    RCLCPP_INFO(this->get_logger(), "Published TwistStamped: linear=(%.3f, %.3f, %.3f), angular=(%.3f, %.3f, %.3f)",
                 twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z,
                 twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z);

    last_pose_ = latest_pose_;
    last_time_ = current_time;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseToVelocityNode>());
  rclcpp::shutdown();
  return 0;
}