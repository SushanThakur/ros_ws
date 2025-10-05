import rclpy
from rclpy.node import Node
from moveit_msgs.msg import MoveItErrorCodes
# from moveit_core.robot_state import RobotState
from moveit.planning import MoveGroupInterface
from geometry_msgs.msg import PoseStamped

class EEFPosePublisher(Node):
    def __init__(self):
        super().__init__('eef_pose_publisher')
        self.move_group = MoveGroupInterface(self, "arm")  # Your planning group
        self.pose_pub = self.create_publisher(PoseStamped, '/end_effector_pose', 10)
        self.timer = self.create_timer(0.1, self.publish_pose)  # 10 Hz

    def publish_pose(self):
        current_pose = self.move_group.get_current_pose()
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "world"  # Your base frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = current_pose.pose
        self.pose_pub.publish(pose_stamped)
        self.get_logger().info(f"Published EE pose: {pose_stamped.pose.position.x}, {pose_stamped.pose.position.y}, {pose_stamped.pose.position.z}")

def main(args=None):
    rclpy.init(args=args)
    node = EEFPosePublisher()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()