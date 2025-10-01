import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

class Ps4MoverSimple(Node):
    def __init__(self):
        super().__init__('ps4_mover_simple')
        # Path to MoveIt config parameters (adjust if your package name differs)
        moveit_config_path = os.path.join(
            get_package_share_directory('moveit_config'),
            'config',
            'moveit_parameters.yaml'  # Adjust to match your config file
        )

        # Initialize MoveItPy
        self.moveit = MoveItPy(
            node_name='moveit_py_node',
            launch_params_filepaths=[moveit_config_path]
        )
        self.arm_group = self.moveit.get_planning_component("arm")  # Replace with your group name
        self.arm_group.set_planning_time(5.0)
        self.arm_group.set_num_planning_attempts(5)

        # PS4 subscription and joint publisher
        self.joy_sub = self.create_subscription(Joy, '/joy', self.ps4_callback, 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Joints and home pose
        self.joints_ = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
        self.home_pose = np.zeros(7)

        self.get_logger().info('Ps4MoverSimple initialized')

    def time_callback(self, pos):
        ms = JointState()
        ms.header.stamp = self.get_clock().now().to_msg()
        ms.name = self.joints_
        ms.position = pos.tolist()
        self.joint_pub.publish(ms)
        self.get_logger().info(f"Published JointState: {pos}")

    def ps4_callback(self, msg):
        buttons = msg.buttons
        self.get_logger().info(f"Buttons: {buttons}")
        if buttons[0]:  # X button: Move to home pose
            self.get_logger().info("X button: Moving to home pose")
            try:
                self.arm_group.set_joint_value_target(self.joints_, self.home_pose.tolist())
                plan = self.arm_group.plan()
                if plan.trajectory:
                    self.arm_group.execute(plan)
                    self.time_callback(self.home_pose)
                else:
                    self.get_logger().warn("Home pose planning failed")
            except Exception as e:
                self.get_logger().error(f"Planning/execution failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Ps4MoverSimple()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
