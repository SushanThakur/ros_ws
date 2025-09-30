import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
import math
import os

class ps4Mover(Node):
    def __init__(self):
        super().__init__('ps4_mover')
        self.ps4_sub = self.create_subscription(
            Joy,
            '/joy',
            self.ps4_callback,
            10
        )
        self.joint_pub = self.create_publisher(
            JointState,
            "joint_states",
            10
        )

        self.freq_ = 0.1
        # self.timer_ = self.create_timer(self.freq_, self.timer_callback)

        self.joints_ = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
        self.home_ = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.move_ = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.move2_ = [3.14, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0]

        self.get_logger().info("Ready")

    def ard_map(self, x, in_min_max, out_min_max):
        in_min, in_max = in_min_max
        out_min, out_max = out_min_max
        if in_max == in_min:  # Prevent division by zero
            return out_min
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def time_callback(self, pos):
        ms = JointState()

        ms.header.stamp = self.get_clock().now().to_msg()
        ms.name = self.joints_

        ms.position = pos

        self.joint_pub.publish(ms)
        self.get_logger().info(f"{self.move_}")

    def ps4_callback(self, msg):
        buttons = msg.buttons
        axes = msg.axes
        x_axis = axes[3]

        # if buttons[0]:
        #     self.get_logger().info("X button was pressed")
        #     self.time_callback(self.move_)
        # elif buttons[3]: 
        #     self.get_logger().info("[] button was pressed")
        #     self.time_callback(self.move2_)
        # else:
        #     self.time_callback(self.home_)

        self.move_[0] = self.ard_map(x_axis, [-1, 1], [3.14, -3.14])
        self.time_callback(self.move_)


def main(): 
    rclpy.init()
    node = ps4Mover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()