import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class SimpleMover(Node):
    def __init__(self):
        super().__init__("simple_mover")
        self.pub_ = self.create_publisher(JointState, "joint_states", 10)
        self.frequency_ = 0.1
        self.get_logger().info("Publishing at %d Hz" % self.frequency_)
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

        self.angle_ = -3.14

    def timerCallback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']

        msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.position[0] = min(self.angle_, 3.14)
        if self.angle_ > 3.14:
            self.angle_ = -3.14
        self.angle_ += 0.1

        self.pub_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.position[0]:.2f}')


def main():
    rclpy.init()
    simple_mover = SimpleMover()
    rclpy.spin(simple_mover)
    simple_mover.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()