import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import os


class PS4Listener(Node):
    def __init__(self):
        super().__init__('ps4_listener')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)

    def joy_callback(self, msg):
        left_stick_x = msg.axes[0]
        left_stick_y = msg.axes[1]
        button_cross = msg.buttons[0]
        os.system("clear")
        self.get_logger().info(
            f'Left Stick: ({left_stick_x:.2f}, {left_stick_y:.2f}) | Cross: {button_cross}')


def main(args=None):
    rclpy.init(args=args)
    node = PS4Listener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
