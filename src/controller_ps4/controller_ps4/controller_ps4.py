import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose
import math
import os

def ard_map(x, in_min_max, out_min_max):
    in_min, in_max = in_min_max
    out_min, out_max = out_min_max
    if in_max == in_min:  # Prevent division by zero
        return out_min
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class controllerPs4(Node):

    def __init__(self):
        super().__init__('controller_ps4')
        self.ps4_sub_ = self.create_subscription(
            Joy,
            '/joy',
            self.ps4_call_,
            10
        )
        self.pos_pub_ = self.create_publisher(
            Pose,
            "ee_pose",
            10
        )
        
        def get_def_pose():
            msg = Pose()
            msg.orientation.w = 0.00
            msg.orientation.x = 0.00
            msg.orientation.y = 0.707
            msg.orientation.z = 0.707
            msg.position.x = -0.107
            msg.position.y = -0.177
            msg.position.z = 0.44
            return msg
        
        self.def_pose_ = get_def_pose()
        self.edit_pose_ = get_def_pose()

    

    def ps4_call_(self, msg):
        buttons = msg.buttons
        axes = msg.axes

        self._update_position_x(axes[6])
        self._update_position_y(axes[7])
        self._update_position_z(buttons)
        self._update_position_x_with_axes3(axes[3])

        self.pos_pub_.publish(self.edit_pose_)

        self.get_logger().info(f"x = {self.edit_pose_.position.x:.4f}, "
                        f"y = {self.edit_pose_.position.y:.4f}, "
                        f"z = {self.edit_pose_.position.z:.4f} "
                        )

    def _update_position_x(self, axis_value):
        if axis_value == 1:
            self.edit_pose_.position.x += 0.01
        elif axis_value == -1:
            self.edit_pose_.position.x -= 0.01

    def _update_position_y(self, axis_value):
        if axis_value == 1:
            self.edit_pose_.position.y += 0.01
        elif axis_value == -1:
            self.edit_pose_.position.y -= 0.01

    def _update_position_z(self, buttons):
        if buttons[4]:
            self.edit_pose_.position.z += 0.01
        if buttons[5]:
            self.edit_pose_.position.z -= 0.01

    def _update_position_x_with_axes3(self, axis_value):
        if axis_value:
            if axis_value > 0:
                temp = ard_map(axis_value, [0, 1], [0.0001, 0.001])
                self.edit_pose_.position.x += temp
            else:
                temp = ard_map(axis_value, [-1, 0], [0.001, 0.0001])
                self.edit_pose_.position.x -= temp
        
def main():
    rclpy.init()
    node = controllerPs4()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()