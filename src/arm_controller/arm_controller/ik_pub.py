import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Joy
import sys

node_name = 'ik_cmd_pub'

default_orient = [0.0, 0.0, 0.707, -0.707]

draw_entity = [
    [0.4,0.3,0.1],
    [0.3,0.3,0.1],
    [0.3,0.2,0.1],
    [0.4,0.2,0.1],
    [0.4,0.1,0.1],
    [0.3,0.1,0.1],
]



class IkPublisher(Node):
    def __init__(self):
        super().__init__(node_name)

        self.ik_pub_ = self.create_publisher(Pose, node_name, 10)
        self.joy_sub_ = self.create_subscription(Joy, 'joy', self.joy_call, 10)

        # self.timer_call = self.create_timer(1, self.ik_pub_call)

        self.point_index = 0
        self.last_button_state = 0
        
    def joy_call(self, msg):
            button = msg.buttons[2]
            if button and not button == self.last_button_state:
                self.ik_pub_call()
            self.last_button_state = button

    def ik_pub_call(self):
        self.get_logger().info("Drawing started")
        while self.point_index < len(draw_entity):
            pub_pose = Pose()
            pub_pose.orientation.w = default_orient[0]
            pub_pose.orientation.x = default_orient[1]
            pub_pose.orientation.y = default_orient[2]
            pub_pose.orientation.z = default_orient[3]

            pub_pose.position.x = draw_entity[self.point_index][0]
            pub_pose.position.y = draw_entity[self.point_index][1]
            pub_pose.position.z = draw_entity[self.point_index][2]

            self.ik_pub_.publish(pub_pose)
            self.get_logger().debug(f"Published pose {self.point_index + 1}: {pub_pose}")
            time.sleep(1)
            self.point_index += 1

        self.get_logger().info("Drawing stopped")
        self.point_index = 0
        # self.destroy_node()
        # sys.exit()

def main():
    try:
        rclpy.init()
        node = IkPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()