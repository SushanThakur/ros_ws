import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Joy
import sys

node_name = 'ik_cmd_pub'

default_orient = [0.0, 0.0, 0.707, -0.707]

# draw_entity = [
#     [0.4,0.3,0.1],
#     [0.3,0.3,0.1],
#     [0.3,0.2,0.1],
#     [0.4,0.2,0.1],
#     [0.4,0.1,0.1],
#     [0.3,0.1,0.1],
# ]

draw_entity = [
[-0.357 ,0.284 ,0.100],
[-0.372 ,0.284 ,0.100],
[-0.372 ,0.180 ,0.100],
[-0.381 ,0.180 ,0.100],
[-0.381 ,0.284 ,0.100],
[-0.396 ,0.284 ,0.100],
[-0.396 ,0.292 ,0.100],
[-0.357 ,0.292 ,0.100],
[-0.317 ,0.188 ,0.200],
[-0.318 ,0.180 ,0.100],
[-0.350 ,0.180 ,0.100],
[-0.350 ,0.284 ,0.100],
[-0.353 ,0.284 ,0.100],
[-0.351 ,0.292 ,0.100],
[-0.317 ,0.292 ,0.100],
[-0.318 ,0.284 ,0.100],
[-0.342 ,0.284 ,0.100],
[-0.342 ,0.240 ,0.100],
[-0.317 ,0.240 ,0.100],
[-0.318 ,0.232 ,0.100],
[-0.342 ,0.232 ,0.100],
[-0.342 ,0.188 ,0.100],
[-0.317 ,0.188 ,0.100],
[-0.302 ,0.212 ,0.200],
[-0.302 ,0.196 ,0.100],
[-0.293 ,0.187 ,0.100],
[-0.302 ,0.190 ,0.100],
[-0.299 ,0.187 ,0.100],
[-0.293 ,0.187 ,0.100],
[-0.290 ,0.187 ,0.100],
[-0.282 ,0.194 ,0.100],
[-0.285 ,0.187 ,0.100],
[-0.282 ,0.189 ,0.100],
[-0.282 ,0.194 ,0.100],
[-0.282 ,0.226 ,0.100],
[-0.306 ,0.237 ,0.100],
[-0.310 ,0.243 ,0.100],
[-0.309 ,0.238 ,0.100],
[-0.310 ,0.240 ,0.100],
[-0.310 ,0.243 ,0.100],
[-0.310 ,0.278 ,0.100],
[-0.293 ,0.293 ,0.100],
[-0.310 ,0.288 ,0.100],
[-0.304 ,0.293 ,0.100],
[-0.293 ,0.293 ,0.100],
[-0.289 ,0.293 ,0.100],
[-0.274 ,0.276 ,0.100],
[-0.279 ,0.292 ,0.100],
[-0.274 ,0.287 ,0.100],
[-0.274 ,0.276 ,0.100],
[-0.274 ,0.254 ,0.100],
[-0.282 ,0.255 ,0.100],
[-0.282 ,0.276 ,0.100],
[-0.290 ,0.284 ,0.100],
[-0.282 ,0.281 ,0.100],
[-0.285 ,0.284 ,0.100],
[-0.290 ,0.284 ,0.100],
[-0.293 ,0.284 ,0.100],
[-0.302 ,0.278 ,0.100],
[-0.299 ,0.284 ,0.100],
[-0.302 ,0.282 ,0.100],
[-0.302 ,0.278 ,0.100],
[-0.301 ,0.244 ,0.100],
[-0.277 ,0.232 ,0.100],
[-0.274 ,0.226 ,0.100],
[-0.275 ,0.231 ,0.100],
[-0.274 ,0.229 ,0.100],
[-0.274 ,0.226 ,0.100],
[-0.274 ,0.194 ,0.100],
[-0.290 ,0.179 ,0.100],
[-0.274 ,0.184 ,0.100],
[-0.279 ,0.179 ,0.100],
[-0.290 ,0.179 ,0.100],
[-0.295 ,0.179 ,0.100],
[-0.310 ,0.196 ,0.100],
[-0.305 ,0.179 ,0.100],
[-0.310 ,0.185 ,0.100],
[-0.310 ,0.196 ,0.100],
[-0.310 ,0.212 ,0.100],
[-0.302 ,0.212 ,0.100],
[-0.231 ,0.292 ,0.200],
[-0.231 ,0.284 ,0.100],
[-0.246 ,0.284 ,0.100],
[-0.246 ,0.180 ,0.100],
[-0.255 ,0.180 ,0.100],
[-0.255 ,0.284 ,0.100],
[-0.270 ,0.284 ,0.100],
[-0.270 ,0.292 ,0.100],
[-0.231 ,0.292 ,0.100],
]

# draw_entity = [
    
# 	# S
#     [0.4, 0.4, 0.1],
#     [0.3, 0.4, 0.1],
#     [0.3, 0.3, 0.1],
#     [0.4, 0.3, 0.1],
#     [0.4, 0.2, 0.1],
#     [0.3, 0.2, 0.1],
    
# 	[0.3, 0.2, 0.2],
#     [0.2, 0.2, 0.2],
    
# 	# U
#     [0.2, 0.2, 0.1],
#     [0.2, 0.4, 0.1],
#     [0.1, 0.4, 0.1],
#     [0.1, 0.2, 0.1],
    
# 	[0.1, 0.2, 0.2],
#     [-0.1, 0.2, 0.2],
    
# 	# S
#     [-0.1, 0.2, 0.1],
#     [0.0, 0.2, 0.1],
#     [0.0, 0.3, 0.1],
#     [-0.1, 0.3, 0.1],
#     [-0.1, 0.4, 0.1],
#     [0.0, 0.4, 0.1],
    
# ]

class IkPublisher(Node):
    def __init__(self):
        super().__init__(node_name)

        self.ik_pub_ = self.create_publisher(Pose, node_name, 10)
        self.joy_sub_ = self.create_subscription(Joy, 'joy', self.joy_call, 10)

        self.timer_call = self.create_timer(1, self.ik_pub_call)

        self.point_index = 0
        self.last_button_state = 0
        
    def joy_call(self, msg):
            button = msg.buttons[2]
            axis = msg.axes[7]
            if axis == 1.0 and not button == self.last_button_state:
                self.ik_pub_call()
            self.last_button_state = axis

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
            self.get_logger().info(f"Published pose {self.point_index + 1}: {pub_pose}")
            time.sleep(0.5)
            self.point_index += 1

        self.get_logger().info("Drawing stopped")
        self.point_index = 0
        self.destroy_node()
        sys.exit()

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