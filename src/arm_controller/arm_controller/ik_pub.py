import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

node_name = 'ik_cmd_pub'

default_orient = [0.0, 0.0, 0.707, -0.707]

letter_S = [
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

        self.timer_call = self.create_timer(1, self.ik_pub_call)

        self.executed = False
        self.point_index = 0

    def ik_pub_call(self):
        if self.point_index >= len(letter_S):
            self.get_logger().info("Finished publishing all poses.")
            return

        pub_pose = Pose()
        pub_pose.orientation.w = default_orient[0]
        pub_pose.orientation.x = default_orient[1]
        pub_pose.orientation.y = default_orient[2]
        pub_pose.orientation.z = default_orient[3]

        pub_pose.position.x = letter_S[self.point_index][0]
        pub_pose.position.y = letter_S[self.point_index][1]
        pub_pose.position.z = letter_S[self.point_index][2]

        self.ik_pub_.publish(pub_pose)
        self.get_logger().info(f"Published pose {self.point_index + 1}: {pub_pose}")
        time.sleep(1)
        self.point_index += 1

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