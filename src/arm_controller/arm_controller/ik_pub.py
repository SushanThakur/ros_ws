import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

node_name = 'ik_cmd_pub'

default_pose = Pose()
default_pose.position.x = -0.107
default_pose.position.y = -0.177
default_pose.position.z = 0.445
default_pose.orientation.w = -0.0
default_pose.orientation.x = -0.0
default_pose.orientation.y = 0.707
default_pose.orientation.z = 0.707

class IkPublisher(Node):
	def __init__(self):
		super().__init__(node_name)

		self.ik_pub_ = self.create_publisher(Pose, node_name, 10)

		self.timer_call = self.create_timer(1, self.ik_pub_call)

	def ik_pub_call(self):
		self.ik_pub_.publish(default_pose)

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