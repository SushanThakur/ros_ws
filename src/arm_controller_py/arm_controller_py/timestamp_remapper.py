import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped

class TimestampRemapper(Node):
	def __init__(self):
		super().__init__('timestamp_remapper')
		self.sub = self.create_subscription(TwistStamped, '/servo_node/delta_twist_cmds', self.callback, 10)
		self.pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

	def callback(self, msg):
		new_msg = TwistStamped()
		new_msg = msg
		new_msg.header.stamp = self.get_clock().now().to_msg()
		self.pub.publish(new_msg)

def main():
	rclpy.init()
	node = TimestampRemapper()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__=='__main__':
	main()