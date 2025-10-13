import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped

FRAME_ID = "base_link"

joints = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7', 'tool_joint']


class TwistPublisher(Node):

	def __init__(self):
		super().__init__('twist_publisher')

		self.ps_listener_sub = self.create_subscription(Joy, 'joy', self.listener_call, 10)
		self.ps_listener_sub

		self.twist_pub = self.create_publisher(TwistStamped, "servo_node/delta_twist_cmds", 10)

	def listener_call(self, msg):
		buttons = msg.buttons
		axes = msg.axes

		# Create a generator expression that ignores some indices of the 'axes' and 'buttons' list.
		# This is done because change in only few buttons on the PS4 needs to publish twist message
		buttons_check_generator = (a == 0 for i, a in enumerate(buttons) if i not in (0,1,2,3,5,7,8,9,10,11,12))
		axes_check_generator = (b == 0.0 for i, b in enumerate(axes) if i not in (2,5,6,7))

		all_zero = all(buttons_check_generator) and all(axes_check_generator)

		if not(all_zero):
			req_twist = TwistStamped()
			req_twist.header.stamp = self.get_clock().now().to_msg()
			req_twist.header.frame_id = FRAME_ID

			req_twist.twist.linear.x = axes[0]
			req_twist.twist.angular.x = axes[4]
			if buttons[4]:
				req_twist.twist.linear.z = axes[1]
				req_twist.twist.angular.y = axes[3]
			else:
				req_twist.twist.linear.y = -1 * axes[1]
				req_twist.twist.angular.z = -1 * axes[3]

			self.twist_pub.publish(req_twist)
			self.get_logger().debug(f"Twist Message Published")
		

def main():
	
	rclpy.init()
	
	twist_publisher = TwistPublisher()
	rclpy.spin(twist_publisher)
	twist_publisher.destroy_node()

	rclpy.shutdown()

			
if __name__ == "__main__":
	main()