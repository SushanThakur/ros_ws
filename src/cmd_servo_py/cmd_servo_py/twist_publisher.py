from urllib import response
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ServoCommandType
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped

joints = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7', 'tool_joint']

class SwitchCmd(Node):

	def __init__(self):
		super().__init__('switch_cmd')

		self.switch_cmd_cli = self.create_client(ServoCommandType, 'servo_node/switch_command_type')
		while not self.switch_cmd_cli.wait_for_service(timeout_sec=1.0):
			self.get_logger().info("Service not available, waiting again...")
		self.req = ServoCommandType.Request()



	def send_request(self, command_type):
		self.req.command_type = command_type
		return self.switch_cmd_cli.call_async(self.req)


class PsListener(Node):

	def __init__(self):
		super().__init__('ps_listener')

		self.ps_listener_sub = self.create_subscription(Joy, 'joy', self.listener_call, 10)
		self.ps_listener_sub

		self.twist_pub = self.create_publisher(TwistStamped, "servo_node/delta_twist_cmds", 10)

	def listener_call(self, msg):
		buttons = msg.buttons
		axes = msg.axes

		# Create a generator expression that ignores indices 2 and 5 of the 'axes' list.
		axes_check_generator = (b == 0.0 for i, b in enumerate(axes) if i not in (2, 5))

		all_zero = all(a == 0 for a in buttons) and all(axes_check_generator)

		if not(all_zero):
				twist = TwistStamped()
				twist.header.stamp = self.get_clock().now().to_msg()
				twist.header.frame_id = "base_link"
				if not buttons[4]:
					twist.twist.linear.x = axes[0]
					twist.twist.linear.y = -1 * axes[1]
					twist.twist.angular.x = axes[4]
					twist.twist.angular.y = axes[3]
				else:
					twist.twist.linear.z = axes[1]
					twist.twist.angular.z = axes[3]

				self.get_logger().info(f"Twist Message Published")
				self.twist_pub.publish(twist)
		


def main():
	rclpy.init()
	switch_cmd = SwitchCmd()

	# Switch command type to TWIST
	request = switch_cmd.send_request(1)
	rclpy.spin_until_future_complete(switch_cmd, request)
	response = request.result()
	switch_cmd.get_logger().info(f"Result = {response}")


	twist_publisher = PsListener()
	rclpy.spin(twist_publisher)
	

	switch_cmd.destroy_node()
	twist_publisher.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()