import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from moveit_msgs.srv import ServoCommandType
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped

FRAME_ID = "base_link"

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
		buttons_check_generator = (a == 0 for i, a in enumerate(buttons) if i not in (0,5,7))
		axes_check_generator = (b == 0.0 for i, b in enumerate(axes) if i not in (2,5))

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
			self.get_logger().info(f"Twist Message Published")
		

def main():
	
	rclpy.init()
	
	switch_cmd = SwitchCmd()
	twist_publisher = TwistPublisher()

	request_cmd = switch_cmd.send_request(1)
	rclpy.spin_until_future_complete(switch_cmd, request_cmd)
	if request_cmd.done() and request_cmd.result is not None:
		response = request_cmd.result()
		switch_cmd.get_logger().info(f"Switch to TWIST: Result = {response}")
		rclpy.spin(twist_publisher)
	else:
		switch_cmd.get_logger().error("Failed to switch to TWIST")
		
	
	switch_cmd.destroy_node()
	twist_publisher.destroy_node()
	rclpy.shutdown()

			
if __name__ == "__main__":
	main()