import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading
from rclpy.node import Node
from moveit_msgs.srv import ServoCommandType
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped, PoseStamped

FRAME_ID = "base_link"

POSE_PUBLISH = False
TWIST_PUBLISH = False

joints = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7', 'tool_joint']
# Default Pose for Robotic Arm
default_pose = PoseStamped()
default_pose.pose.position.x = 0.08949
default_pose.pose.position.y = 0.13793
default_pose.pose.position.z = 0.57731
default_pose.pose.orientation.x = 0.7343
default_pose.pose.orientation.y = 2.6234
default_pose.pose.orientation.z = -3.7148
default_pose.pose.orientation.w = 0.6787


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
		buttons_check_generator = (a == 0 for i, a in enumerate(buttons) if i not in (4,5))
		axes_check_generator = (b == 0.0 for i, b in enumerate(axes) if i not in (2, 5))

		all_zero = all(buttons_check_generator) and all(axes_check_generator)

		if not(all_zero):
			req_twist = TwistStamped()
			req_twist.header.stamp = self.get_clock().now().to_msg()
			req_twist.header.frame_id = FRAME_ID

			req_twist.twist.linear.x = axes[0]
			req_twist.twist.angular.x = axes[4]
			if buttons[4]:
				req_twist.twist.linear.z = axes[1]
				req_twist.twist.angular.z = -1 * axes[3]
			else:
				req_twist.twist.linear.y = -1 * axes[1]
				req_twist.twist.angular.y = axes[3]

			self.twist_pub.publish(req_twist)
			self.get_logger().info(f"Twist Message Published")
		
class PosePublisher(Node):
	def __init__(self):
		super().__init__('pose_publisher')
		self.pose_pub = self.create_publisher(PoseStamped, "servo_node/pose_target_cmds", 10)
		
		req_pose = PoseStamped()
		req_pose.header.stamp = self.get_clock().now().to_msg()
		req_pose.header.frame_id = FRAME_ID
		
		req_pose.pose = default_pose.pose
		
		if POSE_PUBLISH:
			self.pose_pub.publish(req_pose)
			self.get_logger().info(f"Pose Message Published")

def main():
	rclpy.init()
	switch_cmd = SwitchCmd()
	twist_publisher = TwistPublisher()
	pose_publisher = PosePublisher()

	# # Switch command type to TWIST
	# request_cmd = switch_cmd.send_request(1)
	# rclpy.spin_until_future_complete(switch_cmd, request_cmd)
	# response = request_cmd.result()
	# switch_cmd.get_logger().info(f"Result = {response}")


	# rclpy.spin(twist_publisher)
	rclpy.spin(pose_publisher)
	
	while (True):
		print("1=TWIST")
		print("2=POSE")
		print("0=EXIT")
		print("Enter your choice")
		key = int(input())
		if key == 1:
			request_cmd = switch_cmd.send_request(1)
			rclpy.spin_until_future_complete(switch_cmd, request_cmd)
			response = request_cmd.result()
			switch_cmd.get_logger().info(f"Result = {response}")
		elif key == 2:
			request_cmd = switch_cmd.send_request(2)
			rclpy.spin_until_future_complete(switch_cmd, request_cmd)
			response = request_cmd.result()
			switch_cmd.get_logger().info(f"Result = {response}")
			
			POSE_PUBLISH = True
			
		elif key == 0:
			break

	switch_cmd.destroy_node()
	pose_publisher.destroy_node()
	twist_publisher.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()