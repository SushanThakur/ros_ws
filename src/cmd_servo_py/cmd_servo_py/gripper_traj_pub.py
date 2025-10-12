import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import Joy

# -0.040 0.040

FRAME_ID = "tool_link"
joints = ['grip_left', 'grip_right']

default_pose = JointTrajectoryPoint()
default_pose.positions = [0, 0.4944]

open_pose = JointTrajectoryPoint()
close_pose = JointTrajectoryPoint()

open_pose.positions = [0.0, 0.0]
close_pose.positions = [-0.04, 0.04]

class JointTrajectoryPublisher(Node):
	
	def __init__(self):
		super().__init__('custom_joint_trajectory_publisher')
		self.joint_traj_pub = self.create_publisher(JointTrajectory, "gripper_controller/joint_trajectory", 10)
		self.joint_traj_pub
		
		self.joy_sub = self.create_subscription(Joy, "joy", self.joy_callback, 10)
		self.gripper_param_cli = self.create_client(GetParameters, 'gripper_param/gripper_state_param')
		self.gripper_param_set = self.create_client(SetParameters, 'gripper_param/gripper_state_param')

		while not self.gripper_param_cli.wait_for_service(timeout_sec=1.0):
			self.get_logger().info("CLI Service not available, waiting again...")
		while not self.gripper_param_set.wait_for_service(timeout_sec=1.0):
			self.get_logger().info("SET Service not available, waiting again...")

		self.req = GetParameters.Request()
		self.set = SetParameters.Request()

	def request_param(self):
		self.req.names = ['gripper_state_param']

		self.future = self.gripper_param_cli.call_async(self.req)
		rclpy.spin_until_future_complete(self, self.future)
		return self.future.result()
	
	def set_param(self, param_val):
		param = Parameter()
		param.name = "gripper_state_param"
		param.value.type = ParameterType.PARAMETER_STRING
		param.value.string_value = param_val
		self.set.parameters.append(param)

		self.future = self.gripper_param_set.call_async(self.set)
		rclpy.spin_until_future_complete(self, self.future)
		return self.future.result()
		
	def joy_callback(self, msg):
		buttons = msg.buttons
		
		if buttons[7]:
			gripper_state = self.request_param('gripper_state_param')
			self.timer_callback(gripper_state)
			
	def timer_callback(self, state):

		joint_traj = JointTrajectory()
		joint_traj.header.frame_id = FRAME_ID
		joint_traj.header.stamp = self.get_clock().now().to_msg()
		joint_traj.joint_names = joints
		
		temp_point = JointTrajectoryPoint()
		if state == "OPENED":
			self.set_param("CLOSED")
			temp_point.positions = close_pose.positions
		elif state == "CLOSED":
			self.set_param("OPENED")
			temp_point.positions = open_pose.positions
		temp_point.time_from_start = Duration(sec=1, nanosec=0)
		
		joint_traj.points = [temp_point]
		
		self.joint_traj_pub.publish(joint_traj)
		self.get_logger().info("Joint Trajectory Published")
		
	
def main():
	rclpy.init()
	joint_trajectory_publisher = JointTrajectoryPublisher()
	
	rclpy.spin(joint_trajectory_publisher)
	
	joint_trajectory_publisher.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

		