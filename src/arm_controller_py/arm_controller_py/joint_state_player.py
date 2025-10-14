import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Joy
from rcl_interfaces.msg import SetParametersResult
from builtin_interfaces.msg import Duration
import time

PARAM_NAME = 'current_playing_state'
PLAYING = 0

frame_id = "base_link"
# joints = ['gripper_left', 'gripper_right', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7', 'tool_joint']
joints = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7', 'tool_joint']

class JointStateRecorder(Node):
	def __init__(self):
		super().__init__('joint_state_player')

		self.in_file_name = "ignore/pos.txt"
		self.file_mode = 'r'
		self.last_button_state = 0

		self.declare_parameter(PARAM_NAME, PLAYING)
		self.add_on_set_parameters_callback(self.param_change_call)

		self.joint_traj_pub = self.create_publisher(JointTrajectory, "robotic_arm_controller/joint_trajectory", 10)
		self.joint_traj_pub

		self.joy_sub = self.create_subscription(
			Joy,
			'/joy',
			self.joy_call,
			10
		)

	def joy_call(self, msg):
		button = msg.buttons[9]

		if button and not self.last_button_state:
			playing_state = self.get_parameter(PARAM_NAME).value
			self.toggle_playing_state(playing_state)
		self.last_button_state = button

	def toggle_playing_state(self, playing_state):
		new_state = 1 if not playing_state else 0
		self.set_parameters([Parameter(PARAM_NAME, Parameter.Type.INTEGER, new_state)])
		if new_state == 1:
			self.play_file()
	
	def play_file(self):
		try:
			with open(self.in_file_name, self.file_mode) as in_file:
				count = 0
				for line in in_file:
					# trimmed_line = line.strip()[12:-2]
					trimmed_line = line.strip()[21:-2]
					list_str = trimmed_line.split(',')
					list_float = [float(s) for s in list_str]

					joint_traj = JointTrajectory()
					joint_traj.header.frame_id = frame_id
					joint_traj.header.stamp = self.get_clock().now().to_msg()
					joint_traj.joint_names = joints

					temp_point = JointTrajectoryPoint()
					temp_point.positions = list_float
					temp_point.time_from_start = Duration(sec=1, nanosec=0)

					joint_traj.points = [temp_point]
					count += 1
					self.joint_traj_pub.publish(joint_traj)
					time.sleep(0.01)

			self.toggle_playing_state(1)
			self.get_logger().debug(f"Playback complete. Line Count = {count}")
		except FileNotFoundError:
			self.get_logger().error("File not found. ")
		except Exception as e:
			self.get_logger().error(f"Error occured! {str(e)}")


	def param_change_call(self, params):
		for param in params:
			if param.name == PARAM_NAME and param.type_ == Parameter.Type.INTEGER:
				state = "PLAYING" if param.value else "STOPPED PLAYING"
				self.get_logger().info(f"{state}")
		return SetParametersResult(successful=True)

def main():
	rclpy.init()
	joint_state_recorder = JointStateRecorder()
	rclpy.spin(joint_state_recorder)
	joint_state_recorder.destroy_node()
	rclpy.shutdown()

if __name__=='__main__':
	main()