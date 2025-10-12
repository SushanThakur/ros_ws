import rclpy
from rclpy.node import Node
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
		
		# timer_period = 1.0
		# self.timer = self.create_timer(timer_period, self.timer_callback)
		
	def joy_callback(self, msg):
		buttons = msg.buttons
		
		if buttons[7]:
			self.timer_callback("close")
		else:
			self.timer_callback("open")
			
	def timer_callback(self, state):

		joint_traj = JointTrajectory()
		joint_traj.header.frame_id = FRAME_ID
		joint_traj.header.stamp = self.get_clock().now().to_msg()
		joint_traj.joint_names = joints
		
		temp_point = JointTrajectoryPoint()
		if state == "open":
			temp_point.positions = open_pose.positions
		elif state == "close":
			temp_point.positions = close_pose.positions
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

		