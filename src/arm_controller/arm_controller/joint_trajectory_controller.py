import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import Joy


FRAME_ID = "base_link"
joints = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7', 'tool_joint']

default_pose = [0, 0.4944, 0, 1.4434, 0, 1.0277, 0, 0]

write_pose = [-1.56, -0.90, -1.79, -1.31, -1.50, 1.35, -0.25, 0.65]

class JointTrajectoryPublisher(Node):
	
	def __init__(self):
		super().__init__('joint_trajectory_controller')
		self.joint_traj_pub = self.create_publisher(JointTrajectory, "robotic_arm_controller/joint_trajectory", 10)
		self.joint_traj_pub
		
		self.joy_sub = self.create_subscription(Joy, "joy", self.joy_callback, 10)

		self.pose = "default"
		
	def joy_callback(self, msg):
		buttons = msg.buttons
		
		if buttons[0]:
			self.pose = "default"
			self.timer_callback()
		elif buttons[1]:
			self.pose = "write"
			self.timer_callback()
			
	def timer_callback(self):

		joint_traj = JointTrajectory()
		joint_traj.header.frame_id = FRAME_ID
		joint_traj.header.stamp = self.get_clock().now().to_msg()
		joint_traj.joint_names = joints
		
		temp_point = JointTrajectoryPoint()
		if self.pose == "default":
			temp_point.positions = default_pose
		elif self.pose == "write":
			temp_point.positions = write_pose
		temp_point.time_from_start = Duration(sec=1, nanosec=0)
		
		joint_traj.points = [temp_point]
		
		self.joint_traj_pub.publish(joint_traj)
		self.get_logger().debug("Joint Trajectory Published")
		
	
def main():
	try: 
		rclpy.init()
		joint_trajectory_publisher = JointTrajectoryPublisher()		
		rclpy.spin(joint_trajectory_publisher)
	except KeyboardInterrupt:
		pass
	finally:	
		if rclpy.ok():
			joint_trajectory_publisher.destroy_node()
			rclpy.shutdown()

if __name__ == '__main__':
	main()

		