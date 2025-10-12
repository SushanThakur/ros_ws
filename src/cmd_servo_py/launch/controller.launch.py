from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
	
	twist_publisher_node = Node(
		package="cmd_servo_py",
		executable="twist_publisher",
	)
	
	joint_trajectory_pub_node = Node(
		package="cmd_servo_py",
		executable="joint_trajectory_pub",
	)

	gripper_traj_pub_node = Node(
		package="cmd_servo_py",
		executable="gripper_traj_pub"
	)
	
	return LaunchDescription(
		[
			twist_publisher_node,
			joint_trajectory_pub_node,
			gripper_traj_pub_node,
		]
	)