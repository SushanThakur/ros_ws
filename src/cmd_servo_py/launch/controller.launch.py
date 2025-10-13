from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
	
	cmd_type_switcher_node = Node(
		package="cmd_servo_py",
		executable="cmd_type_switcher",
	)

	twist_publisher_node = Node(
		package="cmd_servo_py",
		executable="twist_publisher",
	)

	joint_publisher_node = Node(
		package="cmd_servo_py",
		executable="joint_publisher",
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
			cmd_type_switcher_node,
			twist_publisher_node,
			joint_publisher_node,
			joint_trajectory_pub_node,
			gripper_traj_pub_node,
		]
	)