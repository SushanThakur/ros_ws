from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
	
	cmd_type_switcher_node = Node(
		package="arm_controller_py",
		executable="command_type_switcher",
	)

	twist_publisher_node = Node(
		package="arm_controller_py",
		executable="joy_twist_publisher",
	)

	joint_publisher_node = Node(
		package="arm_controller_py",
		executable="joy_joint_publisher",
	)
	
	joint_trajectory_pub_node = Node(
		package="arm_controller_py",
		executable="joint_trajectory_controller",
	)

	gripper_traj_pub_node = Node(
		package="arm_controller_py",
		executable="gripper_trajectory_controller"
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