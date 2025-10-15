from launch_ros.actions import Node
from launch import LaunchDescription

package_name = 'arm_controller'

def generate_launch_description():
	
	cmd_type_switcher_node = Node(
		package=package_name,
		executable="command_type_switcher",
	)

	twist_publisher_node = Node(
		package=package_name,
		executable="joy_twist_publisher",
	)

	joint_publisher_node = Node(
		package=package_name,
		executable="joy_joint_publisher",
	)
	
	joint_trajectory_pub_node = Node(
		package=package_name,
		executable="joint_trajectory_controller",
	)

	gripper_traj_pub_node = Node(
		package=package_name,
		executable="gripper_trajectory_controller"
	)

	joint_state_recorder_node = Node(
		package=package_name,
		executable="joint_state_recorder"
	)

	joint_state_player_node = Node(
		package=package_name,
		executable="joint_state_player"
	)
	
	return LaunchDescription(
		[
			cmd_type_switcher_node,
			twist_publisher_node,
			joint_publisher_node,
			joint_trajectory_pub_node,
			gripper_traj_pub_node,
			joint_state_recorder_node,
			joint_state_player_node,
		]
	)