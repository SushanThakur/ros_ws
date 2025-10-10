from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='moveit_servo',
            executable='servo_node',
            name='servo_node',
            output='screen',
            parameters=[
                {'command_in_type': 'unitless'},
                {'joint_topic': '/servo_node/delta_joint_cmds'},
                {'moveit_servo.move_group_name': 'robotic_arm'},
                {'robot_description': open('path/to/your_robot.urdf').read()},
                {'robot_description_semantic': open('path/to/your_robot.srdf').read()},
                'path/to/kinematics.yaml'
            ]
        )
    ])