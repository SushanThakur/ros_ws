from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(get_package_share_directory(
            "robotic-arm-urdf-min"), "urdf", "robotic-arm-urdf-min.urdf.xacro"),
        description="Absolute path to the robot URDF file"
    )

    # Set IGN_GAZEBO_RESOURCE_PATH for models
    env_var_resource = SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", os.path.join(get_package_prefix("robotic-arm-urdf-min"), "share"))

    # Process the Xacro file to generate URDF
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen"
    )

    # Start Ignition Gazebo with the updated world file
    world_file = os.path.join(get_package_share_directory("robotic-arm-urdf-min"), "worlds", "empty.sdf")
    start_ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '4', world_file],
        output='screen',
        name='ign_gazebo'
    )

    # Spawn the robot
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "robotic-arm-urdf-min", "-topic", "/robot_description", "-z", "0.1"],
        output="screen"
    )

    # Ensure spawn_robot starts after ign_gazebo
    spawn_robot_after_gazebo = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_ign_gazebo,
            on_start=[spawn_robot]
        )
    )

    return LaunchDescription([
        env_var_resource,
        model_arg,
        robot_state_publisher,
        start_ign_gazebo,
        spawn_robot_after_gazebo
    ])