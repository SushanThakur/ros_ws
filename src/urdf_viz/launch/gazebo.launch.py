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
            "urdf_viz"), "urdf", "urdf_viz.urdf.xacro"),
        description="Absolute path to the robot URDF file"
    )

    env_var_resource = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", get_package_share_directory("urdf_viz"))

    # Process the Xacro file to generate URDF
    robot_description = Command(["xacro ", LaunchConfiguration("model")])

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": ParameterValue(robot_description, value_type=str), "use_sim_time": True}],
        output="screen"
    )

    # Start Gazebo with the updated world file
    world_file = os.path.join(get_package_share_directory("urdf_viz"), "worlds", "world.sdf")
    start_ign_gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world_file],
        output='screen',
        name='ign_gazebo',
        additional_env={'GZ_SIM_RESOURCE_PATH': os.path.join(get_package_prefix("urdf_viz"), "share")}
    )

    # Spawn the robot
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "urdf_viz", "-topic", "/robot_description", "-z", "0.1"],
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