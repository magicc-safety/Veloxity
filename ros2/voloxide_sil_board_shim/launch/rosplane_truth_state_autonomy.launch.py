import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rosplane_dir = get_package_share_directory("rosplane")
    autopilot_params = os.path.join(
        rosplane_dir, "params", "anaconda_autopilot_params.yaml")

    state_topic = LaunchConfiguration("state_topic")
    command_topic = LaunchConfiguration("command_topic")
    controller_command_topic = LaunchConfiguration("controller_command_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("state_topic", default_value="/sim/rosplane/state"),
        DeclareLaunchArgument("command_topic", default_value="/command"),
        DeclareLaunchArgument("controller_command_topic", default_value="/controller_command"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="rosplane",
            executable="controller",
            name="controller",
            output="screen",
            arguments=["default"],
            parameters=[autopilot_params, {"use_sim_time": use_sim_time}],
            remappings=[
                ("/estimated_state", state_topic),
                ("/command", command_topic),
            ],
        ),
        Node(
            package="rosplane",
            executable="path_follower",
            name="path_follower",
            parameters=[autopilot_params, {"use_sim_time": use_sim_time}],
            remappings=[
                ("/estimated_state", state_topic),
                ("/controller_command", controller_command_topic),
            ],
        ),
        Node(
            package="rosplane",
            executable="path_manager",
            name="path_manager",
            parameters=[autopilot_params, {"use_sim_time": use_sim_time}],
            remappings=[("/estimated_state", state_topic)],
        ),
        Node(
            package="rosplane",
            executable="path_planner",
            name="path_planner",
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=[("/estimated_state", state_topic)],
        ),
        Node(
            package="rosplane_sim",
            executable="sim_state_transcriber",
            name="rosplane_truth",
        ),
    ])
