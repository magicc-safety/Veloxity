import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rosflight_sim_dir = get_package_share_directory("rosflight_sim")
    standalone_param_file = os.path.join(
        rosflight_sim_dir, "params", "standalone_sim_params.yaml")
    dynamics_param_file = os.path.join(
        rosflight_sim_dir, "params", "multirotor_dynamics.yaml")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Whether the nodes will use sim time or not",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Whether to launch the standalone RViz visualizer.",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription(
        [
            use_sim_time_arg,
            use_rviz_arg,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rosflight_sim_dir, "launch", "standalone_sim.launch.py")
                ),
                condition=IfCondition(use_rviz),
                launch_arguments={
                    "sim_aircraft_file": os.path.join("common_resource", "multirotor.dae")
                }.items(),
            ),
            Node(
                package="rosflight_sim",
                executable="rosflight_sil_manager",
                name="rosflight_sil_manager",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time, "use_timer": True}],
            ),
            Node(
                package="rosflight_sim",
                executable="sil_board",
                name="sil_board",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="rosflight_sim",
                executable="standalone_sensors",
                name="standalone_sensors",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}, dynamics_param_file],
            ),
            Node(
                package="rosflight_io",
                executable="rosflight_io",
                name="rosflight_io",
                output="screen",
                parameters=[{"udp": True, "use_sim_time": use_sim_time}],
            ),
            Node(
                package="rosflight_sim",
                executable="standalone_time_manager",
                name="standalone_time_manager",
                output="screen",
                condition=IfCondition(use_sim_time),
                parameters=[standalone_param_file],
            ),
            Node(
                package="rosflight_sim",
                executable="multirotor_forces_and_moments",
                name="multirotor_forces_and_moments",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}, dynamics_param_file],
            ),
            Node(
                package="rosflight_sim",
                executable="standalone_dynamics",
                name="standalone_dynamics",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}, dynamics_param_file],
            ),
        ]
    )
