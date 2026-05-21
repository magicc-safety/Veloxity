import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    rosflight_sim_dir = get_package_share_directory("rosflight_sim")
    dynamics_param_file = os.path.join(
        rosflight_sim_dir, "params", "multirotor_dynamics.yaml")
    standalone_param_file = os.path.join(
        rosflight_sim_dir, "params", "standalone_sim_params.yaml")

    firmware = LaunchConfiguration("firmware")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_vimfly = LaunchConfiguration("use_vimfly")
    use_builtin_rc = LaunchConfiguration("use_builtin_rc")
    use_rviz = LaunchConfiguration("use_rviz")

    is_c = PythonExpression(["'", firmware, "' == 'c'"])
    is_voloxide = PythonExpression(["'", firmware, "' == 'voloxide'"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "firmware",
            default_value="c",
            choices=["c", "voloxide"],
            description="Firmware endpoint to run: upstream ROSflight C++ or Voloxide FFI.",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_vimfly", default_value="false"),
        DeclareLaunchArgument(
            "use_builtin_rc",
            default_value="true",
            description="Launch ROSflight rc.py. Disable when a test publishes sim/RC directly.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Open the standalone RViz visualizer.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rosflight_sim_dir, "launch", "standalone_sim.launch.py")
            ),
            condition=IfCondition(use_rviz),
            launch_arguments={
                "sim_aircraft_file": os.path.join("common_resource", "multirotor.dae"),
            }.items(),
        ),
        Node(
            package="rosflight_sim",
            executable="rosflight_sil_manager",
            name="rosflight_sil_manager",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "use_timer": True,
                "service_result_timeout_ms": 100,
            }],
        ),
        Node(
            package="rosflight_sim",
            executable="sil_board",
            name="sil_board",
            output="screen",
            condition=IfCondition(is_c),
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="voloxide_sil_board_shim",
            executable="voloxide_sil_board",
            name="voloxide_sil_board",
            output="screen",
            condition=IfCondition(is_voloxide),
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
            executable="rc.py",
            condition=IfCondition(use_builtin_rc),
            parameters=[{"use_vimfly": use_vimfly, "use_sim_time": use_sim_time}],
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
    ])
