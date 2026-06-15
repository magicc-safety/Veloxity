import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    rosflight_sim_dir = get_package_share_directory("rosflight_sim")
    dynamics_param_file = LaunchConfiguration("dynamics_param_file")
    standalone_param_file = os.path.join(
        rosflight_sim_dir, "params", "standalone_sim_params.yaml")

    firmware = LaunchConfiguration("firmware")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_vimfly = LaunchConfiguration("use_vimfly")
    use_builtin_rc = LaunchConfiguration("use_builtin_rc")
    use_rviz = LaunchConfiguration("use_rviz")
    veloxity_param_dir = LaunchConfiguration("veloxity_param_dir")

    is_c = PythonExpression(["'", firmware, "' == 'c'"])
    is_veloxity = PythonExpression(["'", firmware, "' == 'veloxity'"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "firmware",
            default_value="veloxity",
            choices=["c", "veloxity"],
            description="Firmware endpoint to run: upstream ROSflight C++ or Veloxity FFI.",
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
        DeclareLaunchArgument(
            "veloxity_param_dir",
            default_value=EnvironmentVariable(
                "VELOXITY_SIM_PARAM_DIR",
                default_value="/tmp/veloxity-sim-params/fixedwing",
            ),
            description="Writable runtime parameter directory for the Veloxity FFI firmware.",
        ),
        DeclareLaunchArgument(
            "dynamics_param_file",
            default_value=os.path.join(
                rosflight_sim_dir, "params", "anaconda_dynamics.yaml"),
            description="Fixed-wing dynamics parameter file.",
        ),
        SetEnvironmentVariable(
            "VELOXITY_SIM_PARAM_DIR",
            veloxity_param_dir,
            condition=IfCondition(is_veloxity),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rosflight_sim_dir, "launch", "standalone_sim.launch.py")
            ),
            condition=IfCondition(use_rviz),
            launch_arguments={
                "sim_aircraft_file": os.path.join("common_resource", "skyhunter.dae"),
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
                "service_exists_timeout_ms": 1000,
                "service_result_timeout_ms": 1000,
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
            package="veloxity_sil_board_shim",
            executable="veloxity_sil_board",
            name="veloxity_sil_board",
            output="screen",
            condition=IfCondition(is_veloxity),
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
            executable="fixedwing_forces_and_moments",
            name="fixedwing_forces_and_moments",
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
