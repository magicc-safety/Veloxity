import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import FindExecutable, LaunchConfiguration


def generate_launch_description():
    rosflight_sim_dir = get_package_share_directory("rosflight_sim")
    default_param_file = os.path.join(
        rosflight_sim_dir,
        "params",
        "fixedwing_firmware.yaml",
    )

    param_file = LaunchConfiguration("param_file")
    write_delay_s = LaunchConfiguration("write_delay_s")

    init_firmware = ExecuteProcess(
        cmd=[[
            FindExecutable(name="ros2"),
            " service call /param_load_from_file rosflight_msgs/srv/ParamFile ",
            "\"{filename: ",
            param_file,
            "}\"",
            " && ",
            FindExecutable(name="ros2"),
            " service call /calibrate_imu std_srvs/srv/Trigger",
            " && ",
            FindExecutable(name="ros2"),
            " service call /calibrate_baro std_srvs/srv/Trigger",
            " && sleep ",
            write_delay_s,
            " && ",
            FindExecutable(name="ros2"),
            " service call /param_write std_srvs/srv/Trigger",
        ]],
        shell=True,
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "param_file",
            default_value=default_param_file,
            description="Fixed-wing firmware parameter file to load through rosflight_io.",
        ),
        DeclareLaunchArgument(
            "write_delay_s",
            default_value="10",
            description="Seconds to wait after calibration before writing firmware parameters.",
        ),
        init_firmware,
    ])
