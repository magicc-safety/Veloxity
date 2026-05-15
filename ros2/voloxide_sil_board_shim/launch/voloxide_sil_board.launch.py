from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="voloxide_sil_board_shim",
                executable="voloxide_sil_board",
                name="voloxide_sil_board",
                output="screen",
            ),
        ]
    )
