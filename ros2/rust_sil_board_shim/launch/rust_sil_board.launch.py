from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rust_sil_board_shim",
                executable="rust_sil_board",
                name="rust_sil_board",
                output="screen",
            ),
        ]
    )
