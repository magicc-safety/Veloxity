from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="veloxity_sil_board_shim",
                executable="veloxity_sil_board",
                name="veloxity_sil_board",
                output="screen",
            ),
        ]
    )
