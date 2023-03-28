import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # navigation2 노드
    mechaship_navigation2_params = LaunchConfiguration(
        "mechaship_navigation2_params",
        default=os.path.join(
            get_package_share_directory("mechaship_example"),
            "param",
            "mechaship_navigation2.yaml",
        ),
    )
    mechaship_navigation2_arg = DeclareLaunchArgument(
        "mechaship_navigation2_params",
        default_value=mechaship_navigation2_params,
    )
    mechaship_navigation2 = Node(
        package="mechaship_example",
        executable="mechaship_navigation2_node",
        name="mechaship_navigation2_node",
        output="screen",
        emulate_tty=True,
        parameters=[mechaship_navigation2_params],
    )

    return LaunchDescription(
        [
            mechaship_navigation2_arg,
            mechaship_navigation2,
        ]
    )
