import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir


from launch_ros.actions import Node, PushRosNamespace, LifecycleNode


def generate_launch_description():
    # imu_filter_madgwick_node
    imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick_node",
        parameters=[
            {"mag_bias_x": 0},
            {"mag_bias_y": 0},
            {"mag_bias_z": 0},
            {"gain": 0.001},
        ],
    )

    return LaunchDescription(
        [
            imu_filter_node,
        ]
    )
