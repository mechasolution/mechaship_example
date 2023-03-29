from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # imu_filter_madgwick_node
    imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick_node",
        parameters=[
            {"mag_bias_x": 0.0},
            {"mag_bias_y": 0.0},
            {"mag_bias_z": 0.0},
            {"gain": 0.001},
            {"fixed_frame": "imu_link"},
        ],
    )

    return LaunchDescription([imu_filter_node])
