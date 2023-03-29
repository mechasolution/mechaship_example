import glob
import os
from setuptools import setup

package_name = "mechaship_example"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob.glob("launch/*.launch.py"),
        ),
        (os.path.join("share", package_name, "param"), glob.glob("param/*.yaml")),
        (os.path.join("share", package_name, "model"), glob.glob("model/*.pt")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sydney Kim",
    maintainer_email="sydney@mechasolution.com",
    description="ROS2 Example for Mechasolution Autoship 2023 Project",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mechaship_classify_node = mechaship_example.mechaship_classify_node:main",
            "mechaship_classify_sub_node = mechaship_example.mechaship_classify_sub_node:main",
            "mechaship_detect_node = mechaship_example.mechaship_detect_node:main",
            "mechaship_detect_sub_node = mechaship_example.mechaship_detect_sub_node:main",
            "mechaship_navigation_node = mechaship_example.mechaship_navigation_node:main",
            "mechaship_imu_mag_hard_iron_node = mechaship_example.mechaship_imu_mag_hard_iron_node:main",
            "mechaship_imu_quat_to_euler = mechaship_example.mechaship_imu_quat_to_euler:main",
        ],
    },
)
