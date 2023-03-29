import os
import numpy as np
import datetime
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math


import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from transforms3d import euler


class MechashipImuQuatToEuler(Node):
    def __init__(self):
        super().__init__("mechaship_imu_quat_to_euler")
        self._is_first_data = False

        self.create_subscription(Imu, "imu/data", self._imu_sub_callback, qos_profile_sensor_data)
        self._timer_hd = self.create_timer(5, self._mag_data_monitor)

    def _mag_data_monitor(self):
        if self._is_first_data == False:
            self.get_logger().warning('Waitting for "imu/data" topic...')
        else:
            self._timer_hd.destroy()

    def _imu_sub_callback(self, data: Imu):
        self._is_first_data = True

        quat_data = data.orientation
        x, y, z = euler.quat2euler([quat_data.w, quat_data.x, quat_data.y, quat_data.z], "sxyz")
        x = 359 - ((x * 180 / math.pi + 180) % 360) % 360
        y = 359 - ((y * 180 / math.pi + 180) % 360) % 360
        z = 359 - ((z * 180 / math.pi + 180) % 360) % 360
        print(f"x:{x}\ny:{y}\nz:{z}\n\n")


def main(args=None):
    rclpy.init(args=args)
    node = MechashipImuQuatToEuler()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        rclpy.logging.get_logger("Quitting").info("Done")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
