import os
import numpy as np
import datetime
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3

mag_arr = list()

CURRENT_BIAS = {
    "x": 0,
    "y": 0,
    "z": 0,
}


class MechashipImuMagHardIronNode(Node):
    def __init__(self):
        super().__init__("mechaship_imu_mag_hard_iron_node")
        self._mag_data = Vector3()
        self._is_first_data = False

        self.create_subscription(MagneticField, "imu/mag", self._mag_sub_callback, qos_profile_sensor_data)
        self._timer_hd = self.create_timer(5, self._mag_data_monitor)

    def _mag_data_monitor(self):
        if self._is_first_data == False:
            self.get_logger().warning('Waitting for Magnetic Field sensor data from "imu/mag" topic...')
        else:
            self._timer_hd.destroy()

    def _mag_sub_callback(self, data: MagneticField):
        if self._is_first_data == False:
            self._is_first_data = True
            self.get_logger().info("First Magnetic Field sensor data has been received!")
        self._mag_data = data.magnetic_field
        mag_arr.append(
            [
                self._mag_data.x + CURRENT_BIAS["x"],
                self._mag_data.y + CURRENT_BIAS["y"],
                self._mag_data.z + CURRENT_BIAS["z"],
            ]
        )


def calculate_magnetometer_bias(x, y, z):
    center_x = np.mean(x)
    center_y = np.mean(y)
    center_z = np.mean(z)

    expected_field_strength = 0  # 예시: 테슬라 단위

    bias_x = expected_field_strength - center_x
    bias_y = expected_field_strength - center_y
    bias_z = expected_field_strength - center_z

    return (bias_x, bias_y, bias_z)


def main(args=None):
    rclpy.init(args=args)
    node = MechashipImuMagHardIronNode()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        if len(mag_arr) == 0:
            node.get_logger().error("Magnetic Field sensor data has not been received!")
        else:
            time_str = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
            path_str = os.path.join(os.path.expanduser("~"), "mag_arr_" + time_str + ".csv")
            np.savetxt(path_str, np.array(mag_arr), delimiter=",")
            _xy_mag_arr_trans = np.transpose(np.array(mag_arr))
            _x_mag_arr = _xy_mag_arr_trans[0]
            _y_mag_arr = _xy_mag_arr_trans[1]
            _z_mag_arr = _xy_mag_arr_trans[2]

            # bias 계산
            bias_x, bias_y, bias_z = calculate_magnetometer_bias(_x_mag_arr, _y_mag_arr, _z_mag_arr)
            bias_x = CURRENT_BIAS["x"] + bias_x
            bias_y = CURRENT_BIAS["y"] + bias_y
            bias_z = CURRENT_BIAS["z"] + bias_z
            node.get_logger().info(
                f"\nChange Bias as bellow\nbias X : {bias_x:.8f}\nbias Y : {bias_y:.8f}\nbias Z : {bias_z:.8f}"
            )

            # 데이터 시각화 준비
            _fig = plt.figure()
            _ax = _fig.add_subplot(projection="3d")
            _ax.scatter(_x_mag_arr, _y_mag_arr, _z_mag_arr, c="r")

            # 정상 데이터 (50uT)
            u, v = np.mgrid[0 : 2 * np.pi : 50j, 0 : np.pi : 50j]
            x_normal = 0 + 0.00005 * np.cos(u) * np.sin(v)
            y_normal = 0 + 0.00005 * np.sin(u) * np.sin(v)
            z_normal = 0 + 0.00005 * np.cos(v)
            _ax.plot_surface(x_normal, y_normal, z_normal, color="red", alpha=0.2, label="normal data")

            # 산점도 그래프 보기 설정
            _x_max = max(abs(np.min(_x_mag_arr)), abs(np.max(_x_mag_arr)))
            _y_max = max(abs(np.min(_y_mag_arr)), abs(np.max(_y_mag_arr)))
            _z_max = max(abs(np.min(_z_mag_arr)), abs(np.max(_z_mag_arr)))
            _xyz_max = max(max(max(_x_max, _y_max), _z_max), 0.00005) + 0.00002

            _ax.set_xlim([-_xyz_max, _xyz_max])
            _ax.set_ylim([-_xyz_max, _xyz_max])
            _ax.set_zlim([-_xyz_max, _xyz_max])
            _ax.set_xlabel("X Mag (uT)")
            _ax.set_ylabel("Y Mag (uT)")
            _ax.set_zlabel("Z Mag (uT)")

            plt.show()

    except SystemExit:
        rclpy.logging.get_logger("Quitting").info("Done")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
