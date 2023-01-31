import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from numpy.polynomial import Polynomial
import cv2


class MechashipClassifyNode(Node):
    def __init__(self):
        super().__init__("mechaship_classify_node")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1,
        )

        self.scan_subscription = self.create_subscription(
            LaserScan, "/scan", self.listener_callback, qos_profile
        )
        self.scan_subscription  # prevent unused variable warning

        # self.scan_points_pub_handler = self.create_publisher(
        #     PointCloud, "scan_points", 10
        # )

    def listener_callback(self, data):
        self.get_logger().info("ranges cnt: %s" % (len(data.ranges)))
        self.get_logger().info("intensities cnt: %s" % (len(data.intensities)))
        self.get_logger().info("rad min: %s" % (math.degrees(data.angle_min)))
        self.get_logger().info("rad max: %s" % (math.degrees(data.angle_max)))
        self.get_logger().info("range min: %s" % (data.range_min))
        self.get_logger().info("range max: %s" % (data.range_max))

        rhos = []  # 거리(M)
        thetas = []  # 각도(radian)
        angle = data.angle_min
        for scan_data in data.ranges:
            if (scan_data != 0) and (not math.isinf(scan_data)):
                rhos.append(scan_data)
                thetas.append(angle)
            angle += data.angle_increment

        # 극좌표계 적분해서 기울기로 리스트 분할하고 시각화
        polar_coordinates_group = self.grouping_with_differential(rhos, thetas, 5)

        # rho로 분리하는 방법
        # polar_coordinates_group = self.regrouping_with_max_rho(polar_coordinates_group, 0.1)
        # self.show_polar_coordinates(polar_coordinates_group, (100, 100), True)
        # buoy, wall = self.classify_with_rho(polar_coordinates_group, 10, 0.3)
        # self.show_polar_coordinates(buoy, (100, 100), True)

        # 최소차승법으로 분리하는 방법
        buoy, wall = self.classify_with_LSM(polar_coordinates_group, 10, 100)
        self.show_polar_coordinates(buoy, (100, 100), True)
        # self.show_polar_coordinates(wall, (100, 100), True)
        self.get_logger().info("polar cnt: %s" % (len(polar_coordinates_group)))

        # 데카르트 좌표계로 변환하고 시각화
        cartesian_coordinates = self.convert_scans_to_cartesian_coordinates(
            data, (100, -90)  # scale, turn_degree
        )
        self.show_cartesian_coordinates(
            cartesian_coordinates, (350, 350, 100)  # center_x, center_y, scale
        )
        self.get_logger().info("cartesian cnt: %s" % (len(cartesian_coordinates)))

        cv2.waitKey(1)

    def convert_scans_to_cartesian_coordinates(self, scans, scale=(100, 0)):
        # 각도, 거리 -> 좌표로 변환
        points = []
        angle = scans.angle_min
        for point in scans.ranges:
            if angle > scans.angle_max:
                break

            if math.isinf(point) or point < scans.range_min or point > scans.range_max:
                angle += scans.angle_increment
                continue

            turn_radians = math.radians(scale[1])
            point_x = int(math.cos(-angle + turn_radians) * point * scale[0])
            point_y = int(math.sin(-angle + turn_radians) * point * scale[0])
            angle += scans.angle_increment

            points.append((point_x, point_y))

        return list(reversed(points))

    def grouping_with_differential(self, rhos, thetas, threshold):
        rhos_df = np.r_[0, np.diff(rhos)] / 0.1

        polar_coordinates_group = []
        polar_coordinates = []
        for rho, theta, rho_df in zip(rhos, thetas, rhos_df):
            if abs(rho_df) > threshold:
                polar_coordinates_group.append(polar_coordinates)
                polar_coordinates = []

            polar_coordinates.append((rho, theta))

        return polar_coordinates_group

    def regrouping_with_max_rho(self, polar_coordinates_group, index_range=0):
        # index_range : 0~1
        polar_coordinates_regroup = []
        for polar_coordinates in polar_coordinates_group[2:]:
            rhos = list(map(lambda x: x[0], polar_coordinates))
            max_rho_index = rhos.index(max(rhos))

            if index_range == 0:
                if max_rho_index != 0 and max_rho_index != len(polar_coordinates) - 1:
                    polar_coordinates_regroup.append(polar_coordinates[:max_rho_index])
                    polar_coordinates_regroup.append(polar_coordinates[max_rho_index:])
                else:
                    polar_coordinates_regroup.append(polar_coordinates)

            elif index_range < 1:
                min_index = (len(polar_coordinates) - 1) * index_range
                max_index = (len(polar_coordinates) - 1) * (1 - index_range)
                if max_rho_index > min_index and max_rho_index < max_index:
                    polar_coordinates_regroup.append(polar_coordinates[:max_rho_index])
                    polar_coordinates_regroup.append(polar_coordinates[max_rho_index:])
                else:
                    polar_coordinates_regroup.append(polar_coordinates)

        return polar_coordinates_regroup

    def classify_with_rho(self, polar_coordinates_group, min_length=0, index_range=0):
        # index_range : 0~1
        buoy = []
        wall = []

        for polar_coordinates in polar_coordinates_group:
            length = len(polar_coordinates)
            if length < min_length:
                continue

            rhos = list(map(lambda x: x[0], polar_coordinates))
            min_rho_index = rhos.index(min(rhos))
            if min_rho_index == 0 or min_rho_index == length - 1:
                wall.append(polar_coordinates)
                continue

            left_max_rho = max(rhos[:min_rho_index])
            right_max_rho = max(rhos[min_rho_index:])
            if abs(left_max_rho - right_max_rho) > 0.07:
                wall.append(polar_coordinates)
                continue

            if index_range == 0:
                if min_rho_index == int(len(polar_coordinates) * 0.5 - 1):
                    buoy.append(polar_coordinates)
                else:
                    wall.append(polar_coordinates)

            elif index_range < 1:
                min_index = (len(polar_coordinates) - 1) * (0.5 - index_range)
                max_index = (len(polar_coordinates) - 1) * (0.5 + index_range)
                if min_index < min_rho_index < max_index:
                    buoy.append(polar_coordinates)
                else:
                    wall.append(polar_coordinates)

            elif index_range >= 1:
                min_index = len(polar_coordinates) * 0.5 - 1 - index_range
                max_index = len(polar_coordinates) * 0.5 - 1 + index_range
                if min_index < min_rho_index < max_index:
                    buoy.append(polar_coordinates)
                else:
                    wall.append(polar_coordinates)

        return buoy, wall

    def classify_with_LSM(self, polar_coordinates_group, min_length=0, max_length=100):
        buoy = []
        wall = []

        for polar_coordinates in polar_coordinates_group:
            length = len(polar_coordinates)
            if length < min_length or length > max_length:
                continue

            rhos = list(map(lambda x: x[0], polar_coordinates))
            thetas = list(map(lambda x: x[1], polar_coordinates))

            # 최소 차승법
            fx = Polynomial.fit(thetas, rhos, deg=2).convert()
            axis = fx.coef[1] / (-2.0 * fx.coef[2])
            self.get_logger().info("fx: %s" % (str(fx)))
            self.get_logger().info("axis: %s" % (str(axis)))

            center_theta = thetas[int(len(thetas) / 2.0)]
            self.get_logger().info("center_theta: %s" % (str(center_theta)))
            if fx.coef[2] > 0 and center_theta * 0.9 < axis < center_theta * 1.1:
                buoy.append(polar_coordinates)
            else:
                wall.append(polar_coordinates)

        return buoy, wall

    def show_polar_coordinates(self, polar_coordinates, scale=(100, 100), iscolor=True):
        colors = [
            (0, 0, 255),
            (0, 255, 0),
            (255, 0, 0),
            (0, 255, 255),
            (255, 0, 255),
            (255, 255, 0),
        ]

        height = int(3.5 * scale[0])
        width = int(2 * math.pi * scale[1])
        empty_image = np.zeros((height, width, 3), np.uint8)  # (세로, 가로)
        for idx, polar_coordinate in enumerate(polar_coordinates):
            if iscolor:
                color = colors[idx % (len(colors))]
            else:
                color = (255, 255, 255)
            for rho, theta in polar_coordinate:
                empty_image[int(rho * scale[0])][int(theta * scale[1])] = color

        cv2.imshow("polar_coordinates", np.flip(empty_image, axis=0))

    def show_cartesian_coordinates(self, cartesian_coordinates, scale=(100, 100, 50)):
        image_size = int(7 * scale[2])
        self.get_logger().info("image_size: %s" % (image_size))

        center_point = (scale[0], scale[1])
        empty_image = np.zeros((image_size, image_size, 3), np.uint8)  # (세로, 가로)
        cv2.line(empty_image, center_point, center_point, (255, 0, 0), 3)
        for cartesian_coordinate in cartesian_coordinates:
            point_x = cartesian_coordinate[0] + scale[0]
            point_y = cartesian_coordinate[1] + scale[1]
            cv2.line(
                empty_image, (point_x, point_y), (point_x, point_y), (0, 0, 255), 2
            )

        cv2.imshow("cartesian_coordinates", empty_image)


def main(args=None):
    rclpy.init(args=args)
    node = MechashipClassifyNode()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
