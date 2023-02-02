import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from numpy.polynomial import Polynomial
from mechaship_interfaces.msg import Classification, ClassificationArray


class MechashipClassify(Node):
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

        self.buoy_publisher = self.create_publisher(
            ClassificationArray, "Buoy", qos_profile
        )
        self.wall_publisher = self.create_publisher(
            ClassificationArray, "Wall", qos_profile
        )

    def listener_callback(self, data):
        self.get_logger().info("ranges cnt: %s" % (len(data.ranges)))
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

        # 극좌표계 적분해서 기울기로 그룹핑
        polar_coordinates_group = self.grouping_with_differential(rhos, thetas, 5)

        # rho로 분리하는 방법
        # polar_coordinates_group = self.regrouping_with_max_rho(polar_coordinates_group, 0.1)
        # buoy, wall = self.classify_with_rho(polar_coordinates_group, 10, 0.3)

        # 최소차승법으로 분리하는 방법
        buoy, wall = self.classify_with_LSM(polar_coordinates_group, 10, 100)

        self.buoy_publisher.publish(buoy)
        self.wall_publisher.publish(wall)

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
        buoy = Classification()
        wall = Classification()

        for polar_coordinates in polar_coordinates_group:
            length = len(polar_coordinates)
            if length < min_length:
                continue

            rhos = list(map(lambda x: x[0], polar_coordinates))
            min_rho_index = rhos.index(min(rhos))
            if min_rho_index == 0 or min_rho_index == length - 1:
                wall.ranges.append(polar_coordinates[0])
                wall.thetas.append(polar_coordinates[1])
                continue

            left_max_rho = max(rhos[:min_rho_index])
            right_max_rho = max(rhos[min_rho_index:])
            if abs(left_max_rho - right_max_rho) > 0.07:
                wall.ranges.append(polar_coordinates[0])
                wall.thetas.append(polar_coordinates[1])
                continue

            if index_range == 0:
                if min_rho_index == int(len(polar_coordinates) * 0.5 - 1):
                    buoy.ranges.append(polar_coordinates[0])
                    buoy.thetas.append(polar_coordinates[1])
                else:
                    wall.ranges.append(polar_coordinates[0])
                    wall.thetas.append(polar_coordinates[1])

            elif index_range < 1:
                min_index = (len(polar_coordinates) - 1) * (0.5 - index_range)
                max_index = (len(polar_coordinates) - 1) * (0.5 + index_range)
                if min_index < min_rho_index < max_index:
                    buoy.ranges.append(polar_coordinates[0])
                    buoy.thetas.append(polar_coordinates[1])
                else:
                    wall.ranges.append(polar_coordinates[0])
                    wall.thetas.append(polar_coordinates[1])

            elif index_range >= 1:
                min_index = len(polar_coordinates) * 0.5 - 1 - index_range
                max_index = len(polar_coordinates) * 0.5 - 1 + index_range
                if min_index < min_rho_index < max_index:
                    buoy.ranges.append(polar_coordinates[0])
                    buoy.thetas.append(polar_coordinates[1])
                else:
                    wall.ranges.append(polar_coordinates[0])
                    wall.thetas.append(polar_coordinates[1])

        return buoy, wall

    def classify_with_LSM(self, polar_coordinates_group, min_length=0, max_length=100):
        buoy = ClassificationArray()
        wall = ClassificationArray()

        for polar_coordinates in polar_coordinates_group:
            length = len(polar_coordinates)
            if length < min_length or length > max_length:
                continue

            coords = Classification()

            coords.ranges = list(map(lambda x: x[0], polar_coordinates))
            coords.thetas = list(map(lambda x: x[1], polar_coordinates))

            # 최소 차승법
            fx = Polynomial.fit(coords.thetas, coords.ranges, deg=2).convert()
            axis = fx.coef[1] / (-2.0 * fx.coef[2])
            self.get_logger().info("fx: %s" % (str(fx)))
            self.get_logger().info("axis: %s" % (str(axis)))

            center_theta = coords.thetas[int(len(coords.thetas) / 2.0)]
            if fx.coef[2] > 0 and center_theta * 0.9 < axis < center_theta * 1.1:
                buoy.classifications.append(coords)
            else:
                wall.classifications.append(coords)

        return buoy, wall


def main(args=None):
    rclpy.init(args=args)
    node = MechashipClassify()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
