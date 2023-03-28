import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
import math
from mechaship_interfaces.msg import ClassificationArray, DetectionArray
from mechaship_interfaces.srv import Key, ThrottlePercentage, RGBColor


class MechashipNavigation(Node):
    def __init__(self):
        super().__init__(
            "mechaship_navigation_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.wall_safe_range = (
            self.get_parameter_or(
                "wall_safe_range",
                Parameter("wall_safe_range", Parameter.Type.DOUBLE, 5.0),
            )
            .get_parameter_value()
            .double_value
        )
        self.buoy_safe_range = (
            self.get_parameter_or(
                "buoy_safe_range",
                Parameter("buoy_safe_range", Parameter.Type.DOUBLE, 3.0),
            )
            .get_parameter_value()
            .double_value
        )
        self.image_width = (
            self.get_parameter_or(
                "image_width",
                Parameter("image_width", Parameter.Type.INTEGER, 400),
            )
            .get_parameter_value()
            .integer_value
        )

        self.get_logger().info("wall_safe_range: %s" % (str(self.wall_safe_range)))
        self.get_logger().info("buoy_safe_range: %s" % (str(self.buoy_safe_range)))

        self.buoy_subscription = self.create_subscription(
            ClassificationArray,
            "/Buoy",
            self.buoy_listener_callback,
            qos_profile_sensor_data,
        )
        self.wall_subscription = self.create_subscription(
            ClassificationArray,
            "/Wall",
            self.wall_listener_callback,
            qos_profile_sensor_data,
        )
        self.detection_subscription = self.create_subscription(
            DetectionArray,
            "/DetectionArray",
            self.detection_listener_callback,
            qos_profile_sensor_data,
        )

        self.buoy_subscription  # prevent unused variable warning
        self.wall_subscription  # prevent unused variable warning
        self.detection_subscription  # prevent unused variable warning

        self.create_timer(0.1, self.navigate)

        self.set_key_handler = self.create_client(Key, "/actuators/key/set")
        self.set_throttle_handler = self.create_client(
            ThrottlePercentage, "/actuators/throttle/set_percentage"
        )
        self.set_color_handler = self.create_client(RGBColor, "set_color")

        self.walls = []
        self.buoys = []

        self.isdocking = False
        self.camera_fov = 62.2

    def wall_listener_callback(self, data):
        # self.get_logger().info("wall cnt: %s" % (len(data.classifications)))
        self.walls.append(data.classifications)

        if len(self.walls) > 10:
            del self.walls[0]

    def buoy_listener_callback(self, data):
        # self.get_logger().info("buoy cnt: %s" % (len(data.classifications)))
        self.buoys.append(data.classifications)

        if len(self.buoys) > 10:
            del self.buoys[0]

    def detection_listener_callback(self, data):
        self.get_logger().info("detection cnt: %s" % (len(data.detections)))
        for detection in data.detections:
            if detection.name == "circle":
                color = RGBColor.Request()
                color.red = 255
                color.green = 0
                color.blue = 0
                self.set_color_handler.call_async(color)
                self.docking(detection)

            elif detection.name == "triangle":
                color = RGBColor.Request()
                color.red = 0
                color.green = 255
                color.blue = 0
                self.set_color_handler.call_async(color)
                self.docking(detection)

            elif detection.name == "square":
                color = RGBColor.Request()
                color.red = 0
                color.green = 0
                color.blue = 255
                self.set_color_handler.call_async(color)
                self.docking(detection)

    def navigate(self):
        if self.walls == None or self.buoys == None:
            return

        if self.isdocking:
            return

        cautions = self.get_cautions_map()

        forward_cautions = []
        forward_caution = []
        for caution in cautions[270:] + cautions[:90]:
            if len(forward_caution) > 0 and forward_caution[-1] != caution:
                forward_cautions.append(forward_caution)
                forward_caution = []
            forward_caution.append(caution)
        self.get_logger().info("forward_cautions: %s" % (forward_cautions))

        if forward_cautions == []:
            goal_angle = 90
        else:
            min_cautions = min(map(min, forward_cautions))
            goal_angle = 0
            goal_angles = []
            angle = 0
            for caution in forward_cautions:
                angle += len(caution)
                if caution[0] == min_cautions and len(caution) > len(goal_angles):
                    goal_angles = caution
                    goal_angle = int(angle - len(caution) / 2.0)
        self.get_logger().info("goal_angle: %s" % (str(goal_angle)))

        key = Key.Request()
        if goal_angle < 60:
            key.degree = 60
        elif goal_angle > 120:
            key.degree = 120
        else:
            key.degree = goal_angle

        throttle = ThrottlePercentage.Request()
        wall_distance = self.get_forward_wall_distance()
        if wall_distance < 1.5:
            throttle.percentage = 0
        else:
            throttle.percentage = 20

        self.set_key_handler.call_async(key)
        self.set_throttle_handler.call_async(throttle)

    def docking(self, detection):
        self.isdocking = True

        center_x = (detection.xmin + detection.xmax) / 2.0
        center_angle = int((center_x / self.image_width) * self.camera_fov + 60)

        key = Key.Request()
        key.degree = center_angle
        self.set_key_handler.call_async(key)

    def get_cautions_map(self):
        cautions = [0 for i in range(360)]
        # self.get_logger().info("cautions: %s" % (str(cautions)))

        for wall in self.walls:
            for cc in wall:
                if min(cc.ranges) <= self.wall_safe_range:
                    start_angle = int(math.degrees(cc.thetas[0]))
                    end_angle = int(math.degrees(cc.thetas[-1]))
                    for angle in range(start_angle, end_angle + 1):
                        cautions[angle] += 1
        # self.get_logger().info("cautions: %s" % (str(cautions)))

        for buoy in self.buoys:
            for cc in buoy:
                if min(cc.ranges) <= self.buoy_safe_range:
                    start_angle = int(math.degrees(cc.thetas[0]))
                    end_angle = int(math.degrees(cc.thetas[-1]))
                    for angle in range(start_angle, end_angle + 1):
                        cautions[angle] += 1
        # self.get_logger().info("cautions: %s" % (str(cautions)))

        return cautions

    def get_forward_wall_distance(self):
        forward_ranges = []
        for wall in self.walls:
            for cc in wall:
                for range, theta in zip(cc.ranges, cc.thetas):
                    if theta < math.radians(30 * math.pi) or theta > math.radians(330):
                        forward_ranges.append(range * math.cos(theta))

        if forward_ranges == []:
            distance = 5
        else:
            distance = sum(forward_ranges) / len(forward_ranges)
        self.get_logger().info("wall_distance: %s" % (str(distance)))

        return distance


def main(args=None):
    rclpy.init(args=args)
    node = MechashipNavigation()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
