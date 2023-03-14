import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO, checks
from mechaship_interfaces.msg import Detection, DetectionArray
from glob import glob
from os.path import join, exists
from ament_index_python.packages import get_package_share_directory


class ModelError(Exception):
    def __str__(self):
        return "YOLO 모델 파일이 확인되지 않습니다. 모델 파일의 경로와 파일명을 확인해주세요!"


class MechashipDetect(Node):
    def __init__(self):
        super().__init__(
            "mechaship_detect_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        _model_dir = join(get_package_share_directory(__package__), "model")
        _model_file_name = (
            self.get_parameter_or(
                "model_file_name",
                Parameter(
                    "model_file_name",
                    Parameter.Type.STRING,
                    "",
                ),
            )
            .get_parameter_value()
            .string_value
        )
        if (_model_file_name) and (exists(join(_model_dir, _model_file_name))):
            _model_path = join(_model_dir, _model_file_name)
        elif glob(join(_model_dir, "*.pt")):
            _model_path = glob(join(_model_dir, "*.pt"))[0]
        else:
            raise ModelError()

        self.model_conf = (
            self.get_parameter_or(
                "model_conf",
                Parameter("model_conf", Parameter.Type.DOUBLE, 0.25),
            )
            .get_parameter_value()
            .double_value
        )

        self.get_logger().info("model_path: %s" % (_model_path))
        self.get_logger().info("model_conf: %s" % (self.model_conf))

        checks()
        self.model = YOLO(_model_path)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1,
        )

        self.image_subscription = self.create_subscription(
            Image, "/Image", self.listener_callback, qos_profile
        )
        self.image_subscription  # prevent unused variable warning
        self.br = CvBridge()

        self.detection_publisher = self.create_publisher(
            DetectionArray, "DetectionArray", qos_profile
        )

    def listener_callback(self, data):
        origin_image = self.br.imgmsg_to_cv2(data, "bgr8")
        if len(origin_image):
            # BGR 이미지를 RGB 이미지로 변경
            rgb_img = cv2.cvtColor(origin_image, cv2.COLOR_BGR2RGB)

            results = self.model.predict(source=rgb_img, conf=self.model_conf)[0]
            msg = DetectionArray()
            msg.header.frame_id = data.header.frame_id
            msg.header.stamp = super().get_clock().now().to_msg()
            msg.preprocess = results.speed["preprocess"]
            msg.inference = results.speed["inference"]
            msg.postprocess = results.speed["postprocess"]

            results_list = results.boxes.xyxy
            msg.detection_count = len(results_list)
            for result in results_list:
                detection_msg = Detection()
                detection_msg.xmin = result[0]
                detection_msg.ymin = result[1]
                detection_msg.xmax = result[2]
                detection_msg.ymax = result[3]
                detection_msg.confidence = result[4]
                detection_msg.class_id = int(result[5])
                detection_msg.name = result.names[int(result[5])]
                msg.detections.append(detection_msg)

            self.get_logger().info(str(msg))
            self.detection_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MechashipDetect()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
