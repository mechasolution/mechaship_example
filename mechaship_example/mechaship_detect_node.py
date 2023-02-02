import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import torch
from mechaship_interfaces.msg import Detection, DetectionArray


class CudaError(Exception):
    def __str__(self):
        return "CUDA 설치 여부 확인이 필요합니다!"


class MechashipDetect(Node):
    def __init__(self):
        super().__init__(
            "mechaship_detect_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        _yolov5_path = (
            self.get_parameter_or(
                "yolov5_path",
                Parameter("yolov5_path", Parameter.Type.STRING, "/home/jetson/yolov5"),
            )
            .get_parameter_value()
            .string_value
        )
        _model_path = (
            self.get_parameter_or(
                "model_path",
                Parameter("model_path", Parameter.Type.STRING, "/home/jetson/best.pt"),
            )
            .get_parameter_value()
            .string_value
        )
        _model_conf = (
            self.get_parameter_or(
                "model_conf",
                Parameter("model_conf", Parameter.Type.DOUBLE, 0.25),
            )
            .get_parameter_value()
            .double_value
        )
        _model_iou = (
            self.get_parameter_or(
                "model_iou",
                Parameter("model_iou", Parameter.Type.DOUBLE, 0.45),
            )
            .get_parameter_value()
            .double_value
        )
        _model_max = (
            self.get_parameter_or(
                "model_max",
                Parameter("model_max", Parameter.Type.INTEGER, 1000),
            )
            .get_parameter_value()
            .integer_value
        )
        self.batch_size = (
            self.get_parameter_or(
                "batch_size", Parameter("batch_size", Parameter.Type.INTEGER, 300)
            )
            .get_parameter_value()
            .integer_value
        )

        self.get_logger().info("yolov5_path: %s" % (_yolov5_path))
        self.get_logger().info("model_path: %s" % (_model_path))
        self.get_logger().info("model_conf: %s" % (_model_conf))
        self.get_logger().info("model_iou: %s" % (_model_iou))
        self.get_logger().info("model_max: %s" % (_model_max))
        self.get_logger().info("batch_size: %s" % (self.batch_size))

        if torch.cuda.is_available():
            self.model = torch.hub.load(
                _yolov5_path, "custom", path=_model_path, source="local"
            )
            self.model.conf = _model_conf  # NMS 최소 정확도
            self.model.iou = _model_iou  # NMS 최소 IoU
            self.model.max_det = _model_max  # 이미지 당 최대 detection 개수
        else:
            raise CudaError()

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

            results = self.model(rgb_img, size=self.batch_size)
            msg = DetectionArray()
            msg.header.frame_id = data.header.frame_id
            msg.header.stamp = super().get_clock().now().to_msg()
            msg.preprocess = results.t[0]
            msg.inference = results.t[1]
            msg.nms = results.t[2]

            results_list = results.xyxy[0].tolist()
            msg.detection_count = len(results_list)
            for result in results_list:
                detection_msg = Detection()
                detection_msg.xmin = result[0]
                detection_msg.ymin = result[1]
                detection_msg.xmax = result[2]
                detection_msg.ymax = result[3]
                detection_msg.confidence = result[4]
                detection_msg.class_id = int(result[5])
                detection_msg.name = self.model.names[result[5]]
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
