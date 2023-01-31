import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import torch
from std_msgs.msg import Float32MultiArray


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

        self.subscription = self.create_subscription(
            Image, "Image", self.listener_callback, 0
        )
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()

        if torch.cuda.is_available():
            self.model = torch.hub.load(
                _yolov5_path, "custom", path=_model_path, source="local"
            )
            self.model.conf = _model_conf  # NMS 최소 정확도
            self.model.iou = _model_iou  # NMS 최소 IoU
            self.model.max_det = _model_max  # 이미지 당 최대 detection 개수
        else:
            raise CudaError()

    def listener_callback(self, data):
        origin_image = self.br.imgmsg_to_cv2(data, "bgr8")
        if len(origin_image):
            # BGR 이미지를 RGB 이미지로 변경
            rgb_img = cv2.cvtColor(origin_image, cv2.COLOR_BGR2RGB)

            results = self.model(rgb_img, size=self.batch_size)
            msg = Float32MultiArray()
            # xmin / ymin / xmax / ymax / confidence / class / name
            msg.data = results.pandas().xyxy[0].values.tolist()


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
