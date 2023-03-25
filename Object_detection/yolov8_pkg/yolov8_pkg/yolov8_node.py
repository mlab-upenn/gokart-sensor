import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ultralytics import YOLO
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D
from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import cv2
import random
from ament_index_python.packages import get_package_share_directory
import os


class Yolov8_node(Node):
    def __init__(self):
        # here, super().__init__(<node_name>), while the node_name should be the same as provided in yaml file
        super().__init__("yolov8_node")

        # load yolov8 model
        self.declare_parameters(
            namespace='',
            parameters=[
                ('model_path', None),
                ('device', None),
                ('threshold', None),
                ('class_num', None),
                ('image_topic', None)
            ])
        model = self.get_parameter("model_path").get_parameter_value().string_value
        model = os.path.join(get_package_share_directory('yolov8_pkg'), 'config', model)
        device = self.get_parameter("device").get_parameter_value().string_value
        self.threshold = self.get_parameter("threshold").get_parameter_value().double_value
        # k = self.get_parameter("color_map").get_parameter_value().type
        # t = self.get_parameter("color_map").get_parameter_value()
        # self.get_logger().info(f"{k}, {t}")
        self.color_map = self._init_color_map(self.get_parameter("class_num").get_parameter_value().integer_value)
        self.yolo = YOLO(model)
        self.yolo.to(device)
        self.get_logger().info(f"Successfully load Yolov8 model from: {model}, model on device: {device}")

        # cv bridge to transfer between ros raw image and opencv
        self.cv_bridge = CvBridge()

        # subscribe to camera
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self._pub = self.create_publisher(Detection2DArray, "detections", 10)
        self._dbg_pub = self.create_publisher(Image, "dbg_image", 10)
        self._sub = self.create_subscription(
            Image, self.image_topic, self.image_cb,
            qos_profile_sensor_data
        )

    def _init_color_map(self, cls_num):
        color_map = {}
        for i in range(cls_num):
            color_map[i] = [round(random.random()*255), round(random.random()*255), round(random.random()*255)]
        return color_map

    def image_cb(self, msg: Image) -> None:
        # convert image + predict
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
        results = self.yolo.predict(source=cv_image, verbose=False, stream=True)

        # create detections msg
        detections_msg = Detection2DArray()
        detections_msg.header = msg.header

        for result in results:
            result = result.cpu()
            for b in result.boxes:
                labelId = int(b.cls)
                label = self.yolo.names[int(b.cls)]
                score = float(b.conf)
                if score < self.threshold:
                    continue
                detection = Detection2D()

                box = b.xywh[0]

                # get boxes values
                detection.bbox.center.x = float(box[0])
                detection.bbox.center.y = float(box[1])
                detection.bbox.size_x = float(box[2])
                detection.bbox.size_y = float(box[3])

                # get hypothesis
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.id = label
                hypothesis.score = score
                detection.results.append(hypothesis)

                # draw boxes for debug
                color = self.color_map[labelId]

                min_pt = (round(detection.bbox.center.x - detection.bbox.size_x / 2.0),
                          round(detection.bbox.center.y - detection.bbox.size_y / 2.0))
                max_pt = (round(detection.bbox.center.x + detection.bbox.size_x / 2.0),
                          round(detection.bbox.center.y + detection.bbox.size_y / 2.0))
                cv2.rectangle(cv_image, min_pt, max_pt, color, 2)

                label = "{} {:.3f}".format(label, score)
                pos = (min_pt[0] + 5, min_pt[1] + 20)
                font = cv2.FONT_HERSHEY_SIMPLEX
                thickness = 3
                cv2.putText(cv_image, label, pos, font, 1, color, thickness, cv2.LINE_AA)

                # append msg
                detections_msg.detections.append(detection)

        # publish detections and dbg image
        self._pub.publish(detections_msg)
        self._dbg_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_image, encoding=msg.encoding))



def main(args=None):
    rclpy.init(args=args)
    node = Yolov8_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()