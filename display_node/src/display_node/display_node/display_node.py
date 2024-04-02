from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2


class DisplayNode(Node):
    POSE_PAIRS = [
        [1, 2],
        [1, 5],
        [2, 3],
        [3, 4],
        [5, 6],
        [6, 7],
        [1, 8],
        [8, 9],
        [9, 10],
        [1, 11],
        [11, 12],
        [12, 13],
        [1, 0],
        [0, 14],
        [14, 16],
        [0, 15],
        [15, 17],
        [2, 17],
        [5, 16],
    ]

    def __init__(self, pose_topic="/pose", camera_topic="/camera"):
        super().__init__("display_node")

        self.create_subscription(String, pose_topic, self._sub_pose_callback, 10)
        self.create_subscription(Image, camera_topic, self._sub_camera_callback, 10)
        self._cv_bridge = CvBridge()
        self._pose_buffer = []

    def _sub_camera_callback(self, image: Image):
        self.get_logger().info("Received: /camera")
        cv_image = self._cv_bridge.imgmsg_to_cv2(image)
        for pair in self.POSE_PAIRS:
            part_a = pair[0]
            part_b = pair[1]
            cv2.line(
                cv_image,
                (part_a[0], part_a[1]),
                (part_b[0], part_b[1]),
                (0, 255, 0),
                2,
            )

        cv2.imshow("Display Node", cv_image)
        cv2.waitKey(1)

    def _sub_pose_callback(self, pose: String):
        self.get_logger().info("Received: /pose")
        self._pose_buffer = pose.data
