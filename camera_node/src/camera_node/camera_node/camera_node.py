from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class FrameNotAvailableError(Exception):
    def __init__(self):
        pass
    
    def __str__(self):
        return 'Failed to read the frame'


class CameraNode(Node):
    """
    カメラの画像を取得し、指定したトピックに画像を配信するノード。
    """

    def __init__(self,
                 capture: cv2.VideoCapture,
                 topic_name: str = '/camera',
                 publish_rate: float = 0.5):
        """
        ## Parameters

        - `capture`: カメラのキャプチャオブジェクト。
        - `topic_name`: 画像を配信するトピック名。
        - `publish_rate`: 画像の配信レート。
        """

        super().__init__('camera_node')
        self._capture   = capture
        self._camera_publisher = self.create_publisher(Image, topic_name, 10)
        self._timer     = self.create_timer(publish_rate, self._timer_callback)
        self._cv_bridge = CvBridge()

    def _timer_callback(self):
        """
        タイマーコールバック関数。
        カメラから画像を取得し、指定したトピックに画像を配信する。
        """

        # Read the frame
        has_frame, frame = self._capture.read()

        # Check if the frame is valid
        if not has_frame:
            self.get_logger().error('No frame')
            raise FrameNotAvailableError()

        # Convert the frame and publish it
        image = self._cv_bridge.cv2_to_imgmsg(frame)
        self._camera_publisher.publish(image)
        self.get_logger().info('Publish: /camera')
