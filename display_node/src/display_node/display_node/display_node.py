from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np


class DisplayNode(Node):
    """
    画像とポーズ情報を受信し、画像にポーズを描画するノード。
    転倒判定も行う。
    """

    """OpenPose のキーポイントのペア"""
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

    def __init__(self, pose_topic="/pose", camera_topic="/camera", fall_threshold=0.5):
        """
        ## Parameters

        - pose_topic: ポーズ情報を受信するトピック名
        - camera_topic: カメラ画像を受信するトピック名
        - fall_threshold: 転倒判定の閾値
        """

        super().__init__("display_node")
        self.create_subscription(String, pose_topic, self._sub_pose_callback, 10)
        self.create_subscription(Image, camera_topic, self._sub_camera_callback, 10)

        """CvBridge インスタンス"""
        self._cv_bridge = CvBridge()

        """ポーズ情報をバッファリングするリスト"""
        self._pose_buffer = []

        """転倒判定の閾値"""
        self._fall_threshold = fall_threshold

    def _sub_camera_callback(self, image: Image):
        """
        カメラ画像を受信したときのコールバック関数。
        画像にポーズを描画し、表示する。
        """

        self.get_logger().info("Received: /camera")
        cv_image = self._cv_bridge.imgmsg_to_cv2(image)

        self._draw_pose(cv_image)
        self._put_text(cv_image)

        cv2.imshow("Display Node", cv_image)
        cv2.waitKey(1)

    def _sub_pose_callback(self, pose: String):
        """
        ポーズ情報を受信したときのコールバック関数。
        ポーズ情報をバッファリングする。
        """

        self.get_logger().info("Received: /pose")

        # `/pose` トピックと `/camera` トピックの受信は非同期であるため、
        # バッファリングすることで参照できるようにする
        self._pose_buffer = eval(pose.data)
    def _draw_pose(self, image):
        """画像にポーズを描画する。"""

        for person in self._pose_buffer:
            for pair in self.POSE_PAIRS:
                part_a_index = pair[0]
                part_b_index = pair[1]
                part_a_pos = person[part_a_index]
                part_b_pos = person[part_b_index]

                part_a_pos = (int(part_a_pos[0]), int(part_a_pos[1]))
                part_b_pos = (int(part_b_pos[0]), int(part_b_pos[1]))

                # 座標が 0 に近い場合は描画しない
                if (part_a_pos[0] == 0 and part_a_pos[1] == 0) or (
                    part_b_pos[0] == 0 and part_b_pos[1] == 0
                ):
                    continue

                cv2.line(
                    image,
                    part_a_pos,
                    part_b_pos,
                    (0, 255, 0),
                    2,
                )

    def _put_text(self, image):
        """画像にテキストを描画する処理はここにまとめて記述する。"""

        # 転倒判定の結果を描画
        if self._is_fallen(image.shape[0], self._pose_buffer):
            cv2.putText(
                image,
                "FALLEN",
                (10, 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

    def _is_fallen(self, frame_height, keypoints):
        """
        転倒判定を行う。
        閾値は画像上部からの割合で指定する。
        複数人の場合は、一人でも転倒していれば True を返す。
        """

        if keypoints == []:
            return False

        threshold_height = self._fall_threshold * frame_height

        for person in keypoints:
            flag = True
            for part in person:
                if part[0] == 0.0 and part[1] == 0.0:
                    continue
                if part[1] < threshold_height:
                    flag = False
                    break
            if flag:
                return True
        return False
