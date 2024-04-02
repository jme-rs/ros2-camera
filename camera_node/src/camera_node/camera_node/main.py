import rclpy
import cv2
from .camera_node import CameraNode


def main():
    try:
        capture = cv2.VideoCapture(-1)
        if not capture.isOpened():
            raise Exception('Failed to open the camera')
    except Exception as e:
        print(e)
        return
    
    try:
        rclpy.init()
        camera_node = CameraNode(capture)
        rclpy.spin(camera_node)
    except Exception as e:
        print(e)
    finally:
        capture.release()
        camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
