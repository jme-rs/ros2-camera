import rclpy
import cv2
from .display_node import DisplayNode


def main():
    try:
        rclpy.init()
        display_node = DisplayNode()
        rclpy.spin(display_node)
    except Exception as e:
        print(e)
    finally:
        cv2.destroyAllWindows()
        display_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
