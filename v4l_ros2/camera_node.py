#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from linuxpy.video.device import Device
import cv2


class v4l2Tof:

    _operating_mode = "0"
    _phase_depth_bits = "4"
    _ab_bits = "0"
    _confidence_bits = "0"
    _ab_averaging = "0"
    _depth_enable = "0"

    def init_v4l2(self):
        self.cam = Device.from_id(0)
        self.cam.open()

    def print_card_info(self):
        print(self.cam.info.card)

    def print_available_controls(self):
        for ctrl in self.cam.controls.values():
            print(ctrl)


class ImagePublisher(Node):

    _v4l2Obj = v4l2Tof()

    def __init__(self):
        super().__init__("image_publisher")
        self._v4l2Obj.init_v4l2()

        self.publisher_ = self.create_publisher(Image, "image", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cap = cv2.VideoCapture(
            0
        )  # Change 0 to the appropriate device ID if needed
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:

            self._v4l2Obj.print_card_info()
            self._v4l2Obj.print_available_controls()

            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
            self.get_logger().info("Publishing image frame")

    def __del__(self):
        self.cap.release()


def main(args=None):

    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
