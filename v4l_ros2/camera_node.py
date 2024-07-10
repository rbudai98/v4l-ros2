#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import subprocess
import cv2
import fcntl
from v4l2 import *
from copy import copy
import numpy as np
import select
import os


class V4L2InterfaceNode(Node):

    def __init__(self):
        print(self)

        # Set general variables
        self.vid = None
        self.width = 1280        # Update according to your sensor
        self.height = 720        # Update according to your sensor
        self.process = None
        self.query_ctrl_list = []
        self.streamOn = False
        self.control_device = "/dev/video0"    # Update according to your sensor
        self.buffer = np.zeros(self.width * self.height * 2, dtype=np.uint8)

        super(V4L2InterfaceNode, self).__init__("camera_node")
        self.get_logger().info("Selfe initialized")

        try:
            self.video_control_fd = open(self.control_device, "rb+", buffering=0)
        except:
            self.get_logger().info("Could not open requested device")
            return
        self.bridge = CvBridge()

        self.publisher = self.create_publisher(Image, "video_frames", 10)

        self.start_stop_params()
        self.declare_parameters_local()
        self.add_on_set_parameters_callback(self.on_parameter_event)
        self.timer = self.create_timer(
            10.0 / 30.0, self.publish_frame
        )  # Adjust frame rate as needed
        self.get_logger().info("V4L2 Interface node has been started")

    def start_stop_params(self):
        # Declare parameters
        self.declare_parameter('start_button', False)
        # Get initial parameter values
        self.start_button = self.get_parameter('start_button').get_parameter_value().bool_value
        self.get_logger().info(f'Start button: {self.start_button}')


    def declare_parameters_local(self):
        queryctrl = v4l2_queryctrl()
        queryctrl.id = V4L2_CID_BASE
        while True:
            try:
                fcntl.ioctl(self.video_control_fd, VIDIOC_QUERYCTRL, queryctrl)
            except IOError:
                break

            if queryctrl.name.decode() != "Camera Controls":
                self.query_ctrl_list.append(copy(queryctrl))
                self.get_logger().info("Found command: %s " % queryctrl.name.decode())
                desc = ParameterDescriptor(description=queryctrl.name.decode())
                self.declare_parameter(
                    queryctrl.name.decode(), int(self.get_control(queryctrl.id)), desc
                )
            queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL

    def get_control(self, control_id):
        control = v4l2_control()
        control.id = control_id
        fcntl.ioctl(self.video_control_fd, VIDIOC_G_CTRL, control)
        self.get_logger().info("Get value for: %d" % control_id)
        return control.value

    def set_control(self, control_id, value):
        control = v4l2_control()
        control.id = control_id
        control.value = value
        self.get_logger().info("Set value for: %d" % control_id)
        fcntl.ioctl(self.video_control_fd, VIDIOC_S_CTRL, control)

    def on_parameter_event(self, params):
        for parameter in params:
            for tmp in range(len(self.query_ctrl_list)):
                if self.query_ctrl_list[tmp].name.decode() == parameter.name:
                    control_id = self.query_ctrl_list[tmp].id
                    self.set_control(control_id, parameter.value)

        return SetParametersResult(successful=True)

    def publish_frame(self):
        self.start_button = self.get_parameter('start_button').get_parameter_value().bool_value
        if not self.start_button and self.streamOn:
            self.vid.release()
            self.streamOn = False
        elif self.start_button and not self.streamOn:
            self.vid = cv2.VideoCapture(0) 
            self.streamOn = True

        if (self.streamOn):
            ret, frame = self.vid.read()
            raw_reshape = frame.reshape((self.height, self.width))
            normalized_image = cv2.normalize(raw_reshape, None, 0, 0x0fff, cv2.NORM_MINMAX)
            grayscale_image = np.uint8(normalized_image)
            msg = self.bridge.cv2_to_imgmsg(grayscale_image, encoding='mono8')
            self.publisher.publish(msg)
        print("Publishing")

    def destroy_node(self):
        if self.process:
            self.process.terminate()
        super().destroy_node()


def main(args=None):
    print("Initializing ROS")
    rclpy.init(args=args)
    print("Creating node")
    node = V4L2InterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.video_control_fd.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
