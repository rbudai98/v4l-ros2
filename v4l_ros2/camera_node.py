#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import fcntl
from v4l2 import *
import os

class V4L2InterfaceNode(Node):
    def __init__(self):
        print("Before super called")
        print(self)
        super(V4L2InterfaceNode, self).__init__("camera_node")
        print("Selfe initialized")

        self.device = "/dev/video2"  # Adjust to your device
        self.video_fd = open(self.device, "rb+", buffering=0)

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, "video_frames", 10)

        self.declare_parameters_local()

        self.timer = self.create_timer(
            1.0 / 30.0, self.publish_frame
        )  # Adjust frame rate as needed

        self.get_logger().info("V4L2 Interface node has been started")

    def declare_parameters_local(self):

        vd = os.open(self.device, os.O_RDWR)
        # Prepare the query control structure
        queryctrl = v4l2_queryctrl()
        queryctrl.id = V4L2_CID_BASE
        # Loop to query all controls
        while True:
            try:
            # Query the next control
                fcntl.ioctl(self.video_fd, VIDIOC_QUERYCTRL, queryctrl)
            except IOError:
            # If we get an IOError, break the loop
                os.close(vd)
                break

            # Print the control information
        #     print(f"Control ID: {queryctrl.id}")
        #     print(f"Name: {queryctrl.name.decode()}")
        #     print(f"Type: {queryctrl.type}")
        #     print(f"Minimum: {queryctrl.minimum}")
        #     print(f"Maximum: {queryctrl.maximum}")
        #     print(f"Step: {queryctrl.step}")
        #     print(f"Default Value: {queryctrl.default_value}")
        #     print(f"Flags: {queryctrl.flags}")
        #     print()
            if (queryctrl.name.decode() != "Camera Controls"):
                print("Found command: ", queryctrl.name.decode())
                desc = ParameterDescriptor(description=queryctrl.name.decode())
                self.declare_parameter(
                        queryctrl.name.decode(), int(self.get_control(queryctrl.id)), desc
                )
                # Increment the control ID to query the next control
            queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL

    def get_control(self, control_id):
        control = v4l2_control()
        control.id = control_id
        fcntl.ioctl(self.video_fd, VIDIOC_G_CTRL, control)
        return control.value

    def set_control(self, control_id, value):
        control = v4l2_control()
        control.id = control_id
        control.value = value
        fcntl.ioctl(self.video_fd, VIDIOC_S_CTRL, control)

    def on_parameter_event(self, event):
        for changed_parameter in event.changed_parameters:
            control_id = self.get_control_id_by_name(changed_parameter.name)
            if control_id:
                self.set_control(control_id, changed_parameter.value)

    def get_control_id_by_name(self, name):
        queryctrl = v4l2_queryctrl()
        queryctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL

        while True:
            try:
                fcntl.ioctl(self.video_fd, VIDIOC_QUERYCTRL, queryctrl)
                if queryctrl.flags & V4L2_CTRL_FLAG_DISABLED:
                    queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL
                    continue
                if queryctrl.name.decode() == name:
                    return queryctrl.id
                queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL
            except Exception as e:
                break
        return None

    def publish_frame(self):
        ret, frame = self.capture_frame()
        if not ret:
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher.publish(msg)

    def capture_frame(self):
        cap = cv2.VideoCapture(self.device)
        ret, frame = cap.read()
        cap.release()
        return ret, frame


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
        node.video_fd.close(node.video_fd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
