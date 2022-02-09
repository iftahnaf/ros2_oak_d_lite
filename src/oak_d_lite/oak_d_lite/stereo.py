#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import sys
import os
import cv2
import time

import depthai as dai

class StereoPublisher(Node):
    def __init__(self):
        super().__init__('stereo_publisher')
# Create pipeline
        self.pipeline = dai.Pipeline()

        # Define sources and outputs
        self.monoLeft = self.pipeline.create(dai.node.MonoCamera)
        self.monoRight = self.pipeline.create(dai.node.MonoCamera)
        self.xoutLeft = self.pipeline.create(dai.node.XLinkOut)
        self.xoutRight = self.pipeline.create(dai.node.XLinkOut)

        self.xoutLeft.setStreamName('left')
        self.xoutRight.setStreamName('right')

        # Properties
        self.monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        self.monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        self.monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        self.monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)

        # Linking
        self.monoRight.out.link(self.xoutRight.input)
        self.monoLeft.out.link(self.xoutLeft.input)

        # ros2 publishers
        self.left_pub = self.create_publisher(Image, '/camera/left/image_raw', 10)
        self.right_pub = self.create_publisher(Image, '/camera/right/image_raw', 10)
        self.left_info_pub = self.create_publisher(CameraInfo, 'left/camera_info', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, 'right/camera_info', 10)

        # bridge
        self.bridge = CvBridge()
        self.left_info = CameraInfo()
        self.right_info = CameraInfo()
        self.get_logger().info("initiate Oak-D-Lite ros2 node ...")
        self.frame_grabber()

        # Connect to device and start pipeline
    def frame_grabber(self):
        with dai.Device(self.pipeline) as device:

            # Output queues will be used to get the grayscale frames from the outputs defined above
            qLeft = device.getOutputQueue(name="left", maxSize=4, blocking=False)
            qRight = device.getOutputQueue(name="right", maxSize=4, blocking=False)
                # Instead of get (blocking), we use tryGet (non-blocking) which will return the available data or None otherwise
            while True:
                inLeft = qLeft.tryGet()
                inRight = qRight.tryGet()
                now = self.get_clock().now().to_msg()
                if inLeft is not None:
                    
                    left_frame = inLeft.getCvFrame()
                    left_frame_ros = self.bridge.cv2_to_imgmsg(left_frame,'mono8')
                    self.left_info.header.stamp = now
                    left_frame_ros.header.stamp = now
                    self.left_pub.publish(left_frame_ros)
                    self.left_info_pub.publish(self.left_info)

                if inRight is not None:
                    right_frame =  inRight.getCvFrame()
                    right_frame_ros = self.bridge.cv2_to_imgmsg(right_frame, 'mono8')
                    self.right_info.header.stamp = now
                    right_frame_ros.header.stamp = now
                    self.right_pub.publish(right_frame_ros)
                    self.right_info_pub.publish(self.right_info)

                time.sleep(0.1)
                
                
            
def main(args=None):
    rclpy.init(args=args)
    stereo_publisher = StereoPublisher()
    rclpy.spin(stereo_publisher)
    stereo_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)