#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, Imu
from std_msgs.msg import String
import depthai as dai
from time import sleep, time
import threading

class StereoInertialBridge(Node):

    def __init__(self):
        super().__init__('stereo_imu_bridge')

        self.image_rate = 30
        self.imu_rate = 200

        self.create_pipeline()

        self.ImageGrabber = threading.Thread(target=self.image_grabber)
        self.ImuGrabber = threading.Thread(target=self.imu_grabber)

        self.run()

    def create_pipeline(self):
        self.pipeline = dai.Pipeline()

        # Define sources and outputs
        self.monoLeft = self.pipeline.create(dai.node.MonoCamera)
        self.monoRight = self.pipeline.create(dai.node.MonoCamera)
        self.xoutLeft = self.pipeline.create(dai.node.XLinkOut)
        self.xoutRight = self.pipeline.create(dai.node.XLinkOut)
        self.imu = self.pipeline.create(dai.node.IMU)
        self.xlinkOut = self.pipeline.create(dai.node.XLinkOut)

        self.xoutLeft.setStreamName('left')
        self.xoutRight.setStreamName('right')
        self.xlinkOut.setStreamName("imu")

        # Properties
        self.monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        self.monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
        self.monoLeft.setFps(self.image_rate)
        self.monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        self.monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
        self.monoRight.setFps(self.image_rate)
        self.imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 200)
        self.imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 200)
        self.imu.setBatchReportThreshold(1)
        self.imu.setMaxBatchReports(10)

        # Linking
        self.monoRight.out.link(self.xoutRight.input)
        self.monoLeft.out.link(self.xoutLeft.input)
        self.imu.out.link(self.xlinkOut.input)

        # ros publishers
        self.left_pub = self.create_publisher(Image, '/camera/left/image_raw', 1)
        self.right_pub = self.create_publisher(Image, '/camera/right/image_raw', 1)
        self.left_info_pub = self.create_publisher(CameraInfo, 'left/camera_info', 1)
        self.right_info_pub = self.create_publisher(CameraInfo, 'right/camera_info', 1)
        self.imu_pub = self.create_publisher(Imu, '/imu', 1)

        # bridge
        self.bridge = CvBridge()
        self.left_info = CameraInfo()
        self.right_info = CameraInfo()
        self.imu_msg = Imu()
        self.imu_status_msg = String()
        self.camera_status_msg = String()

        self.get_logger().info("initiate Oak-D-Lite ROS Stereo-Inertial Node ...")

    def image_grabber(self):
        while True:
            
            start = time()

            if self.qLeft.has():
                inLeft = self.qLeft.tryGet()
            else:
                inLeft = None
            
            if self.qRight.has():
                inRight = self.qRight.tryGet()
            else:
                inRight = None

            now = self.get_clock().now().to_msg()

            if inLeft is not None:
                left_frame = inLeft.getCvFrame()
                left_frame_ros = self.bridge.cv2_to_imgmsg(left_frame,'mono8')
                self.left_info.header.stamp = now
                left_frame_ros.header.stamp = now
                left_frame_ros.header.frame_id = "left_camera"
                self.left_pub.publish(left_frame_ros)
                self.left_info_pub.publish(self.left_info)
                self.camera_status_msg.data = "left"
                inLeft = None

            if inRight is not None:
                right_frame =  inRight.getCvFrame()
                right_frame_ros = self.bridge.cv2_to_imgmsg(right_frame, 'mono8')
                self.right_info.header.stamp = now
                right_frame_ros.header.stamp = now
                right_frame_ros.header.frame_id = "right_camera"
                self.right_pub.publish(right_frame_ros)
                self.right_info_pub.publish(self.right_info)
                self.camera_status_msg.data = "right"
                inRight = None

            dt = time() - start

            if  dt > (1 / self.image_rate)*1.1:
                self.get_logger().warn(f"Loop took too much time = {dt}")

            self.get_logger().info("[IMAGE] Working ...", throttle_duration_sec=5.0)

    def imu_grabber(self):
        while True:

            if self.imuQueue.has():
                imuData = self.imuQueue.tryGet()  # blocking call, will wait until a new data has arrived
            else:
                imuData = None

            self.imu_status_msg = "imu"

            now = self.get_clock().now().to_msg()

            try:
                imuPackets = imuData.packets
                for imuPacket in imuPackets:
                    acceleroValues = imuPacket.acceleroMeter
                    gyroValues = imuPacket.gyroscope

                    self.imu_msg.header.stamp = now
                    self.imu_msg.header.frame_id = "imu"
                    self.imu_msg.angular_velocity.x = gyroValues.x
                    self.imu_msg.angular_velocity.y = gyroValues.y
                    self.imu_msg.angular_velocity.z = gyroValues.z

                    self.imu_msg.linear_acceleration.x = acceleroValues.x
                    self.imu_msg.linear_acceleration.y = acceleroValues.y
                    self.imu_msg.linear_acceleration.z = acceleroValues.z

                    self.imu_pub.publish(self.imu_msg)
                    
                    self.get_logger().info("[IMU] Working ...", throttle_duration_sec=5.0)
            except Exception:
                continue

    def run(self):
        with dai.Device(self.pipeline) as device:

            # Output queues will be used to get the grayscale frames from the outputs defined above
            self.qLeft = device.getOutputQueue(name="left", maxSize=1, blocking=False)
            self.qRight = device.getOutputQueue(name="right", maxSize=1, blocking=False)
            self.imuQueue = device.getOutputQueue(name="imu", maxSize=1, blocking=False)

            self.ImageGrabber.start()
            self.ImuGrabber.start()

            self.ImageGrabber.join()
            self.ImuGrabber.join()

def main(args=None):
    rclpy.init(args=args)
    sib = StereoInertialBridge()
    rclpy.spin(sib)
    sib.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)