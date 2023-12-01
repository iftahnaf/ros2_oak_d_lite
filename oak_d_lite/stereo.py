import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import depthai as dai
import time

class Bridge(Node):

    def __init__(self):
        super().__init__('oak_d_lite')

        self.rate = 30 # 30hz
        self.create_pipeline()
        self.run()

    def create_pipeline(self):
        self.pipeline = dai.Pipeline()

        # Define sources and outputs
        self.monoLeft = self.pipeline.create(dai.node.MonoCamera)
        self.monoRight = self.pipeline.create(dai.node.MonoCamera)
        self.xoutLeft = self.pipeline.create(dai.node.XLinkOut)
        self.xoutRight = self.pipeline.create(dai.node.XLinkOut)

        self.xoutLeft.setStreamName('left')
        self.xoutRight.setStreamName('right')

        # Properties
        self.monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        self.monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
        self.monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        self.monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)

        # Linking
        self.monoRight.out.link(self.xoutRight.input)
        self.monoLeft.out.link(self.xoutLeft.input)

        # ros publishers
        self.left_pub = self.create_publisher(Image, '/camera/left/image_raw', 1)
        self.right_pub = self.create_publisher(Image, '/camera/right/image_raw', 1)
        self.left_info_pub = self.create_publisher(CameraInfo, 'left/camera_info', 1)
        self.right_info_pub = self.create_publisher(CameraInfo, 'right/camera_info', 1)

        # bridge
        self.bridge = CvBridge()
        self.left_info = CameraInfo()
        self.right_info = CameraInfo()

        self.get_logger().info("initiate Oak-D-Lite ROS Stereo Node ...")

    def run(self):
        with dai.Device(self.pipeline) as device:

            self.get_logger().info(f"Device ready for use ...")

            # Output queues will be used to get the grayscale frames from the outputs defined above

            qLeft = device.getOutputQueue(name="left", maxSize=1, blocking=False)
            qRight = device.getOutputQueue(name="right", maxSize=1, blocking=False)
            # Instead of get (blocking), we use tryGet (non-blocking) which will return the available data or None otherwise
            left_counter = 0
            right_counter = 0
            while True:
                start = time.time()

                try:
                    inLeft = qLeft.tryGet()
                except Exception:
                    self.get_logger().warn("Failed to get image from left queue")
                    continue
                try:
                    inRight = qRight.tryGet()
                except Exception:
                    self.get_logger().warn("Failed to get image from right queue")
                    continue

                now = self.get_clock().now().to_msg()
            
                if inLeft is not None:
                    left_frame = inLeft.getCvFrame()
                    left_frame_ros = self.bridge.cv2_to_imgmsg(left_frame,'mono8')
                    self.left_info.header.stamp = now
                    left_frame_ros.header.stamp = now
                    left_frame_ros.header.frame_id = f'{left_counter}'
                    self.left_pub.publish(left_frame_ros)
                    self.left_info_pub.publish(self.left_info)
                    left_counter += 1
                    inLeft = None
                    
                if inRight is not None:
                    right_frame =  inRight.getCvFrame()
                    right_frame_ros = self.bridge.cv2_to_imgmsg(right_frame, 'mono8')
                    self.right_info.header.stamp = now
                    right_frame_ros.header.stamp = now
                    right_frame_ros.header.frame_id = f'{right_counter}'
                    self.right_pub.publish(right_frame_ros)
                    self.right_info_pub.publish(self.right_info)
                    right_counter += 1
                    inRight = None

                dt = time.time() - start

                if  dt > (1 / self.rate)*1.1:
                    self.get_logger().warn(f"Loop took too much time = {dt}")

                self.get_logger().info("Working ...", throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    bridge = Bridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)