#!/usr/bin/env python3
"""
Camera Capture Bridge Node
อ่านกล้องเพียงครั้งเดียว แล้ว publish ไปยัง ROS2 topic
ให้ nodes อื่น subscribe และใช้ภาพร่วมกัน
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraCaptureNode(Node):
    def __init__(self):
        super().__init__('camera_capture')
        
        # Parameters
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('fps', 30)
        
        self.camera_id = self.get_parameter('camera_id').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.fps = self.get_parameter('fps').value
        
        # Camera setup
        self.cap = None
        self.bridge = CvBridge()
        
        try:
            self.cap = cv2.VideoCapture(self.camera_id)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            
            if not self.cap.isOpened():
                self.get_logger().error(f'Cannot open camera {self.camera_id}')
                return
            
            self.get_logger().info(f'✓ Camera {self.camera_id} opened: {self.image_width}x{self.image_height}@{self.fps}fps')
        except Exception as e:
            self.get_logger().error(f'Failed to open camera: {e}')
            return
        
        # Publisher (broadcast to all subscribers)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=5  # Keep last 5 frames
        )
        
        self.image_pub = self.create_publisher(
            Image,
            'camera/image_raw',
            qos
        )
        
        # Timer - read and publish
        self.create_timer(1.0/self.fps, self.publish_frame)
        
        self.get_logger().info('Camera Capture Bridge Node started')
        self.get_logger().info('Publishing to: /camera/image_raw')
        self.get_logger().info('Subscribers can connect without resource conflict')
    
    def publish_frame(self):
        """Read frame and publish to topic"""
        if self.cap is None or not self.cap.isOpened():
            return
        
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame')
            return
        
        try:
            # Convert BGR to RGB
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Convert to ROS Image
            img_msg = self.bridge.cv2_to_imgmsg(frame_rgb, encoding="rgb8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera'
            
            # Publish (all subscribers get copy)
            self.image_pub.publish(img_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing frame: {e}')
    
    def destroy_node(self):
        """Release camera"""
        if self.cap:
            self.cap.release()
        self.get_logger().info('Camera released')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraCaptureNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
