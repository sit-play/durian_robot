#!/usr/bin/env python3
"""
ROS2 Camera Controller Node
จัดการกล้อง USB - ถ่ายรูปและบันทึกลงดิสก์
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
import threading

class CameraControllerNode(Node):
    def __init__(self):
        super().__init__('camera_controller')
        
        # ========== PARAMETERS ==========
        self.declare_parameter('camera_id', 0)  # Default: /dev/video0
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('output_dir', os.path.expanduser('~/project_ws/data/images'))
        self.declare_parameter('fps', 30)
        
        self.camera_id = self.get_parameter('camera_id').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.output_dir = self.get_parameter('output_dir').value
        self.fps = self.get_parameter('fps').value
        
        # ========== CAMERA SETUP ==========
        self.cap = None
        self.bridge = CvBridge()
        self.capture_count = 0
        self.lock = threading.Lock()
        
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
        
        # ========== CREATE OUTPUT DIRECTORY ==========
        os.makedirs(self.output_dir, exist_ok=True)
        self.get_logger().info(f'✓ Output directory: {self.output_dir}')
        
        # ========== ROS2 PUBLISHERS ==========
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        self.image_pub = self.create_publisher(
            Image,
            'camera/image_raw',
            qos
        )
        
        self.status_pub = self.create_publisher(
            String,
            'camera/status',
            10
        )
        
        # ========== ROS2 SUBSCRIBERS ==========
        self.capture_sub = self.create_subscription(
            String,
            'camera/capture',
            self.capture_callback,
            10
        )
        
        # ========== TIMERS ==========
        self.create_timer(1.0/self.fps, self.camera_read_timer)
        
        self.get_logger().info('Camera Controller Node initialized')
    
    # ========== CAMERA READ TIMER ==========
    def camera_read_timer(self):
        """Read frame from camera and publish"""
        if self.cap is None or not self.cap.isOpened():
            return
        
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame')
            return
        
        try:
            # Convert BGR to RGB
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Convert to ROS Image message
            img_msg = self.bridge.cv2_to_imgmsg(frame_rgb, encoding="rgb8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera'
            
            # Publish
            self.image_pub.publish(img_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing frame: {e}')
    
    # ========== CAPTURE CALLBACK ==========
    def capture_callback(self, msg: String):
        """
        Receive capture command
        msg.data can be:
        - "CAPTURE" = save single image
        - "START" = start continuous capture
        - "STOP" = stop continuous capture
        """
        command = msg.data.strip().upper()
        
        if command == "CAPTURE":
            self.capture_single_image()
        elif command == "START":
            self.get_logger().info('Continuous capture started')
            # Can be extended for video recording
        elif command == "STOP":
            self.get_logger().info('Capture stopped')
    
    # ========== CAPTURE SINGLE IMAGE ==========
    def capture_single_image(self):
        """Capture and save single image"""
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().error('Camera not available')
            return
        
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture image')
            return
        
        with self.lock:
            self.capture_count += 1
            
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            filename = f"image_{self.capture_count:04d}_{timestamp}.jpg"
            filepath = os.path.join(self.output_dir, filename)
            
            try:
                # Save image
                cv2.imwrite(filepath, frame)
                
                # Log
                self.get_logger().info(f'✓ Image saved: {filename}')
                
                # Publish status
                status_msg = String()
                status_msg.data = f'SAVED: {filename}'
                self.status_pub.publish(status_msg)
                
            except Exception as e:
                self.get_logger().error(f'Failed to save image: {e}')
    
    # ========== CLEANUP ==========
    def destroy_node(self):
        """Release camera"""
        if self.cap:
            self.cap.release()
        self.get_logger().info('Camera released')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
