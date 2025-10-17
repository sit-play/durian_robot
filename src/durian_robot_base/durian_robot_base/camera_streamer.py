#!/usr/bin/env python3
"""
MJPEG Camera Streamer - FIXED
Subscribe from /camera/image_raw และ stream ไป browser
(ไม่พยายามเปิดกล้องเอง)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
import cv2

class CameraStreamerNode(Node):
    def __init__(self):
        super().__init__('camera_streamer')
        
        self.bridge = CvBridge()
        self.frame = None
        self.lock = threading.Lock()
        
        # Subscribe to camera topic
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=5
        )
        
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            qos
        )
        
        self.get_logger().info('✓ Camera Streamer Node started')
        self.get_logger().info('✓ Subscribed to /camera/image_raw')
    
    def image_callback(self, msg: Image):
        """Receive image from topic"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.frame = cv_image
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
    
    def get_frame(self):
        """Get latest frame"""
        with self.lock:
            if self.frame is None:
                return None
            return self.frame.copy()

# Global streamer node
streamer_node = None

class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            html = """
            <html>
            <head>
                <title>Durian Robot Camera Stream</title>
                <style>
                    body { 
                        font-family: Arial; 
                        text-align: center; 
                        background: #f0f0f0;
                        margin: 0;
                        padding: 20px;
                    }
                    img { 
                        max-width: 100%; 
                        border: 2px solid #333;
                        border-radius: 8px;
                        box-shadow: 0 4px 6px rgba(0,0,0,0.1);
                    }
                    h1 { color: #333; }
                    .info { 
                        margin-top: 20px; 
                        font-size: 14px;
                        color: #666;
                    }
                </style>
            </head>
            <body>
                <h1>🤖 Durian Robot Camera Stream</h1>
                <img src="/stream" width="640">
                <div class="info">
                    <p>Real-time video from Camera Bridge</p>
                    <p>Resolution: 640x480 | FPS: 30</p>
                </div>
            </body>
            </html>
            """
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(html.encode())
        
        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            
            try:
                while True:
                    if streamer_node is None:
                        time.sleep(0.1)
                        continue
                    
                    frame = streamer_node.get_frame()
                    if frame is None:
                        time.sleep(0.033)
                        continue
                    
                    ret, buffer = cv2.imencode('.jpg', frame)
                    frame_bytes = buffer.tobytes()
                    
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n')
                    self.wfile.write(b'Content-Length: ' + str(len(frame_bytes)).encode() + b'\r\n\r\n')
                    self.wfile.write(frame_bytes)
                    self.wfile.write(b'\r\n')
                    
                    time.sleep(0.033)
            except:
                pass
        else:
            self.send_response(404)
            self.end_headers()
    
    def log_message(self, format, *args):
        pass

def main(args=None):
    global streamer_node
    
    rclpy.init(args=args)
    streamer_node = CameraStreamerNode()
    
    # Start HTTP server in background thread
    def run_server():
        server = HTTPServer(('0.0.0.0', 8080), StreamHandler)
        print("✓ Stream server started at http://0.0.0.0:8080")
        print("✓ Access from: http://localhost:8080")
        server.serve_forever()
    
    server_thread = threading.Thread(target=run_server, daemon=True)
    server_thread.start()
    
    try:
        rclpy.spin(streamer_node)
    except KeyboardInterrupt:
        pass
    finally:
        streamer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()