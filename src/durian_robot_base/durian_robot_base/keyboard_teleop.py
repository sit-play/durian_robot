#!/usr/bin/env python3
"""
ROS2 Keyboard Teleop + Camera Control
ควบคุมหุ่นยนต์ด้วยแป้นพิมพ์ + ถ่ายรูป
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys
import termios
import tty

class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # ========== PARAMETERS ==========
        self.declare_parameter('linear_speed', 0.5)    # m/s
        self.declare_parameter('angular_speed', 1.0)   # rad/s
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        
        # ========== PUBLISHERS ==========
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.capture_pub = self.create_publisher(String, 'camera/capture', 10)
        
        # ========== KEY MAPPINGS ==========
        self.keys = {
            'w': (self.linear_speed, 0.0),      # Forward
            's': (-self.linear_speed, 0.0),     # Backward
            'a': (0.0, self.angular_speed),     # Turn left
            'd': (0.0, -self.angular_speed),    # Turn right
            'q': (self.linear_speed, self.angular_speed),    # Forward-left
            'e': (self.linear_speed, -self.angular_speed),   # Forward-right
            'z': (-self.linear_speed, self.angular_speed),   # Backward-left
            'c': (-self.linear_speed, -self.angular_speed),  # Backward-right
            ' ': (0.0, 0.0),                    # Stop (space)
        }
        
        # ========== TERMINAL SETTINGS ==========
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info("""
╔════════════════════════════════════════════════════╗
║       Keyboard Teleop + Camera Control             ║
╠════════════════════════════════════════════════════╣
║  Movement:                                          ║
║    W  - Forward                                     ║
║    S  - Backward                                    ║
║    A  - Turn Left                                   ║
║    D  - Turn Right                                  ║
║    Q  - Forward-Left                                ║
║    E  - Forward-Right                               ║
║    Z  - Backward-Left                               ║
║    C  - Backward-Right                              ║
║    SPACE - Stop                                     ║
║                                                     ║
║  Camera:                                            ║
║    P  - Capture Photo (save image)                  ║
║    V  - View image stream                           ║
║                                                     ║
║  System:                                            ║
║    X  - Exit                                        ║
║                                                     ║
║  Linear Speed:  %.2f m/s                          ║
║  Angular Speed: %.2f rad/s                         ║
╚════════════════════════════════════════════════════╝
""" % (self.linear_speed, self.angular_speed))
        
        # Start keyboard listener
        self.start_listener()
    
    def start_listener(self):
        """Start listening for keyboard input"""
        try:
            tty.setraw(sys.stdin)
            
            while rclpy.ok():
                # Read single character
                ch = sys.stdin.read(1)
                
                if ch.lower() == 'x':
                    # Exit
                    self.get_logger().info('Exiting...')
                    self.stop_robot()
                    break
                
                elif ch.lower() == 'p':
                    # Capture photo
                    self.capture_photo()
                    self.show_status(ch.lower(), 'CAPTURE PHOTO', '')
                
                elif ch.lower() == 'v':
                    # View stream (just log)
                    self.get_logger().info('Image stream available at /camera/image_raw')
                    self.show_status(ch.lower(), 'VIEW STREAM', '')
                
                elif ch.lower() in self.keys:
                    # Movement
                    linear, angular = self.keys[ch.lower()]
                    self.publish_velocity(linear, angular)
                    self.show_status(ch.lower(), linear, angular)
                
        except KeyboardInterrupt:
            self.get_logger().info('Interrupted')
            self.stop_robot()
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    
    def publish_velocity(self, linear_x, angular_z):
        """Publish velocity command"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)
    
    def capture_photo(self):
        """Send capture command to camera"""
        msg = String()
        msg.data = 'CAPTURE'
        self.capture_pub.publish(msg)
    
    def stop_robot(self):
        """Stop the robot"""
        self.publish_velocity(0.0, 0.0)
    
    def show_status(self, key, linear_or_action, angular=''):
        """Show current command status"""
        if isinstance(linear_or_action, str):
            # Action (like CAPTURE)
            print(f"\rKey: {key:1s} | Action: {linear_or_action:20s}         ", 
                  end='', flush=True)
        else:
            # Movement
            print(f"\rKey: {key:1s} | Linear: {linear_or_action:+.2f} | Angular: {angular:+.2f} rad/s", 
                  end='', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()