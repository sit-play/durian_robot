#!/usr/bin/env python3
"""
ROS2 Motor Controller Node - FIXED
ควบคุมหุ่นยนต์ผ่าน cmd_vel (geometry_msgs/Twist) messages
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import serial
import threading
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String
import time

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # ========== PARAMETERS ==========
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheelbase', 0.2)  # meters
        self.declare_parameter('max_linear_vel', 1.0)  # m/s
        self.declare_parameter('max_angular_vel', 2.0)  # rad/s
        self.declare_parameter('cmd_timeout', 2.0)  # seconds
        
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        
        # ========== SERIAL CONNECTION ==========
        self.serial_conn = None
        self.connected = False
        self.lock = threading.Lock()
        
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0
            )
            self.connected = True
            self.get_logger().info(f'Connected to {self.port}@{self.baudrate}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to ESP32: {e}')
            self.connected = False
        
        # ========== ROS2 SUBSCRIPTIONS ==========
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # ✅ SUBSCRIBE cmd_vel - THIS IS IMPORTANT!
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            qos
        )
        self.get_logger().info('✓ Subscribed to /cmd_vel')
        
        # ========== ROS2 PUBLISHERS ==========
        self.motor_status_pub = self.create_publisher(
            Float32MultiArray,
            'motor_status',
            10
        )
        
        self.debug_pub = self.create_publisher(
            String,
            'motor_debug',
            10
        )
        
        # ========== STATE TRACKING ==========
        self.last_cmd_time = time.time()
        self.current_left_speed = 0.0
        self.current_right_speed = 0.0
        
        # ========== TIMERS ==========
        self.create_timer(0.1, self.status_timer_callback)  # 10 Hz
        self.create_timer(0.05, self.cmd_timeout_check)  # 20 Hz
        
        self.get_logger().info('Motor Controller Node initialized')
    
    # ========== SUBSCRIBER CALLBACK ==========
    def cmd_vel_callback(self, msg: Twist):
        """
        ✅ Receive cmd_vel messages
        Convert (linear_vel, angular_vel) to motor speeds
        """
        if not self.connected:
            self.get_logger().warn('Not connected to ESP32')
            return
        
        # Extract velocities
        linear_vel = msg.linear.x   # m/s
        angular_vel = msg.angular.z  # rad/s
        
        self.get_logger().info(f'Received: linear={linear_vel:.2f}, angular={angular_vel:.2f}')
        
        # Clamp velocities
        linear_vel = max(-self.max_linear_vel, min(self.max_linear_vel, linear_vel))
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))
        
        # Convert to motor speeds
        left_speed, right_speed = self.differential_steering(linear_vel, angular_vel)
        
        # Send to ESP32
        self.send_motor_command(left_speed, right_speed)
        
        # Update timestamp
        self.last_cmd_time = time.time()
    
    # ========== DIFFERENTIAL STEERING ==========
    def differential_steering(self, linear_vel, angular_vel):
        """
        Convert (linear_vel, angular_vel) to (left_speed, right_speed)
        v_left = v - (w * wheelbase / 2)
        v_right = v + (w * wheelbase / 2)
        """
        half_wheelbase = self.wheelbase / 2.0
        
        left_speed = linear_vel - (angular_vel * half_wheelbase)
        right_speed = linear_vel + (angular_vel * half_wheelbase)
        
        # Normalize if needed
        max_speed = max(abs(left_speed), abs(right_speed), self.max_linear_vel)
        if max_speed > self.max_linear_vel:
            left_speed = left_speed / max_speed * self.max_linear_vel
            right_speed = right_speed / max_speed * self.max_linear_vel
        
        return left_speed, right_speed
    
    # ========== SEND COMMAND TO ESP32 ==========
    def send_motor_command(self, left_speed, right_speed):
        """
        Send motor command to ESP32
        Format: "M,left_speed,right_speed\n"
        """
        if not self.connected:
            return
        
        # Store current speeds
        self.current_left_speed = left_speed
        self.current_right_speed = right_speed
        
        # Format command
        cmd = f"M,{left_speed:.2f},{right_speed:.2f}\n"
        
        with self.lock:
            try:
                self.serial_conn.write(cmd.encode())
                
                # Publish debug message
                debug_msg = String()
                debug_msg.data = f"Sent: {cmd.strip()}"
                self.debug_pub.publish(debug_msg)
                
            except Exception as e:
                self.get_logger().error(f'Failed to send command: {e}')
                self.connected = False
    
    # ========== TIMEOUT CHECK ==========
    def cmd_timeout_check(self):
        """
        Safety: Stop motors if no command for timeout
        """
        if not self.connected:
            return
        
        elapsed = time.time() - self.last_cmd_time
        
        if elapsed > self.cmd_timeout:
            if self.current_left_speed != 0.0 or self.current_right_speed != 0.0:
                self.get_logger().warn(f'Command timeout ({elapsed:.1f}s) - stopping motors')
                self.send_motor_command(0.0, 0.0)
    
    # ========== STATUS PUBLISHER ==========
    def status_timer_callback(self):
        """
        Publish current motor status
        """
        if not self.connected:
            return
        
        status_msg = Float32MultiArray()
        status_msg.data = [
            self.current_left_speed,
            self.current_right_speed,
            time.time() - self.last_cmd_time,
            1.0 if self.connected else 0.0,
        ]
        
        self.motor_status_pub.publish(status_msg)
    
    # ========== CLEANUP ==========
    def destroy_node(self):
        """Clean up serial connection"""
        if self.serial_conn and self.serial_conn.is_open:
            self.send_motor_command(0.0, 0.0)
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()