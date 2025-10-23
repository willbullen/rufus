#!/usr/bin/env python3
"""
DDSM315 Chassis Driver for RUFUS
Handles communication with Waveshare DDSM315 motors via RS485
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import serial
import struct
import math
import time


class DDSM315Driver(Node):
    """Driver for DDSM315 tracked chassis motors"""
    
    def __init__(self):
        super().__init__('ddsm315_driver')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('track_width', 0.48)  # Distance between tracks in meters
        self.declare_parameter('wheel_radius', 0.05)  # Wheel radius in meters
        self.declare_parameter('simulation_mode', True)  # For testing without hardware
        
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.track_width = self.get_parameter('track_width').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.simulation_mode = self.get_parameter('simulation_mode').value
        
        # Serial connection
        self.serial = None
        if not self.simulation_mode:
            try:
                self.serial = serial.Serial(self.port, self.baudrate, timeout=0.1)
                self.get_logger().info(f'Connected to DDSM315 on {self.port}')
            except Exception as e:
                self.get_logger().error(f'Failed to open serial port: {e}')
                self.simulation_mode = True
        else:
            self.get_logger().info('Running in simulation mode')
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
        # Simulated motor states
        self.left_velocity = 0.0
        self.right_velocity = 0.0
        self.left_position = 0.0
        self.right_position = 0.0
        
        # Timer for reading feedback and publishing odometry
        self.create_timer(0.02, self.update_loop)  # 50 Hz
        
        self.get_logger().info('DDSM315 driver initialized')
    
    def cmd_vel_callback(self, msg):
        """Convert Twist message to motor commands"""
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Differential drive kinematics
        # v_left = v - (w * track_width / 2)
        # v_right = v + (w * track_width / 2)
        left_velocity = linear - (angular * self.track_width / 2.0)
        right_velocity = linear + (angular * self.track_width / 2.0)
        
        # Store for simulation
        self.left_velocity = left_velocity
        self.right_velocity = right_velocity
        
        # Send commands to motors
        if not self.simulation_mode:
            self.send_motor_command(1, left_velocity)   # Left front
            self.send_motor_command(2, left_velocity)   # Left rear
            self.send_motor_command(3, right_velocity)  # Right front
            self.send_motor_command(4, right_velocity)  # Right rear
    
    def send_motor_command(self, motor_id, velocity):
        """Send RS485 command to motor"""
        if self.serial is None:
            return
        
        # Convert m/s to RPM
        # velocity (m/s) = (RPM * 2 * pi * radius) / 60
        # RPM = (velocity * 60) / (2 * pi * radius)
        rpm = int((velocity * 60.0) / (2.0 * math.pi * self.wheel_radius))
        
        # Clamp to motor limits (-315 to 315 RPM for DDSM315)
        rpm = max(-315, min(315, rpm))
        
        # DDSM315 protocol: [Header, ID, Mode, Speed_H, Speed_L, CRC]
        speed_bytes = struct.pack('>h', rpm)
        
        packet = bytearray([
            0xFF,  # Header
            motor_id,
            0x02,  # Speed mode
            speed_bytes[0],
            speed_bytes[1],
            0x00   # CRC placeholder
        ])
        
        packet[-1] = self.calculate_crc(packet[:-1])
        
        try:
            self.serial.write(packet)
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')
    
    def update_loop(self):
        """Update odometry and publish joint states"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt <= 0:
            return
        
        # Update simulated positions
        self.left_position += self.left_velocity * dt / self.wheel_radius
        self.right_position += self.right_velocity * dt / self.wheel_radius
        
        # Calculate odometry
        # Average velocity
        v = (self.left_velocity + self.right_velocity) / 2.0
        # Angular velocity
        w = (self.right_velocity - self.left_velocity) / self.track_width
        
        # Update pose
        delta_x = v * math.cos(self.theta) * dt
        delta_y = v * math.sin(self.theta) * dt
        delta_theta = w * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Publish odometry
        self.publish_odometry(current_time, v, w)
        
        # Publish joint states
        self.publish_joint_states(current_time)
        
        # Publish TF
        self.publish_tf(current_time)
    
    def publish_odometry(self, timestamp, linear_vel, angular_vel):
        """Publish odometry message"""
        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Velocity
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = angular_vel
        
        self.odom_pub.publish(odom)
    
    def publish_joint_states(self, timestamp):
        """Publish joint states for tracks"""
        joint_state = JointState()
        joint_state.header.stamp = timestamp.to_msg()
        joint_state.name = ['left_track_joint', 'right_track_joint']
        joint_state.position = [self.left_position, self.right_position]
        joint_state.velocity = [
            self.left_velocity / self.wheel_radius,
            self.right_velocity / self.wheel_radius
        ]
        joint_state.effort = []
        
        self.joint_state_pub.publish(joint_state)
    
    def publish_tf(self, timestamp):
        """Publish transform from odom to base_footprint"""
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(t)
    
    def calculate_crc(self, data):
        """Calculate CRC8 checksum"""
        crc = 0
        for byte in data:
            crc ^= byte
        return crc
    
    def destroy_node(self):
        """Cleanup"""
        if self.serial is not None:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DDSM315Driver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

