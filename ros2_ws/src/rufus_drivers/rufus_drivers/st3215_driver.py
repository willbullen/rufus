#!/usr/bin/env python3
"""
ST3215 Arm Driver for RUFUS
Handles communication with Feetech ST3215 servos via RS485
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer
import serial
import struct
import math


class ST3215Driver(Node):
    """Driver for ST3215 robotic arm servos"""
    
    def __init__(self):
        super().__init__('st3215_driver')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('simulation_mode', True)
        
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.simulation_mode = self.get_parameter('simulation_mode').value
        
        # Serial connection
        self.serial = None
        if not self.simulation_mode:
            try:
                self.serial = serial.Serial(self.port, self.baudrate, timeout=0.1)
                self.get_logger().info(f'Connected to ST3215 on {self.port}')
            except Exception as e:
                self.get_logger().error(f'Failed to open serial port: {e}')
                self.simulation_mode = True
        else:
            self.get_logger().info('Running in simulation mode')
        
        # Joint names
        self.joint_names = [
            'base_joint',
            'shoulder_joint',
            'elbow_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint',
            'gripper_joint'
        ]
        
        # Current joint states (simulated)
        self.joint_positions = [0.0] * 6
        self.joint_velocities = [0.0] * 6
        self.target_positions = [0.0] * 6
        
        # Subscribers
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )
        
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Action server for trajectory following
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            self.execute_trajectory
        )
        
        # Timer for reading feedback and updating simulation
        self.create_timer(0.02, self.update_loop)  # 50 Hz
        
        self.get_logger().info('ST3215 driver initialized')
    
    def trajectory_callback(self, msg):
        """Execute joint trajectory"""
        if len(msg.points) == 0:
            return
        
        # For now, just take the first point
        point = msg.points[0]
        
        if len(point.positions) != len(self.joint_names):
            self.get_logger().warn(f'Expected {len(self.joint_names)} positions, got {len(point.positions)}')
            return
        
        for i, position in enumerate(point.positions):
            self.target_positions[i] = position
            if not self.simulation_mode:
                self.set_servo_position(i + 1, position)
    
    async def execute_trajectory(self, goal_handle):
        """Execute trajectory action"""
        self.get_logger().info('Executing trajectory...')
        
        trajectory = goal_handle.request.trajectory
        
        # Execute each point in the trajectory
        for point in trajectory.points:
            if len(point.positions) == len(self.joint_names):
                for i, position in enumerate(point.positions):
                    self.target_positions[i] = position
                    if not self.simulation_mode:
                        self.set_servo_position(i + 1, position)
            
            # Wait for the time specified in the trajectory
            if point.time_from_start.sec > 0 or point.time_from_start.nanosec > 0:
                await self.get_clock().sleep_for(
                    rclpy.duration.Duration(
                        seconds=point.time_from_start.sec,
                        nanoseconds=point.time_from_start.nanosec
                    )
                )
        
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        return result
    
    def set_servo_position(self, servo_id, position_rad):
        """Set servo position in radians"""
        if self.serial is None:
            return
        
        # Convert radians to servo units (0-4095 for 360 degrees)
        # ST3215 has 4096 positions per revolution
        position_units = int((position_rad / (2.0 * math.pi)) * 4096) % 4096
        
        # Clamp to valid range
        position_units = max(0, min(4095, position_units))
        
        # ST3215 write position command
        # Packet format: [0xFF, 0xFF, ID, Length, Instruction, Address, Data..., Checksum]
        packet = bytearray([
            0xFF, 0xFF,  # Header
            servo_id,    # Servo ID
            0x07,        # Length (7 bytes after ID)
            0x03,        # Write command
            0x2A,        # Position register address
            position_units & 0xFF,          # Position low byte
            (position_units >> 8) & 0xFF,   # Position high byte
            0x00, 0x00,  # Speed (0 = max speed)
            0x00         # Checksum placeholder
        ])
        
        # Calculate checksum
        packet[-1] = self.calculate_checksum(packet[2:-1])
        
        try:
            self.serial.write(packet)
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')
    
    def read_servo_position(self, servo_id):
        """Read servo position (placeholder for real implementation)"""
        # In simulation, return the target position
        if servo_id > 0 and servo_id <= len(self.joint_positions):
            return self.joint_positions[servo_id - 1]
        return 0.0
    
    def update_loop(self):
        """Update joint states and simulation"""
        # Simulate smooth movement towards target positions
        for i in range(len(self.joint_positions)):
            error = self.target_positions[i] - self.joint_positions[i]
            # Simple proportional control for simulation
            self.joint_velocities[i] = error * 2.0  # Gain of 2.0
            self.joint_positions[i] += self.joint_velocities[i] * 0.02  # dt = 0.02s
        
        # Publish joint states
        self.publish_joint_states()
    
    def publish_joint_states(self):
        """Publish current joint states"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = self.joint_positions.copy()
        joint_state.velocity = self.joint_velocities.copy()
        joint_state.effort = []
        
        self.joint_state_pub.publish(joint_state)
    
    def calculate_checksum(self, data):
        """Calculate checksum for ST3215 protocol"""
        return (~sum(data)) & 0xFF
    
    def destroy_node(self):
        """Cleanup"""
        if self.serial is not None:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ST3215Driver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

