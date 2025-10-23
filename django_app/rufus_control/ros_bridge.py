"""
ROS Bridge Connection Manager for RUFUS
Handles communication between Django and ROS2 via rosbridge
"""

import roslibpy
import logging
from django.conf import settings

logger = logging.getLogger(__name__)


class ROSBridge:
    """Singleton class to manage ROS bridge connection"""
    
    _instance = None
    _client = None
    _connected = False
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(ROSBridge, cls).__new__(cls)
        return cls._instance
    
    def __init__(self):
        if not self._connected:
            self.connect()
    
    def connect(self):
        """Connect to rosbridge server"""
        try:
            host = getattr(settings, 'ROSBRIDGE_HOST', 'localhost')
            port = getattr(settings, 'ROSBRIDGE_PORT', 9090)
            
            self._client = roslibpy.Ros(host=host, port=port)
            self._client.run()
            self._connected = True
            logger.info(f'Connected to ROS bridge at {host}:{port}')
        except Exception as e:
            logger.error(f'Failed to connect to ROS bridge: {e}')
            self._connected = False
    
    def disconnect(self):
        """Disconnect from rosbridge server"""
        if self._client:
            try:
                self._client.terminate()
                self._connected = False
                logger.info('Disconnected from ROS bridge')
            except Exception as e:
                logger.error(f'Error disconnecting from ROS bridge: {e}')
    
    @property
    def is_connected(self):
        """Check if connected to ROS bridge"""
        return self._connected and self._client and self._client.is_connected
    
    @property
    def client(self):
        """Get the ROS client"""
        if not self.is_connected:
            self.connect()
        return self._client
    
    def publish_velocity(self, linear_x=0.0, angular_z=0.0):
        """Publish velocity command to chassis"""
        if not self.is_connected:
            logger.warning('Not connected to ROS bridge')
            return False
        
        try:
            topic = roslibpy.Topic(
                self._client,
                '/cmd_vel',
                'geometry_msgs/Twist'
            )
            
            message = roslibpy.Message({
                'linear': {'x': linear_x, 'y': 0.0, 'z': 0.0},
                'angular': {'x': 0.0, 'y': 0.0, 'z': angular_z}
            })
            
            topic.publish(message)
            return True
        except Exception as e:
            logger.error(f'Failed to publish velocity: {e}')
            return False
    
    def publish_joint_trajectory(self, joint_positions, duration_sec=1.0):
        """Publish joint trajectory to arm"""
        if not self.is_connected:
            logger.warning('Not connected to ROS bridge')
            return False
        
        try:
            topic = roslibpy.Topic(
                self._client,
                '/arm_controller/joint_trajectory',
                'trajectory_msgs/JointTrajectory'
            )
            
            message = roslibpy.Message({
                'joint_names': [
                    'base_joint',
                    'shoulder_joint',
                    'elbow_joint',
                    'wrist_pitch_joint',
                    'wrist_roll_joint',
                    'gripper_joint'
                ],
                'points': [{
                    'positions': joint_positions,
                    'velocities': [],
                    'accelerations': [],
                    'effort': [],
                    'time_from_start': {
                        'sec': int(duration_sec),
                        'nanosec': int((duration_sec % 1) * 1e9)
                    }
                }]
            })
            
            topic.publish(message)
            return True
        except Exception as e:
            logger.error(f'Failed to publish joint trajectory: {e}')
            return False
    
    def subscribe_joint_states(self, callback):
        """Subscribe to joint states"""
        if not self.is_connected:
            logger.warning('Not connected to ROS bridge')
            return None
        
        try:
            topic = roslibpy.Topic(
                self._client,
                '/joint_states',
                'sensor_msgs/JointState'
            )
            
            topic.subscribe(callback)
            return topic
        except Exception as e:
            logger.error(f'Failed to subscribe to joint states: {e}')
            return None
    
    def subscribe_odometry(self, callback):
        """Subscribe to odometry"""
        if not self.is_connected:
            logger.warning('Not connected to ROS bridge')
            return None
        
        try:
            topic = roslibpy.Topic(
                self._client,
                '/odom',
                'nav_msgs/Odometry'
            )
            
            topic.subscribe(callback)
            return topic
        except Exception as e:
            logger.error(f'Failed to subscribe to odometry: {e}')
            return None
    
    def call_navigation_service(self, x, y, qz, qw):
        """Call navigation service to move to goal"""
        if not self.is_connected:
            logger.warning('Not connected to ROS bridge')
            return False
        
        try:
            # This would use the Nav2 action server
            # For now, just log the request
            logger.info(f'Navigation goal: x={x}, y={y}, qz={qz}, qw={qw}')
            return True
        except Exception as e:
            logger.error(f'Failed to call navigation service: {e}')
            return False


# Global instance
ros_bridge = ROSBridge()

