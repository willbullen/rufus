"""
WebSocket consumers for VR teleoperation
"""

import json
import logging
from channels.generic.websocket import AsyncWebsocketConsumer
from rufus_control.ros_bridge import ros_bridge

logger = logging.getLogger(__name__)


class VRTeleopConsumer(AsyncWebsocketConsumer):
    """WebSocket consumer for VR teleoperation"""
    
    async def connect(self):
        """Handle WebSocket connection"""
        await self.accept()
        logger.info(f'VR client connected: {self.channel_name}')
        
        # Send connection confirmation
        await self.send(text_data=json.dumps({
            'type': 'connection',
            'status': 'connected',
            'message': 'Connected to RUFUS VR teleoperation'
        }))
    
    async def disconnect(self, close_code):
        """Handle WebSocket disconnection"""
        logger.info(f'VR client disconnected: {self.channel_name}')
        
        # Stop robot on disconnect
        ros_bridge.publish_velocity(0.0, 0.0)
    
    async def receive(self, text_data):
        """Handle incoming WebSocket messages"""
        try:
            data = json.loads(text_data)
            message_type = data.get('type')
            
            if message_type == 'vr_pose':
                await self.handle_vr_pose(data)
            elif message_type == 'chassis_control':
                await self.handle_chassis_control(data)
            elif message_type == 'arm_control':
                await self.handle_arm_control(data)
            elif message_type == 'ping':
                await self.send(text_data=json.dumps({'type': 'pong'}))
            else:
                logger.warning(f'Unknown message type: {message_type}')
        
        except json.JSONDecodeError as e:
            logger.error(f'Invalid JSON received: {e}')
        except Exception as e:
            logger.error(f'Error processing message: {e}')
    
    async def handle_vr_pose(self, data):
        """Handle VR controller pose data"""
        try:
            left_hand = data.get('left_hand', {})
            right_hand = data.get('right_hand', {})
            
            # Extract positions and rotations
            left_pos = left_hand.get('position', {})
            right_pos = right_hand.get('position', {})
            
            # Perform inverse kinematics (simplified for now)
            # In production, this would use a proper IK solver
            joint_positions = self.simple_ik(left_pos, right_pos)
            
            # Send to arm
            ros_bridge.publish_joint_trajectory(joint_positions, duration_sec=0.1)
            
            # Send acknowledgment
            await self.send(text_data=json.dumps({
                'type': 'pose_ack',
                'status': 'success'
            }))
        
        except Exception as e:
            logger.error(f'Error handling VR pose: {e}')
            await self.send(text_data=json.dumps({
                'type': 'error',
                'message': str(e)
            }))
    
    async def handle_chassis_control(self, data):
        """Handle chassis control commands"""
        try:
            linear = data.get('linear', 0.0)
            angular = data.get('angular', 0.0)
            
            # Clamp values
            linear = max(-1.0, min(1.0, linear))
            angular = max(-2.0, min(2.0, angular))
            
            # Send to chassis
            ros_bridge.publish_velocity(linear, angular)
            
            await self.send(text_data=json.dumps({
                'type': 'chassis_ack',
                'status': 'success'
            }))
        
        except Exception as e:
            logger.error(f'Error handling chassis control: {e}')
    
    async def handle_arm_control(self, data):
        """Handle direct arm control commands"""
        try:
            joint_positions = data.get('joint_positions', [0.0] * 6)
            
            if len(joint_positions) != 6:
                raise ValueError('Expected 6 joint positions')
            
            # Send to arm
            ros_bridge.publish_joint_trajectory(joint_positions, duration_sec=0.5)
            
            await self.send(text_data=json.dumps({
                'type': 'arm_ack',
                'status': 'success'
            }))
        
        except Exception as e:
            logger.error(f'Error handling arm control: {e}')
    
    def simple_ik(self, left_pos, right_pos):
        """
        Simple inverse kinematics solver
        In production, this would use a proper IK library like IKPy or KDL
        """
        # Placeholder implementation
        # Maps VR hand positions to joint angles
        
        # For now, return neutral pose
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class TelemetryConsumer(AsyncWebsocketConsumer):
    """WebSocket consumer for real-time telemetry streaming"""
    
    async def connect(self):
        """Handle WebSocket connection"""
        await self.accept()
        logger.info(f'Telemetry client connected: {self.channel_name}')
        
        # TODO: Start streaming telemetry data
        # This would subscribe to ROS topics and forward to WebSocket
    
    async def disconnect(self, close_code):
        """Handle WebSocket disconnection"""
        logger.info(f'Telemetry client disconnected: {self.channel_name}')
    
    async def receive(self, text_data):
        """Handle incoming messages"""
        try:
            data = json.loads(text_data)
            
            if data.get('type') == 'subscribe':
                topics = data.get('topics', [])
                logger.info(f'Client subscribed to topics: {topics}')
            
        except Exception as e:
            logger.error(f'Error in telemetry consumer: {e}')

