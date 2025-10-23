# RUFUS Implementation Guide

This comprehensive guide walks you through building the RUFUS robot from scratch, including hardware assembly, software installation, and system configuration.

## Table of Contents

1. [Hardware Assembly](#1-hardware-assembly)
2. [Jetson Orin NX Setup](#2-jetson-orin-nx-setup)
3. [Docker Environment Setup](#3-docker-environment-setup)
4. [ROS2 Workspace Configuration](#4-ros2-workspace-configuration)
5. [Django Web Application Setup](#5-django-web-application-setup)
6. [Hardware Driver Development](#6-hardware-driver-development)
7. [VR Teleoperation Setup](#7-vr-teleoperation-setup)
8. [Testing and Validation](#8-testing-and-validation)
9. [Troubleshooting](#9-troubleshooting)

## 1. Hardware Assembly

### 1.1 Bill of Materials

| Component | Quantity | Estimated Cost | Supplier |
|-----------|----------|----------------|----------|
| NVIDIA Jetson Orin NX | 1 | $499 | NVIDIA |
| Waveshare DDSM315 Motors | 4 | $80 each | Waveshare |
| Feetech ST3215 Servos | 6 | $25 each | Feetech |
| SO-101 Arm Frame Kit | 1 | $150 | Various |
| Tracked Chassis Frame | 1 | $200 | Custom/Waveshare |
| USB-to-RS485 Converters | 2 | $15 each | Amazon |
| Lidar (MS200 or equivalent) | 1 | $150 | Slamtec |
| Depth Camera (RealSense D435) | 1 | $200 | Intel |
| 6S LiPo Battery (22.2V, 5000mAh) | 1 | $80 | Hobby stores |
| Power Management Unit | 1 | $50 | Custom/COTS |
| Voltage Regulators (24V, 12V, 5V) | 3 | $30 total | Amazon |
| Wiring, Connectors, Fasteners | - | $100 | Various |
| **Total Estimated Cost** | - | **~$2,500** | - |

### 1.2 Chassis Assembly

1. **Frame Construction**
   - Assemble the tracked chassis frame according to manufacturer instructions
   - Ensure structural rigidity for mounting Jetson and arm
   - Install mounting plates for motors and electronics

2. **Motor Installation**
   - Mount 4x DDSM315 motors to chassis (2 per track)
   - Connect motors in parallel for each track
   - Secure wiring with cable ties and routing channels

3. **Power Distribution**
   - Install Power Management Unit (PMU) in central location
   - Connect main battery to PMU input
   - Wire voltage regulators:
     - 24V rail → DDSM315 motors
     - 12V rail → ST3215 servos
     - 5V rail → Jetson Orin NX, sensors

### 1.3 Arm Assembly

1. **SO-101 Frame**
   - Assemble the 6-DOF arm frame
   - Install ST3215 servos in each joint
   - Ensure proper alignment and range of motion

2. **Mounting**
   - Secure arm base to chassis mounting plate
   - Verify stability under load
   - Route servo cables through cable management

3. **Gripper**
   - Install gripper mechanism
   - Connect gripper servo
   - Test open/close motion

### 1.4 Electronics Integration

1. **Jetson Orin NX**
   - Mount Jetson in protective enclosure
   - Connect to 5V power rail
   - Install heatsink and cooling fan

2. **RS485 Converters**
   - Connect USB-to-RS485 Converter 1 to Jetson USB port
   - Wire RS485 A/B terminals to chassis motor bus
   - Connect USB-to-RS485 Converter 2 to Jetson USB port
   - Wire RS485 A/B terminals to arm servo bus

3. **Sensors**
   - Mount Lidar at front of chassis
   - Connect Lidar to Jetson via USB
   - Mount depth camera on pan-tilt or fixed mount
   - Connect camera to Jetson USB 3.0 port

4. **Wiring Verification**
   - Check all power connections with multimeter
   - Verify RS485 A/B polarity
   - Ensure no shorts or loose connections

## 2. Jetson Orin NX Setup

### 2.1 JetPack Installation

1. **Download JetPack SDK**
   - Visit NVIDIA Developer website
   - Download JetPack 6.0 or later
   - Includes Ubuntu 22.04, CUDA, cuDNN, TensorRT

2. **Flash Jetson**
   - Use NVIDIA SDK Manager on host PC
   - Connect Jetson via USB-C in recovery mode
   - Follow SDK Manager wizard to flash OS

3. **Initial Boot**
   - Complete Ubuntu setup wizard
   - Set username: `rufus`
   - Set hostname: `rufus-robot`
   - Connect to WiFi/Ethernet

### 2.2 System Configuration

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install essential tools
sudo apt install -y \
    git \
    curl \
    wget \
    nano \
    htop \
    net-tools \
    python3-pip \
    build-essential

# Configure power mode (25W for full performance)
sudo nvpmodel -m 0
sudo jetson_clocks
```

### 2.3 Docker Installation

```bash
# Docker is pre-installed with JetPack, verify:
docker --version

# Install Docker Compose
sudo apt install -y docker-compose

# Add user to docker group
sudo usermod -aG docker $USER
newgrp docker

# Verify Docker can access GPU
docker run --rm --runtime=nvidia dustynv/ros:humble-ros-base-l4t-r36.2.0 nvidia-smi
```

### 2.4 NVIDIA Container Runtime

```bash
# Configure NVIDIA runtime as default
sudo nano /etc/docker/daemon.json
```

Add:
```json
{
  "default-runtime": "nvidia",
  "runtimes": {
    "nvidia": {
      "path": "nvidia-container-runtime",
      "runtimeArgs": []
    }
  }
}
```

```bash
# Restart Docker
sudo systemctl restart docker
```

## 3. Docker Environment Setup

### 3.1 Clone RUFUS Repository

```bash
cd ~
git clone https://github.com/willbullen/rufus.git
cd rufus
```

### 3.2 Configure Environment

```bash
# Copy example environment file
cp .env.example .env

# Edit with your settings
nano .env
```

Update key values:
- `POSTGRES_PASSWORD`: Strong database password
- `DJANGO_SECRET_KEY`: Random 50-character string
- `CHASSIS_PORT`: Verify with `ls /dev/ttyUSB*`
- `ARM_PORT`: Verify with `ls /dev/ttyUSB*`

### 3.3 Build Docker Images

```bash
# Build all images (this will take 30-60 minutes)
docker-compose build

# Verify images
docker images | grep rufus
```

### 3.4 Initialize Database

```bash
# Start database and redis only
docker-compose up -d db redis

# Wait for database to be ready
docker-compose logs -f db

# Run Django migrations
docker-compose run --rm web-backend python manage.py migrate

# Create superuser
docker-compose run --rm web-backend python manage.py createsuperuser
```

## 4. ROS2 Workspace Configuration

### 4.1 Create ROS2 Packages

```bash
# Enter ROS2 core container
docker-compose run --rm ros2-core bash

# Navigate to workspace
cd /workspace/ros2_ws/src

# Create packages
ros2 pkg create --build-type ament_python rufus_bringup
ros2 pkg create --build-type ament_python rufus_drivers
ros2 pkg create --build-type ament_python rufus_description
ros2 pkg create --build-type ament_python rufus_navigation
ros2 pkg create --build-type ament_python rufus_perception
ros2 pkg create --build-type ament_python rufus_manipulation
ros2 pkg create --build-type ament_python rufus_teleop

# Exit container
exit
```

### 4.2 Build Workspace

```bash
# Build all packages
docker-compose run --rm ros2-core bash -c "cd /workspace/ros2_ws && colcon build --symlink-install"

# Source workspace
docker-compose run --rm ros2-core bash -c "source /workspace/ros2_ws/install/setup.bash && ros2 pkg list | grep rufus"
```

## 5. Django Web Application Setup

### 5.1 Create Django Apps

```bash
# Enter backend container
docker-compose run --rm web-backend bash

# Create apps
cd /app
python manage.py startapp rufus_control
python manage.py startapp rufus_telemetry
python manage.py startapp rufus_vr
python manage.py startapp rufus_arm
python manage.py startapp rufus_chassis
python manage.py startapp rufus_vision

# Exit container
exit
```

### 5.2 Configure Django Settings

Edit `django_app/rufus_web/settings.py`:

```python
INSTALLED_APPS = [
    # ... existing apps ...
    'channels',
    'rest_framework',
    'corsheaders',
    'rufus_control',
    'rufus_telemetry',
    'rufus_vr',
    'rufus_arm',
    'rufus_chassis',
    'rufus_vision',
]

# Channels configuration
ASGI_APPLICATION = 'rufus_web.asgi.application'
CHANNEL_LAYERS = {
    'default': {
        'BACKEND': 'channels_redis.core.RedisChannelLayer',
        'CONFIG': {
            "hosts": [('redis', 6379)],
        },
    },
}

# CORS configuration
CORS_ALLOW_ALL_ORIGINS = True  # For development only
```

### 5.3 Create ASGI Configuration

Edit `django_app/rufus_web/asgi.py`:

```python
import os
from django.core.asgi import get_asgi_application
from channels.routing import ProtocolTypeRouter, URLRouter
from channels.auth import AuthMiddlewareStack
from rufus_vr import routing as vr_routing

os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'rufus_web.settings')

application = ProtocolTypeRouter({
    "http": get_asgi_application(),
    "websocket": AuthMiddlewareStack(
        URLRouter(
            vr_routing.websocket_urlpatterns
        )
    ),
})
```

## 6. Hardware Driver Development

### 6.1 DDSM315 Chassis Driver

Create `ros2_ws/src/rufus_drivers/rufus_drivers/ddsm315_driver.py`:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import serial
import struct

class DDSM315Driver(Node):
    def __init__(self):
        super().__init__('ddsm315_driver')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        
        # Serial connection
        self.serial = serial.Serial(port, baudrate, timeout=0.1)
        
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
        
        # Timer for reading feedback
        self.create_timer(0.02, self.read_feedback)  # 50 Hz
        
        self.get_logger().info(f'DDSM315 driver initialized on {port}')
    
    def cmd_vel_callback(self, msg):
        """Convert Twist message to motor commands"""
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Differential drive kinematics
        left_speed = linear - angular * 0.3  # 0.3m track width
        right_speed = linear + angular * 0.3
        
        # Send commands to motors
        self.send_motor_command(1, left_speed)   # Left front
        self.send_motor_command(2, left_speed)   # Left rear
        self.send_motor_command(3, right_speed)  # Right front
        self.send_motor_command(4, right_speed)  # Right rear
    
    def send_motor_command(self, motor_id, speed):
        """Send RS485 command to motor"""
        # DDSM315 protocol: [Header, ID, Mode, Speed_H, Speed_L, CRC]
        speed_rpm = int(speed * 100)  # Convert m/s to RPM
        speed_bytes = struct.pack('>h', speed_rpm)
        
        packet = bytearray([
            0xFF,  # Header
            motor_id,
            0x02,  # Speed mode
            speed_bytes[0],
            speed_bytes[1],
            0x00   # CRC placeholder
        ])
        
        packet[-1] = self.calculate_crc(packet[:-1])
        self.serial.write(packet)
    
    def read_feedback(self):
        """Read encoder feedback from motors"""
        # Implementation for reading motor feedback
        pass
    
    def calculate_crc(self, data):
        """Calculate CRC8 checksum"""
        crc = 0
        for byte in data:
            crc ^= byte
        return crc

def main(args=None):
    rclpy.init(args=args)
    node = DDSM315Driver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 6.2 ST3215 Arm Driver

Create `ros2_ws/src/rufus_drivers/rufus_drivers/st3215_driver.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import serial

class ST3215Driver(Node):
    def __init__(self):
        super().__init__('st3215_driver')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 1000000)
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        
        # Serial connection
        self.serial = serial.Serial(port, baudrate, timeout=0.1)
        
        # Subscribers
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory',
            self.trajectory_callback,
            10
        )
        
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Joint names
        self.joint_names = [
            'base_joint',
            'shoulder_joint',
            'elbow_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint',
            'gripper_joint'
        ]
        
        # Timer for reading feedback
        self.create_timer(0.02, self.read_joint_states)  # 50 Hz
        
        self.get_logger().info(f'ST3215 driver initialized on {port}')
    
    def trajectory_callback(self, msg):
        """Execute joint trajectory"""
        if len(msg.points) == 0:
            return
        
        point = msg.points[0]
        for i, position in enumerate(point.positions):
            self.set_servo_position(i + 1, position)
    
    def set_servo_position(self, servo_id, position_rad):
        """Set servo position in radians"""
        # Convert radians to servo units (0-4095 for 360 degrees)
        position_units = int((position_rad / (2 * 3.14159)) * 4096) % 4096
        
        # ST3215 write position command
        packet = bytearray([
            0xFF, 0xFF,  # Header
            servo_id,
            0x07,  # Length
            0x03,  # Write command
            0x2A,  # Position register
            position_units & 0xFF,
            (position_units >> 8) & 0xFF,
            0x00, 0x00,  # Speed
            0x00   # Checksum placeholder
        ])
        
        packet[-1] = self.calculate_checksum(packet[2:-1])
        self.serial.write(packet)
    
    def read_joint_states(self):
        """Read current joint positions"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = []
        
        for i in range(1, 7):
            position = self.read_servo_position(i)
            joint_state.position.append(position)
        
        self.joint_state_pub.publish(joint_state)
    
    def read_servo_position(self, servo_id):
        """Read servo position"""
        # Implementation for reading servo position
        return 0.0
    
    def calculate_checksum(self, data):
        """Calculate checksum"""
        return (~sum(data)) & 0xFF

def main(args=None):
    rclpy.init(args=args)
    node = ST3215Driver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 7. VR Teleoperation Setup

### 7.1 Meta Quest App Development

Refer to the VR development guide created earlier. Key steps:

1. Install Unity with Android Build Support
2. Install Meta XR SDK
3. Create VR project with hand tracking
4. Implement WebSocket client for communication
5. Build and deploy to Meta Quest

### 7.2 Django WebSocket Handler

Create `django_app/rufus_vr/consumers.py`:

```python
import json
from channels.generic.websocket import AsyncWebsocketConsumer
import roslibpy

class VRTeleopConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        await self.accept()
        
        # Connect to ROS bridge
        self.ros_client = roslibpy.Ros(host='rosbridge', port=9090)
        self.ros_client.run()
        
        # Create publishers
        self.cmd_vel_pub = roslibpy.Topic(
            self.ros_client,
            '/cmd_vel',
            'geometry_msgs/Twist'
        )
        
        self.joint_trajectory_pub = roslibpy.Topic(
            self.ros_client,
            '/joint_trajectory',
            'trajectory_msgs/JointTrajectory'
        )
    
    async def disconnect(self, close_code):
        if hasattr(self, 'ros_client'):
            self.ros_client.terminate()
    
    async def receive(self, text_data):
        data = json.loads(text_data)
        
        if data['type'] == 'vr_pose':
            # Process VR controller pose
            await self.handle_vr_pose(data)
        elif data['type'] == 'chassis_control':
            # Process chassis control
            await self.handle_chassis_control(data)
    
    async def handle_vr_pose(self, data):
        # Perform IK and send to arm
        # This is a simplified example
        joint_angles = self.inverse_kinematics(
            data['left_hand'],
            data['right_hand']
        )
        
        self.joint_trajectory_pub.publish(roslibpy.Message({
            'joint_names': ['base_joint', 'shoulder_joint', ...],
            'points': [{
                'positions': joint_angles,
                'time_from_start': {'sec': 0, 'nanosec': 100000000}
            }]
        }))
    
    async def handle_chassis_control(self, data):
        self.cmd_vel_pub.publish(roslibpy.Message({
            'linear': {'x': data['linear'], 'y': 0, 'z': 0},
            'angular': {'x': 0, 'y': 0, 'z': data['angular']}
        }))
    
    def inverse_kinematics(self, left_hand, right_hand):
        # Implement IK solver
        # This is a placeholder
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

## 8. Testing and Validation

### 8.1 Hardware Tests

```bash
# Test chassis motors
docker exec -it rufus_ros2_core bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

# Test arm servos
ros2 topic pub /joint_trajectory trajectory_msgs/JointTrajectory "..."

# Monitor joint states
ros2 topic echo /joint_states
```

### 8.2 Navigation Tests

```bash
# Start SLAM
ros2 launch rufus_navigation slam.launch.py

# Visualize in RViz (on remote PC)
ros2 launch rufus_navigation rviz.launch.py

# Send navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "..."
```

### 8.3 Web Interface Tests

1. Open browser to `http://<jetson-ip>:3000`
2. Login with superuser credentials
3. Test manual control with virtual joystick
4. Verify camera feeds display correctly
5. Test arm control sliders

## 9. Troubleshooting

### Common Issues

**Issue: USB devices not detected**
```bash
# Check USB devices
lsusb

# Check serial ports
ls -l /dev/ttyUSB*

# Add user to dialout group
sudo usermod -aG dialout $USER
```

**Issue: Docker containers can't access GPU**
```bash
# Verify NVIDIA runtime
docker run --rm --runtime=nvidia dustynv/ros:humble-ros-base-l4t-r36.2.0 nvidia-smi

# Check daemon.json
cat /etc/docker/daemon.json
```

**Issue: ROS2 nodes can't communicate**
```bash
# Check ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID

# List active nodes
ros2 node list

# Check topic list
ros2 topic list
```

**Issue: WebSocket connection fails**
```bash
# Check rosbridge is running
docker ps | grep rosbridge

# Test rosbridge connection
wscat -c ws://localhost:9090
```

## Next Steps

1. Complete hardware driver implementation
2. Develop VR application
3. Integrate MoveIt2 for arm planning
4. Configure Nav2 for autonomous navigation
5. Implement object detection with YOLOv8
6. Create custom web interface pages
7. Conduct field testing and optimization

## References

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Jetson Orin NX Developer Guide](https://docs.nvidia.com/jetson/)
- [Django Channels Documentation](https://channels.readthedocs.io/)
- [MoveIt2 Tutorials](https://moveit.picknik.ai/main/index.html)
- [Nav2 Documentation](https://navigation.ros.org/)

