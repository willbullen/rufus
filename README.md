# RUFUS: Robotic Universal Field Utility System

![RUFUS Logo](docs/images/rufus_logo.png)

**A mobile manipulation robot with tracked chassis, SO-101 robotic arm, ROS2 on Jetson Orin NX, Docker deployment, Django web interface, and VR teleoperation capabilities.**

## Overview

RUFUS is a sophisticated mobile manipulation platform designed for research, development, and practical applications in robotics. It combines a robust tracked chassis with a 6-DOF robotic arm, powered by an NVIDIA Jetson Orin NX running ROS2 Humble in Docker containers. The system features multiple control modes including a modern web interface and immersive VR teleoperation via Meta Quest.

### Key Features

- **Mobile Platform**: Tracked chassis with 4x Waveshare DDSM315 direct-drive servo motors
- **Manipulation**: SO-101 robotic arm with 6x ST3215 servos for precise manipulation
- **Edge AI**: NVIDIA Jetson Orin NX for real-time perception and control
- **ROS2 Humble**: Modern robotics middleware in Docker containers
- **Web Interface**: Django-based control panel with React frontend
- **VR Teleoperation**: Meta Quest integration for immersive robot control
- **Autonomous Navigation**: Nav2 stack with SLAM and path planning
- **Computer Vision**: YOLOv8 object detection and depth perception
- **Modular Architecture**: Docker Compose orchestration for easy deployment

## Hardware Components

| Component | Specification | Purpose |
|-----------|--------------|---------|
| **Compute** | NVIDIA Jetson Orin NX | Main controller, AI processing |
| **Chassis Motors** | 4x Waveshare DDSM315 | Tracked chassis drive |
| **Arm Servos** | 6x Feetech ST3215 | Robotic arm actuation |
| **Lidar** | MS200 or equivalent | 2D laser scanning |
| **Depth Camera** | Intel RealSense or equivalent | RGB-D perception |
| **IMU** | Integrated in motor controller | Orientation sensing |
| **Battery** | 6S LiPo (22.2V) | Power supply |

## Software Stack

### Core Technologies

- **ROS2 Humble**: Robotics middleware
- **Docker**: Container orchestration
- **Django 4.2**: Web backend
- **React 19**: Web frontend
- **Channels**: WebSocket support
- **Celery**: Task queue
- **PostgreSQL**: Database
- **Redis**: Cache and message broker

### ROS2 Packages

- **Navigation2**: Autonomous navigation
- **SLAM Toolbox**: Mapping and localization
- **MoveIt2**: Arm motion planning
- **rosbridge_server**: Web-ROS2 bridge
- **image_transport**: Camera streaming
- **robot_localization**: Sensor fusion

## Project Structure

```
rufus/
├── docker/                    # Docker configurations
│   ├── ros2_core/            # Core ROS2 services
│   ├── drivers/              # Hardware drivers
│   ├── perception/           # Vision and sensing
│   ├── manipulation/         # Arm control
│   ├── web_backend/          # Django application
│   └── web_frontend/         # React application
├── ros2_ws/                  # ROS2 workspace
│   └── src/                  # ROS2 packages
│       ├── rufus_bringup/    # Launch files
│       ├── rufus_drivers/    # Motor/servo drivers
│       ├── rufus_description/# Robot URDF
│       ├── rufus_navigation/ # Nav2 configuration
│       ├── rufus_perception/ # Vision processing
│       ├── rufus_manipulation/# MoveIt2 config
│       └── rufus_teleop/     # Teleoperation
├── django_app/               # Django web application
│   ├── rufus_control/        # Robot control API
│   ├── rufus_telemetry/      # Real-time telemetry
│   ├── rufus_vr/             # VR interface
│   ├── rufus_arm/            # Arm control
│   ├── rufus_chassis/        # Chassis control
│   └── rufus_vision/         # Camera feeds
├── frontend/                 # React frontend
│   └── src/                  # Source code
├── docs/                     # Documentation
├── config/                   # Configuration files
├── scripts/                  # Utility scripts
├── hardware/                 # Hardware schematics/CAD
├── docker-compose.yml        # Container orchestration
└── README.md                 # This file
```

## Quick Start

### Prerequisites

- NVIDIA Jetson Orin NX with JetPack 6.0+
- Docker and Docker Compose installed
- Git installed
- 2x USB-to-RS485 converters
- Assembled RUFUS hardware

### Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/willbullen/rufus.git
   cd rufus
   ```

2. **Configure environment variables**
   ```bash
   cp .env.example .env
   nano .env  # Edit with your settings
   ```

3. **Build Docker images**
   ```bash
   docker-compose build
   ```

4. **Start the system**
   ```bash
   docker-compose up -d
   ```

5. **Access the web interface**
   - Open browser to `http://<jetson-ip>:3000`
   - Default credentials: `admin` / `admin`

### Hardware Setup

1. **Connect RS485 converters**
   - Converter 1 → Chassis motors (DDSM315)
   - Converter 2 → Arm servos (ST3215)

2. **Connect sensors**
   - Lidar → USB 2.0 port
   - Depth camera → USB 3.0 port

3. **Power connections**
   - Main battery → PMU
   - PMU 24V → Chassis motors
   - PMU 12V → Arm servos
   - PMU 5V → Jetson Orin NX

4. **Verify connections**
   ```bash
   ls /dev/ttyUSB*  # Should show USB0 and USB1
   ```

## Usage

### Web Interface

The web interface provides comprehensive control and monitoring:

- **Dashboard**: System status, battery, temperature
- **Manual Control**: Virtual joystick for chassis, arm sliders
- **VR Teleoperation**: Start/stop VR sessions, view metrics
- **Navigation**: Set waypoints, view SLAM map
- **Arm Control**: Joint/Cartesian control, saved poses
- **Perception**: Camera feeds, object detection
- **System Monitor**: Container health, logs

### VR Teleoperation

1. **Setup Meta Quest**
   - Install the RUFUS VR app (see `docs/vr_setup.md`)
   - Connect to same network as Jetson

2. **Start VR session**
   - Put on headset
   - Launch RUFUS VR app
   - Enter Jetson IP address
   - Press "Connect"

3. **Control the robot**
   - Left hand → Left arm control
   - Right hand → Right arm control (or chassis)
   - Grip buttons → Gripper control
   - Trigger → Record demonstrations

### Autonomous Navigation

1. **Create a map**
   ```bash
   docker exec rufus_ros2_core ros2 launch rufus_navigation slam.launch.py
   ```

2. **Save the map**
   ```bash
   docker exec rufus_ros2_core ros2 run nav2_map_server map_saver_cli -f /maps/my_map
   ```

3. **Navigate autonomously**
   - Load map in web interface
   - Click destination on map
   - Robot plans path and navigates

## Development

### Building ROS2 Packages

```bash
# Enter the ROS2 core container
docker exec -it rufus_ros2_core bash

# Build workspace
cd /workspace/ros2_ws
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### Running Tests

```bash
# ROS2 package tests
docker exec rufus_ros2_core colcon test

# Django tests
docker exec rufus_backend python manage.py test

# Frontend tests
docker exec rufus_frontend npm test
```

### Viewing Logs

```bash
# All containers
docker-compose logs -f

# Specific container
docker-compose logs -f ros2-core

# ROS2 logs
docker exec rufus_ros2_core ros2 topic echo /diagnostics
```

## Documentation

Comprehensive documentation is available in the `docs/` directory:

- [System Architecture](docs/ARCHITECTURE.md)
- [Hardware Assembly](docs/HARDWARE_ASSEMBLY.md)
- [Software Installation](docs/SOFTWARE_INSTALLATION.md)
- [ROS2 Package Guide](docs/ROS2_PACKAGES.md)
- [Web Interface Guide](docs/WEB_INTERFACE.md)
- [VR Teleoperation Guide](docs/VR_TELEOPERATION.md)
- [API Reference](docs/API_REFERENCE.md)
- [Troubleshooting](docs/TROUBLESHOOTING.md)

## Contributing

Contributions are welcome! Please read our [Contributing Guidelines](CONTRIBUTING.md) before submitting pull requests.

### Development Workflow

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- **Hiwonder LanderPi**: Inspiration for the ROS2 architecture
- **Waveshare**: DDSM315 motors and hardware components
- **NVIDIA**: Jetson platform and jetson-containers
- **ROS2 Community**: Navigation2, MoveIt2, and other packages
- **Django Community**: Web framework and ecosystem
- **dusty-nv**: jetson-containers project

## Support

- **Issues**: [GitHub Issues](https://github.com/willbullen/rufus/issues)
- **Discussions**: [GitHub Discussions](https://github.com/willbullen/rufus/discussions)
- **Email**: support@rufus-robot.com

## Roadmap

- [x] Core system architecture
- [x] Docker containerization
- [x] Web interface
- [ ] VR teleoperation implementation
- [ ] Autonomous navigation integration
- [ ] MoveIt2 arm control
- [ ] Object detection and tracking
- [ ] Multi-robot coordination
- [ ] ROS2 Iron migration
- [ ] Cloud deployment options

## Citation

If you use RUFUS in your research, please cite:

```bibtex
@software{rufus2025,
  author = {Bullen, Will},
  title = {RUFUS: Robotic Universal Field Utility System},
  year = {2025},
  url = {https://github.com/willbullen/rufus}
}
```

---

**Built with ❤️ for the robotics community**

