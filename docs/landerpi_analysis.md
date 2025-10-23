# Hiwonder LanderPi Analysis

## Overview
LanderPi is a ROS2-powered mobile manipulation robot built on Raspberry Pi 5 platform. Available with Mecanum, Ackerman, or Tank chassis configurations.

## Key Hardware Components

### Main Controller
- **Raspberry Pi 5**: ROS2 controller, runs deep learning frameworks, ROS2 in Docker
- **STM32F407VET6**: Low-level motion controller
  - ARM Cortex-M4 core @ 168 MHz
  - 512 KB Flash, 192 KB RAM
  - Onboard IMU sensor
  - Supports up to 4 encoder-equipped DC motors
  - Multiple expansion interfaces: I2C, UART, GPIO, PWM

### Sensors
- **Lidar**: MS200
- **Depth Camera**: Aurora930 with detailed parameters
- **IMU**: Onboard on STM32 controller
- **Encoders**: For motor feedback

### Power
- 7.4V 2200mAh lithium battery

### Control Options
- PS2 wireless controller
- App via Bluetooth
- RC transmitter
- ROS2 commands

## Software Architecture

### STM32 Firmware (FreeRTOS-based)
**Responsibilities:**
- Motor PID control
- Encoder and IMU data acquisition
- RGB headlight control
- Serial communication with ROS2

**Tasks:**
- `Robot_Task`: Main control, kinematics, IMU data, transmission
- `Miscellaneous Task`: Battery management, IMU calibration, buzzer

**Control Flow:**
1. Target velocity received from ROS2
2. Inverse kinematics calculates motor speeds
3. Motor PID controllers process speeds
4. PWM signals output to motor drivers
5. Encoders provide feedback for closed-loop control

### ROS2 Structure
**Directory Layout:**
- `ros2_ws/`: Main workspace
  - `src/`: Package folders
    - `app/`: Application-level features
    - `interfaces/`: Message definitions
    - `simulations/`: Gazebo/simulation
    - `bringup/`: Launch files
    - `driver/`: Hardware drivers
    - `navigation/`: Nav2 stack
    - `slam/`: Mapping packages
    - `peripherals/`: Sensor interfaces
    - `yolov8_detect/`: AI vision
    - `calibration/`: Sensor calibration

**Features Supported:**
- Mapping and navigation
- Path planning
- Object tracking
- Vision tracking
- Human-robot interaction
- Voice control
- AI vision (YOLOv8)
- Group control

## Communication Architecture
- STM32 ↔ ROS2: Serial interface
  - ROS2 sends: Target velocity vectors
  - STM32 sends: Odometry, IMU data, battery voltage
- ROS2 nodes communicate via topics/services
- Docker containerization for ROS2 environment

## Key Insights for RUFUS Project
1. **Proven Architecture**: STM32 for low-level control + ROS2 for high-level intelligence
2. **Docker Deployment**: Already uses Docker for ROS2
3. **Modular Design**: Clear separation between drivers, navigation, perception
4. **Multiple Control Methods**: Easy to add VR control alongside existing methods
5. **FreeRTOS Base**: Real-time task scheduling for motor control

## Differences for RUFUS
- Replace Raspberry Pi 5 with Jetson Orin NX
- Replace Hiwonder motors with Waveshare DDSM315
- Add SO-101 arm with ST3215 servos
- Add VR control interface
- Use custom Django web interface
- Potentially replace STM32 with Waveshare servo controller

