# Waveshare Hardware Components for RUFUS

## DDSM315 Direct Drive Servo Motor

### Overview
Direct drive servo motor combining outer rotor brushless motor, encoder, and servo driver into a highly reliable permanent magnet synchronous motor (PMSM). Designed for mobile robot wheel drive applications.

### Key Specifications
- **Motor Type**: PMSM, outer rotor brushless, direct drive (no external reducer)
- **Control**: Integrated FOC (Field Oriented Control) algorithm
- **Voltage**: 18V DC rated (12-24V DC operating range)
- **Speed**: 
  - No-load: 315 ±10 RPM
  - Rated: 200 RPM
- **Torque**:
  - Rated: 0.55 Nm
  - Stall: 1.1 Nm
- **Current**:
  - Rated: 0.5 A
  - No-load: ≤0.25 A
  - Stall: ≤3.5 A
- **Encoder**: 12-bit (4096 positions/revolution)
- **Weight**: 349 ±5 g
- **Single Wheel Load**: Up to 10 kg
- **Operating Temp**: -20°C to 45°C
- **Protection**: IP54 (dust and water jets resistant)
- **Efficiency**: Up to 99.99% (zero-backlash, no gear friction)
- **Noise**: ≤50-55 dB(A)

### Communication & Control
- **Interface**: RS485 serial bus (multi-drop)
- **Topology**: Single RS485 channel controls multiple servos simultaneously
- **Control Modes**:
  - Current loop mode
  - Position loop mode
  - Velocity loop mode
- **Real-time Feedback**: Position, speed, current, temperature, error codes
- **Protection**: Over-current protection, Hall position detection, electric brake

### RS485 Communication Protocol

#### Protocol 1: Drive Motor to Rotate
- Controls motor speed/position in different modes
- CRC8 checksum for data integrity
- Supports current, speed, and position control

#### Protocol 2: Obtain Feedback
- Read motor status: position, speed, current, temperature
- Error code reporting
- Real-time diagnostic data

#### Protocol 3: Motor Mode Switch
- Switch between current/speed/position modes
- Configure control parameters

#### Protocol 4: Motor ID Set
- Assign unique ID to each motor on RS485 bus
- Essential for multi-motor control

#### Protocol 5: Motor ID Query
- Discover motors on bus
- Verify motor connectivity

### Advantages for RUFUS
- **Zero Backlash**: Direct drive eliminates gear train
- **High Precision**: 12-bit encoder provides 4096 steps/revolution
- **Low Noise**: Ideal for indoor robotics
- **Efficient**: 99.99% efficiency reduces power consumption
- **Robust**: IP54 rating for outdoor/dusty environments
- **Multi-Motor Control**: Single RS485 bus for all chassis motors
- **Real-time Feedback**: Closed-loop control with encoder data

### Application in RUFUS
- **Tracked Chassis Drive**: 2-4 motors for differential drive tracks
- **Wheel Configuration**: Direct hub mounting
- **Load Capacity**: Each motor supports 10kg, suitable for robot weight
- **Speed Range**: 200-315 RPM appropriate for mobile platform

## ST3215 Servo (for SO-101 Arm)

### Overview
High-torque programmable servo designed for robotic joints and manipulators. Conventional servo with internal reduction gearing.

### Key Specifications
- **Drive Type**: Conventional servo with internal reduction
- **Motor**: Brushed/brushless with reduction gear
- **Encoder**: 360° magnetic encoder, absolute position
- **Control Interface**: Programmable serial bus (TTL/RS485)
- **Torque**: Much higher than DDSM315 (ST3215-HS: up to 3.2+ Nm)
- **Speed**: Lower speeds, optimized for precise positioning
- **Feedback**: Position, speed, torque lock, mode
- **Mounting**: Traditional servo horn
- **Backlash**: Minimal (with gear train), but not zero

### Comparison: DDSM315 vs ST3215

| Feature | DDSM315 | ST3215 |
|---------|---------|--------|
| **Application** | Wheel drive, mobile platforms | Robotic joints, arms, manipulators |
| **Drive Type** | Direct drive, hub-style | Conventional with gearing |
| **Torque** | 0.55 Nm (rated), 1.1 Nm (stall) | 3.2+ Nm (much higher) |
| **Speed** | Up to 315 RPM | Lower, precision-focused |
| **Backlash** | Zero | Minimal (gear train) |
| **Encoder** | 12-bit (4096 steps) | 360° magnetic, absolute |
| **Mounting** | Hub/wheel direct | Servo horn |
| **Communication** | RS485 | TTL/RS485 serial |
| **Use Case** | Chassis locomotion | Arm articulation |

### Application in RUFUS
- **SO-101 Arm Joints**: 6 DOF control
- **Gripper Control**: Precise position/torque control
- **High Precision**: Absolute positioning for manipulation tasks
- **Serial Bus**: Daisy-chain multiple servos on single bus

## Integration Architecture for RUFUS

### Hardware Topology
```
Jetson Orin NX (ROS2 Controller)
    |
    ├─ RS485 Bus 1 → DDSM315 Motors (Chassis)
    |   ├─ Motor ID 1 (Left Track Front)
    |   ├─ Motor ID 2 (Left Track Rear)
    |   ├─ Motor ID 3 (Right Track Front)
    |   └─ Motor ID 4 (Right Track Rear)
    |
    └─ RS485/TTL Bus 2 → ST3215 Servos (Arm)
        ├─ Servo ID 1 (Base)
        ├─ Servo ID 2 (Shoulder)
        ├─ Servo ID 3 (Elbow)
        ├─ Servo ID 4 (Wrist Pitch)
        ├─ Servo ID 5 (Wrist Roll)
        └─ Servo ID 6 (Gripper)
```

### Communication Requirements
- **2x RS485/USB Converters**: One for chassis, one for arm
- **Power Distribution**: 
  - 18V for DDSM315 motors
  - 12V for ST3215 servos
- **Control Frequency**: 50-100 Hz for smooth motion
- **Latency**: <10ms for real-time control

### ROS2 Driver Requirements
- **DDSM315 Driver Node**: 
  - Implements RS485 protocol
  - Publishes odometry from encoders
  - Subscribes to velocity commands
  - Provides motor diagnostics
  
- **ST3215 Driver Node**:
  - Implements servo protocol
  - Publishes joint states
  - Subscribes to joint commands
  - Supports MoveIt integration

## Open Source Resources
Waveshare provides open-source four-wheel drive off-road vehicle structure model with DDSM315 motors, including:
- CAD models
- Engineering files
- Reference implementations

This can serve as a starting point for RUFUS chassis design.

