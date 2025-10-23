# Jetson Orin NX ROS2 Docker Architecture for RUFUS

## Overview

The NVIDIA Jetson Orin NX provides a powerful edge computing platform for running ROS2-based robotics applications in Docker containers. This document outlines best practices and architecture for RUFUS.

## 1. Docker Setup on Jetson Orin NX

### NVIDIA Container Runtime

The NVIDIA Container Runtime is essential for GPU acceleration in Docker containers. It must be configured as the default runtime to ensure all containers have access to CUDA, cuDNN, TensorRT, and hardware codecs.

**Installation Steps:**
1. Install Docker Engine (comes with JetPack)
2. Install NVIDIA Container Runtime
3. Set as default runtime in `/etc/docker/daemon.json`:

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

4. Restart Docker daemon: `sudo systemctl restart docker`

### Base Images

Use L4T (Linux for Tegra) based images for Jetson compatibility:
- `nvcr.io/nvidia/l4t-base:<tag>` - NVIDIA official base
- `dustynv/ros:humble-ros-base-l4t-r36.x` - ROS2 Humble with GPU support
- Ensure L4T version matches JetPack version to avoid library conflicts

**Key Resource:** The [jetson-containers project](https://github.com/dusty-nv/jetson-containers) by Dusty-NV is the definitive reference for Jetson Docker images with ROS2 and GPU acceleration.

## 2. ROS2 Humble/Iron Compatibility

### Recommended: ROS2 Humble

ROS2 Humble Hawksbill (LTS) is the recommended distribution for production robotics on Jetson:
- **Long-term Support**: Until May 2027
- **Widespread Support**: Extensive community and package ecosystem
- **Proven Stability**: Well-tested on Jetson platforms
- **Container Availability**: Pre-built images from dusty-nv and NVIDIA

**Container Image:**
```bash
docker pull dustynv/ros:humble-ros-base-l4t-r36.2.0
```

### ROS2 Iron

ROS2 Iron is newer but may require building from source or awaiting updated container support. Humble's LTS status makes it the better choice for RUFUS.

### Compatibility Considerations

- **JetPack Version**: Ensure host JetPack matches L4T version in Docker image
- **CUDA Version**: Verify CUDA compatibility between host and container
- **Library Conflicts**: Mix-matching versions causes library conflicts

## 3. GPU Acceleration in Docker

### Enabling GPU Access

When NVIDIA runtime is set as default, containers automatically have GPU access. Otherwise, explicitly enable:

```bash
docker run --runtime=nvidia --gpus all <image>
```

### GPU-Accelerated Libraries

Containers can access:
- **CUDA**: Parallel computing platform
- **cuDNN**: Deep learning primitives
- **TensorRT**: High-performance inference
- **Hardware Codecs**: Video encoding/decoding
- **VPI (Vision Programming Interface)**: Accelerated computer vision

### Verification

Test GPU access in container:
```bash
nvidia-smi
nvcc --version
```

## 4. Real-Time Performance Considerations

### Network Configuration

Use `--net=host` for low-latency ROS2 DDS communication:
```bash
docker run --net=host <image>
```

The default bridge network introduces significant overhead for ROS2 discovery and data transfer.

### Resource Allocation

Limit CPU and GPU allocations to avoid over-commitment:
```bash
docker run --cpus="4.0" --memory="8g" --gpus="device=0" <image>
```

### Real-Time Kernel

- Standard JetPack images are sufficient for most robotics workloads
- For hard real-time requirements, consider PREEMPT-RT patches
- Most mobile manipulation tasks don't require hard real-time

### Minimize Contention

- Isolate real-time ROS2 nodes in separate containers
- Avoid running heavy non-RT workloads concurrently
- Use CPU affinity for critical processes

## 5. Multi-Container Architecture for RUFUS

### Container Decomposition

RUFUS should use a multi-container architecture for modularity and isolation:

#### Container 1: ROS2 Core & Navigation
- **Purpose**: Core ROS2 nodes, navigation stack, SLAM
- **Packages**: nav2, slam_toolbox, robot_localization
- **Resources**: 2-3 CPU cores, 4GB RAM
- **Network**: host mode for low latency

#### Container 2: Hardware Drivers
- **Purpose**: DDSM315 chassis driver, ST3215 arm driver
- **Devices**: `/dev/ttyUSB0`, `/dev/ttyUSB1` (RS485 converters)
- **Resources**: 1 CPU core, 1GB RAM
- **Network**: host mode

#### Container 3: Perception & AI
- **Purpose**: Camera processing, object detection, depth processing
- **GPU**: Required for inference
- **Packages**: YOLOv8, depth_image_proc
- **Resources**: 2 CPU cores, 4GB RAM, GPU access
- **Network**: host mode

#### Container 4: VR Teleoperation Server
- **Purpose**: Django web server, WebSocket handler, VR interface
- **Ports**: 8000 (Django), 8001 (WebSocket)
- **Resources**: 1 CPU core, 2GB RAM
- **Network**: bridge mode with port mapping

#### Container 5: Manipulation & MoveIt
- **Purpose**: Arm motion planning, trajectory execution
- **Packages**: moveit2, arm controllers
- **Resources**: 2 CPU cores, 3GB RAM
- **Network**: host mode

### Docker Compose Orchestration

Use Docker Compose for multi-container deployment with inter-container dependencies and networking.

## 6. Resource Management and Optimization

### Jetson Orin NX Specifications
- **CPU**: 8-core ARM Cortex-A78AE
- **GPU**: 1024-core NVIDIA Ampere with 32 Tensor Cores
- **RAM**: 8GB or 16GB unified memory
- **Power Modes**: 10W, 15W, 25W

### Resource Allocation Strategy

Total available resources must be distributed across containers:

| Container | CPU Cores | RAM | GPU | Priority |
|-----------|-----------|-----|-----|----------|
| ROS2 Core & Navigation | 2.5 | 4GB | No | High |
| Hardware Drivers | 1.0 | 1GB | No | Critical |
| Perception & AI | 2.0 | 4GB | Yes | High |
| VR Teleoperation | 1.0 | 2GB | No | Medium |
| Manipulation & MoveIt | 1.5 | 3GB | No | High |
| **Total** | **8.0** | **14GB** | **Shared** | - |

### Device Mounting

Mount only necessary devices to each container:
```bash
--device /dev/ttyUSB0:/dev/ttyUSB0  # Chassis RS485
--device /dev/ttyUSB1:/dev/ttyUSB1  # Arm RS485
--device /dev/video0:/dev/video0    # Camera
```

### Volume Mounting

Use bind mounts for development and named volumes for persistence:
```bash
-v ./ros2_ws:/workspace/ros2_ws     # Source code
-v rufus_logs:/var/log/rufus        # Persistent logs
-v rufus_maps:/maps                 # SLAM maps
```

### Image Optimization

- Use multi-stage builds to reduce image size
- Leverage Docker layer caching
- Clean up apt cache: `rm -rf /var/lib/apt/lists/*`
- Combine RUN commands to minimize layers

### Monitoring

Use `tegrastats` on Jetson host to monitor resource usage:
```bash
sudo tegrastats
```

Monitor per-container resources:
```bash
docker stats
```

## 7. Example Dockerfiles and Docker Compose

### Dockerfile: ROS2 Core Container

```dockerfile
FROM dustynv/ros:humble-ros-base-l4t-r36.2.0

# Install navigation and SLAM packages
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-twist-mux \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    transforms3d \
    numpy

# Create workspace
RUN mkdir -p /workspace/ros2_ws/src
WORKDIR /workspace/ros2_ws

# Copy source code
COPY ./src /workspace/ros2_ws/src

# Build workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Setup entrypoint
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
```

### Dockerfile: Hardware Drivers Container

```dockerfile
FROM dustynv/ros:humble-ros-base-l4t-r36.2.0

# Install serial communication libraries
RUN apt-get update && apt-get install -y \
    python3-serial \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    pyserial \
    crcmod

# Create workspace
RUN mkdir -p /workspace/ros2_ws/src
WORKDIR /workspace/ros2_ws

# Copy driver source code
COPY ./src/rufus_drivers /workspace/ros2_ws/src/rufus_drivers

# Build workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Setup entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "rufus_drivers", "bringup.launch.py"]
```

### Dockerfile: VR Teleoperation Container

```dockerfile
FROM python:3.10-slim

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install Django and dependencies
RUN pip3 install --no-cache-dir \
    django==4.2 \
    channels==4.0 \
    channels-redis==4.1 \
    daphne==4.0 \
    roslibpy==1.5

# Create application directory
WORKDIR /app

# Copy Django application
COPY ./django_app /app

# Expose ports
EXPOSE 8000 8001

# Run Django with Daphne (ASGI server)
CMD ["daphne", "-b", "0.0.0.0", "-p", "8000", "rufus_web.asgi:application"]
```

### docker-compose.yml: Complete RUFUS System

```yaml
version: '3.8'

services:
  ros2-core:
    build:
      context: ./docker/ros2_core
      dockerfile: Dockerfile
    image: rufus/ros2-core:latest
    container_name: rufus_ros2_core
    runtime: nvidia
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=42
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      - CYCLONEDDS_URI=file:///cyclonedds.xml
    volumes:
      - ./ros2_ws:/workspace/ros2_ws
      - ./config:/config
      - rufus_logs:/var/log/rufus
      - rufus_maps:/maps
    deploy:
      resources:
        limits:
          cpus: '2.5'
          memory: 4G
    restart: unless-stopped
    command: ros2 launch rufus_bringup bringup.launch.py

  hardware-drivers:
    build:
      context: ./docker/drivers
      dockerfile: Dockerfile
    image: rufus/drivers:latest
    container_name: rufus_drivers
    runtime: nvidia
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=42
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0  # Chassis RS485
      - /dev/ttyUSB1:/dev/ttyUSB1  # Arm RS485
    volumes:
      - ./ros2_ws/src/rufus_drivers:/workspace/ros2_ws/src/rufus_drivers
    deploy:
      resources:
        limits:
          cpus: '1.0'
          memory: 1G
    restart: unless-stopped
    depends_on:
      - ros2-core

  perception:
    build:
      context: ./docker/perception
      dockerfile: Dockerfile
    image: rufus/perception:latest
    container_name: rufus_perception
    runtime: nvidia
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=42
      - NVIDIA_VISIBLE_DEVICES=0
    devices:
      - /dev/video0:/dev/video0  # Camera
    volumes:
      - ./ros2_ws/src/rufus_perception:/workspace/ros2_ws/src/rufus_perception
      - ./models:/models
    deploy:
      resources:
        limits:
          cpus: '2.0'
          memory: 4G
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]
    restart: unless-stopped
    depends_on:
      - ros2-core

  vr-teleoperation:
    build:
      context: ./docker/vr_teleop
      dockerfile: Dockerfile
    image: rufus/vr-teleop:latest
    container_name: rufus_vr_teleop
    network_mode: bridge
    ports:
      - "8000:8000"  # Django HTTP
      - "8001:8001"  # WebSocket
    environment:
      - DJANGO_SETTINGS_MODULE=rufus_web.settings
      - ROS_BRIDGE_HOST=172.17.0.1  # Docker bridge gateway
      - ROS_BRIDGE_PORT=9090
    volumes:
      - ./django_app:/app
      - ./static:/app/static
    deploy:
      resources:
        limits:
          cpus: '1.0'
          memory: 2G
    restart: unless-stopped
    depends_on:
      - ros2-core

  manipulation:
    build:
      context: ./docker/manipulation
      dockerfile: Dockerfile
    image: rufus/manipulation:latest
    container_name: rufus_manipulation
    runtime: nvidia
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=42
    volumes:
      - ./ros2_ws/src/rufus_manipulation:/workspace/ros2_ws/src/rufus_manipulation
      - ./config/moveit:/config/moveit
    deploy:
      resources:
        limits:
          cpus: '1.5'
          memory: 3G
    restart: unless-stopped
    depends_on:
      - ros2-core
      - hardware-drivers

  rosbridge:
    image: dustynv/ros:humble-ros-base-l4t-r36.2.0
    container_name: rufus_rosbridge
    runtime: nvidia
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=42
    command: ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
    deploy:
      resources:
        limits:
          cpus: '0.5'
          memory: 512M
    restart: unless-stopped
    depends_on:
      - ros2-core

volumes:
  rufus_logs:
  rufus_maps:
```

### ros_entrypoint.sh

```bash
#!/bin/bash
set -e

# Source ROS2 setup
source /opt/ros/humble/setup.bash

# Source workspace if it exists
if [ -f /workspace/ros2_ws/install/setup.bash ]; then
    source /workspace/ros2_ws/install/setup.bash
fi

# Execute command
exec "$@"
```

## Key Takeaways for RUFUS

1. **Use ROS2 Humble**: LTS support, proven stability, extensive ecosystem
2. **Multi-Container Architecture**: Isolate concerns, improve maintainability
3. **Host Networking for ROS2**: Minimize latency for DDS communication
4. **Resource Limits**: Prevent any single container from monopolizing resources
5. **GPU Acceleration**: Enable for perception and AI workloads
6. **Docker Compose**: Orchestrate complex multi-container system
7. **jetson-containers**: Leverage Dusty-NV's pre-built images and scripts
8. **Monitoring**: Use tegrastats and docker stats for performance tuning

## References

- [jetson-containers GitHub](https://github.com/dusty-nv/jetson-containers)
- [NVIDIA Jetson Linux Developer Guide](https://docs.nvidia.com/jetson/archives/r36.2/DeveloperGuide/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Docker Compose Documentation](https://docs.docker.com/compose/)

