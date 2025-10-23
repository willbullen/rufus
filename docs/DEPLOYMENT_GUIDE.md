# RUFUS Deployment and Testing Guide

This guide provides step-by-step instructions for deploying and testing the RUFUS robot system.

## Quick Start (Simulation Mode)

The system is configured to run in simulation mode by default, allowing you to test without physical hardware.

### Prerequisites

- Docker and Docker Compose installed
- Git installed
- 16GB+ RAM recommended
- For Jetson deployment: NVIDIA Jetson Orin NX with JetPack 6.0+

### 1. Clone and Configure

```bash
# Clone the repository
git clone https://github.com/willbullen/rufus.git
cd rufus

# Copy environment template
cp .env.example .env

# Edit environment variables
nano .env
```

### 2. Start the System

```bash
# Build all containers (first time only, takes 10-20 minutes)
docker-compose build

# Start all services
docker-compose up -d

# View logs
docker-compose logs -f
```

### 3. Access the Web Interface

Open your browser to:
- **Frontend**: http://localhost:3000
- **Django Admin**: http://localhost:8000/admin
- **API**: http://localhost:8000/api

Default credentials (create via Django admin):
- Username: admin
- Password: (set during first setup)

## System Architecture

### Container Services

| Service | Port | Purpose |
|---------|------|---------|
| **db** | 5432 | PostgreSQL database |
| **redis** | 6379 | Cache and message broker |
| **web-backend** | 8000 | Django API with WebSocket |
| **web-frontend** | 3000 | React UI |
| **celery-worker** | - | Background tasks |
| **celery-beat** | - | Scheduled tasks |

### ROS2 Services (Jetson Only)

| Service | Purpose |
|---------|---------|
| **ros2-core** | Navigation, SLAM, core services |
| **drivers** | Hardware drivers (DDSM315, ST3215) |
| **perception** | Vision and Lidar processing |
| **manipulation** | Arm motion planning |
| **rosbridge** | WebSocket bridge to ROS2 |

## Testing the System

### 1. Test Database Connection

```bash
# Check database is running
docker-compose ps db

# Run Django migrations
docker-compose exec web-backend python manage.py migrate

# Create superuser
docker-compose exec web-backend python manage.py createsuperuser
```

### 2. Test WebSocket Connection

Open browser console on http://localhost:3000 and run:

```javascript
const ws = new WebSocket('ws://localhost:8000/ws/vr/');
ws.onopen = () => console.log('Connected!');
ws.onmessage = (e) => console.log('Message:', e.data);
ws.send(JSON.stringify({type: 'ping'}));
```

Expected response: `{type: 'pong'}`

### 3. Test Chassis Control

From the Manual Control page:
1. Click the arrow buttons or use keyboard (WASD/arrows)
2. Check browser console for WebSocket messages
3. Verify no errors in backend logs:
   ```bash
   docker-compose logs web-backend | grep ERROR
   ```

### 4. Test Arm Control

From the Manual Control page:
1. Move the joint sliders
2. Click "Home Position" button
3. Verify joint positions update smoothly

### 5. Test ROS2 Integration (Jetson Only)

```bash
# Check ROS2 nodes are running
docker exec rufus_ros2_core ros2 node list

# Expected output:
# /ddsm315_driver
# /st3215_driver
# /robot_state_publisher

# Check topics
docker exec rufus_ros2_core ros2 topic list

# Test publishing velocity
docker exec rufus_ros2_core ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}" --once

# Monitor joint states
docker exec rufus_ros2_core ros2 topic echo /joint_states
```

## Deployment to Jetson Orin NX

### 1. Prepare Jetson

```bash
# SSH into Jetson
ssh rufus@<jetson-ip>

# Clone repository
git clone https://github.com/willbullen/rufus.git
cd rufus

# Configure environment
cp .env.example .env
nano .env
```

Update `.env` for hardware:
```bash
# Disable simulation mode
SIMULATION_MODE=false

# Set hardware ports
CHASSIS_PORT=/dev/ttyUSB0
ARM_PORT=/dev/ttyUSB1

# Set ROS domain
ROS_DOMAIN_ID=42
```

### 2. Connect Hardware

1. **Chassis Motors (DDSM315)**
   - Connect USB-to-RS485 converter to USB port
   - Verify: `ls /dev/ttyUSB*` shows `/dev/ttyUSB0`
   - Wire RS485 A/B to motor bus

2. **Arm Servos (ST3215)**
   - Connect second USB-to-RS485 converter
   - Verify: `ls /dev/ttyUSB*` shows `/dev/ttyUSB1`
   - Wire RS485 A/B to servo bus

3. **Sensors**
   - Connect Lidar to USB 2.0 port
   - Connect depth camera to USB 3.0 port

### 3. Deploy Containers

```bash
# Build for Jetson (uses ARM64 images)
docker-compose build

# Start all services
docker-compose up -d

# Check all containers are running
docker-compose ps
```

### 4. Verify Hardware Communication

```bash
# Test chassis motors
docker exec rufus_drivers python3 -c "
import serial
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
print('Chassis port open:', ser.is_open)
ser.close()
"

# Test arm servos
docker exec rufus_drivers python3 -c "
import serial
ser = serial.Serial('/dev/ttyUSB1', 1000000, timeout=1)
print('Arm port open:', ser.is_open)
ser.close()
"
```

### 5. Test Robot Movement

```bash
# Test chassis (should move forward slowly)
docker exec rufus_ros2_core ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}" --once

# EMERGENCY STOP
docker exec rufus_ros2_core ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" --once

# Test arm (move to home position)
docker exec rufus_ros2_core ros2 topic pub /arm_controller/joint_trajectory \
  trajectory_msgs/JointTrajectory \
  "{joint_names: ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'wrist_roll_joint', 'gripper_joint'], \
   points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}" --once
```

## Troubleshooting

### Container Issues

**Problem**: Container won't start
```bash
# Check logs
docker-compose logs <service-name>

# Rebuild container
docker-compose build <service-name>
docker-compose up -d <service-name>
```

**Problem**: Database connection error
```bash
# Check database is running
docker-compose ps db

# Restart database
docker-compose restart db

# Wait for database to be ready
docker-compose logs -f db | grep "ready to accept connections"
```

### WebSocket Issues

**Problem**: WebSocket connection fails
```bash
# Check backend is running
docker-compose ps web-backend

# Check Redis is running
docker-compose ps redis

# View backend logs
docker-compose logs -f web-backend

# Test Redis connection
docker-compose exec redis redis-cli ping
# Should return: PONG
```

### ROS2 Issues

**Problem**: ROS2 nodes not communicating
```bash
# Check ROS_DOMAIN_ID matches
docker exec rufus_ros2_core printenv ROS_DOMAIN_ID
docker exec rufus_drivers printenv ROS_DOMAIN_ID

# List nodes
docker exec rufus_ros2_core ros2 node list

# Check topic list
docker exec rufus_ros2_core ros2 topic list

# Monitor topic
docker exec rufus_ros2_core ros2 topic echo /joint_states
```

**Problem**: Hardware not responding
```bash
# Check USB devices
lsusb

# Check serial ports
ls -l /dev/ttyUSB*

# Add user to dialout group
sudo usermod -aG dialout $USER
# Logout and login again

# Check permissions
ls -l /dev/ttyUSB*
# Should show: crw-rw---- 1 root dialout
```

### Performance Issues

**Problem**: High CPU/memory usage
```bash
# Check resource usage
docker stats

# Limit container resources in docker-compose.yml:
services:
  service-name:
    deploy:
      resources:
        limits:
          cpus: '2.0'
          memory: 4G
```

**Problem**: Slow response
```bash
# Check network latency
ping <jetson-ip>

# Check WebSocket latency in browser console
# Should be < 50ms for good VR performance
```

## Monitoring and Maintenance

### View System Status

```bash
# All containers
docker-compose ps

# Resource usage
docker stats

# Disk usage
docker system df

# Logs
docker-compose logs -f --tail=100
```

### Backup Database

```bash
# Backup
docker-compose exec db pg_dump -U rufus rufus > backup_$(date +%Y%m%d).sql

# Restore
cat backup_20250123.sql | docker-compose exec -T db psql -U rufus rufus
```

### Update System

```bash
# Pull latest code
git pull origin main

# Rebuild containers
docker-compose build

# Restart services
docker-compose down
docker-compose up -d

# Run migrations
docker-compose exec web-backend python manage.py migrate
```

### Clean Up

```bash
# Stop all services
docker-compose down

# Remove volumes (WARNING: deletes data)
docker-compose down -v

# Remove images
docker-compose down --rmi all

# Clean Docker system
docker system prune -a
```

## Production Deployment

### Security Checklist

- [ ] Change default passwords
- [ ] Set strong `DJANGO_SECRET_KEY`
- [ ] Set `DJANGO_DEBUG=False`
- [ ] Configure `ALLOWED_HOSTS`
- [ ] Enable HTTPS/SSL
- [ ] Set up firewall rules
- [ ] Enable 2FA for admin users
- [ ] Regular security updates
- [ ] Backup database regularly

### Performance Optimization

1. **Database**
   - Enable connection pooling
   - Add database indexes
   - Regular VACUUM operations

2. **Redis**
   - Increase memory limit
   - Enable persistence
   - Monitor memory usage

3. **Django**
   - Enable caching
   - Use gunicorn workers
   - Compress static files

4. **Frontend**
   - Build production bundle
   - Enable gzip compression
   - Use CDN for static assets

### Monitoring

Set up monitoring for:
- Container health
- CPU/GPU temperature
- Battery voltage
- Network latency
- Error rates
- Response times

Recommended tools:
- Prometheus + Grafana
- ELK Stack (Elasticsearch, Logstash, Kibana)
- Netdata

## Support

For issues and questions:
- GitHub Issues: https://github.com/willbullen/rufus/issues
- Documentation: https://github.com/willbullen/rufus/docs
- Email: support@rufus-robot.com

## License

MIT License - see LICENSE file for details

