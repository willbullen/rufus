from django.db import models
from django.contrib.auth.models import User


class RobotStatus(models.Model):
    """Current status of the RUFUS robot"""
    timestamp = models.DateTimeField(auto_now=True)
    is_connected = models.BooleanField(default=False)
    battery_voltage = models.FloatField(default=0.0)
    battery_percentage = models.FloatField(default=0.0)
    cpu_temperature = models.FloatField(default=0.0)
    gpu_temperature = models.FloatField(default=0.0)
    cpu_usage = models.FloatField(default=0.0)
    memory_usage = models.FloatField(default=0.0)
    
    class Meta:
        verbose_name = "Robot Status"
        verbose_name_plural = "Robot Status"
        ordering = ['-timestamp']
    
    def __str__(self):
        return f"Status at {self.timestamp}"


class ControlSession(models.Model):
    """Track control sessions for logging and analysis"""
    SESSION_TYPES = [
        ('web', 'Web Interface'),
        ('vr', 'VR Teleoperation'),
        ('auto', 'Autonomous'),
    ]
    
    user = models.ForeignKey(User, on_delete=models.CASCADE)
    session_type = models.CharField(max_length=10, choices=SESSION_TYPES)
    start_time = models.DateTimeField(auto_now_add=True)
    end_time = models.DateTimeField(null=True, blank=True)
    duration_seconds = models.IntegerField(default=0)
    commands_sent = models.IntegerField(default=0)
    
    class Meta:
        ordering = ['-start_time']
    
    def __str__(self):
        return f"{self.user.username} - {self.session_type} - {self.start_time}"


class SavedPose(models.Model):
    """Saved arm poses for quick recall"""
    name = models.CharField(max_length=100)
    description = models.TextField(blank=True)
    user = models.ForeignKey(User, on_delete=models.CASCADE)
    created_at = models.DateTimeField(auto_now_add=True)
    
    # Joint positions (in radians)
    base_joint = models.FloatField(default=0.0)
    shoulder_joint = models.FloatField(default=0.0)
    elbow_joint = models.FloatField(default=0.0)
    wrist_pitch_joint = models.FloatField(default=0.0)
    wrist_roll_joint = models.FloatField(default=0.0)
    gripper_joint = models.FloatField(default=0.0)
    
    class Meta:
        ordering = ['name']
    
    def __str__(self):
        return self.name
    
    def to_joint_array(self):
        """Return joint positions as array"""
        return [
            self.base_joint,
            self.shoulder_joint,
            self.elbow_joint,
            self.wrist_pitch_joint,
            self.wrist_roll_joint,
            self.gripper_joint
        ]


class NavigationWaypoint(models.Model):
    """Saved navigation waypoints"""
    name = models.CharField(max_length=100)
    description = models.TextField(blank=True)
    user = models.ForeignKey(User, on_delete=models.CASCADE)
    created_at = models.DateTimeField(auto_now_add=True)
    
    # Position
    x = models.FloatField()
    y = models.FloatField()
    z = models.FloatField(default=0.0)
    
    # Orientation (quaternion)
    qx = models.FloatField(default=0.0)
    qy = models.FloatField(default=0.0)
    qz = models.FloatField(default=0.0)
    qw = models.FloatField(default=1.0)
    
    class Meta:
        ordering = ['name']
    
    def __str__(self):
        return self.name

