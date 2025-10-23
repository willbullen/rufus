"""
Scheduler Models

This module defines models for managing scheduled tasks that are stored in the database
and executed by Celery Beat. Tasks can be managed through the Django admin interface.
"""

from django.db import models
from django.contrib.auth import get_user_model
from django.core.validators import MinValueValidator
from django_celery_beat.models import PeriodicTask, CrontabSchedule, IntervalSchedule
import json

User = get_user_model()


class ScheduledTaskTemplate(models.Model):
    """
    Template for creating scheduled tasks.
    Defines reusable task configurations that can be instantiated multiple times.
    """
    
    TASK_TYPES = [
        ('email', 'Send Email'),
        ('report', 'Generate Report'),
        ('cleanup', 'Data Cleanup'),
        ('backup', 'Database Backup'),
        ('api_call', 'API Call'),
        ('custom', 'Custom Task'),
    ]
    
    name = models.CharField(max_length=200, unique=True, help_text="Unique name for this task template")
    task_type = models.CharField(max_length=50, choices=TASK_TYPES, default='custom')
    celery_task = models.CharField(max_length=200, help_text="Celery task path (e.g., apps.scheduler.tasks.send_email)")
    description = models.TextField(blank=True, help_text="Description of what this task does")
    default_args = models.JSONField(default=dict, blank=True, help_text="Default arguments for the task")
    default_kwargs = models.JSONField(default=dict, blank=True, help_text="Default keyword arguments for the task")
    is_active = models.BooleanField(default=True)
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)
    
    class Meta:
        verbose_name = "Task Template"
        verbose_name_plural = "Task Templates"
        ordering = ['name']
    
    def __str__(self):
        return f"{self.name} ({self.get_task_type_display()})"


class ScheduledTask(models.Model):
    """
    Individual scheduled task instance.
    Links to a PeriodicTask from django-celery-beat for actual scheduling.
    """
    
    STATUS_CHOICES = [
        ('active', 'Active'),
        ('paused', 'Paused'),
        ('completed', 'Completed'),
        ('failed', 'Failed'),
    ]
    
    template = models.ForeignKey(
        ScheduledTaskTemplate,
        on_delete=models.CASCADE,
        related_name='scheduled_tasks',
        null=True,
        blank=True,
        help_text="Optional template to base this task on"
    )
    periodic_task = models.OneToOneField(
        PeriodicTask,
        on_delete=models.CASCADE,
        related_name='scheduled_task_info',
        help_text="Link to Celery Beat periodic task"
    )
    name = models.CharField(max_length=200, help_text="Human-readable name for this task")
    description = models.TextField(blank=True)
    status = models.CharField(max_length=20, choices=STATUS_CHOICES, default='active')
    created_by = models.ForeignKey(User, on_delete=models.SET_NULL, null=True, blank=True, related_name='created_tasks')
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)
    last_run_at = models.DateTimeField(null=True, blank=True)
    next_run_at = models.DateTimeField(null=True, blank=True)
    total_run_count = models.IntegerField(default=0, validators=[MinValueValidator(0)])
    
    class Meta:
        verbose_name = "Scheduled Task"
        verbose_name_plural = "Scheduled Tasks"
        ordering = ['-created_at']
    
    def __str__(self):
        return f"{self.name} ({self.get_status_display()})"
    
    def pause(self):
        """Pause the scheduled task"""
        self.periodic_task.enabled = False
        self.periodic_task.save()
        self.status = 'paused'
        self.save()
    
    def resume(self):
        """Resume the scheduled task"""
        self.periodic_task.enabled = True
        self.periodic_task.save()
        self.status = 'active'
        self.save()
    
    def get_schedule_display(self):
        """Get human-readable schedule information"""
        if self.periodic_task.crontab:
            return f"Crontab: {self.periodic_task.crontab}"
        elif self.periodic_task.interval:
            return f"Every {self.periodic_task.interval}"
        return "No schedule"


class TaskExecutionLog(models.Model):
    """
    Log of task executions for monitoring and debugging.
    """
    
    STATUS_CHOICES = [
        ('pending', 'Pending'),
        ('running', 'Running'),
        ('success', 'Success'),
        ('failure', 'Failure'),
        ('retry', 'Retry'),
    ]
    
    scheduled_task = models.ForeignKey(
        ScheduledTask,
        on_delete=models.CASCADE,
        related_name='execution_logs',
        null=True,
        blank=True
    )
    task_id = models.CharField(max_length=255, unique=True, help_text="Celery task ID")
    task_name = models.CharField(max_length=200)
    status = models.CharField(max_length=20, choices=STATUS_CHOICES, default='pending')
    args = models.JSONField(default=list, blank=True)
    kwargs = models.JSONField(default=dict, blank=True)
    result = models.JSONField(null=True, blank=True)
    error_message = models.TextField(blank=True)
    traceback = models.TextField(blank=True)
    started_at = models.DateTimeField(null=True, blank=True)
    completed_at = models.DateTimeField(null=True, blank=True)
    duration_seconds = models.FloatField(null=True, blank=True)
    retry_count = models.IntegerField(default=0)
    created_at = models.DateTimeField(auto_now_add=True)
    
    class Meta:
        verbose_name = "Task Execution Log"
        verbose_name_plural = "Task Execution Logs"
        ordering = ['-created_at']
        indexes = [
            models.Index(fields=['-created_at']),
            models.Index(fields=['status']),
            models.Index(fields=['task_id']),
        ]
    
    def __str__(self):
        return f"{self.task_name} - {self.get_status_display()} ({self.created_at})"
    
    def calculate_duration(self):
        """Calculate task duration if both timestamps are available"""
        if self.started_at and self.completed_at:
            delta = self.completed_at - self.started_at
            self.duration_seconds = delta.total_seconds()
            self.save()


class TaskNotification(models.Model):
    """
    Notification settings for task execution events.
    """
    
    EVENT_TYPES = [
        ('success', 'On Success'),
        ('failure', 'On Failure'),
        ('retry', 'On Retry'),
        ('all', 'All Events'),
    ]
    
    NOTIFICATION_METHODS = [
        ('email', 'Email'),
        ('webhook', 'Webhook'),
        ('slack', 'Slack'),
        ('discord', 'Discord'),
    ]
    
    scheduled_task = models.ForeignKey(
        ScheduledTask,
        on_delete=models.CASCADE,
        related_name='notifications'
    )
    event_type = models.CharField(max_length=20, choices=EVENT_TYPES)
    notification_method = models.CharField(max_length=20, choices=NOTIFICATION_METHODS)
    recipient = models.CharField(max_length=255, help_text="Email address, webhook URL, etc.")
    is_active = models.BooleanField(default=True)
    created_at = models.DateTimeField(auto_now_add=True)
    
    class Meta:
        verbose_name = "Task Notification"
        verbose_name_plural = "Task Notifications"
        unique_together = ['scheduled_task', 'event_type', 'notification_method', 'recipient']
    
    def __str__(self):
        return f"{self.scheduled_task.name} - {self.get_event_type_display()} via {self.get_notification_method_display()}"

