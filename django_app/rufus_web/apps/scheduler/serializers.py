"""
Scheduler Serializers

Serializers for the scheduler API endpoints.
"""

from rest_framework import serializers
from django_celery_beat.models import PeriodicTask, CrontabSchedule, IntervalSchedule
from .models import (
    ScheduledTaskTemplate,
    ScheduledTask,
    TaskExecutionLog,
    TaskNotification
)
import json


class ScheduledTaskTemplateSerializer(serializers.ModelSerializer):
    """Serializer for task templates"""
    
    task_type_display = serializers.CharField(source='get_task_type_display', read_only=True)
    scheduled_tasks_count = serializers.SerializerMethodField()
    
    class Meta:
        model = ScheduledTaskTemplate
        fields = [
            'id',
            'name',
            'task_type',
            'task_type_display',
            'celery_task',
            'description',
            'default_args',
            'default_kwargs',
            'is_active',
            'scheduled_tasks_count',
            'created_at',
            'updated_at'
        ]
        read_only_fields = ['created_at', 'updated_at']
    
    def get_scheduled_tasks_count(self, obj):
        """Get count of tasks using this template"""
        return obj.scheduled_tasks.count()


class ScheduledTaskSerializer(serializers.ModelSerializer):
    """Serializer for scheduled tasks"""
    
    status_display = serializers.CharField(source='get_status_display', read_only=True)
    schedule_display = serializers.CharField(source='get_schedule_display', read_only=True)
    template_name = serializers.CharField(source='template.name', read_only=True)
    created_by_username = serializers.CharField(source='created_by.username', read_only=True)
    periodic_task_name = serializers.CharField(source='periodic_task.name', read_only=True)
    periodic_task_enabled = serializers.BooleanField(source='periodic_task.enabled', read_only=True)
    
    class Meta:
        model = ScheduledTask
        fields = [
            'id',
            'template',
            'template_name',
            'periodic_task',
            'periodic_task_name',
            'periodic_task_enabled',
            'name',
            'description',
            'status',
            'status_display',
            'schedule_display',
            'created_by',
            'created_by_username',
            'created_at',
            'updated_at',
            'last_run_at',
            'next_run_at',
            'total_run_count'
        ]
        read_only_fields = [
            'created_at',
            'updated_at',
            'last_run_at',
            'next_run_at',
            'total_run_count',
            'created_by'
        ]


class CreateScheduledTaskSerializer(serializers.Serializer):
    """Serializer for creating a new scheduled task"""
    
    SCHEDULE_TYPES = [
        ('crontab', 'Crontab'),
        ('interval', 'Interval'),
    ]
    
    name = serializers.CharField(max_length=200)
    description = serializers.CharField(required=False, allow_blank=True)
    template_id = serializers.IntegerField(required=False, allow_null=True)
    celery_task = serializers.CharField(max_length=200)
    
    schedule_type = serializers.ChoiceField(choices=SCHEDULE_TYPES)
    
    # Crontab fields
    crontab_minute = serializers.CharField(required=False, default='*')
    crontab_hour = serializers.CharField(required=False, default='*')
    crontab_day_of_week = serializers.CharField(required=False, default='*')
    crontab_day_of_month = serializers.CharField(required=False, default='*')
    crontab_month_of_year = serializers.CharField(required=False, default='*')
    
    # Interval fields
    interval_every = serializers.IntegerField(required=False)
    interval_period = serializers.ChoiceField(
        choices=['seconds', 'minutes', 'hours', 'days'],
        required=False
    )
    
    # Task arguments
    args = serializers.JSONField(required=False, default=list)
    kwargs = serializers.JSONField(required=False, default=dict)
    
    enabled = serializers.BooleanField(default=True)
    
    def validate(self, data):
        """Validate schedule configuration"""
        schedule_type = data.get('schedule_type')
        
        if schedule_type == 'interval':
            if not data.get('interval_every') or not data.get('interval_period'):
                raise serializers.ValidationError(
                    "interval_every and interval_period are required for interval schedule"
                )
        
        return data
    
    def create(self, validated_data):
        """Create scheduled task with periodic task"""
        from django.contrib.auth import get_user_model
        
        # Extract data
        name = validated_data['name']
        description = validated_data.get('description', '')
        template_id = validated_data.get('template_id')
        celery_task = validated_data['celery_task']
        schedule_type = validated_data['schedule_type']
        args = validated_data.get('args', [])
        kwargs = validated_data.get('kwargs', {})
        enabled = validated_data.get('enabled', True)
        
        # Get template if provided
        template = None
        if template_id:
            template = ScheduledTaskTemplate.objects.get(id=template_id)
        
        # Create schedule
        if schedule_type == 'crontab':
            schedule, _ = CrontabSchedule.objects.get_or_create(
                minute=validated_data.get('crontab_minute', '*'),
                hour=validated_data.get('crontab_hour', '*'),
                day_of_week=validated_data.get('crontab_day_of_week', '*'),
                day_of_month=validated_data.get('crontab_day_of_month', '*'),
                month_of_year=validated_data.get('crontab_month_of_year', '*'),
            )
            schedule_kwargs = {'crontab': schedule}
        else:
            schedule, _ = IntervalSchedule.objects.get_or_create(
                every=validated_data['interval_every'],
                period=validated_data['interval_period']
            )
            schedule_kwargs = {'interval': schedule}
        
        # Create periodic task
        periodic_task = PeriodicTask.objects.create(
            name=f"{name}_{timezone.now().timestamp()}",
            task=celery_task,
            args=json.dumps(args),
            kwargs=json.dumps(kwargs),
            enabled=enabled,
            **schedule_kwargs
        )
        
        # Create scheduled task
        scheduled_task = ScheduledTask.objects.create(
            template=template,
            periodic_task=periodic_task,
            name=name,
            description=description,
            status='active' if enabled else 'paused',
            created_by=self.context['request'].user
        )
        
        return scheduled_task


class TaskExecutionLogSerializer(serializers.ModelSerializer):
    """Serializer for task execution logs"""
    
    status_display = serializers.CharField(source='get_status_display', read_only=True)
    scheduled_task_name = serializers.CharField(source='scheduled_task.name', read_only=True)
    duration_display = serializers.SerializerMethodField()
    
    class Meta:
        model = TaskExecutionLog
        fields = [
            'id',
            'scheduled_task',
            'scheduled_task_name',
            'task_id',
            'task_name',
            'status',
            'status_display',
            'args',
            'kwargs',
            'result',
            'error_message',
            'traceback',
            'started_at',
            'completed_at',
            'duration_seconds',
            'duration_display',
            'retry_count',
            'created_at'
        ]
    
    def get_duration_display(self, obj):
        """Get human-readable duration"""
        if obj.duration_seconds is not None:
            if obj.duration_seconds < 1:
                return f"{obj.duration_seconds * 1000:.0f}ms"
            elif obj.duration_seconds < 60:
                return f"{obj.duration_seconds:.1f}s"
            else:
                minutes = int(obj.duration_seconds // 60)
                seconds = int(obj.duration_seconds % 60)
                return f"{minutes}m {seconds}s"
        return None


class TaskNotificationSerializer(serializers.ModelSerializer):
    """Serializer for task notifications"""
    
    event_type_display = serializers.CharField(source='get_event_type_display', read_only=True)
    notification_method_display = serializers.CharField(source='get_notification_method_display', read_only=True)
    scheduled_task_name = serializers.CharField(source='scheduled_task.name', read_only=True)
    
    class Meta:
        model = TaskNotification
        fields = [
            'id',
            'scheduled_task',
            'scheduled_task_name',
            'event_type',
            'event_type_display',
            'notification_method',
            'notification_method_display',
            'recipient',
            'is_active',
            'created_at'
        ]
        read_only_fields = ['created_at']

