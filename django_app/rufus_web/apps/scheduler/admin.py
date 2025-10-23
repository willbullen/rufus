"""
Scheduler Admin Interface

Provides a comprehensive admin interface for managing scheduled tasks,
viewing execution logs, and configuring notifications.
"""

from django.contrib import admin
from django.utils.html import format_html
from django.urls import reverse
from django.utils import timezone
from django_celery_beat.models import PeriodicTask, CrontabSchedule, IntervalSchedule
from .models import (
    ScheduledTaskTemplate,
    ScheduledTask,
    TaskExecutionLog,
    TaskNotification
)


@admin.register(ScheduledTaskTemplate)
class ScheduledTaskTemplateAdmin(admin.ModelAdmin):
    """Admin interface for task templates"""
    
    list_display = ['name', 'task_type', 'celery_task', 'is_active', 'created_at']
    list_filter = ['task_type', 'is_active', 'created_at']
    search_fields = ['name', 'celery_task', 'description']
    readonly_fields = ['created_at', 'updated_at']
    
    fieldsets = (
        ('Basic Information', {
            'fields': ('name', 'task_type', 'celery_task', 'description', 'is_active')
        }),
        ('Default Arguments', {
            'fields': ('default_args', 'default_kwargs'),
            'classes': ('collapse',)
        }),
        ('Metadata', {
            'fields': ('created_at', 'updated_at'),
            'classes': ('collapse',)
        }),
    )
    
    def get_queryset(self, request):
        qs = super().get_queryset(request)
        return qs.prefetch_related('scheduled_tasks')


class TaskNotificationInline(admin.TabularInline):
    """Inline admin for task notifications"""
    model = TaskNotification
    extra = 1
    fields = ['event_type', 'notification_method', 'recipient', 'is_active']


@admin.register(ScheduledTask)
class ScheduledTaskAdmin(admin.ModelAdmin):
    """Admin interface for scheduled tasks"""
    
    list_display = [
        'name',
        'status_badge',
        'get_schedule',
        'last_run_at',
        'next_run_at',
        'total_run_count',
        'action_buttons'
    ]
    list_filter = ['status', 'created_at', 'template__task_type']
    search_fields = ['name', 'description', 'periodic_task__name']
    readonly_fields = [
        'created_at',
        'updated_at',
        'last_run_at',
        'next_run_at',
        'total_run_count',
        'get_schedule_display'
    ]
    inlines = [TaskNotificationInline]
    
    fieldsets = (
        ('Basic Information', {
            'fields': ('name', 'description', 'template', 'status')
        }),
        ('Schedule Configuration', {
            'fields': ('periodic_task', 'get_schedule_display'),
            'description': 'Link to the Celery Beat periodic task configuration'
        }),
        ('Execution Information', {
            'fields': ('last_run_at', 'next_run_at', 'total_run_count'),
            'classes': ('collapse',)
        }),
        ('Metadata', {
            'fields': ('created_by', 'created_at', 'updated_at'),
            'classes': ('collapse',)
        }),
    )
    
    def status_badge(self, obj):
        """Display status as a colored badge"""
        colors = {
            'active': '#28a745',
            'paused': '#ffc107',
            'completed': '#17a2b8',
            'failed': '#dc3545',
        }
        color = colors.get(obj.status, '#6c757d')
        return format_html(
            '<span style="background-color: {}; color: white; padding: 3px 10px; '
            'border-radius: 3px; font-weight: bold;">{}</span>',
            color,
            obj.get_status_display()
        )
    status_badge.short_description = 'Status'
    
    def get_schedule(self, obj):
        """Display schedule information"""
        return obj.get_schedule_display()
    get_schedule.short_description = 'Schedule'
    
    def action_buttons(self, obj):
        """Display action buttons for pause/resume"""
        if obj.status == 'active':
            return format_html(
                '<a class="button" href="{}">Pause</a>',
                reverse('admin:scheduler_pause_task', args=[obj.pk])
            )
        elif obj.status == 'paused':
            return format_html(
                '<a class="button" href="{}">Resume</a>',
                reverse('admin:scheduler_resume_task', args=[obj.pk])
            )
        return '-'
    action_buttons.short_description = 'Actions'
    
    def save_model(self, request, obj, form, change):
        """Set created_by to current user if creating new task"""
        if not change:
            obj.created_by = request.user
        super().save_model(request, obj, form, change)
    
    actions = ['pause_tasks', 'resume_tasks']
    
    def pause_tasks(self, request, queryset):
        """Bulk action to pause tasks"""
        count = 0
        for task in queryset:
            task.pause()
            count += 1
        self.message_user(request, f'{count} task(s) paused successfully.')
    pause_tasks.short_description = 'Pause selected tasks'
    
    def resume_tasks(self, request, queryset):
        """Bulk action to resume tasks"""
        count = 0
        for task in queryset:
            task.resume()
            count += 1
        self.message_user(request, f'{count} task(s) resumed successfully.')
    resume_tasks.short_description = 'Resume selected tasks'


@admin.register(TaskExecutionLog)
class TaskExecutionLogAdmin(admin.ModelAdmin):
    """Admin interface for task execution logs"""
    
    list_display = [
        'task_name',
        'status_badge',
        'started_at',
        'duration_display',
        'retry_count',
        'view_details'
    ]
    list_filter = ['status', 'started_at', 'scheduled_task']
    search_fields = ['task_name', 'task_id', 'error_message']
    readonly_fields = [
        'task_id',
        'task_name',
        'status',
        'args',
        'kwargs',
        'result',
        'error_message',
        'traceback',
        'started_at',
        'completed_at',
        'duration_seconds',
        'retry_count',
        'created_at'
    ]
    
    fieldsets = (
        ('Task Information', {
            'fields': ('scheduled_task', 'task_id', 'task_name', 'status')
        }),
        ('Execution Details', {
            'fields': ('args', 'kwargs', 'result')
        }),
        ('Timing', {
            'fields': ('started_at', 'completed_at', 'duration_seconds', 'retry_count')
        }),
        ('Error Information', {
            'fields': ('error_message', 'traceback'),
            'classes': ('collapse',)
        }),
    )
    
    def has_add_permission(self, request):
        """Disable adding logs manually"""
        return False
    
    def has_change_permission(self, request, obj=None):
        """Make logs read-only"""
        return False
    
    def status_badge(self, obj):
        """Display status as a colored badge"""
        colors = {
            'pending': '#6c757d',
            'running': '#007bff',
            'success': '#28a745',
            'failure': '#dc3545',
            'retry': '#ffc107',
        }
        color = colors.get(obj.status, '#6c757d')
        return format_html(
            '<span style="background-color: {}; color: white; padding: 3px 10px; '
            'border-radius: 3px; font-weight: bold;">{}</span>',
            color,
            obj.get_status_display()
        )
    status_badge.short_description = 'Status'
    
    def duration_display(self, obj):
        """Display duration in human-readable format"""
        if obj.duration_seconds is not None:
            if obj.duration_seconds < 1:
                return f"{obj.duration_seconds * 1000:.0f}ms"
            elif obj.duration_seconds < 60:
                return f"{obj.duration_seconds:.1f}s"
            else:
                minutes = int(obj.duration_seconds // 60)
                seconds = int(obj.duration_seconds % 60)
                return f"{minutes}m {seconds}s"
        return '-'
    duration_display.short_description = 'Duration'
    
    def view_details(self, obj):
        """Link to view full details"""
        return format_html(
            '<a class="button" href="{}">View Details</a>',
            reverse('admin:scheduler_taskexecutionlog_change', args=[obj.pk])
        )
    view_details.short_description = 'Details'


@admin.register(TaskNotification)
class TaskNotificationAdmin(admin.ModelAdmin):
    """Admin interface for task notifications"""
    
    list_display = [
        'scheduled_task',
        'event_type',
        'notification_method',
        'recipient',
        'is_active',
        'created_at'
    ]
    list_filter = ['event_type', 'notification_method', 'is_active', 'created_at']
    search_fields = ['scheduled_task__name', 'recipient']
    readonly_fields = ['created_at']
    
    fieldsets = (
        ('Notification Configuration', {
            'fields': ('scheduled_task', 'event_type', 'notification_method', 'recipient', 'is_active')
        }),
        ('Metadata', {
            'fields': ('created_at',),
            'classes': ('collapse',)
        }),
    )


# Customize django-celery-beat admin
class CustomPeriodicTaskAdmin(admin.ModelAdmin):
    """Enhanced admin for Celery Beat periodic tasks"""
    
    list_display = [
        'name',
        'enabled',
        'interval',
        'crontab',
        'last_run_at',
        'total_run_count'
    ]
    list_filter = ['enabled', 'last_run_at']
    search_fields = ['name', 'task']
    
    fieldsets = (
        ('Task Configuration', {
            'fields': ('name', 'task', 'enabled')
        }),
        ('Schedule', {
            'fields': ('interval', 'crontab', 'solar', 'clocked'),
            'description': 'Choose one schedule type'
        }),
        ('Arguments', {
            'fields': ('args', 'kwargs'),
            'classes': ('collapse',)
        }),
        ('Execution Options', {
            'fields': ('queue', 'exchange', 'routing_key', 'priority', 'expires', 'expire_seconds'),
            'classes': ('collapse',)
        }),
        ('Statistics', {
            'fields': ('last_run_at', 'total_run_count', 'date_changed'),
            'classes': ('collapse',)
        }),
    )


# Unregister default and register custom
admin.site.unregister(PeriodicTask)
admin.site.register(PeriodicTask, CustomPeriodicTaskAdmin)

