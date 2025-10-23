"""
Scheduler API Views

REST API endpoints for managing scheduled tasks programmatically.
"""

from rest_framework import viewsets, status
from rest_framework.decorators import action
from rest_framework.response import Response
from rest_framework.permissions import IsAuthenticated
from django_celery_beat.models import PeriodicTask, CrontabSchedule, IntervalSchedule
from django.utils import timezone
from .models import (
    ScheduledTaskTemplate,
    ScheduledTask,
    TaskExecutionLog,
    TaskNotification
)
from .serializers import (
    ScheduledTaskTemplateSerializer,
    ScheduledTaskSerializer,
    TaskExecutionLogSerializer,
    TaskNotificationSerializer,
    CreateScheduledTaskSerializer
)
import json


class ScheduledTaskTemplateViewSet(viewsets.ModelViewSet):
    """
    ViewSet for managing task templates.
    
    list: Get all task templates
    retrieve: Get a specific task template
    create: Create a new task template
    update: Update a task template
    destroy: Delete a task template
    """
    
    queryset = ScheduledTaskTemplate.objects.all()
    serializer_class = ScheduledTaskTemplateSerializer
    permission_classes = [IsAuthenticated]
    
    def get_queryset(self):
        """Filter active templates by default"""
        queryset = super().get_queryset()
        
        if self.request.query_params.get('active_only', 'true').lower() == 'true':
            queryset = queryset.filter(is_active=True)
        
        return queryset.order_by('name')


class ScheduledTaskViewSet(viewsets.ModelViewSet):
    """
    ViewSet for managing scheduled tasks.
    
    list: Get all scheduled tasks
    retrieve: Get a specific scheduled task
    create: Create a new scheduled task
    update: Update a scheduled task
    destroy: Delete a scheduled task
    pause: Pause a scheduled task
    resume: Resume a scheduled task
    run_now: Execute a task immediately
    """
    
    queryset = ScheduledTask.objects.all()
    permission_classes = [IsAuthenticated]
    
    def get_serializer_class(self):
        """Use different serializers for different actions"""
        if self.action == 'create':
            return CreateScheduledTaskSerializer
        return ScheduledTaskSerializer
    
    def get_queryset(self):
        """Filter tasks with optional status filter"""
        queryset = super().get_queryset()
        
        status_filter = self.request.query_params.get('status')
        if status_filter:
            queryset = queryset.filter(status=status_filter)
        
        return queryset.select_related('template', 'periodic_task', 'created_by')
    
    def perform_create(self, serializer):
        """Set created_by to current user"""
        serializer.save(created_by=self.request.user)
    
    @action(detail=True, methods=['post'])
    def pause(self, request, pk=None):
        """Pause a scheduled task"""
        task = self.get_object()
        task.pause()
        
        return Response({
            'status': 'success',
            'message': f'Task "{task.name}" has been paused',
            'task': ScheduledTaskSerializer(task).data
        })
    
    @action(detail=True, methods=['post'])
    def resume(self, request, pk=None):
        """Resume a paused task"""
        task = self.get_object()
        task.resume()
        
        return Response({
            'status': 'success',
            'message': f'Task "{task.name}" has been resumed',
            'task': ScheduledTaskSerializer(task).data
        })
    
    @action(detail=True, methods=['post'])
    def run_now(self, request, pk=None):
        """Execute a task immediately (one-time)"""
        task = self.get_object()
        
        try:
            # Get the Celery task
            from celery import current_app
            celery_task = current_app.tasks.get(task.periodic_task.task)
            
            if not celery_task:
                return Response({
                    'status': 'error',
                    'message': f'Celery task "{task.periodic_task.task}" not found'
                }, status=status.HTTP_400_BAD_REQUEST)
            
            # Parse arguments
            args = json.loads(task.periodic_task.args) if task.periodic_task.args else []
            kwargs = json.loads(task.periodic_task.kwargs) if task.periodic_task.kwargs else {}
            
            # Execute task asynchronously
            result = celery_task.apply_async(args=args, kwargs=kwargs)
            
            return Response({
                'status': 'success',
                'message': f'Task "{task.name}" has been queued for execution',
                'task_id': result.id
            })
        
        except Exception as e:
            return Response({
                'status': 'error',
                'message': str(e)
            }, status=status.HTTP_500_INTERNAL_SERVER_ERROR)
    
    @action(detail=True, methods=['get'])
    def execution_history(self, request, pk=None):
        """Get execution history for a task"""
        task = self.get_object()
        logs = task.execution_logs.all()[:50]  # Last 50 executions
        
        serializer = TaskExecutionLogSerializer(logs, many=True)
        
        return Response({
            'task': task.name,
            'total_executions': task.total_run_count,
            'recent_executions': serializer.data
        })


class TaskExecutionLogViewSet(viewsets.ReadOnlyModelViewSet):
    """
    ViewSet for viewing task execution logs.
    
    list: Get all execution logs
    retrieve: Get a specific execution log
    """
    
    queryset = TaskExecutionLog.objects.all()
    serializer_class = TaskExecutionLogSerializer
    permission_classes = [IsAuthenticated]
    
    def get_queryset(self):
        """Filter logs with optional filters"""
        queryset = super().get_queryset()
        
        # Filter by status
        status_filter = self.request.query_params.get('status')
        if status_filter:
            queryset = queryset.filter(status=status_filter)
        
        # Filter by task
        task_id = self.request.query_params.get('task_id')
        if task_id:
            queryset = queryset.filter(scheduled_task_id=task_id)
        
        # Filter by date range
        start_date = self.request.query_params.get('start_date')
        end_date = self.request.query_params.get('end_date')
        
        if start_date:
            queryset = queryset.filter(created_at__gte=start_date)
        if end_date:
            queryset = queryset.filter(created_at__lte=end_date)
        
        return queryset.select_related('scheduled_task').order_by('-created_at')
    
    @action(detail=False, methods=['get'])
    def statistics(self, request):
        """Get execution statistics"""
        from django.db.models import Count, Avg, Sum
        
        stats = TaskExecutionLog.objects.aggregate(
            total_executions=Count('id'),
            successful=Count('id', filter=models.Q(status='success')),
            failed=Count('id', filter=models.Q(status='failure')),
            avg_duration=Avg('duration_seconds'),
            total_duration=Sum('duration_seconds')
        )
        
        return Response(stats)


class TaskNotificationViewSet(viewsets.ModelViewSet):
    """
    ViewSet for managing task notifications.
    
    list: Get all notifications
    retrieve: Get a specific notification
    create: Create a new notification
    update: Update a notification
    destroy: Delete a notification
    """
    
    queryset = TaskNotification.objects.all()
    serializer_class = TaskNotificationSerializer
    permission_classes = [IsAuthenticated]
    
    def get_queryset(self):
        """Filter notifications by task if specified"""
        queryset = super().get_queryset()
        
        task_id = self.request.query_params.get('task_id')
        if task_id:
            queryset = queryset.filter(scheduled_task_id=task_id)
        
        return queryset.select_related('scheduled_task')

