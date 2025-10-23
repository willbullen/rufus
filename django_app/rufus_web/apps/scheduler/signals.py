"""
Scheduler Signals

Handles Celery task events and updates the database accordingly.
"""

from django.db.models.signals import post_save, pre_delete
from django.dispatch import receiver
from celery.signals import (
    task_prerun,
    task_postrun,
    task_failure,
    task_retry,
    task_success
)
from django.utils import timezone
from .models import ScheduledTask, TaskExecutionLog, TaskNotification
import logging

logger = logging.getLogger(__name__)


@receiver(post_save, sender=ScheduledTask)
def update_periodic_task_status(sender, instance, created, **kwargs):
    """Update periodic task when scheduled task is modified"""
    if not created:
        # Sync status with periodic task
        if instance.status == 'paused':
            instance.periodic_task.enabled = False
            instance.periodic_task.save()
        elif instance.status == 'active':
            instance.periodic_task.enabled = True
            instance.periodic_task.save()


@receiver(pre_delete, sender=ScheduledTask)
def delete_periodic_task(sender, instance, **kwargs):
    """Delete associated periodic task when scheduled task is deleted"""
    try:
        if instance.periodic_task:
            instance.periodic_task.delete()
    except Exception as e:
        logger.error(f"Error deleting periodic task: {str(e)}")


# Celery task signals
@task_prerun.connect
def task_prerun_handler(sender=None, task_id=None, task=None, args=None, kwargs=None, **extra):
    """Handle task pre-run event"""
    try:
        # Create or update execution log
        log, created = TaskExecutionLog.objects.get_or_create(
            task_id=task_id,
            defaults={
                'task_name': task.name,
                'status': 'running',
                'args': list(args) if args else [],
                'kwargs': dict(kwargs) if kwargs else {},
                'started_at': timezone.now()
            }
        )
        
        if not created:
            log.status = 'running'
            log.started_at = timezone.now()
            log.save()
        
        logger.info(f"Task {task.name} ({task_id}) started")
    
    except Exception as e:
        logger.error(f"Error in task_prerun_handler: {str(e)}")


@task_postrun.connect
def task_postrun_handler(sender=None, task_id=None, task=None, args=None, kwargs=None, 
                         retval=None, state=None, **extra):
    """Handle task post-run event"""
    try:
        log = TaskExecutionLog.objects.filter(task_id=task_id).first()
        
        if log:
            log.completed_at = timezone.now()
            log.result = retval
            log.calculate_duration()
            
            # Update scheduled task statistics if linked
            if log.scheduled_task:
                log.scheduled_task.last_run_at = timezone.now()
                log.scheduled_task.total_run_count += 1
                log.scheduled_task.save()
        
        logger.info(f"Task {task.name} ({task_id}) completed")
    
    except Exception as e:
        logger.error(f"Error in task_postrun_handler: {str(e)}")


@task_success.connect
def task_success_handler(sender=None, result=None, **kwargs):
    """Handle task success event"""
    try:
        task_id = kwargs.get('task_id')
        
        if task_id:
            log = TaskExecutionLog.objects.filter(task_id=task_id).first()
            
            if log:
                log.status = 'success'
                log.save()
                
                # Send success notifications
                if log.scheduled_task:
                    send_task_notifications(log.scheduled_task, 'success', log)
        
        logger.info(f"Task {task_id} succeeded")
    
    except Exception as e:
        logger.error(f"Error in task_success_handler: {str(e)}")


@task_failure.connect
def task_failure_handler(sender=None, task_id=None, exception=None, args=None, 
                        kwargs=None, traceback=None, einfo=None, **extra):
    """Handle task failure event"""
    try:
        log = TaskExecutionLog.objects.filter(task_id=task_id).first()
        
        if log:
            log.status = 'failure'
            log.error_message = str(exception)
            log.traceback = str(traceback) if traceback else ''
            log.completed_at = timezone.now()
            log.calculate_duration()
            log.save()
            
            # Send failure notifications
            if log.scheduled_task:
                send_task_notifications(log.scheduled_task, 'failure', log)
        
        logger.error(f"Task {task_id} failed: {str(exception)}")
    
    except Exception as e:
        logger.error(f"Error in task_failure_handler: {str(e)}")


@task_retry.connect
def task_retry_handler(sender=None, task_id=None, reason=None, einfo=None, **kwargs):
    """Handle task retry event"""
    try:
        log = TaskExecutionLog.objects.filter(task_id=task_id).first()
        
        if log:
            log.status = 'retry'
            log.retry_count += 1
            log.error_message = str(reason)
            log.save()
            
            # Send retry notifications
            if log.scheduled_task:
                send_task_notifications(log.scheduled_task, 'retry', log)
        
        logger.warning(f"Task {task_id} retrying: {str(reason)}")
    
    except Exception as e:
        logger.error(f"Error in task_retry_handler: {str(e)}")


def send_task_notifications(scheduled_task, event_type, execution_log):
    """
    Send notifications for task events.
    
    Args:
        scheduled_task: ScheduledTask instance
        event_type: Type of event (success, failure, retry)
        execution_log: TaskExecutionLog instance
    """
    try:
        from .tasks import send_notification
        
        # Get active notifications for this event
        notifications = scheduled_task.notifications.filter(
            is_active=True
        ).filter(
            event_type__in=[event_type, 'all']
        )
        
        for notification in notifications:
            message = format_notification_message(scheduled_task, event_type, execution_log)
            
            # Send notification asynchronously
            send_notification.delay(
                notification_type=notification.notification_method,
                recipient=notification.recipient,
                message=message,
                subject=f"Task {event_type.title()}: {scheduled_task.name}"
            )
    
    except Exception as e:
        logger.error(f"Error sending task notifications: {str(e)}")


def format_notification_message(scheduled_task, event_type, execution_log):
    """Format notification message based on event type"""
    
    base_message = f"""
Task: {scheduled_task.name}
Status: {event_type.upper()}
Executed at: {execution_log.started_at}
Duration: {execution_log.duration_seconds}s
    """
    
    if event_type == 'failure':
        base_message += f"\nError: {execution_log.error_message}"
    
    elif event_type == 'retry':
        base_message += f"\nRetry count: {execution_log.retry_count}"
    
    return base_message.strip()

