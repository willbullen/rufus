"""
Celery Configuration for Django Template Project
"""

import os
from celery import Celery
from celery.signals import task_prerun, task_postrun, task_failure

# Set the default Django settings module
os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'project.settings')

app = Celery('django_template')

# Load configuration from Django settings with CELERY namespace
app.config_from_object('django.conf:settings', namespace='CELERY')

# Auto-discover tasks from all installed apps
app.autodiscover_tasks()


@app.task(bind=True, ignore_result=True)
def debug_task(self):
    """Debug task to test Celery is working"""
    print(f'Request: {self.request!r}')


# Task event handlers
@task_prerun.connect
def task_prerun_handler(sender=None, task_id=None, task=None, **kwargs):
    """Log when a task starts"""
    print(f'Task {task.name} ({task_id}) is starting')


@task_postrun.connect
def task_postrun_handler(sender=None, task_id=None, task=None, **kwargs):
    """Log when a task completes"""
    print(f'Task {task.name} ({task_id}) has completed')


@task_failure.connect
def task_failure_handler(sender=None, task_id=None, exception=None, **kwargs):
    """Log when a task fails"""
    print(f'Task {task_id} failed with exception: {exception}')

