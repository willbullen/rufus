"""
Scheduler App Configuration
"""

from django.apps import AppConfig


class SchedulerConfig(AppConfig):
    default_auto_field = 'django.db.models.BigAutoField'
    name = 'apps.scheduler'
    verbose_name = 'Task Scheduler'
    
    def ready(self):
        """Import signals when app is ready"""
        try:
            import apps.scheduler.signals
        except ImportError:
            pass

