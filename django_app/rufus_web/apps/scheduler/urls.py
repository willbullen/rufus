"""
Scheduler URL Configuration
"""

from django.urls import path, include
from rest_framework.routers import DefaultRouter
from .views import (
    ScheduledTaskTemplateViewSet,
    ScheduledTaskViewSet,
    TaskExecutionLogViewSet,
    TaskNotificationViewSet
)

app_name = 'scheduler'

router = DefaultRouter()
router.register(r'templates', ScheduledTaskTemplateViewSet, basename='template')
router.register(r'tasks', ScheduledTaskViewSet, basename='task')
router.register(r'logs', TaskExecutionLogViewSet, basename='log')
router.register(r'notifications', TaskNotificationViewSet, basename='notification')

urlpatterns = [
    path('', include(router.urls)),
]

