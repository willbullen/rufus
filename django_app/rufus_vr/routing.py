"""
WebSocket URL routing
"""

from django.urls import re_path
from . import consumers

websocket_urlpatterns = [
    re_path(r'ws/vr/$', consumers.VRTeleopConsumer.as_asgi()),
    re_path(r'ws/telemetry/$', consumers.TelemetryConsumer.as_asgi()),
]

