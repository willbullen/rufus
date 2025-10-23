"""
Core Middleware

Custom middleware for request/response processing.
"""

import time
import logging
from django.utils.deprecation import MiddlewareMixin

logger = logging.getLogger(__name__)


class RequestLoggingMiddleware(MiddlewareMixin):
    """
    Middleware to log all requests with timing information.
    """
    
    def process_request(self, request):
        """Store request start time"""
        request.start_time = time.time()
    
    def process_response(self, request, response):
        """Log request details with duration"""
        if hasattr(request, 'start_time'):
            duration = time.time() - request.start_time
            
            logger.info(
                f"{request.method} {request.path} - "
                f"Status: {response.status_code} - "
                f"Duration: {duration:.3f}s"
            )
        
        return response


class HealthCheckMiddleware(MiddlewareMixin):
    """
    Middleware to handle health check requests without authentication.
    """
    
    def process_request(self, request):
        """Return 200 OK for health check endpoints"""
        if request.path in ['/health/', '/healthz/', '/ping/']:
            from django.http import JsonResponse
            return JsonResponse({'status': 'healthy'})
        
        return None

