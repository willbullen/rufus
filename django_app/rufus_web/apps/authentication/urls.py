from django.urls import path
from rest_framework_simplejwt.views import TokenRefreshView
from . import views

urlpatterns = [
    # Authentication endpoints
    path('register/', views.register, name='register'),
    path('login/', views.login, name='login'),
    path('verify-otp/', views.verify_otp, name='verify_otp'),
    path('token/refresh/', TokenRefreshView.as_view(), name='token_refresh'),
    
    # 2FA management endpoints
    path('2fa/enable/', views.enable_2fa, name='enable_2fa'),
    path('2fa/confirm/', views.confirm_2fa, name='confirm_2fa'),
    path('2fa/disable/', views.disable_2fa, name='disable_2fa'),
    
    # User info
    path('user/', views.get_user, name='get_user'),
]

