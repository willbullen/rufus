from rest_framework import status
from rest_framework.decorators import api_view, permission_classes
from rest_framework.permissions import AllowAny, IsAuthenticated
from rest_framework.response import Response
from rest_framework_simplejwt.tokens import RefreshToken
from django.contrib.auth.models import User
from django_otp.plugins.otp_totp.models import TOTPDevice
from django_otp import match_token
import qrcode
import io
import base64

from .serializers import (
    UserRegistrationSerializer,
    LoginSerializer,
    OTPVerifySerializer,
    UserSerializer
)


def get_tokens_for_user(user):
    """Generate JWT tokens for a user"""
    refresh = RefreshToken.for_user(user)
    return {
        'refresh': str(refresh),
        'access': str(refresh.access_token),
    }


@api_view(['POST'])
@permission_classes([AllowAny])
def register(request):
    """Register a new user"""
    serializer = UserRegistrationSerializer(data=request.data)
    if serializer.is_valid():
        user = serializer.save()
        tokens = get_tokens_for_user(user)
        return Response({
            'user': UserSerializer(user).data,
            'tokens': tokens,
            'message': 'User registered successfully'
        }, status=status.HTTP_201_CREATED)
    return Response(serializer.errors, status=status.HTTP_400_BAD_REQUEST)


@api_view(['POST'])
@permission_classes([AllowAny])
def login(request):
    """Login user - returns temp token if 2FA is enabled, full token if not"""
    serializer = LoginSerializer(data=request.data)
    if serializer.is_valid():
        user = serializer.validated_data['user']
        
        # Check if user has 2FA enabled
        has_2fa = TOTPDevice.objects.filter(user=user, confirmed=True).exists()
        
        if has_2fa:
            # Return a temporary indicator that 2FA is required
            # Store user ID in session or return a temporary token
            return Response({
                'requires_2fa': True,
                'user_id': user.id,
                'message': 'Please enter your 2FA code'
            }, status=status.HTTP_200_OK)
        else:
            # No 2FA, return full tokens
            tokens = get_tokens_for_user(user)
            return Response({
                'requires_2fa': False,
                'user': UserSerializer(user).data,
                'tokens': tokens,
                'message': 'Login successful'
            }, status=status.HTTP_200_OK)
    
    return Response(serializer.errors, status=status.HTTP_400_BAD_REQUEST)


@api_view(['POST'])
@permission_classes([AllowAny])
def verify_otp(request):
    """Verify OTP code and return full JWT tokens"""
    serializer = OTPVerifySerializer(data=request.data)
    user_id = request.data.get('user_id')
    
    if not user_id:
        return Response({'error': 'user_id is required'}, status=status.HTTP_400_BAD_REQUEST)
    
    if serializer.is_valid():
        try:
            user = User.objects.get(id=user_id)
        except User.DoesNotExist:
            return Response({'error': 'User not found'}, status=status.HTTP_404_NOT_FOUND)
        
        otp_code = serializer.validated_data['otp_code']
        
        # Verify the OTP code
        device = match_token(user, otp_code)
        
        if device:
            # OTP is valid, return full tokens
            tokens = get_tokens_for_user(user)
            return Response({
                'user': UserSerializer(user).data,
                'tokens': tokens,
                'message': '2FA verification successful'
            }, status=status.HTTP_200_OK)
        else:
            return Response({
                'error': 'Invalid OTP code'
            }, status=status.HTTP_400_BAD_REQUEST)
    
    return Response(serializer.errors, status=status.HTTP_400_BAD_REQUEST)


@api_view(['POST'])
@permission_classes([IsAuthenticated])
def enable_2fa(request):
    """Enable 2FA for the current user and return QR code"""
    user = request.user
    
    # Check if user already has 2FA enabled
    existing_device = TOTPDevice.objects.filter(user=user, confirmed=True).first()
    if existing_device:
        return Response({
            'error': '2FA is already enabled for this user'
        }, status=status.HTTP_400_BAD_REQUEST)
    
    # Delete any unconfirmed devices
    TOTPDevice.objects.filter(user=user, confirmed=False).delete()
    
    # Create a new TOTP device
    device = TOTPDevice.objects.create(
        user=user,
        name=f"{user.username}'s device",
        confirmed=False
    )
    
    # Generate QR code
    url = device.config_url
    qr = qrcode.QRCode(version=1, box_size=10, border=5)
    qr.add_data(url)
    qr.make(fit=True)
    
    img = qr.make_image(fill_color="black", back_color="white")
    buffer = io.BytesIO()
    img.save(buffer, format='PNG')
    qr_code_base64 = base64.b64encode(buffer.getvalue()).decode()
    
    return Response({
        'qr_code': f'data:image/png;base64,{qr_code_base64}',
        'secret': device.key,
        'message': 'Scan the QR code with your authenticator app and verify with a code'
    }, status=status.HTTP_200_OK)


@api_view(['POST'])
@permission_classes([IsAuthenticated])
def confirm_2fa(request):
    """Confirm 2FA setup by verifying an OTP code"""
    user = request.user
    serializer = OTPVerifySerializer(data=request.data)
    
    if serializer.is_valid():
        otp_code = serializer.validated_data['otp_code']
        
        # Get the unconfirmed device
        device = TOTPDevice.objects.filter(user=user, confirmed=False).first()
        
        if not device:
            return Response({
                'error': 'No pending 2FA setup found'
            }, status=status.HTTP_400_BAD_REQUEST)
        
        # Verify the OTP code
        if device.verify_token(otp_code):
            device.confirmed = True
            device.save()
            return Response({
                'message': '2FA has been successfully enabled'
            }, status=status.HTTP_200_OK)
        else:
            return Response({
                'error': 'Invalid OTP code'
            }, status=status.HTTP_400_BAD_REQUEST)
    
    return Response(serializer.errors, status=status.HTTP_400_BAD_REQUEST)


@api_view(['POST'])
@permission_classes([IsAuthenticated])
def disable_2fa(request):
    """Disable 2FA for the current user"""
    user = request.user
    
    # Delete all TOTP devices for this user
    deleted_count, _ = TOTPDevice.objects.filter(user=user).delete()
    
    if deleted_count > 0:
        return Response({
            'message': '2FA has been disabled'
        }, status=status.HTTP_200_OK)
    else:
        return Response({
            'error': '2FA was not enabled for this user'
        }, status=status.HTTP_400_BAD_REQUEST)


@api_view(['GET'])
@permission_classes([IsAuthenticated])
def get_user(request):
    """Get current user information"""
    return Response({
        'user': UserSerializer(request.user).data
    }, status=status.HTTP_200_OK)

