import { useState, useEffect } from 'react';
import { Button } from '@/components/ui/button.jsx';
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from '@/components/ui/card.jsx';
import { authAPI } from '@/lib/api';
import { Shield, ShieldCheck, LogOut } from 'lucide-react';

export function Dashboard({ user, onLogout }) {
  const [qrCode, setQrCode] = useState('');
  const [secret, setSecret] = useState('');
  const [otpCode, setOtpCode] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');
  const [success, setSuccess] = useState('');
  const [showSetup, setShowSetup] = useState(false);

  const handleEnable2FA = async () => {
    setError('');
    setSuccess('');
    setLoading(true);

    try {
      const response = await authAPI.enable2FA();
      setQrCode(response.qr_code);
      setSecret(response.secret);
      setShowSetup(true);
      setSuccess('Scan the QR code with your authenticator app');
    } catch (err) {
      setError(err.message || 'Failed to enable 2FA');
    } finally {
      setLoading(false);
    }
  };

  const handleConfirm2FA = async () => {
    if (!otpCode || otpCode.length !== 6) {
      setError('Please enter a 6-digit code');
      return;
    }

    setError('');
    setSuccess('');
    setLoading(true);

    try {
      await authAPI.confirm2FA(otpCode);
      setSuccess('2FA has been successfully enabled!');
      setShowSetup(false);
      setQrCode('');
      setSecret('');
      setOtpCode('');
      // Refresh user data
      window.location.reload();
    } catch (err) {
      setError(err.message || 'Invalid code. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  const handleDisable2FA = async () => {
    if (!confirm('Are you sure you want to disable 2FA?')) {
      return;
    }

    setError('');
    setSuccess('');
    setLoading(true);

    try {
      await authAPI.disable2FA();
      setSuccess('2FA has been disabled');
      // Refresh user data
      window.location.reload();
    } catch (err) {
      setError(err.message || 'Failed to disable 2FA');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-background via-background to-primary/5 p-6">
      <div className="max-w-4xl mx-auto space-y-6">
        {/* Header */}
        <div className="flex items-center justify-between">
          <div>
            <h1 className="text-3xl font-bold">Welcome back, {user.first_name || user.username}!</h1>
            <p className="text-muted-foreground">Manage your account security</p>
          </div>
          <Button variant="outline" onClick={onLogout}>
            <LogOut className="mr-2 h-4 w-4" />
            Logout
          </Button>
        </div>

        {/* User Info Card */}
        <Card className="border-border/50 bg-card/50 backdrop-blur-sm">
          <CardHeader>
            <CardTitle>Account Information</CardTitle>
            <CardDescription>Your account details</CardDescription>
          </CardHeader>
          <CardContent className="space-y-2">
            <div className="flex justify-between">
              <span className="text-muted-foreground">Username:</span>
              <span className="font-medium">{user.username}</span>
            </div>
            <div className="flex justify-between">
              <span className="text-muted-foreground">Email:</span>
              <span className="font-medium">{user.email}</span>
            </div>
            <div className="flex justify-between items-center">
              <span className="text-muted-foreground">Two-Factor Auth:</span>
              <div className="flex items-center gap-2">
                {user.has_2fa ? (
                  <>
                    <ShieldCheck className="h-4 w-4 text-green-500" />
                    <span className="font-medium text-green-500">Enabled</span>
                  </>
                ) : (
                  <>
                    <Shield className="h-4 w-4 text-muted-foreground" />
                    <span className="font-medium text-muted-foreground">Disabled</span>
                  </>
                )}
              </div>
            </div>
          </CardContent>
        </Card>

        {/* 2FA Management Card */}
        <Card className="border-border/50 bg-card/50 backdrop-blur-sm">
          <CardHeader>
            <CardTitle>Two-Factor Authentication</CardTitle>
            <CardDescription>
              Add an extra layer of security to your account
            </CardDescription>
          </CardHeader>
          <CardContent className="space-y-4">
            {error && (
              <div className="text-sm text-destructive bg-destructive/10 p-3 rounded-md border border-destructive/20">
                {error}
              </div>
            )}

            {success && (
              <div className="text-sm text-green-500 bg-green-500/10 p-3 rounded-md border border-green-500/20">
                {success}
              </div>
            )}

            {!user.has_2fa && !showSetup && (
              <div className="space-y-4">
                <p className="text-sm text-muted-foreground">
                  Two-factor authentication adds an extra layer of security to your account.
                  You'll need to enter a code from your authenticator app in addition to your password.
                </p>
                <Button onClick={handleEnable2FA} disabled={loading}>
                  <Shield className="mr-2 h-4 w-4" />
                  Enable 2FA
                </Button>
              </div>
            )}

            {showSetup && qrCode && (
              <div className="space-y-4">
                <div className="flex flex-col items-center space-y-4">
                  <img src={qrCode} alt="QR Code" className="w-64 h-64 rounded-lg border border-border" />
                  <div className="text-center space-y-2">
                    <p className="text-sm text-muted-foreground">
                      Scan this QR code with your authenticator app
                    </p>
                    <p className="text-xs text-muted-foreground">
                      Or enter this secret key manually: <code className="bg-muted px-2 py-1 rounded">{secret}</code>
                    </p>
                  </div>
                </div>

                <div className="space-y-2">
                  <label className="text-sm font-medium">Enter verification code</label>
                  <input
                    type="text"
                    maxLength="6"
                    value={otpCode}
                    onChange={(e) => setOtpCode(e.target.value.replace(/\D/g, ''))}
                    placeholder="000000"
                    className="w-full px-3 py-2 bg-background/50 border border-border/50 rounded-md text-center text-lg tracking-widest"
                  />
                </div>

                <div className="flex gap-2">
                  <Button onClick={handleConfirm2FA} disabled={loading || otpCode.length !== 6} className="flex-1">
                    Confirm & Enable
                  </Button>
                  <Button 
                    variant="outline" 
                    onClick={() => {
                      setShowSetup(false);
                      setQrCode('');
                      setSecret('');
                      setOtpCode('');
                    }}
                    className="flex-1"
                  >
                    Cancel
                  </Button>
                </div>
              </div>
            )}

            {user.has_2fa && (
              <div className="space-y-4">
                <div className="flex items-center gap-2 text-green-500">
                  <ShieldCheck className="h-5 w-5" />
                  <span className="font-medium">Two-factor authentication is enabled</span>
                </div>
                <p className="text-sm text-muted-foreground">
                  Your account is protected with two-factor authentication. You'll need to enter a code from your authenticator app when logging in.
                </p>
                <Button variant="destructive" onClick={handleDisable2FA} disabled={loading}>
                  Disable 2FA
                </Button>
              </div>
            )}
          </CardContent>
        </Card>
      </div>
    </div>
  );
}

