import { useState } from 'react';
import { Button } from '@/components/ui/button.jsx';
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from '@/components/ui/card.jsx';
import {
  InputOTP,
  InputOTPGroup,
  InputOTPSlot,
} from '@/components/ui/input-otp.jsx';
import { authAPI } from '@/lib/api';

export function OTPForm({ userId, onVerifySuccess, onBack }) {
  const [otp, setOtp] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');

  const handleSubmit = async (e) => {
    e.preventDefault();
    
    if (otp.length !== 6) {
      setError('Please enter a 6-digit code');
      return;
    }

    setError('');
    setLoading(true);

    try {
      const response = await authAPI.verifyOTP(userId, otp);
      authAPI.setTokens(response.tokens.access, response.tokens.refresh);
      onVerifySuccess(response.user);
    } catch (err) {
      setError(err.message || 'Invalid verification code. Please try again.');
      setOtp('');
    } finally {
      setLoading(false);
    }
  };

  return (
    <Card className="w-full max-w-sm border-border/50 bg-card/50 backdrop-blur-sm">
      <CardHeader>
        <CardTitle className="text-2xl font-bold">Enter verification code</CardTitle>
        <CardDescription>
          We sent a 6-digit code to your authenticator app
        </CardDescription>
      </CardHeader>
      <CardContent>
        <form onSubmit={handleSubmit} className="space-y-6">
          <div className="flex flex-col items-center space-y-4">
            <InputOTP
              maxLength={6}
              value={otp}
              onChange={setOtp}
              className="gap-2.5"
            >
              <InputOTPGroup className="gap-2">
                <InputOTPSlot 
                  index={0} 
                  className="w-12 h-12 text-lg bg-background/50 border-border/50"
                />
                <InputOTPSlot 
                  index={1} 
                  className="w-12 h-12 text-lg bg-background/50 border-border/50"
                />
                <InputOTPSlot 
                  index={2} 
                  className="w-12 h-12 text-lg bg-background/50 border-border/50"
                />
                <InputOTPSlot 
                  index={3} 
                  className="w-12 h-12 text-lg bg-background/50 border-border/50"
                />
                <InputOTPSlot 
                  index={4} 
                  className="w-12 h-12 text-lg bg-background/50 border-border/50"
                />
                <InputOTPSlot 
                  index={5} 
                  className="w-12 h-12 text-lg bg-background/50 border-border/50"
                />
              </InputOTPGroup>
            </InputOTP>

            <p className="text-sm text-muted-foreground text-center">
              Enter the 6-digit code from your authenticator app
            </p>
          </div>

          {error && (
            <div className="text-sm text-destructive bg-destructive/10 p-3 rounded-md border border-destructive/20">
              {error}
            </div>
          )}

          <div className="space-y-2">
            <Button 
              type="submit" 
              className="w-full" 
              disabled={loading || otp.length !== 6}
            >
              {loading ? 'Verifying...' : 'Verify'}
            </Button>
            
            <Button
              type="button"
              variant="outline"
              className="w-full"
              onClick={onBack}
            >
              Back to login
            </Button>
          </div>

          <div className="text-center text-sm text-muted-foreground">
            Didn't receive a code?{' '}
            <a href="#" className="text-primary hover:underline">
              Resend
            </a>
          </div>
        </form>
      </CardContent>
    </Card>
  );
}

