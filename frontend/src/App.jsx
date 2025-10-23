import { useState, useEffect } from 'react';
import { LoginForm } from '@/components/LoginForm';
import { OTPForm } from '@/components/OTPForm';
import { Dashboard } from '@/components/Dashboard';
import { authAPI } from '@/lib/api';
import './App.css';

function App() {
  const [view, setView] = useState('login'); // 'login', 'otp', 'dashboard'
  const [user, setUser] = useState(null);
  const [userId, setUserId] = useState(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    // Check if user is already logged in
    const checkAuth = async () => {
      if (authAPI.isAuthenticated()) {
        try {
          const response = await authAPI.getUser();
          setUser(response.user);
          setView('dashboard');
        } catch (error) {
          // Token might be expired, clear it
          authAPI.clearTokens();
        }
      }
      setLoading(false);
    };

    checkAuth();
  }, []);

  const handleLoginSuccess = (userData) => {
    setUser(userData);
    setView('dashboard');
  };

  const handleRequires2FA = (id) => {
    setUserId(id);
    setView('otp');
  };

  const handleVerifySuccess = (userData) => {
    setUser(userData);
    setView('dashboard');
  };

  const handleBackToLogin = () => {
    setView('login');
    setUserId(null);
  };

  const handleLogout = () => {
    authAPI.clearTokens();
    setUser(null);
    setView('login');
  };

  if (loading) {
    return (
      <div className="min-h-screen flex items-center justify-center bg-gradient-to-br from-background via-background to-primary/5">
        <div className="text-center space-y-4">
          <div className="w-16 h-16 border-4 border-primary border-t-transparent rounded-full animate-spin mx-auto"></div>
          <p className="text-muted-foreground">Loading...</p>
        </div>
      </div>
    );
  }

  return (
    <div className="min-h-screen">
      {view === 'login' && (
        <div className="flex min-h-screen w-full items-center justify-center p-6 md:p-10 bg-gradient-to-br from-background via-background to-primary/5">
          <div className="w-full max-w-sm">
            <div className="mb-8 text-center">
              <h1 className="text-4xl font-bold mb-2 bg-gradient-to-r from-primary to-primary/50 bg-clip-text text-transparent">
                Django Auth 2FA
              </h1>
              <p className="text-muted-foreground">Secure authentication with two-factor protection</p>
            </div>
            <LoginForm
              onLoginSuccess={handleLoginSuccess}
              onRequires2FA={handleRequires2FA}
            />
          </div>
        </div>
      )}

      {view === 'otp' && (
        <div className="flex min-h-screen w-full items-center justify-center p-6 md:p-10 bg-gradient-to-br from-background via-background to-primary/5">
          <div className="w-full max-w-sm">
            <OTPForm
              userId={userId}
              onVerifySuccess={handleVerifySuccess}
              onBack={handleBackToLogin}
            />
          </div>
        </div>
      )}

      {view === 'dashboard' && user && (
        <Dashboard user={user} onLogout={handleLogout} />
      )}
    </div>
  );
}

export default App;

