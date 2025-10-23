import { useState, useEffect } from 'react';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { Badge } from '@/components/ui/badge';
import { Activity, Battery, Cpu, Thermometer, Wifi, WifiOff } from 'lucide-react';

export default function Dashboard() {
  const [robotStatus, setRobotStatus] = useState({
    isConnected: false,
    batteryPercentage: 0,
    batteryVoltage: 0,
    cpuTemperature: 0,
    gpuTemperature: 0,
    cpuUsage: 0,
    memoryUsage: 0,
  });

  const [telemetryWs, setTelemetryWs] = useState(null);

  useEffect(() => {
    // Connect to telemetry WebSocket
    const ws = new WebSocket(`ws://${window.location.hostname}:8000/ws/telemetry/`);
    
    ws.onopen = () => {
      console.log('Connected to telemetry');
      ws.send(JSON.stringify({
        type: 'subscribe',
        topics: ['status', 'battery', 'system']
      }));
    };

    ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if (data.type === 'status_update') {
        setRobotStatus(prev => ({ ...prev, ...data.status }));
      }
    };

    ws.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    ws.onclose = () => {
      console.log('Disconnected from telemetry');
    };

    setTelemetryWs(ws);

    return () => {
      if (ws) ws.close();
    };
  }, []);

  return (
    <div className="container mx-auto p-6 space-y-6">
      <div className="flex items-center justify-between">
        <h1 className="text-4xl font-bold text-white">RUFUS Dashboard</h1>
        <Badge variant={robotStatus.isConnected ? "success" : "destructive"} className="text-lg px-4 py-2">
          {robotStatus.isConnected ? (
            <><Wifi className="w-4 h-4 mr-2" /> Connected</>
          ) : (
            <><WifiOff className="w-4 h-4 mr-2" /> Disconnected</>
          )}
        </Badge>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6">
        {/* Battery Status */}
        <Card className="bg-slate-900 border-slate-800">
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium text-slate-300">
              Battery
            </CardTitle>
            <Battery className="h-4 w-4 text-blue-400" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold text-white">
              {robotStatus.batteryPercentage.toFixed(0)}%
            </div>
            <p className="text-xs text-slate-400 mt-1">
              {robotStatus.batteryVoltage.toFixed(2)}V
            </p>
          </CardContent>
        </Card>

        {/* CPU Temperature */}
        <Card className="bg-slate-900 border-slate-800">
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium text-slate-300">
              CPU Temp
            </CardTitle>
            <Thermometer className="h-4 w-4 text-orange-400" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold text-white">
              {robotStatus.cpuTemperature.toFixed(1)}°C
            </div>
            <p className="text-xs text-slate-400 mt-1">
              GPU: {robotStatus.gpuTemperature.toFixed(1)}°C
            </p>
          </CardContent>
        </Card>

        {/* CPU Usage */}
        <Card className="bg-slate-900 border-slate-800">
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium text-slate-300">
              CPU Usage
            </CardTitle>
            <Cpu className="h-4 w-4 text-green-400" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold text-white">
              {robotStatus.cpuUsage.toFixed(1)}%
            </div>
            <p className="text-xs text-slate-400 mt-1">
              4 cores active
            </p>
          </CardContent>
        </Card>

        {/* Memory Usage */}
        <Card className="bg-slate-900 border-slate-800">
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium text-slate-300">
              Memory
            </CardTitle>
            <Activity className="h-4 w-4 text-purple-400" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold text-white">
              {robotStatus.memoryUsage.toFixed(1)}%
            </div>
            <p className="text-xs text-slate-400 mt-1">
              8GB total
            </p>
          </CardContent>
        </Card>
      </div>

      {/* Quick Actions */}
      <Card className="bg-slate-900 border-slate-800">
        <CardHeader>
          <CardTitle className="text-white">Quick Actions</CardTitle>
          <CardDescription className="text-slate-400">
            Common robot control operations
          </CardDescription>
        </CardHeader>
        <CardContent className="flex gap-4">
          <Button 
            variant="default" 
            className="bg-blue-600 hover:bg-blue-700"
            onClick={() => window.location.href = '/control'}
          >
            Manual Control
          </Button>
          <Button 
            variant="default" 
            className="bg-purple-600 hover:bg-purple-700"
            onClick={() => window.location.href = '/vr'}
          >
            VR Teleoperation
          </Button>
          <Button 
            variant="default" 
            className="bg-green-600 hover:bg-green-700"
            onClick={() => window.location.href = '/navigation'}
          >
            Navigation
          </Button>
          <Button 
            variant="destructive"
            onClick={() => {
              // Emergency stop
              fetch('/api/emergency-stop/', { method: 'POST' });
            }}
          >
            Emergency Stop
          </Button>
        </CardContent>
      </Card>

      {/* System Status */}
      <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
        <Card className="bg-slate-900 border-slate-800">
          <CardHeader>
            <CardTitle className="text-white">Chassis Status</CardTitle>
          </CardHeader>
          <CardContent className="text-slate-300">
            <div className="space-y-2">
              <div className="flex justify-between">
                <span>Left Track:</span>
                <span className="text-green-400">Operational</span>
              </div>
              <div className="flex justify-between">
                <span>Right Track:</span>
                <span className="text-green-400">Operational</span>
              </div>
              <div className="flex justify-between">
                <span>Odometry:</span>
                <span className="text-green-400">Active</span>
              </div>
            </div>
          </CardContent>
        </Card>

        <Card className="bg-slate-900 border-slate-800">
          <CardHeader>
            <CardTitle className="text-white">Arm Status</CardTitle>
          </CardHeader>
          <CardContent className="text-slate-300">
            <div className="space-y-2">
              <div className="flex justify-between">
                <span>Servos:</span>
                <span className="text-green-400">6/6 Online</span>
              </div>
              <div className="flex justify-between">
                <span>Position:</span>
                <span className="text-green-400">Home</span>
              </div>
              <div className="flex justify-between">
                <span>Gripper:</span>
                <span className="text-green-400">Open</span>
              </div>
            </div>
          </CardContent>
        </Card>
      </div>
    </div>
  );
}

