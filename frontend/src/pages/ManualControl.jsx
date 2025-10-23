import { useState, useEffect, useRef } from 'react';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { Slider } from '@/components/ui/slider';
import { ArrowUp, ArrowDown, ArrowLeft, ArrowRight, Hand } from 'lucide-react';

export default function ManualControl() {
  const [ws, setWs] = useState(null);
  const [connected, setConnected] = useState(false);
  const [jointPositions, setJointPositions] = useState([0, 0, 0, 0, 0, 0]);
  const joystickRef = useRef(null);
  const [joystickActive, setJoystickActive] = useState(false);

  const jointNames = [
    'Base',
    'Shoulder',
    'Elbow',
    'Wrist Pitch',
    'Wrist Roll',
    'Gripper'
  ];

  useEffect(() => {
    // Connect to VR WebSocket (also used for manual control)
    const websocket = new WebSocket(`ws://${window.location.hostname}:8000/ws/vr/`);
    
    websocket.onopen = () => {
      console.log('Connected to control WebSocket');
      setConnected(true);
    };

    websocket.onmessage = (event) => {
      const data = JSON.parse(event.data);
      console.log('Received:', data);
    };

    websocket.onerror = (error) => {
      console.error('WebSocket error:', error);
      setConnected(false);
    };

    websocket.onclose = () => {
      console.log('Disconnected from control');
      setConnected(false);
    };

    setWs(websocket);

    return () => {
      if (websocket) websocket.close();
    };
  }, []);

  const sendChassisCommand = (linear, angular) => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({
        type: 'chassis_control',
        linear,
        angular
      }));
    }
  };

  const sendArmCommand = (positions) => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({
        type: 'arm_control',
        joint_positions: positions
      }));
    }
  };

  const handleJointChange = (index, value) => {
    const newPositions = [...jointPositions];
    newPositions[index] = value[0];
    setJointPositions(newPositions);
    sendArmCommand(newPositions);
  };

  const handleKeyDown = (e) => {
    switch(e.key) {
      case 'ArrowUp':
      case 'w':
        sendChassisCommand(0.5, 0);
        break;
      case 'ArrowDown':
      case 's':
        sendChassisCommand(-0.5, 0);
        break;
      case 'ArrowLeft':
      case 'a':
        sendChassisCommand(0, 0.5);
        break;
      case 'ArrowRight':
      case 'd':
        sendChassisCommand(0, -0.5);
        break;
      case ' ':
        sendChassisCommand(0, 0);
        break;
    }
  };

  const handleKeyUp = () => {
    sendChassisCommand(0, 0);
  };

  useEffect(() => {
    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [ws]);

  return (
    <div className="container mx-auto p-6 space-y-6">
      <div className="flex items-center justify-between">
        <h1 className="text-4xl font-bold text-white">Manual Control</h1>
        <div className={`px-4 py-2 rounded ${connected ? 'bg-green-600' : 'bg-red-600'}`}>
          {connected ? 'Connected' : 'Disconnected'}
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        {/* Chassis Control */}
        <Card className="bg-slate-900 border-slate-800">
          <CardHeader>
            <CardTitle className="text-white">Chassis Control</CardTitle>
            <CardDescription className="text-slate-400">
              Use arrow keys or WASD to control movement
            </CardDescription>
          </CardHeader>
          <CardContent>
            <div className="flex flex-col items-center space-y-4">
              <div className="grid grid-cols-3 gap-2">
                <div></div>
                <Button
                  size="lg"
                  className="bg-blue-600 hover:bg-blue-700"
                  onMouseDown={() => sendChassisCommand(0.5, 0)}
                  onMouseUp={() => sendChassisCommand(0, 0)}
                  onTouchStart={() => sendChassisCommand(0.5, 0)}
                  onTouchEnd={() => sendChassisCommand(0, 0)}
                >
                  <ArrowUp />
                </Button>
                <div></div>
                
                <Button
                  size="lg"
                  className="bg-blue-600 hover:bg-blue-700"
                  onMouseDown={() => sendChassisCommand(0, 0.5)}
                  onMouseUp={() => sendChassisCommand(0, 0)}
                  onTouchStart={() => sendChassisCommand(0, 0.5)}
                  onTouchEnd={() => sendChassisCommand(0, 0)}
                >
                  <ArrowLeft />
                </Button>
                <Button
                  size="lg"
                  variant="destructive"
                  onClick={() => sendChassisCommand(0, 0)}
                >
                  STOP
                </Button>
                <Button
                  size="lg"
                  className="bg-blue-600 hover:bg-blue-700"
                  onMouseDown={() => sendChassisCommand(0, -0.5)}
                  onMouseUp={() => sendChassisCommand(0, 0)}
                  onTouchStart={() => sendChassisCommand(0, -0.5)}
                  onTouchEnd={() => sendChassisCommand(0, 0)}
                >
                  <ArrowRight />
                </Button>
                
                <div></div>
                <Button
                  size="lg"
                  className="bg-blue-600 hover:bg-blue-700"
                  onMouseDown={() => sendChassisCommand(-0.5, 0)}
                  onMouseUp={() => sendChassisCommand(0, 0)}
                  onTouchStart={() => sendChassisCommand(-0.5, 0)}
                  onTouchEnd={() => sendChassisCommand(0, 0)}
                >
                  <ArrowDown />
                </Button>
                <div></div>
              </div>

              <p className="text-sm text-slate-400 text-center">
                Keyboard: Arrow keys or WASD<br />
                Space bar to stop
              </p>
            </div>
          </CardContent>
        </Card>

        {/* Arm Control */}
        <Card className="bg-slate-900 border-slate-800">
          <CardHeader>
            <CardTitle className="text-white">Arm Control</CardTitle>
            <CardDescription className="text-slate-400">
              Adjust individual joint positions
            </CardDescription>
          </CardHeader>
          <CardContent>
            <div className="space-y-6">
              {jointNames.map((name, index) => (
                <div key={index} className="space-y-2">
                  <div className="flex justify-between text-sm">
                    <label className="text-slate-300">{name}</label>
                    <span className="text-slate-400">
                      {jointPositions[index].toFixed(2)} rad
                    </span>
                  </div>
                  <Slider
                    value={[jointPositions[index]]}
                    onValueChange={(value) => handleJointChange(index, value)}
                    min={-Math.PI}
                    max={Math.PI}
                    step={0.01}
                    className="w-full"
                  />
                </div>
              ))}

              <div className="flex gap-2 mt-4">
                <Button
                  className="flex-1 bg-green-600 hover:bg-green-700"
                  onClick={() => {
                    const homePosition = [0, 0, 0, 0, 0, 0];
                    setJointPositions(homePosition);
                    sendArmCommand(homePosition);
                  }}
                >
                  Home Position
                </Button>
                <Button
                  className="flex-1 bg-purple-600 hover:bg-purple-700"
                  onClick={() => {
                    // Open gripper
                    const newPositions = [...jointPositions];
                    newPositions[5] = 0.5;
                    setJointPositions(newPositions);
                    sendArmCommand(newPositions);
                  }}
                >
                  <Hand className="mr-2" />
                  Open Gripper
                </Button>
              </div>
            </div>
          </CardContent>
        </Card>
      </div>

      {/* Camera Feed Placeholder */}
      <Card className="bg-slate-900 border-slate-800">
        <CardHeader>
          <CardTitle className="text-white">Camera Feed</CardTitle>
        </CardHeader>
        <CardContent>
          <div className="aspect-video bg-slate-800 rounded flex items-center justify-center">
            <p className="text-slate-400">Camera feed will appear here</p>
          </div>
        </CardContent>
      </Card>
    </div>
  );
}

