import React, { useEffect, useRef, useState } from 'react';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { Camera, Video, Grid2X2, Maximize2, Minimize2, RefreshCw } from 'lucide-react';
import { ScrollArea } from '@/components/ui/scroll-area';
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs';

// Types
interface CameraConfig {
  CAMERAS: Record<string, string>;
  CAMERA_SERVERS: Record<string, {
    device_info: {
      hostname: string;
      ip: string;
      model: string;
    };
    streams: Record<string, {
      rtsp_url: string;
      status: string;
    }>;
  }>;
}

interface WebRTCMessage {
  type: string;
  camera_id?: string;
  config?: CameraConfig;
  data?: any;
  sdp?: string;
  message?: string;
}

interface CameraStreamProps {
  cameraId: string;
  devicePath: string;
  wsRef: React.MutableRefObject<WebSocket | null>;
  onMaximize: (cameraId: string) => void;
  isMaximized: boolean;
}

const CameraStream: React.FC<CameraStreamProps> = ({ cameraId, devicePath, wsRef, onMaximize, isMaximized }) => {
  const videoRef = useRef<HTMLVideoElement>(null);
  const peerConnectionRef = useRef<RTCPeerConnection | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [stats, setStats] = useState<Record<string, number>>({});
  const statsIntervalRef = useRef<number | null>(null);

  const logError = (error: any, context: string) => {
    console.error(`${context}:`, error);
    setError(`${context}: ${error.message || error}`);
  };

  const collectStats = async () => {
    if (!peerConnectionRef.current) return;

    try {
      const stats = await peerConnectionRef.current.getStats();
      const statsObj: Record<string, number> = {};

      stats.forEach(stat => {
        if (stat.type === 'inbound-rtp' && stat.kind === 'video') {
          statsObj.bytesReceived = stat.bytesReceived;
          statsObj.packetsReceived = stat.packetsReceived;
          statsObj.framesDecoded = stat.framesDecoded;
        }
      });

      setStats(statsObj);
    } catch (e) {
      console.warn('Failed to collect stats:', e);
    }
  };

  const initWebRTC = async () => {
    try {
      if (!wsRef.current) throw new Error("WebSocket not connected");

      // Clean up any existing connection
      if (peerConnectionRef.current) {
        peerConnectionRef.current.close();
        peerConnectionRef.current = null;
      }

      console.log('Creating new RTCPeerConnection');
      const pc = new RTCPeerConnection({
        iceServers: [
          { urls: 'stun:stun.l.google.com:19302' },
          { urls: 'stun:stun1.l.google.com:19302' }
        ],
        iceTransportPolicy: 'all',
        bundlePolicy: 'max-bundle',
        rtcpMuxPolicy: 'require',
      });

      peerConnectionRef.current = pc;

      // Add video transceiver
      console.log('Adding video transceiver');
      pc.addTransceiver('video', {
        direction: 'recvonly',
        streams: [new MediaStream()]
      });

      // Track event handlers
      pc.ontrack = (event) => {
        console.log('Received track:', event.track.kind);
        if (videoRef.current && event.streams[0]) {
          console.log('Setting video stream');
          videoRef.current.srcObject = event.streams[0];

          // Start collecting stats
          if (statsIntervalRef.current) {
            clearInterval(statsIntervalRef.current);
          }
          statsIntervalRef.current = window.setInterval(collectStats, 1000);
        }
      };

      pc.onconnectionstatechange = () => {
        console.log('Connection state changed:', pc.connectionState);
        setIsConnected(pc.connectionState === 'connected');
        if (pc.connectionState === 'failed') {
          logError(new Error('Connection failed'), 'WebRTC Connection');
        }
      };

      pc.oniceconnectionstatechange = () => {
        console.log('ICE Connection State:', pc.iceConnectionState);
      };

      pc.onicegatheringstatechange = () => {
        console.log('ICE Gathering State:', pc.iceGatheringState);
      };

      pc.onsignalingstatechange = () => {
        console.log('Signaling State:', pc.signalingState);
      };

      pc.onicecandidate = (event) => {
        if (event.candidate) {
          console.log('Sending ICE candidate:', event.candidate);
          wsRef.current.send(JSON.stringify({
            type: 'ice-candidate',
            camera_id: cameraId,
            candidate: event.candidate.toJSON()
          }));
        }
      };

      // Request stream
      console.log('Requesting stream from server');
      wsRef.current.send(JSON.stringify({
        type: 'request-stream',
        camera_id: cameraId,
        codec: 'h264'
      }));

    } catch (err) {
      logError(err, 'WebRTC Initialization');
    }
  };

  const disconnectStream = () => {
    console.log('Disconnecting stream');
    if (statsIntervalRef.current) {
      clearInterval(statsIntervalRef.current);
      statsIntervalRef.current = null;
    }

    if (peerConnectionRef.current) {
      peerConnectionRef.current.close();
      peerConnectionRef.current = null;
    }

    if (videoRef.current) {
      videoRef.current.srcObject = null;
    }

    setIsConnected(false);
    setError(null);
    setStats({});
  };

  useEffect(() => {
    if (!wsRef.current) return;

    const handleMessage = async (event: MessageEvent) => {
      try {
        const message: WebRTCMessage = JSON.parse(event.data);
        if (message.camera_id !== cameraId) return;

        console.log('Received message:', message.type);

        const pc = peerConnectionRef.current;
        if (!pc) return;

        switch (message.type) {
          case 'offer':
            console.log('Setting remote description (offer)');
            await pc.setRemoteDescription(new RTCSessionDescription({
              type: 'offer',
              sdp: message.sdp!
            }));

            console.log('Creating answer');
            const answer = await pc.createAnswer();

            console.log('Setting local description');
            await pc.setLocalDescription(answer);

            console.log('Sending answer');
            wsRef.current!.send(JSON.stringify({
              type: 'answer',
              camera_id: cameraId,
              sdp: answer.sdp
            }));
            break;

          case 'ice-candidate':
            if (message.data && pc.remoteDescription) {
              console.log('Adding ICE candidate');
              await pc.addIceCandidate(new RTCIceCandidate(message.data));
            }
            break;

          case 'error':
            logError(new Error(message.message), 'Server Error');
            break;
        }
      } catch (err) {
        logError(err, 'Message Handler');
      }
    };

    wsRef.current.addEventListener('message', handleMessage);
    return () => {
      wsRef.current?.removeEventListener('message', handleMessage);
    };
  }, [cameraId]);

  useEffect(() => {
    return () => {
      disconnectStream();
    };
  }, []);

  return (
    <Card className={`${isMaximized ? 'w-full h-full' : 'w-full md:w-1/2 lg:w-1/3'}`}>
      <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
        <CardTitle className="text-lg font-bold">{cameraId}</CardTitle>
        <div className="flex items-center space-x-2">
          <Button
            onClick={isConnected ? disconnectStream : initWebRTC}
            className={isConnected ? "bg-red-500 hover:bg-red-600" : "bg-blue-500 hover:bg-blue-600"}
            size="sm"
          >
            {isConnected ? (
              <Video className="h-4 w-4" />
            ) : (
              <Camera className="h-4 w-4" />
            )}
          </Button>
          <Button
            onClick={() => onMaximize(cameraId)}
            variant="outline"
            size="sm"
          >
            {isMaximized ? (
              <Minimize2 className="h-4 w-4" />
            ) : (
              <Maximize2 className="h-4 w-4" />
            )}
          </Button>
        </div>
      </CardHeader>
      <CardContent>
        <video
          ref={videoRef}
          className="w-full rounded-lg bg-black aspect-video"
          autoPlay
          playsInline
        />
        {error && (
          <div className="mt-2 text-sm text-red-500">
            Error: {error}
          </div>
        )}
        <div className="mt-2 text-sm space-y-1">
          <div className="text-gray-500">{devicePath}</div>
          {Object.entries(stats).length > 0 && (
            <div className="text-gray-400">
              Frames: {stats.framesDecoded || 0} |
              Packets: {stats.packetsReceived || 0} |
              Bytes: {Math.round((stats.bytesReceived || 0) / 1024)}KB
            </div>
          )}
        </div>
      </CardContent>
    </Card>
  );
};

const CameraServer: React.FC<{
  serverId: string;
  serverInfo: {
    device_info: {
      hostname: string;
      ip: string;
      model: string;
    };
    streams: Record<string, {
      rtsp_url: string;
      status: string;
    }>;
  };
  maximizedCamera: string | null;
  wsRef: React.MutableRefObject<WebSocket | null>;
  onMaximize: (cameraId: string) => void;
}> = ({ serverId, serverInfo, maximizedCamera, wsRef, onMaximize }) => {
  return (
    <div className="mt-4">
      <h2 className="text-xl font-bold mb-2">{serverInfo.device_info.hostname || serverId}</h2>
      <div className="text-sm text-gray-500 mb-4">
        <p>IP: {serverInfo.device_info.ip || 'Unknown'}</p>
        <p>Model: {serverInfo.device_info.model || 'Unknown'}</p>
      </div>

      <div className="flex flex-wrap gap-4">
        {Object.entries(serverInfo.streams).map(([streamId, streamInfo]) => {
          const fullCameraId = `${serverId}_${streamId}`;

          if (maximizedCamera && fullCameraId !== maximizedCamera) {
            return null;
          }

          return (
            <CameraStream
              key={fullCameraId}
              cameraId={fullCameraId}
              devicePath={streamInfo.rtsp_url}
              wsRef={wsRef}
              onMaximize={onMaximize}
              isMaximized={maximizedCamera === fullCameraId}
            />
          );
        })}
      </div>
    </div>
  );
};

const MultiCameraViewer: React.FC = () => {
  const [cameraConfig, setCameraConfig] = useState<CameraConfig | null>(null);
  const [maximizedCamera, setMaximizedCamera] = useState<string | null>(null);
  const [layout, setLayout] = useState<'grid' | 'single'>('grid');
  const [activeTab, setActiveTab] = useState<string>('all');
  const wsRef = useRef<WebSocket | null>(null);

  useEffect(() => {
    const ws = new WebSocket('ws://localhost:8080/ws/camera-webrtc');
    wsRef.current = ws;

    const handleMessage = (event: MessageEvent): void => {
      const message: WebRTCMessage = JSON.parse(event.data);
      if (message.type === 'camera-config' && message.config) {
        setCameraConfig(message.config);

        // Set first server as active tab if available
        if (message.config.CAMERA_SERVERS) {
          const servers = Object.keys(message.config.CAMERA_SERVERS);
          if (servers.length > 0) {
            setActiveTab(servers[0]);
          }
        }
      }
    };

    ws.addEventListener('message', handleMessage);

    return () => {
      ws.removeEventListener('message', handleMessage);
      ws.close();
      wsRef.current = null;
    };
  }, []);

  const handleMaximize = (cameraId: string): void => {
    setMaximizedCamera(maximizedCamera === cameraId ? null : cameraId);
  };

  const refreshCameraConfig = async (): Promise<void> => {
    try {
      const response = await fetch('http://localhost:8080/camera-status');
      const data = await response.json();
      setCameraConfig(data);
    } catch (error) {
      console.error('Failed to refresh camera configuration:', error);
    }
  };

  if (!cameraConfig) {
    return (
      <div className="flex items-center justify-center h-screen">
        <p>Loading camera configuration...</p>
      </div>
    );
  }

  return (
    <div className="container mx-auto p-4">
      <div className="mb-4 flex justify-between items-center">
        <h1 className="text-2xl font-bold">Camera Streams</h1>
        <div className="flex space-x-2">
          <Button
            onClick={refreshCameraConfig}
            variant="outline"
            size="sm"
          >
            <RefreshCw className="h-4 w-4 mr-2" />
            Refresh
          </Button>
          <Button
            onClick={() => setLayout(layout === 'grid' ? 'single' : 'grid')}
            variant="outline"
            size="sm"
          >
            <Grid2X2 className="h-4 w-4 mr-2" />
            {layout === 'grid' ? 'Single View' : 'Grid View'}
          </Button>
        </div>
      </div>

      {cameraConfig.CAMERA_SERVERS && Object.keys(cameraConfig.CAMERA_SERVERS).length > 0 ? (
        <Tabs value={activeTab} onValueChange={setActiveTab} className="w-full">
          <TabsList className="mb-4">
            <TabsTrigger value="all">All Cameras</TabsTrigger>
            {Object.keys(cameraConfig.CAMERA_SERVERS).map(serverId => (
              <TabsTrigger key={serverId} value={serverId}>
                {cameraConfig.CAMERA_SERVERS[serverId].device_info.hostname || serverId}
              </TabsTrigger>
            ))}
          </TabsList>

          <ScrollArea className="h-[calc(100vh-12rem)]">
            <TabsContent value="all">
              <div className="space-y-6">
                {Object.entries(cameraConfig.CAMERA_SERVERS).map(([serverId, serverInfo]) => (
                  <CameraServer
                    key={serverId}
                    serverId={serverId}
                    serverInfo={serverInfo}
                    maximizedCamera={maximizedCamera}
                    wsRef={wsRef}
                    onMaximize={handleMaximize}
                  />
                ))}
              </div>
            </TabsContent>

            {Object.entries(cameraConfig.CAMERA_SERVERS).map(([serverId, serverInfo]) => (
              <TabsContent key={serverId} value={serverId}>
                <CameraServer
                  serverId={serverId}
                  serverInfo={serverInfo}
                  maximizedCamera={maximizedCamera}
                  wsRef={wsRef}
                  onMaximize={handleMaximize}
                />
              </TabsContent>
            ))}
          </ScrollArea>
        </Tabs>
      ) : (
        <div className="text-center p-8">
          <p className="text-lg text-gray-500">No camera servers detected.</p>
          <Button
            onClick={refreshCameraConfig}
            variant="outline"
            className="mt-4"
          >
            <RefreshCw className="h-4 w-4 mr-2" />
            Scan for Cameras
          </Button>
        </div>
      )}
    </div>
  );
};

export default MultiCameraViewer;