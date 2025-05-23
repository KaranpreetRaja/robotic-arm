import { useEffect, useState } from 'react';
import ArmModel from './components/model'
import Form from 'react-bootstrap/Form';
import ScratchBlocks from './components/scratch';

function App() {
  const [value1, setValue1] = useState(0);
  const [value2, setValue2] = useState(0);
  const [value3, setValue3] = useState(0);
  const [value4, setValue4] = useState(0);
  const [value5, setValue5] = useState(0);
  const [value6, setValue6] = useState(0);
  const [websocketPub, setWebsocketPub] = useState<WebSocket | null>(null);
  const [websocketSub, setWebsocketSub] = useState<WebSocket | null>(null);
  const [pubConnectionStatus, pubSetConnectionStatus] = useState<'disconnected' | 'connected' | 'error'>('disconnected');
  const [subConnectionStatus, subSetConnectionStatus] = useState<'disconnected' | 'connected' | 'error'>('disconnected');
  const [jointStates, setJointStates] = useState<any>([]);

  useEffect(() => {
    const ws = new WebSocket('ws://127.0.0.1:8080/ws_publish');

    ws.onopen = () => {
      console.log('Publisher WebSocket Connected');
      pubSetConnectionStatus('connected');
    };

    ws.onclose = () => {
      console.log('Publisher WebSocket Disconnected');
      pubSetConnectionStatus('disconnected');
    };

    ws.onerror = (error) => {
      console.error('Publisher WebSocket Error:', error);
      pubSetConnectionStatus('error');
    };

    setWebsocketPub(ws);

    return () => {
      ws.close();
    };
  }, []);

  useEffect(() => {
    const ws = new WebSocket('ws://127.0.0.1:8080/ws_sub');

    ws.onopen = () => {
      console.log('Subscriber WebSocket Connected');
      subSetConnectionStatus('connected');
      ws.send(JSON.stringify({ topic: '/string/joint_states' }));
    };

    ws.onmessage = (e) => {
      const data = JSON.parse(e.data);
      console.log(data)

      if (data.topic === "/string/joint_states") {
        const jointData = JSON.parse(data.message);
        const jointValues = Object.values(jointData);
        setJointStates(jointValues);
      };
    }

    ws.onclose = () => {
      console.log('Subscriber WebSocket Disconnected');
      subSetConnectionStatus('disconnected');
    };

    ws.onerror = (error) => {
      console.error('Subscriber WebSocket Error:', error);
      subSetConnectionStatus('error');
    };

    setWebsocketSub(ws);

    return () => {
      ws.close();
    };
  }, []);

  useEffect(() => {
    console.log(jointStates)
  }, [jointStates])


  const sendJointAnglesToServer = () => {
    if (!websocketPub || pubConnectionStatus !== 'connected') {
      console.error('Cannot send pose: WebSocket not connected or pose not available');
      return;
    }

    const poseData = {
      orientation: {
        value1,
        value2,
        value3,
        value4,
        value5,
        value6,
      },
    };

    const message = {
      topic: "/robot/raw/joint_angles",
      message: JSON.stringify(poseData)
    };

    websocketPub.send(JSON.stringify(message));
  }

  return (
    <div className='h-screen w-screen bg-base flex flex-col'>
      <div className='w-full h-1/2 flex flex-row px-6 py-4 space-x-4'>
        <div className='w-full h-full'>
          <ScratchBlocks websocketPub={websocketPub} pubConnectionStatus={pubConnectionStatus} />
        </div>
      </div>
      <div className='w-full h-1/2 px-6 flex flex-row space-x-4'>
        <ArmModel jvalue1={value1} jvalue2={value2} jvalue3={value3} jvalue4={value4} jvalue5={value5} jvalue6={value6}
          websocketPub={websocketPub} pubConnectionStatus={pubConnectionStatus} subConnectionStatus={subConnectionStatus} 
          jointStates={jointStates}
          />
        <div className='bg-[#333] w-full h-[600px] rounded-lg flex flex-col space-y-4 p-6'>
          <button
            onClick={sendJointAnglesToServer}
            className='bg-blue-600 hover:bg-blue-700 text-white py-2 px-4 text-2xl rounded-lg transition duration-300'>
            Send Arm Orientation
          </button>
          <div className="flex items-center gap-2">
            <div className={`w-3 h-3 rounded-full ${pubConnectionStatus === 'connected' ? 'bg-green-500' :
              pubConnectionStatus === 'error' ? 'bg-red-500' : 'bg-gray-500'
              }`}></div>
            <span className="text-2xl text-white">
              Publisher WebSocket: {pubConnectionStatus}
            </span>
          </div>
          <div className="flex items-center gap-2">
            <div className={`w-3 h-3 rounded-full ${subConnectionStatus === 'connected' ? 'bg-green-500' :
              subConnectionStatus === 'error' ? 'bg-red-500' : 'bg-gray-500'
              }`}></div>
            <span className="text-2xl text-white">
              Subscriber WebSocket: {subConnectionStatus}
            </span>
          </div>
          <h1 className='text-white font-medium text-3xl mt-11'>Telemetry Data</h1>

          <div className='bg-[#444] p-4 rounded-lg text-2xl space-y-2'>
            <p className='text-white font-medium'>Data 1: <span className='text-gray-300'>xyz</span></p>
            <p className='text-white font-medium'>Data 2: <span className='text-gray-300'>xyz</span></p>
            <p className='text-white font-medium'>Data 3: <span className='text-gray-300'>xyz</span></p>
            <p className='text-white font-medium'>Data 4: <span className='text-gray-300'>xyz</span></p>
            <p className='text-white font-medium'>Data 5: <span className='text-gray-300'>xyz</span></p>
            <p className='text-white font-medium'>Data 6: <span className='text-gray-300'>xyz</span></p>
          </div>
        </div>

        <div>
          <div className="controller-container-2">
            <div>
              <Form.Label>Arm Joint 1</Form.Label>
              <div className="bar-control">
                <Form.Range min={-360} max={360} value={value1} onChange={e => setValue1(e.target.valueAsNumber)} />
                <span className="bar-max">{value1} degrees</span>
              </div>
            </div>
            <div>
              <Form.Label>Arm Joint 2</Form.Label>
              <div className="bar-control">
                <Form.Range min={-360} max={360} value={value2} onChange={e => setValue2(e.target.valueAsNumber)} />
                <span className="bar-max">{value2} degrees</span>
              </div>
            </div>
            <div>
              <Form.Label>Arm Joint 3</Form.Label>
              <div className="bar-control">
                <Form.Range min={-360} max={360} value={value3} onChange={e => setValue3(e.target.valueAsNumber)} />
                <span className="bar-max">{value3} degrees</span>
              </div>
            </div>
            <div>
              <Form.Label>Arm Joint 4</Form.Label>
              <div className="bar-control">
                <Form.Range min={-360} max={360} value={value4} onChange={e => setValue4(e.target.valueAsNumber)} />
                <span className="bar-max">{value4} degrees</span>
              </div>
            </div>
            <div>
              <Form.Label>Arm Joint 5</Form.Label>
              <div className="bar-control">
                <Form.Range min={-360} max={360} value={value5} onChange={e => setValue5(e.target.valueAsNumber)} />
                <span className="bar-max">{value5} degrees</span>
              </div>
            </div>
            <div>
              <Form.Label>Arm Joint 6</Form.Label>
              <div className="bar-control">
                <Form.Range min={-360} max={360} value={value6} onChange={e => setValue6(e.target.valueAsNumber)} />
                <span className="bar-max">{value6} degrees</span>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  )
}

export default App