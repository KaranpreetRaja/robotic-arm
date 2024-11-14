import { useState } from 'react';
import ArmModel from './components/model'
import * as THREE from 'three';
import Form from 'react-bootstrap/Form';

function App() {
  const [value1, setValue1] = useState(0);
  const [value2, setValue2] = useState(0);
  const [value3, setValue3] = useState(0);
  const [value4, setValue4] = useState(0);
  const [value5, setValue5] = useState(0);
  const [value6, setValue6] = useState(0);

  const handleEndEffectorMove = (position: THREE.Vector3) => {
    fetch('your-ik-solver-endpoint', {
      method: 'POST',
      body: JSON.stringify({
        x: position.x,
        y: position.y,
        z: position.z
      })
    });
  };

  return (
    <div className='h-screen w-screen bg-base flex flex-col'>
      <div className='w-full h-1/2 flex flex-row px-6 py-4 space-x-4'>
        <div className='w-1/2 h-full bg-white flex justify-center items-center'>
          Camera 1
        </div>
        <div className='w-1/2 h-full bg-white flex justify-center items-center'>
          Camera 2
        </div>
      </div>
      <div className='w-full h-1/2 px-6 flex flex-row space-x-4'>
        <ArmModel jvalue1={value1} jvalue2={value2} jvalue3={value3} jvalue4={value4} jvalue5={value5} jvalue6={value6}
          onEndEffectorMove={handleEndEffectorMove} />
        <div className='bg-[#333] w-full h-[600px] rounded-lg flex flex-col space-y-4 p-6'>
          <button className='bg-blue-600 hover:bg-blue-700 text-white py-2 px-4 text-2xl rounded-lg transition duration-300'>
            Move Arm
          </button>
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
                <Form.Range min={-150} max={150} value={value1} onChange={e => setValue1(e.target.valueAsNumber)} />
                <span className="bar-max">{value1} degrees</span>
              </div>
            </div>
            <div>
              <Form.Label>Arm Joint 2</Form.Label>
              <div className="bar-control">
                <Form.Range min={-150} max={150} value={value2} onChange={e => setValue2(e.target.valueAsNumber)} />
                <span className="bar-max">{value2} degrees</span>
              </div>
            </div>
            <div>
              <Form.Label>Arm Joint 3</Form.Label>
              <div className="bar-control">
                <Form.Range min={-150} max={150} value={value3} onChange={e => setValue3(e.target.valueAsNumber)} />
                <span className="bar-max">{value3} degrees</span>
              </div>
            </div>
            <div>
              <Form.Label>Arm Joint 4</Form.Label>
              <div className="bar-control">
                <Form.Range min={-150} max={150} value={value4} onChange={e => setValue4(e.target.valueAsNumber)} />
                <span className="bar-max">{value4} degrees</span>
              </div>
            </div>
            <div>
              <Form.Label>Arm Joint 5</Form.Label>
              <div className="bar-control">
                <Form.Range min={-150} max={150} value={value5} onChange={e => setValue5(e.target.valueAsNumber)} />
                <span className="bar-max">{value5} degrees</span>
              </div>
            </div>
            <div>
              <Form.Label>Arm Joint 6</Form.Label>
              <div className="bar-control">
                <Form.Range min={-150} max={150} value={value6} onChange={e => setValue6(e.target.valueAsNumber)} />
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
