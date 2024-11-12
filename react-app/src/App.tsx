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
      <div className='w-full h-2/5'>

      </div>
      <div className='w-full h-3/5 px-6 flex flex-row'>
        <ArmModel jvalue1={value1} jvalue2={value2} jvalue3={value3} jvalue4={value4} jvalue5={value5} jvalue6={value6}
          onEndEffectorMove={handleEndEffectorMove} />
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
