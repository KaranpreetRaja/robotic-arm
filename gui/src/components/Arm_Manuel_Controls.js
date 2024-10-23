import { useState, useEffect, useRef } from 'react';
import Form from 'react-bootstrap/Form';
import ArmModel from './ArmModel';

export default function ArmManuelControl() {
    // set and send Arm Positions
    const [value1, setValue1] = useState(0);
    const [value2, setValue2] = useState(0);
    const [value3, setValue3] = useState(0);
    const [value4, setValue4] = useState(0);
    const [value5, setValue5] = useState(0);
    const [value6, setValue6] = useState(0);
    const [value7, setValue7] = useState(0);

    const intervalRef = useRef(null);

    const startClosingClaw = () => {
      intervalRef.current = setInterval(() => {
        setValue7(prevValue => prevValue - 1);
      }, 100); 
    };
  
    const startOpeningClaw = () => {
      intervalRef.current = setInterval(() => {
        setValue7(prevValue => prevValue + 1);
      }, 100); 
    };
  
    const stopChanging = () => {
      clearInterval(intervalRef.current);
    };

    // const sendControlValues = useSendTopic("/arm_control_values");

    // Receive Arm Positions
    const [receivedPositions, setReceivedPositions] = useState(null);
    // useReceiveTopic("/GUI_commence", setReceivedPositions)

    
    const [currentPosition1, setCurrentPosition1] = useState(0);
    const [currentPosition2, setCurrentPosition2] = useState(0);
    const [currentPosition3, setCurrentPosition3] = useState(0);
    const [currentPosition4, setCurrentPosition4] = useState(0);
    const [currentPosition5, setCurrentPosition5] = useState(0);
    const [currentPosition6, setCurrentPosition6] = useState(0);
    const [currentPosition7, setCurrentPosition7] = useState(0);

    // useEffect(() => {
    //     //Send
    //     sendControlValues([value1, value2, value3, value4, value5, value6, value7]);
    //     console.log([value1, value2, value3, value4, value5, value6, value7])

    // },[sendControlValues, value1, value2, value3, value4, value5, value6, value7])

    useEffect(() => {
        //Receive
        if (receivedPositions != null){
            const positions = receivedPositions.position
            setCurrentPosition1(positions[0])
            setCurrentPosition2(positions[1])
            setCurrentPosition3(positions[2])
            setCurrentPosition4(positions[3])
            setCurrentPosition5(positions[4])
            setCurrentPosition6(positions[5])
            setCurrentPosition7(positions[6])
        }
        console.log(`${currentPosition1} ${currentPosition2} ${currentPosition3} ${currentPosition4} ${currentPosition5} ${currentPosition6} ${currentPosition7}`)
    }, [receivedPositions])

    function OpenArmModalWindow() {
        const armModelContent = document.getElementById('arm-modal-content').outerHTML;
        const newWindow = window.open("", "", "width=500,height=400");
        
        if (newWindow) {
            newWindow.document.write("<!DOCTYPE html><html><head><title>Modal Window</title></head><body>");
            newWindow.document.write(armModelContent);
            newWindow.document.write("</body></html>");
        }
    }
    
    return (
        <div className='w-full z-10'>

            {/* Grip Controls */}

            <div className="controller-container-2">
                <button onMouseDown={startClosingClaw} onMouseUp={stopChanging} onMouseLeave={stopChanging} type="button" class="btn btn-dark w-full text-2xl font-bold mb-4">
                    Hold to close claw
                </button>
                <button onMouseDown={startOpeningClaw} onMouseUp={stopChanging} onMouseLeave={stopChanging} type="button" class="btn btn-dark w-full text-2xl font-bold mb-4">
                    Hold to open claw
                </button>
                <div className='w-full flex justify-center'>
                    <span className='text-2xl font-bold text-white'>{value7} degrees</span>
                </div>
            </div>

            {/* Arm Controls: */}

            {/* Manuel Way */}
            <div className="random-wrapper">
                <div className="title-container">
                 <span className="arm-component-title">ManualWay</span>
                </div>
            </div>

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

        <div id="arm-modal-content">
            <ArmModel jvalue1={value1} jvalue2={value2} jvalue3={value3} jvalue4={value4} jvalue5={value5} jvalue6={value6} jvalue7={value7}/>
        </div>

        {/* <div className='w-full flex justify-center mt-96'> 
            <button className='' onClick={OpenArmModalWindow} type="button" class="btn btn-light">Open Modal Window</button>
        </div> */}
        </div>
    );
}
