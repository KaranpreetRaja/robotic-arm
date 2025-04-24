import { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import URDFLoader from 'urdf-loader';
import { OrbitControls, TransformControls } from 'three-stdlib';

interface JointInterface {
    jvalue1: number;
    jvalue2: number;
    jvalue3: number;
    jvalue4: number;
    jvalue5: number;
    jvalue6: number;
    websocketPub: any;
    pubConnectionStatus: any;
    subConnectionStatus: any;
}

interface EndEffectorPose {
    position: THREE.Vector3;
    rotation: THREE.Euler;
}

export default function ArmModel({ 
    jvalue1, 
    jvalue2, 
    jvalue3, 
    jvalue4, 
    jvalue5, 
    jvalue6,
    websocketPub,
    pubConnectionStatus,
    subConnectionStatus
}: JointInterface) {
    const containerRef = useRef<HTMLDivElement>(null);
    const robotRef = useRef<any>(null);
    const targetRef = useRef<THREE.Group | null>(null);
    const transformControlsRef = useRef<TransformControls | null>(null);
    const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
    const sceneRef = useRef<THREE.Scene | null>(null);
    const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
    const orbitControlsRef = useRef<OrbitControls | null>(null);
    const [controlMode, setControlMode] = useState<'translate' | 'rotate'>('translate');
    const [currentPose, setCurrentPose] = useState<EndEffectorPose | null>(null);
    const [continuousSend, setContinuousSend] = useState(false);
    const [updateFrequency, setUpdateFrequency] = useState(10); // Hz
    const intervalRef = useRef<NodeJS.Timeout | null>(null);


    useEffect(() => {
       
        if (intervalRef.current) {
            clearInterval(intervalRef.current);
            intervalRef.current = null;
        }

        
        if (continuousSend && pubConnectionStatus === 'connected' && currentPose) {
            const intervalTime = Math.floor(1000 / updateFrequency); 
            intervalRef.current = setInterval(() => {
                sendPoseToServer();
            }, intervalTime);
            
            console.log(`Started continuous pose updates at ${updateFrequency}Hz (${intervalTime}ms)`);
        }

        
        return () => {
            if (intervalRef.current) {
                clearInterval(intervalRef.current);
                intervalRef.current = null;
            }
        };
    }, [continuousSend, pubConnectionStatus, updateFrequency, currentPose]);

    
    const sendPoseToServer = () => {
        if (!websocketPub || pubConnectionStatus !== 'connected' || !currentPose) {
            console.error('Cannot send pose: WebSocket not connected or pose not available');
            return;
        }
        
        if (targetRef.current) {
            const position = new THREE.Vector3();
            targetRef.current.getWorldPosition(position);
            
            const rotation = new THREE.Euler().setFromQuaternion(targetRef.current.quaternion);
            
            
            const poseData = {
                position: {
                    x: position.x.toFixed(4),
                    y: position.y.toFixed(4),
                    z: position.z.toFixed(4)
                },
                rotation: {
                    x: rotation.x.toFixed(4),
                    y: rotation.y.toFixed(4),
                    z: rotation.z.toFixed(4)
                }
            };
            
            
            const message = {
                topic: "/robot/raw/target_pose", 
                message: JSON.stringify(poseData)
            };
            
        
            websocketPub.send(JSON.stringify(message));
        }
    };

    const getEndEffectorPosition = () => {
        if (!robotRef.current) return null;
        
        const endEffectorNames = ['tool0', 'link7', 'Link7', 'tool', 'Tool', 'end_effector', 'gripper', 'eef'];
        let endEffectorLink = null;
        
        for (const name of endEffectorNames) {
            if (robotRef.current.links[name]) {
                console.log(`Found end effector link: ${name}`);
                endEffectorLink = robotRef.current.links[name];
                break;
            }
        }
        
        if (endEffectorLink) {
            const position = new THREE.Vector3();
            endEffectorLink.getWorldPosition(position);
            return position;
        } else if (robotRef.current.joints['J6']) {
            const j6Joint = robotRef.current.joints['J6'];
            const position = new THREE.Vector3();
            j6Joint.getWorldPosition(position);
            return position;
        }
        
        return new THREE.Vector3(0, 0, 0);
    };

    useEffect(() => {
        if (!containerRef.current) return;

        const scene = new THREE.Scene();
        sceneRef.current = scene;
        
        const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
        camera.position.z = 2;
        cameraRef.current = camera;

        const renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(1250, 600);
        renderer.setClearColor(0xFFFFFF);
        containerRef.current.appendChild(renderer.domElement);
        rendererRef.current = renderer;

        const ambientLight = new THREE.AmbientLight(0x404040);
        scene.add(ambientLight);

        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.5);
        directionalLight.position.set(1, 1, 1);
        scene.add(directionalLight);

        const orbitControls = new OrbitControls(camera, renderer.domElement);
        orbitControls.update();
        orbitControlsRef.current = orbitControls;

        const targetGroup = new THREE.Group();
        targetRef.current = targetGroup;
        scene.add(targetGroup);

        const sphereGeometry = new THREE.SphereGeometry(0.05, 32, 32);
        const sphereMaterial = new THREE.MeshStandardMaterial({ 
            color: 0x0088ff,
            transparent: true,
            opacity: 0.7
        });
        const sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
        targetGroup.add(sphere);

        const axisLength = 0.1;
        
        const xAxisGeometry = new THREE.CylinderGeometry(0.005, 0.005, axisLength);
        const xAxisMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
        const xAxis = new THREE.Mesh(xAxisGeometry, xAxisMaterial);
        xAxis.rotation.z = Math.PI / 2;
        xAxis.position.x = axisLength / 2;
        targetGroup.add(xAxis);
        
        const yAxisGeometry = new THREE.CylinderGeometry(0.005, 0.005, axisLength);
        const yAxisMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
        const yAxis = new THREE.Mesh(yAxisGeometry, yAxisMaterial);
        yAxis.position.y = axisLength / 2;
        targetGroup.add(yAxis);
        
        const zAxisGeometry = new THREE.CylinderGeometry(0.005, 0.005, axisLength);
        const zAxisMaterial = new THREE.MeshBasicMaterial({ color: 0x0000ff });
        const zAxis = new THREE.Mesh(zAxisGeometry, zAxisMaterial);
        zAxis.rotation.x = Math.PI / 2;
        zAxis.position.z = axisLength / 2;
        targetGroup.add(zAxis);

        const transformControls = new TransformControls(camera, renderer.domElement);
        transformControls.attach(targetGroup);
        transformControls.setMode('translate');
        transformControls.setSize(0.75);
        scene.add(transformControls);
        transformControlsRef.current = transformControls;

        transformControls.addEventListener('dragging-changed', (event) => {
            if (orbitControlsRef.current) {
                orbitControlsRef.current.enabled = !event.value;
            }
        });

        transformControls.addEventListener('objectChange', () => {
            if (targetRef.current) {
                const position = new THREE.Vector3();
                targetRef.current.getWorldPosition(position);
                
                const pose: EndEffectorPose = {
                    position: position,
                    rotation: new THREE.Euler().setFromQuaternion(targetRef.current.quaternion)
                };
                
                setCurrentPose(pose);
            }
        });

        const manager = new THREE.LoadingManager();
        const loader = new URDFLoader(manager);

        loader.load('RoboticArmv2/urdf/RoboticArmv2.urdf', robot => {
            console.log("Arm URDF loaded");
            robotRef.current = robot;

            robot.rotation.set(Math.PI / 2, Math.PI, Math.PI);

            robot.joints[`J1`].setJointValue(THREE.MathUtils.degToRad(jvalue1));
            robot.joints[`J2`].setJointValue(THREE.MathUtils.degToRad(jvalue2));
            robot.joints[`J3`].setJointValue(THREE.MathUtils.degToRad(jvalue3));
            robot.joints[`J4`].setJointValue(THREE.MathUtils.degToRad(jvalue4));
            robot.joints[`J5`].setJointValue(THREE.MathUtils.degToRad(jvalue5));
            robot.joints[`J6`].setJointValue(THREE.MathUtils.degToRad(jvalue6));

            scene.add(robot);
            
            setTimeout(() => {
                const endEffectorPosition = getEndEffectorPosition();
                if (endEffectorPosition && targetRef.current) {
                    targetRef.current.position.copy(endEffectorPosition);
                    
                    setCurrentPose({
                        position: new THREE.Vector3().copy(endEffectorPosition),
                        rotation: new THREE.Euler().setFromQuaternion(targetRef.current.quaternion)
                    });
                }
            }, 100);
        },
        undefined,
        error => {
            console.error('An error occurred while loading the URDF model:', error.message);
        });

        const handleKeyDown = (event: KeyboardEvent) => {
            if (event.key === 'r' || event.key === 'R') {
                if (transformControlsRef.current) {
                    transformControlsRef.current.setMode('rotate');
                    setControlMode('rotate');
                }
            } else if (event.key === 't' || event.key === 'T') {
                if (transformControlsRef.current) {
                    transformControlsRef.current.setMode('translate');
                    setControlMode('translate');
                }
            } else if (event.key === 's' || event.key === 'S') {
                sendPoseToServer();
            } else if (event.key === 'c' || event.key === 'C') {
                setContinuousSend(prev => !prev);
            }
        };

        window.addEventListener('keydown', handleKeyDown);

        const animate = () => {
            requestAnimationFrame(animate);
            if (orbitControlsRef.current) {
                orbitControlsRef.current.update();
            }
            if (sceneRef.current && cameraRef.current && rendererRef.current) {
                rendererRef.current.render(sceneRef.current, cameraRef.current);
            }
        };
        animate();

        return () => {
            window.removeEventListener('keydown', handleKeyDown);
            
            if (rendererRef.current) {
                rendererRef.current.dispose();
            }
            if (containerRef.current && rendererRef.current) {
                containerRef.current.removeChild(rendererRef.current.domElement);
            }
        };
    }, []);

    useEffect(() => {
        if (!robotRef.current) return;

        robotRef.current.joints[`J1`].setJointValue(THREE.MathUtils.degToRad(jvalue1));
        robotRef.current.joints[`J2`].setJointValue(THREE.MathUtils.degToRad(jvalue2));
        robotRef.current.joints[`J3`].setJointValue(THREE.MathUtils.degToRad(jvalue3));
        robotRef.current.joints[`J4`].setJointValue(THREE.MathUtils.degToRad(jvalue4));
        robotRef.current.joints[`J5`].setJointValue(THREE.MathUtils.degToRad(jvalue5));
        robotRef.current.joints[`J6`].setJointValue(THREE.MathUtils.degToRad(jvalue6));
    }, [jvalue1, jvalue2, jvalue3, jvalue4, jvalue5, jvalue6]);

    return (
        <div className="flex flex-col">
            <div className='w-full h-48' ref={containerRef} />
            <div className="mt-48 flex flex-col gap-2 ml-4">
                <div className="flex items-center gap-2">
                    <div className={`w-3 h-3 rounded-full ${
                        pubConnectionStatus === 'connected' ? 'bg-green-500' : 
                        pubConnectionStatus === 'error' ? 'bg-red-500' : 'bg-gray-500'
                    }`}></div>
                    <span className="text-xl">
                        Publisher WebSocket: {pubConnectionStatus}
                    </span>
                </div>

                <div className="flex items-center gap-2">
                    <div className={`w-3 h-3 rounded-full ${
                        subConnectionStatus === 'connected' ? 'bg-green-500' : 
                        subConnectionStatus === 'error' ? 'bg-red-500' : 'bg-gray-500'
                    }`}></div>
                    <span className="text-xl">
                        Subscriber WebSocket: {subConnectionStatus}
                    </span>
                </div>
                
                <div className="flex gap-2 flex-wrap">
                    <button 
                        onClick={() => {
                            if (transformControlsRef.current) {
                                transformControlsRef.current.setMode('translate');
                                setControlMode('translate');
                            }
                        }}
                        className={`px-3 py-1 text-sm rounded ${
                            controlMode === 'translate' 
                                ? 'bg-blue-600 text-white' 
                                : 'bg-gray-200 text-gray-800'
                        }`}
                    >
                        Position (T)
                    </button>
                    
                    <button 
                        onClick={() => {
                            if (transformControlsRef.current) {
                                transformControlsRef.current.setMode('rotate');
                                setControlMode('rotate');
                            }
                        }}
                        className={`px-3 py-1 text-sm rounded ${
                            controlMode === 'rotate' 
                                ? 'bg-blue-600 text-white' 
                                : 'bg-gray-200 text-gray-800'
                        }`}
                    >
                        Rotation (R)
                    </button>
                    
                    <button 
                        onClick={sendPoseToServer}
                        disabled={pubConnectionStatus !== 'connected' || !currentPose}
                        className={`px-3 py-1 text-sm rounded ${
                            pubConnectionStatus === 'connected' && currentPose
                                ? 'bg-green-600 text-white hover:bg-green-700' 
                                : 'bg-gray-300 text-gray-500 cursor-not-allowed'
                        }`}
                    >
                        Send Once (S)
                    </button>
                    
                    <button 
                        onClick={() => setContinuousSend(!continuousSend)}
                        disabled={pubConnectionStatus !== 'connected' || !currentPose}
                        className={`px-3 py-1 text-sm rounded ${
                            pubConnectionStatus === 'connected' && currentPose
                                ? (continuousSend 
                                    ? 'bg-red-600 text-white hover:bg-red-700'
                                    : 'bg-green-600 text-white hover:bg-green-700')
                                : 'bg-gray-300 text-gray-500 cursor-not-allowed'
                        }`}
                    >
                        {continuousSend ? 'Stop Streaming (C)' : 'Start Streaming (C)'}
                    </button>
                </div>
                
                {continuousSend && (
                    <div className="flex items-center gap-2 mt-2">
                        <span className="text-sm">Update Rate:</span>
                        <input 
                            type="range" 
                            min="1" 
                            max="30" 
                            value={updateFrequency} 
                            onChange={(e) => setUpdateFrequency(parseInt(e.target.value))}
                            className="w-32"
                        />
                        <span className="text-sm font-mono">{updateFrequency} Hz</span>
                    </div>
                )}
                
                {currentPose && (
                    <div className="mt-2 text-xl font-mono bg-gray-100 p-2 rounded">
                        <div>Sphere Position: 
                            x: {currentPose.position.x.toFixed(4)}, 
                            y: {currentPose.position.y.toFixed(4)}, 
                            z: {currentPose.position.z.toFixed(4)}
                        </div>
                        <div>Sphere Rotation: 
                            x: {currentPose.rotation.x.toFixed(4)}, 
                            y: {currentPose.rotation.y.toFixed(4)}, 
                            z: {currentPose.rotation.z.toFixed(4)}
                        </div>
                        {continuousSend && (
                            <div className="mt-1 text-green-600">
                                Streaming data at {updateFrequency} Hz
                            </div>
                        )}
                    </div>
                )}
            </div>
        </div>
    );
}