import { useEffect, useRef } from 'react';
import * as THREE from 'three';
import URDFLoader from 'urdf-loader';
import { OrbitControls, DragControls } from 'three-stdlib';

interface JointInterface {
    jvalue1: number;
    jvalue2: number;
    jvalue3: number;
    jvalue4: number;
    jvalue5: number;
    jvalue6: number;
}

export default function ArmModel({ jvalue1, jvalue2, jvalue3, jvalue4, jvalue5, jvalue6 }: JointInterface) {
    const containerRef = useRef<HTMLDivElement>(null);
    const robotRef = useRef<any>(null);
    const sphereRef = useRef<THREE.Mesh | null>(null);
    const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
    const sceneRef = useRef<THREE.Scene | null>(null);
    const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
    const orbitControlsRef = useRef<OrbitControls | null>(null);

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

        const sphereGeometry = new THREE.SphereGeometry(0.05, 32, 32);
        const sphereMaterial = new THREE.MeshStandardMaterial({ 
            color: 0x0088ff,
            transparent: true,
            opacity: 0.7
        });
        const sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
        sphere.position.set(0, 0, 0);
        sphereRef.current = sphere;
        scene.add(sphere);

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
                if (endEffectorPosition && sphereRef.current) {
                    sphereRef.current.position.copy(endEffectorPosition);
                }
                
                if (sphereRef.current && cameraRef.current && rendererRef.current) {
                    const dragControls = new DragControls(
                        [sphereRef.current], 
                        cameraRef.current, 
                        rendererRef.current.domElement
                    );
                    
                    dragControls.addEventListener('dragstart', () => {
                        if (orbitControlsRef.current) {
                            orbitControlsRef.current.enabled = false;
                        }
                    });
                    
                    dragControls.addEventListener('dragend', () => {
                        if (orbitControlsRef.current) {
                            orbitControlsRef.current.enabled = true;
                        }
                    });
                    
                    dragControls.addEventListener('hoveron', () => {
                        if (sphereRef.current) {
                            document.body.style.cursor = 'pointer';
                            sphereRef.current.material.opacity = 0.9;
                            sphereRef.current.material.color.set(0x00AAFF);
                        }
                    });
                    
                    dragControls.addEventListener('hoveroff', () => {
                        if (sphereRef.current) {
                            document.body.style.cursor = 'auto';
                            sphereRef.current.material.opacity = 0.7;
                            sphereRef.current.material.color.set(0x0088FF);
                        }
                    });

                    dragControls.addEventListener('drag', (event) => {
                        console.log(`Sphere Position: x=${sphere.position.x.toFixed(3)}, y=${sphere.position.y.toFixed(3)}, z=${sphere.position.z.toFixed(3)}`);
                    });
                }
            }, 100);
        },
        undefined,
        error => {
            console.error('An error occurred while loading the URDF model:', error.message);
        });

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
            if (rendererRef.current) {
                rendererRef.current.dispose();
                containerRef.current?.removeChild(rendererRef.current.domElement);
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

    return <div className='w-full h-48' ref={containerRef} />;
}