import { useEffect, useState, useRef } from 'react';
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
    onEndEffectorMove?: (position: THREE.Vector3) => void;
}

export default function ArmModel({ 
    jvalue1, 
    jvalue2, 
    jvalue3, 
    jvalue4, 
    jvalue5, 
    jvalue6,
    onEndEffectorMove 
}: JointInterface) {
    const containerRef = useRef<HTMLDivElement>(null);
    const endEffectorRef = useRef<THREE.Group | null>(null);
    const transformControlsRef = useRef<typeof TransformControls | null>(null);
    const robotRef = useRef<any>(null);

    useEffect(() => {
        if (!containerRef.current) return;

        // Scene setup
        const scene = new THREE.Scene();
        const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
        camera.position.z = 2;

        const renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(1250, 600);
        renderer.setClearColor(0xFFFFFF);
        containerRef.current.appendChild(renderer.domElement);

        // Lighting
        const ambientLight = new THREE.AmbientLight(0x404040);
        scene.add(ambientLight);

        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.5);
        directionalLight.position.set(1, 1, 1);
        scene.add(directionalLight);

        // Controls
        const orbitControls = new OrbitControls(camera, renderer.domElement);
        orbitControls.update();

        // Create end effector marker
        const endEffectorMarker = new THREE.Group();
        
        // Add a small sphere to mark the end effector position
        const sphereGeometry = new THREE.SphereGeometry(0.02);
        const sphereMaterial = new THREE.MeshStandardMaterial({ color: 0x00ff00 });
        const sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
        endEffectorMarker.add(sphere);

        // Create transform controls
        const transformControls = new TransformControls(camera, renderer.domElement);
        transformControls.setMode('translate');
        transformControls.setSize(0.5);
        transformControls.attach(endEffectorMarker);
        scene.add(transformControls);
        transformControlsRef.current = transformControls;

        // Handle transform control events
        transformControls.addEventListener('dragging-changed', (event) => {
            orbitControls.enabled = !event.value;
            
            if (!event.value && onEndEffectorMove) {
                // Send position to IK solver when dragging ends
                onEndEffectorMove(endEffectorMarker.position);
            }
        });

        transformControls.addEventListener('objectChange', () => {
            if (onEndEffectorMove) {
                // Continuously update position during drag
                onEndEffectorMove(endEffectorMarker.position);
            }
        });

        // Load URDF
        const manager = new THREE.LoadingManager();
        const loader = new URDFLoader(manager);

        loader.load('RoboticArmv2/urdf/RoboticArmv2.urdf', robot => {
            console.log("Arm urdf loaded");
            robotRef.current = robot;
            
            robot.rotation.z = Math.PI / 1;
            robot.rotation.y = Math.PI / 1;
            robot.rotation.x = Math.PI / 2;

            // Set joint values
            // robot.joints[`arm_j1`].setJointValue(THREE.MathUtils.degToRad(jvalue1));
            // robot.joints[`arm_j2`].setJointValue(THREE.MathUtils.degToRad(jvalue2));
            // robot.joints[`arm_j3`].setJointValue(THREE.MathUtils.degToRad(jvalue3));
            // robot.joints[`arm_j4`].setJointValue(THREE.MathUtils.degToRad(jvalue4));
            // robot.joints[`arm_j5`].setJointValue(THREE.MathUtils.degToRad(jvalue5));
            // robot.joints[`arm_j6`].setJointValue(THREE.MathUtils.degToRad(jvalue6));

            // Add end effector marker at the last joint
            // const lastJoint = robot.joints[`arm_j6`];
            // if (lastJoint) {
            //     const position = new THREE.Vector3();
            //     lastJoint.getWorldPosition(position);
            //     endEffectorMarker.position.copy(position);
            // }

            scene.add(robot);
            // scene.add(endEffectorMarker);
            // endEffectorRef.current = endEffectorMarker;
        },
        undefined,
        error => {
            console.error('An error occurred while loading the URDF model:', error.message);
            console.log('Hello:', error.message)
        });

        // Animation loop
        const animate = () => {
            requestAnimationFrame(animate);
            orbitControls.update();
            renderer.render(scene, camera);

            // Update end effector position if robot moves
            if (robotRef.current && endEffectorRef.current) {
                const lastJoint = robotRef.current.joints[`arm_j6`];
                if (lastJoint) {
                    const position = new THREE.Vector3();
                    lastJoint.getWorldPosition(position);
                    if (!transformControls.dragging) {
                        endEffectorRef.current.position.copy(position);
                    }
                }
            }
        };
        animate();

        // Cleanup
        return () => {
            renderer.dispose();
            containerRef.current?.removeChild(renderer.domElement);
        };

    }, [jvalue1, jvalue2, jvalue3, jvalue4, jvalue5, jvalue6, onEndEffectorMove]);

    return (
        <div className='w-full h-48' ref={containerRef} />
    );
}