import { useEffect, useRef } from 'react';
import * as THREE from 'three';
import URDFLoader from 'urdf-loader';
import { OrbitControls } from 'three-stdlib';

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

        // Load URDF
        const manager = new THREE.LoadingManager();
        const loader = new URDFLoader(manager);

        loader.load('RoboticArmv2/urdf/RoboticArmv2.urdf', robot => {
            console.log("Arm URDF loaded");
            robotRef.current = robot;

            robot.rotation.set(Math.PI / 2, Math.PI, Math.PI);

            // Set joint values
            robot.joints[`J1`].setJointValue(THREE.MathUtils.degToRad(jvalue1));
            robot.joints[`J2`].setJointValue(THREE.MathUtils.degToRad(jvalue2));
            robot.joints[`J3`].setJointValue(THREE.MathUtils.degToRad(jvalue3));
            robot.joints[`J4`].setJointValue(THREE.MathUtils.degToRad(jvalue4));
            robot.joints[`J5`].setJointValue(THREE.MathUtils.degToRad(jvalue5));
            robot.joints[`J6`].setJointValue(THREE.MathUtils.degToRad(jvalue6));

            scene.add(robot);
        },
        undefined,
        error => {
            console.error('An error occurred while loading the URDF model:', error.message);
        });

        // Animation loop
        const animate = () => {
            requestAnimationFrame(animate);
            orbitControls.update();
            renderer.render(scene, camera);
        };
        animate();

        // Cleanup
        return () => {
            renderer.dispose();
            containerRef.current?.removeChild(renderer.domElement);
        };
    }, [jvalue1, jvalue2, jvalue3, jvalue4, jvalue5, jvalue6]);

    return <div className='w-full h-48' ref={containerRef} />;
}