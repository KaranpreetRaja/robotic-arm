import { useEffect, useState, useRef  } from 'react';
import * as THREE from 'three';
import URDFLoader from 'urdf-loader';
import { OrbitControls } from 'three-stdlib';

export default function ArmModel({ jvalue1, jvalue2, jvalue3, jvalue4, jvalue5, jvalue6, jvalue7}) {
    const containerRef = useRef(null);

    useEffect(() => {
        const scene = new THREE.Scene();
        const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
        camera.position.z = 2;

        const renderer = new THREE.WebGLRenderer();
        renderer.setSize(1290, 700);
        containerRef.current.appendChild(renderer.domElement);
        renderer.setClearColor(0xFFACAC);

        const ambientLight = new THREE.AmbientLight(0x404040);
        scene.add(ambientLight);

        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.5);
        directionalLight.position.set(1, 1, 1);
        scene.add(directionalLight);

        const controls = new OrbitControls(camera, renderer.domElement);
        controls.update();

        const manager = new THREE.LoadingManager();
        const loader = new URDFLoader(manager);

        loader.load('arm_urdf/abdi.urdf', robot => {
            console.log("Arm urdf loaded");

            const singleColorMaterial = new THREE.MeshStandardMaterial({
                color: 0xff0000 
            });

            robot.traverse((child) => {
                if (child instanceof THREE.Mesh) {
                    child.visible = true;
                    child.material = singleColorMaterial;
                }
            });
            robot.scale.set(1.5, 1.5, 1.5);
            robot.rotation.z = Math.PI / 1;
            robot.rotation.y = Math.PI / 1;
            robot.rotation.x = Math.PI / 2;

            robot.joints[`arm_j1`].setJointValue(THREE.MathUtils.degToRad(jvalue1));
            robot.joints[`arm_j2`].setJointValue(THREE.MathUtils.degToRad(jvalue2));
            robot.joints[`arm_j3`].setJointValue(THREE.MathUtils.degToRad(jvalue3));
            robot.joints[`arm_j4`].setJointValue(THREE.MathUtils.degToRad(jvalue4));
            robot.joints[`arm_j5`].setJointValue(THREE.MathUtils.degToRad(jvalue5));
            robot.joints[`arm_j6`].setJointValue(THREE.MathUtils.degToRad(jvalue6));
            // robot.joints[`arm_j7`].setJointValue(THREE.MathUtils.degToRad(jvalue7));

            scene.add(robot);
        },
            undefined,
            error => {
                console.error('An error occurred while loading the URDF model:', error.message);
            }
        );

        const animate = () => {
            requestAnimationFrame(animate);
            controls.update();
            renderer.render(scene, camera);
        };
        animate();

        return () => {
            containerRef.current.removeChild(renderer.domElement)
        };

    }, [jvalue1, jvalue2, jvalue3, jvalue4, jvalue5, jvalue6, jvalue7]);
    
    return(
        <div className='w-full h-96' ref={containerRef} />
    )
}
