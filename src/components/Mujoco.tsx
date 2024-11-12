import { memo, useEffect, useRef, useState } from "react";

import * as THREE from "three";

import { useFrame, useThree } from "@react-three/fiber";

import { Body, loadMujocoModule, loadScene, updateThreeScene } from "./mujocoUtils";
import { MujocoContainer } from "./MujocoContainer";

export interface MujocoProps {
  sceneUrl: string;
}

export const MujocoComponent = ({ sceneUrl }: MujocoProps) => {
  // The 35ms threshold acts as a safeguard to prevent the simulation from
  // accumulating too much lag, which could degrade performance or accuracy.
  const MAX_SIMULATION_LAG_MS = 35.0;

  const { scene } = useThree();

  // This is to block the scene rendering until the scene has been loaded.
  const loadingScene = useRef<boolean>(false);

  const mujocoTimeRef = useRef(0);
  const bodiesRef = useRef<{ [key: number]: Body }>({});
  const lightsRef = useRef<THREE.Light[]>([]);
  const cylindersRef = useRef<THREE.InstancedMesh<THREE.CylinderGeometry>>();
  const spheresRef = useRef<THREE.InstancedMesh<THREE.SphereGeometry>>();
  const tmpVecRef = useRef<THREE.Vector3>(new THREE.Vector3(0, 0, 0));

  const [mujocoContainer, setMujocoContainer] = useState<MujocoContainer | null>(null);

  // Load MuJoCo WASM with a default empty scene when the component mounts.
  useEffect(() => {
    const setupMujocoModule = async () => {
      try {
        const mujocoContainer = await loadMujocoModule();
        if (mujocoContainer) {
          setMujocoContainer(mujocoContainer);
        }
      } catch (error: unknown) {
        console.error(error);
      }
    };
    setupMujocoModule();
  }, []);

  // Load a scene each time the scene URL changes.
  useEffect(() => {
    const setupMujocoScene = async () => {
      try {
        if (mujocoContainer) {
          const result = await loadScene(mujocoContainer, sceneUrl, scene);
          bodiesRef.current = result.bodies;
          lightsRef.current = result.lights;
          cylindersRef.current = result.cylinders;
          spheresRef.current = result.spheres;
        }
      } catch (error: unknown) {
        console.error(error);
      }
    };
    if (mujocoContainer) {
      try {
        loadingScene.current = true;
        setupMujocoScene();
      } finally {
        loadingScene.current = false;
      }
    }
  }, [mujocoContainer, scene, sceneUrl]);

  // Update the Three.js scene with information from the MuJoCo simulation.
  useFrame(({ clock }) => {
    if (!mujocoContainer || loadingScene.current) {
      return;
    }
    const simulation = mujocoContainer.getSimulation();
    const model = simulation.model();
    if (!model || !simulation) {
      return;
    }

    const timeMS = clock.getElapsedTime() * 1000;
    const timestep = model.getOptions().timestep;

    // If the real elapsed time (timeMS) has advanced more than 35 milliseconds
    // beyond the simulation's current time, the simulation time is reset to
    // match the real time. This prevents the simulation from falling too far
    // behind real time, which could happen if the rendering or simulation steps lag.
    if (timeMS - mujocoTimeRef.current > MAX_SIMULATION_LAG_MS) {
      mujocoTimeRef.current = timeMS;
    }
    // This while loop ensures that the simulation progresses in fixed timesteps
    // until it catches up with the real elapsed time.
    while (mujocoTimeRef.current < timeMS) {
      simulation.step();
      mujocoTimeRef.current += timestep * 1000;
    }

    // Update the three.js scene with the current state of the simulation.
    if (!cylindersRef.current || !spheresRef.current) {
      return;
    }
    updateThreeScene(
      mujocoContainer,
      bodiesRef.current,
      lightsRef.current,
      cylindersRef.current,
      spheresRef.current,
      tmpVecRef.current
    );
  });

  return null; // This component doesn't render anything directly.
};

// Memoize the named component
export const Mujoco = memo(MujocoComponent);
