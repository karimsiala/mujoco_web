import { useEffect, useRef, useState } from "react";

import * as THREE from "three";

import { useFrame, useThree } from "@react-three/fiber";

import { Model, MujocoModule, Simulation } from "../wasm/mujoco_wasm";
import {
  Body,
  loadMujocoModule,
  loadMujocoScene,
  loadThreeScene,
  updateThreeScene
} from "./mujocoUtils";

export interface MujocoProps {
  sceneUrl: string;
}

export const Mujoco = ({ sceneUrl }: MujocoProps) => {
  const { scene } = useThree();

  const mujocoTimeRef = useRef(0);
  const bodiesRef = useRef<{ [key: number]: Body }>({});
  const lightsRef = useRef<THREE.Light[]>([]);
  const cylindersRef = useRef<THREE.InstancedMesh<THREE.CylinderGeometry>>();
  const spheresRef = useRef<THREE.InstancedMesh<THREE.SphereGeometry>>();
  const tmpVecRef = useRef<THREE.Vector3>(new THREE.Vector3(0, 0, 0));

  const [mujocoModule, setMujocoModule] = useState<MujocoModule | null>(null);
  const [simulation, setSimulation] = useState<Simulation | null>(null);
  const [model, setModel] = useState<Model | null>(null);

  // Load MuJoCo WASM when the component mounts.
  useEffect(() => {
    const setupMujocoModule = async () => {
      try {
        const mujocoModule = await loadMujocoModule();
        if (mujocoModule) {
          setMujocoModule(mujocoModule);
        }
      } catch (error: unknown) {
        console.error(error);
      }
    };
    if (!mujocoModule) {
      setupMujocoModule();
    }
  }, [mujocoModule]);

  // Start the simulation after MuJoCo WASM is loaded.
  useEffect(() => {
    const setupMujocoScene = async () => {
      try {
        if (mujocoModule) {
          const { model: model, simulation: simulation } = await loadMujocoScene(
            mujocoModule,
            sceneUrl
          );
          if (model) {
            setModel(model);
          }
          if (simulation) {
            setSimulation(simulation);
          }
        }
      } catch (error: unknown) {
        console.error(error);
      }
    };
    if (mujocoModule) {
      setupMujocoScene();
    }
  }, [mujocoModule, sceneUrl]);

  useEffect(() => {
    const setupThreeScene = async () => {
      try {
        if (model) {
          const result = await loadThreeScene(model, scene);
          bodiesRef.current = result.bodies;
          lightsRef.current = result.lights;
          cylindersRef.current = result.cylinders;
          spheresRef.current = result.spheres;
        }
      } catch (error: unknown) {
        console.error(error);
      }
    };
    if (model) {
      setupThreeScene();
    }
  }, [model, simulation, scene]);

  useFrame(({ clock }) => {
    if (!model || !simulation) {
      return;
    }

    const timeMS = clock.getElapsedTime() * 1000; // Convert time to milliseconds
    const timestep = model.getOptions().timestep;

    if (timeMS - mujocoTimeRef.current > 35.0) {
      mujocoTimeRef.current = timeMS; // Update mujoco_time if it’s been over 35 ms
    }
    while (mujocoTimeRef.current < timeMS) {
      simulation.step();
      mujocoTimeRef.current += timestep * 1000;
    }

    if (!cylindersRef.current || !spheresRef.current) {
      return;
    }
    updateThreeScene(
      model,
      simulation,
      bodiesRef.current,
      lightsRef.current,
      cylindersRef.current,
      spheresRef.current,
      tmpVecRef.current
    );
  });

  return null; // This component doesn't render anything directly
};
