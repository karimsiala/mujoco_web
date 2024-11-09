import * as THREE from "three";
import { useEffect, useRef, useState } from "react";
import { getMujocoModule } from "./mujocoLoader";
import { Model, MujocoModule, Simulation } from "../wasm/mujoco_wasm";
import { useFrame, useThree } from "@react-three/fiber";
import { loadScene, updateScene } from "./sceneLoader";
import { Body } from "./mujocoDemo";

// Define the initial scene
const INITIAL_SCENE = "humanoid.xml";

// Virtual Filesystem used by the WASM container.
const VIRTUAL_FILE_SYSTEM = "/working";

export interface MujocoProps {
  sceneUrl: string;
}

export const Mujoco = ({ sceneUrl }: MujocoProps) => {
  const { scene } = useThree();

  const mujocoModuleRef = useRef<MujocoModule | null>(null);
  const mujocoTimeRef = useRef(0);
  const modelRef = useRef<Model | null>(null);
  const simulationRef = useRef<Simulation | null>(null);
  const bodiesRef = useRef<{ [key: number]: Body }>({})
  const lightsRef = useRef<THREE.Light[]>([]);
  const cylindersRef = useRef<THREE.InstancedMesh<THREE.CylinderGeometry>>();
  const spheresRef = useRef<THREE.InstancedMesh<THREE.SphereGeometry>>();

  const [mujocoModuleLoaded, setMujocoModuleLoaded] = useState(false);

  // Load MuJoCo WASM when the component mounts.
  useEffect(() => {
    let isMounted = true;

    const loadModule = async () => {
      try {
        const mujocoModule = await getMujocoModule();
        if (mujocoModule && isMounted) {
          console.log("MuJoCo WASM module loaded successfully.");
          mujocoModuleRef.current = mujocoModule;
          setMujocoModuleLoaded(true);
        } else {
          throw new Error("MuJoCo WASM module failed to load.");
        }
      } catch (error: unknown) {
        console.error(error);
      }
    };
    loadModule();

    return () => {
      isMounted = false;
    };
  }, []);

  // Start the simulation afterr MuJoCo WASM is loaded.
  useEffect(() => {
    const initializeSimulation = async () => {
      try {
        if (!mujocoModuleRef.current) {
          return;
        }

        // Set up Emscripten's Virtual File System.
        mujocoModuleRef.current!.FS.mkdir(VIRTUAL_FILE_SYSTEM);
        mujocoModuleRef.current.FS.mount(mujocoModuleRef.current.MEMFS, { root: "." }, VIRTUAL_FILE_SYSTEM);

        // Fetch and write the initial scene file.
        const sceneResponse = await fetch(sceneUrl);
        if (!sceneResponse.ok) {
          throw new Error(
            `Failed to fetch scene file: ${sceneResponse.status} ${sceneResponse.statusText}`
          );
        }

        const sceneText = await sceneResponse.text();

        mujocoModuleRef.current.FS.writeFile(`${VIRTUAL_FILE_SYSTEM}/${INITIAL_SCENE}`, sceneText);

        // TODO: update the c++ code to handle the situation where
        // the model is not loaded, e.g. with a global error state.

        // const model1 = mujocoModule.Model.load_from_xml(
        //   `${VIRTUAL_FILE_SYSTEM}/${INITIAL_SCENE}`
        // );

        modelRef.current = new mujocoModuleRef.current.Model(`${VIRTUAL_FILE_SYSTEM}/${INITIAL_SCENE}`);
        const state = new mujocoModuleRef.current.State(modelRef.current);
        simulationRef.current = new mujocoModuleRef.current.Simulation(modelRef.current, state);
  
        console.log("MuJoCo model and simulation initialized successfully.");

        const result = loadScene(mujocoModuleRef.current, modelRef.current, scene);
        bodiesRef.current = result.bodies;
        lightsRef.current = result.lights;
        cylindersRef.current = result.cylinders;
        spheresRef.current = result.spheres;

        updateScene(modelRef.current, simulationRef.current, bodiesRef.current, lightsRef.current, cylindersRef.current, spheresRef.current);

      } catch (error: unknown) {
        console.error(error);
      }
    };

    if (mujocoModuleLoaded) {
      initializeSimulation();
    }
  }, [mujocoModuleLoaded, scene, sceneUrl]);

  useFrame(({ clock }) => {
    if (!modelRef.current || !simulationRef.current) {
      return;
    }

    const timeMS = clock.getElapsedTime() * 1000; // Convert time to milliseconds
    const timestep = modelRef.current.getOptions().timestep;

    if (timeMS - mujocoTimeRef.current > 35.0) {
      mujocoTimeRef.current = timeMS; // Update mujoco_time if it’s been over 35 ms
    }
    while (mujocoTimeRef.current < timeMS) {
      simulationRef.current.step();
      mujocoTimeRef.current += timestep * 1000; 
    }

    if (!cylindersRef.current || !spheresRef.current) {
      return;
    }
    updateScene(modelRef.current, simulationRef.current, bodiesRef.current, lightsRef.current, cylindersRef.current, spheresRef.current)
  });


  return null; // This component doesn't render anything directly
};
