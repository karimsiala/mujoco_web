import { useEffect, useRef, useState } from "react";
import { getMujocoModule } from "./mujocoLoader";
import { MujocoModule, Simulation } from "../wasm/mujoco_wasm";
import { useFrame, useThree } from "@react-three/fiber";
import { loadScene } from "./sceneLoader";

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
  const simulationRef = useRef<Simulation | null>(null);

  const [mujocoModuleLoaded, setMujocoModuleLoaded] = useState(false);

  // Load MuJoCo WASM when the component mounts.
  useEffect(() => {
    const loadModule = async () => {
      try {
        const mujocoModule = await getMujocoModule();
        if (mujocoModule) {
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
  }, []);

  // Start the simulation afterr MuJoCo WASM is loaded.
  useEffect(() => {
    const initializeSimulation = async () => {
      try {
        const mujocoModule = mujocoModuleRef.current;
        if (!mujocoModule) {
          return;
        }

        // Set up Emscripten's Virtual File System.
        mujocoModule!.FS.mkdir(VIRTUAL_FILE_SYSTEM);
        mujocoModule.FS.mount(mujocoModule.MEMFS, { root: "." }, VIRTUAL_FILE_SYSTEM);

        // Fetch and write the initial scene file.
        const sceneResponse = await fetch(sceneUrl);
        if (!sceneResponse.ok) {
          throw new Error(
            `Failed to fetch scene file: ${sceneResponse.status} ${sceneResponse.statusText}`
          );
        }

        const sceneText = await sceneResponse.text();

        mujocoModule.FS.writeFile(`${VIRTUAL_FILE_SYSTEM}/${INITIAL_SCENE}`, sceneText);

        // TODO: update the c++ code to handle the situation where
        // the model is not loaded, e.g. with a global error state.

        // const model1 = mujocoModule.Model.load_from_xml(
        //   `${VIRTUAL_FILE_SYSTEM}/${INITIAL_SCENE}`
        // );

        const model = new mujocoModule.Model(`${VIRTUAL_FILE_SYSTEM}/${INITIAL_SCENE}`);
        const state = new mujocoModule.State(model);
        const simulation = new mujocoModule.Simulation(model, state);

        simulationRef.current = simulation;

        console.log("MuJoCo model and simulation initialized successfully.");

        loadScene(mujocoModule, model, scene);
      } catch (error: unknown) {
        console.error(error);
      }
    };
    if (mujocoModuleLoaded) {
      initializeSimulation();
    }
  }, [mujocoModuleLoaded, scene, sceneUrl]);

  // To keep the R3F scene in sync with the MuJoCo simulation, use the useFrame
  // hook to update positions, rotations, and other properties on each renderframe.
  useFrame(() => {
    if (mujocoModuleRef.current) {
      //   mujoCoRef.current.stepSimulation(); // Advance the simulation
      //   // Update geometries based on simulation
      //   const updatedGeometries = mujoCoRef.current.getUpdatedGeometries();
      //   updatedGeometries.forEach((geom: any, index: number) => {
      //     const mesh = mujocoSceneRef.current.children[index] as THREE.Mesh;
      //     mesh.position.set(geom.x, geom.y, geom.z);
      //     mesh.rotation.set(geom.rotX, geom.rotY, geom.rotZ);
      //   });
    }
  });

  return null; // This component doesn't render anything directly
};
