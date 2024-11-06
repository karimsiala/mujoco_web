import { useEffect, useState } from "react";
import load_mujoco from '../../public/wasm/mujoco_wasm.js';
import type { MujocoModule } from '../types/mujoco_wasm'; 

// Define the initial scene
const INITIAL_SCENE = "humanoid.xml";

// Virtual Filesystem used by the WASM container.
const VIRTUAL_FILE_SYSTEM = "/working";

export interface MujocoComponentProps {
  sceneUrl: string;
}

export const MujocoComponent = ({ sceneUrl }: MujocoComponentProps) => {
  const [mujocoLoaded, setMujocoLoaded] = useState(false);
  const [mujocoModule, setMujocoModule] = useState<MujocoModule|null>(null);

  // Load MuJoCo WASM when the component mounts.
  useEffect(() => {
    const initialize = async () => {
      try {
        const mujocoModule = await load_mujoco();
        if (!mujocoModule) {
          throw new Error("MuJoCo WASM module failed to load.");
        }

        // Set up Emscripten's Virtual File System.
        mujocoModule.FS.mkdir(VIRTUAL_FILE_SYSTEM);
        mujocoModule.FS.mount(mujocoModule.MEMFS, { root: "." }, VIRTUAL_FILE_SYSTEM);

        // Fetch and write the initial scene file.
        const sceneResponse = await fetch(sceneUrl);
        if (!sceneResponse.ok) {
          throw new Error(
            `Failed to fetch scene file: ${sceneResponse.status} ${sceneResponse.statusText}`
          );
        }

        const sceneText = await sceneResponse.text();

        mujocoModule.FS.writeFile(
          `${VIRTUAL_FILE_SYSTEM}/${INITIAL_SCENE}`,
          sceneText
        );

        // TODO: update the c++ code to handle the situation where
        // the model is not loaded, e.g. with a global error state.

        const model = new mujocoModule.Model(
          `${VIRTUAL_FILE_SYSTEM}/${INITIAL_SCENE}`
        );

        const state = new mujocoModule.State(model);
        // const simulation = new mujocoModule.Simulation(model, state);

        console.log("MuJoCo model and simulation initialized successfully.");

        setMujocoModule(mujocoModule);
        setMujocoLoaded(true);
        
      } catch (error: unknown) {
        console.error(error as string)
      }
    };
    initialize();
  }, [sceneUrl]);

  return <div>Test</div>;
};
