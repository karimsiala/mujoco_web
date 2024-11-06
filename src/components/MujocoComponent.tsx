import { useEffect, useRef, useState } from "react";
import { getMujocoModule } from "./mujocoLoader";
import { MujocoModule, Simulation } from "../wasm/mujoco_wasm";

// Define the initial scene
const INITIAL_SCENE = "humanoid.xml";

// Virtual Filesystem used by the WASM container.
const VIRTUAL_FILE_SYSTEM = "/working";

export interface MujocoComponentProps {
  sceneUrl: string;
}

export const MujocoComponent = ({ sceneUrl }: MujocoComponentProps) => {
  const mujocoModuleRef = useRef<MujocoModule | null>(null);
  const simulationRef = useRef<Simulation | null>(null);

  const [mujocoModuleLoaded, setMujocoModuleLoaded] = useState(false);
  const [error, setError] = useState<boolean>(false);

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
        setError(true);
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
      } catch (error: unknown) {
        console.error(error as string);
        setError(true);
      }
    };
    if (mujocoModuleLoaded) {
      initializeSimulation();
    }
  }, [mujocoModuleLoaded, sceneUrl]);

  return (
    <div>
      {error
        ? "Unfortunately there was an error. Check the console."
        : "MuJoCo model and simulation initialized successfully."}
    </div>
  );
};
