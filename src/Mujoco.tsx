import { useEffect, useState } from "react";
import load_mujoco from "./wasm/mujoco_wasm";

// Define the initial scene
const INITIAL_SCENE = "humanoid.xml";
const BASE_URL = `${window.location.origin}${import.meta.env.BASE_URL}`;

const VIRTUAL_FILE_SYSTEM = "/working";

export interface MujocoProps {
  sceneUrl: string;
}

export const Mujoco = ({ sceneUrl }: MujocoProps) => {
  const [mujocoLoaded, setMujocoLoaded] = useState(false);
  const [error, setError] = useState<string>("");

  // Load MuJoCo WASM when the component mounts.
  useEffect(() => {
    const initialize = async () => {
      try {
        const mujoco = await load_mujoco();
        if (!mujoco) {
          throw new Error("MuJoCo WASM module failed to load.");
        }

        // Set up Emscripten's Virtual File System.
        mujoco.FS.mkdir(VIRTUAL_FILE_SYSTEM);
        mujoco.FS.mount(mujoco.MEMFS, { root: "." }, VIRTUAL_FILE_SYSTEM);

        // Fetch and write the initial scene file.
        const sceneResponse = await fetch(sceneUrl);
        if (!sceneResponse.ok) {
          throw new Error(
            `Failed to fetch scene file: ${sceneResponse.status} ${sceneResponse.statusText}`
          );
        }

        const sceneText = await sceneResponse.text();
        mujoco.FS.writeFile(
          `${VIRTUAL_FILE_SYSTEM}/${INITIAL_SCENE}`,
          sceneText
        );

        // TODO: update the c++ code to handle the situation where
        // the model is not loaded, e.g. with a global error state.

        const model = new mujoco.Model(
          `${VIRTUAL_FILE_SYSTEM}/${INITIAL_SCENE}`
        );
        const state = new mujoco.State(model);
        const simulation = new mujoco.Simulation(model, state);

        console.log("MuJoCo model and simulation initialized successfully.");
        setMujocoLoaded(true);
      } catch (error: unknown) {
        if (error instanceof Error) {
          const errorMessage = error.message;
          console.error(errorMessage);
          setError(errorMessage);
        } else {
          const errorMessage = "An unexpected error occurred.";
          console.error(errorMessage);
          setError(errorMessage);
        }
      }
    };
    initialize();
  }, [sceneUrl]);

  return <div>{error}</div>;
};
