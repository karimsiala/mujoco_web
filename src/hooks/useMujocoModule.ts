// useMujocoModule.ts
import { useState, useEffect } from "react";
import load_mujoco, { MujocoModule, Model, State, Simulation } from "../wasm/mujoco_wasm";

// Virtual Filesystem used by the WASM container.
const VIRTUAL_FILE_SYSTEM = "/working";
const DEFAULT_SCENE_NAME = "scene.xml";

let mujocoModulePromise: Promise<MujocoModule> | null = null;

/**
 * Custom hook to load the MuJoCo WASM module with caching.
 * Initializes the model, state, and simulation.
 */
export const useMujocoModule = (sceneUrl: string) => {
    const [mujocoModule, setMujocoModule] = useState<MujocoModule | null>(null);
    const [model, setModel] = useState<Model | null>(null);
    const [state, setState] = useState<State | null>(null);
    const [simulation, setSimulation] = useState<Simulation | null>(null);
    const [error, setError] = useState<Error | null>(null);

    useEffect(() => {
        let isMounted = true; // To handle component unmounting
        let previousModel: Model | null = null;
        let previousState: State | null = null;
        let previousSimulation: Simulation | null = null;

        const loadModuleAndInitialize = async () => {
            try {
                // Load the MuJoCo module if not already loaded
                if (!mujocoModulePromise) {
                    mujocoModulePromise = load_mujoco();
                }
                const module = await mujocoModulePromise;

                if (!isMounted) return;

                setMujocoModule(module);

                // Set up Emscripten's Virtual File System.
                if (!module.FS.analyzePath(VIRTUAL_FILE_SYSTEM).exists) {
                    module.FS.mkdir(VIRTUAL_FILE_SYSTEM);
                    module.FS.mount(module.MEMFS, { root: "." }, VIRTUAL_FILE_SYSTEM);
                }

                // Fetch and write the scene file.
                const sceneResponse = await fetch(sceneUrl);
                if (!sceneResponse.ok) {
                    throw new Error(
                        `Failed to fetch scene file: ${sceneResponse.status} ${sceneResponse.statusText}`
                    );
                }

                const sceneText = await sceneResponse.text();

                const scenePath = `${VIRTUAL_FILE_SYSTEM}/${DEFAULT_SCENE_NAME}`;
                module.FS.writeFile(scenePath, sceneText);

                // Initialize the model
                const modelInstance = module.Model.load_from_xml(sceneText);
                if (!isMounted) return;
                setModel(modelInstance);
                previousModel = modelInstance; // For cleanup

                // Initialize the state
                const stateInstance = new module.State(modelInstance);
                if (!isMounted) return;
                setState(stateInstance);
                previousState = stateInstance;

                // Initialize the simulation
                const simulationInstance = new module.Simulation(modelInstance, stateInstance);
                if (!isMounted) return;
                setSimulation(simulationInstance);
                previousSimulation = simulationInstance;
            } catch (err) {
                console.error("Error initializing MuJoCo:", err);
                if (isMounted) {
                    setError(err as Error);
                }
            }
        };

        loadModuleAndInitialize();

        return () => {
            isMounted = false;
            previousSimulation?.free();
            previousState?.free();
            previousModel?.free();
        };
    }, [sceneUrl]);

    return { mujocoModule, model, state, simulation, error };
};
