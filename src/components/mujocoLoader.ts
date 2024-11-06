import load_mujoco, { MujocoModule } from "../wasm/mujoco_wasm";

let mujocoModulePromise: Promise<MujocoModule> | null = null;

/**
 * Loads the MuJoCo WASM module with caching.
 * Ensures the module is loaded only once.
 */
export const getMujocoModule = (): Promise<MujocoModule> => {
    if (!mujocoModulePromise) {
        mujocoModulePromise = load_mujoco();
    }
    return mujocoModulePromise;
}