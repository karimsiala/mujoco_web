// Standard types from https://github.com/DefinitelyTyped/DefinitelyTyped
import { EmscriptenModule } from "emscripten";

export interface Model {
  load_from_xml(str: string): Model;
  /** Free the memory associated with the model */
  free(): void;
  /** Retrive various parameters of the current simulation */
  getOptions(): unknown;
}

export interface State {
  /** Free the memory associated with the state */
  free(): void;
}

export interface Simulation {
  state(): State;
  model(): Model;
  /** Free the memory associated with both the model and the state in the simulation */
  free(): void;
  /** Apply cartesian force and torque (outside xfrc_applied mechanism) */
  applyForce(fx: number, fy: number, fz: number, tx: number, ty: number, tz: number, px: number, py: number, pz: number, body_id: number): void;

  /** sets perturb pos,quat in d->mocap when selected body is mocap, and in d->qpos otherwise
   * d->qpos written only if flg_paused and subtree root for selected body has free joint */
  applyPose(bodyID: number,
    refPosX: number, refPosY: number, refPosZ: number,
    refQuat1: number, refQuat2: number, refQuat3: number, refQuat4: number,
    flg_paused: number): void;
}

export interface MujocoModule extends EmscriptenModule {
  Model: Model;
  State: State;
  Simulation: Simulation;
}

declare module "../../public/wasm/mujoco_wasm.js" {
  const load_mujoco: EmscriptenModuleFactory<MujocoModule>;
  export default load_mujoco;
}

