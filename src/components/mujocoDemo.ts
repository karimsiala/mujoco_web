import { Model, MujocoModule, Simulation, State } from "../wasm/mujoco_wasm";
import * as THREE from "three";

/**
 * Add a body ID to the Mesh object.
 */
export class BodyMesh extends THREE.Mesh {
  bodyID?: number;
}

/**
 * Add a body ID to the Group object and a boolean to indicate if the mesh has
 * been customized.
 */
export class Body extends THREE.Group {
  bodyID?: number;
  has_custom_mesh?: boolean;
}
