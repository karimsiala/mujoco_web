import * as THREE from "three";
import { Body } from "./mujocoUtils";

/**
 * This class is just a container of different variables used to update the
 * Three.js scene.
 */
export class UpdateProps {

    private bodies: { [key: number]: Body } = {};
    private lights: THREE.Light[] = [];
    private cylinders: THREE.InstancedMesh<THREE.CylinderGeometry> | null = null;
    private spheres: THREE.InstancedMesh<THREE.SphereGeometry> | null = null;

    constructor(bodies: { [key: number]: Body }, lights: THREE.Light[], cylinders: THREE.InstancedMesh<THREE.CylinderGeometry>, spheres: THREE.InstancedMesh<THREE.SphereGeometry>) {
        this.bodies = bodies;
        this.lights = lights;
        this.cylinders = cylinders;
        this.spheres = spheres;
    }

    public getBodies(): { [key: number]: Body } {
        return this.bodies;
    }

    public getLights(): THREE.Light[] {
        return this.lights;
    }

    public getCylinders(): THREE.InstancedMesh<THREE.CylinderGeometry> | null {
        return this.cylinders;
    }

    public getSpheres(): THREE.InstancedMesh<THREE.SphereGeometry> | null {
        return this.spheres;
    }

}