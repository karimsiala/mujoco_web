import { MujocoModule, Simulation } from "../wasm/mujoco_wasm";

/**
 * A container for the MuJoCo WASM module and simulation.
 */
export class MujocoContainer {
    private mujocoModule: MujocoModule;
    private simulation: Simulation;
    constructor(mujocoModule: MujocoModule, simulation: Simulation) {
        this.mujocoModule = mujocoModule;
        this.simulation = simulation;
    }
    public getMujocoModule(): MujocoModule {
        return this.mujocoModule;
    }
    public setMujocoModule(mujocoModule: MujocoModule): void {
        this.mujocoModule = mujocoModule;
    }
    public getSimulation(): Simulation {
        return this.simulation;
    }
    public setSimulation(simulation: Simulation): void {
        this.simulation = simulation;
    }
}