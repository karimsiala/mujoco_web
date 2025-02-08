import { memo, useEffect, useRef, useState } from "react";

import * as THREE from "three";

import { useFrame, useThree } from "@react-three/fiber";

import { UpdateProps } from "./UpdateProps";
import { MujocoContainer } from "./MujocoContainer";
import { BodyInfo } from "../types/BodyInfo";
import {
  loadMujocoModule,
  buildThreeScene,
  updateThreeScene,
  loadMujocoScene
} from "./mujocoUtils";

export interface MujocoProps {
  sceneUrl: string;
  onContainerReady?: (container: MujocoContainer) => void;
  onBodySelect?: (bodyInfo: BodyInfo | null) => void;
}

export const MujocoComponent = ({ sceneUrl, onContainerReady, onBodySelect }: MujocoProps) => {
  // The 35ms threshold acts as a safeguard to prevent the simulation from
  // accumulating too much lag, which could degrade performance or accuracy.
  const MAX_SIMULATION_LAG_MS = 35.0;

  const { scene, gl, camera } = useThree();

  // This is to block the scene rendering until the scene has been loaded.
  const loadingSceneRef = useRef<boolean>(false);

  // True if the scene fails to load.
  const errorRef = useRef<boolean>(false);

  // Variables used to update the ThreeJS scene.
  const mujocoTimeRef = useRef(0);
  const updatePropsRef = useRef<UpdateProps>();
  const tmpVecRef = useRef<THREE.Vector3>(new THREE.Vector3(0, 0, 0));

  // The container of the MuJoCo module, model, state and simulation.
  const [mujocoContainer, setMujocoContainer] = useState<MujocoContainer | null>(null);

  // Load MuJoCo WASM with a default empty scene when the component mounts.
  useEffect(() => {
    const setupMujocoModule = async () => {
      try {
        const mujocoContainer = await loadMujocoModule();
        if (mujocoContainer) {
          setMujocoContainer(mujocoContainer);
        }
      } catch (error: unknown) {
        errorRef.current = true;
        console.error(error);
      }
    };
    setupMujocoModule();
  }, []);

  // Load a scene each time the scene URL changes.
  useEffect(() => {
    const setupMujocoScene = async () => {
      try {
        if (mujocoContainer) {
          // Time-consuming operation that should be moved into a Worker.
          loadMujocoScene(mujocoContainer, sceneUrl);

          updatePropsRef.current = await buildThreeScene(mujocoContainer, scene);
        }
      } catch (error: unknown) {
        errorRef.current = true;
        console.error(error);
      }
    };
    if (mujocoContainer) {
      try {
        loadingSceneRef.current = true;
        setupMujocoScene();
      } finally {
        loadingSceneRef.current = false;
      }
    }
  }, [mujocoContainer, scene, sceneUrl]);

  // Create a function to apply forces to bodies and expose it globally
  useEffect(() => {
    if (mujocoContainer) {
      (window as any).applyForceToBody = (bodyId: number, force: [number, number, number], point: [number, number, number]) => {
        const simulation = mujocoContainer.getSimulation();
        const model = simulation.model();
        if (!model) return;

        // Scale force by timestep to make it frame-rate independent
        const timestep = model.getOptions().timestep;
        const scaledForce = force.map(f => f * timestep) as [number, number, number];
        
        simulation.applyForce(
          scaledForce[0], scaledForce[1], scaledForce[2],  // force vector
          0, 0, 0,                                          // torque vector (zero for now)
          point[0], point[1], point[2],                     // application point
          bodyId                                            // body ID
        );
      };
    }
    return () => {
      delete (window as any).applyForceToBody;
    };
  }, [mujocoContainer]);

  useEffect(() => {
    const setupMujocoModule = async () => {
      try {
        const mujocoContainer = await loadMujocoModule();
        if (mujocoContainer) {
          setMujocoContainer(mujocoContainer);
          onContainerReady?.(mujocoContainer);
        }
      } catch (error: unknown) {
        errorRef.current = true;
        console.error(error);
      }
    };
    setupMujocoModule();
  }, [onContainerReady]);

  // Handle pointer events for body selection
  useEffect(() => {
    const handlePointerDown = (event: MouseEvent) => {
      if (!mujocoContainer) return;
      
      const simulation = mujocoContainer.getSimulation();
      const rect = (event.target as HTMLElement).getBoundingClientRect();
      const x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
      const y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
      
      const raycaster = new THREE.Raycaster();
      const pointer = new THREE.Vector2(x, y);
      
      raycaster.setFromCamera(pointer, camera as THREE.Camera);
      
      const intersects = raycaster.intersectObjects(scene.children, true);
      
      if (intersects.length > 0) {
        const bodyObject = intersects.find(obj => 
          obj.object.parent && 'bodyID' in obj.object.parent
        )?.object.parent;
        
        if (bodyObject) {
          const position = bodyObject.position.toArray() as [number, number, number];
          const rotation = bodyObject.rotation.toArray() as [number, number, number];
          onBodySelect?.({
            bodyID: bodyObject.bodyID,
            name: bodyObject.name,
            position,
            rotation,
            velocity: simulation.getBodyVelocity?.(bodyObject.bodyID) || [0, 0, 0]
          });
          return;
        }
      }
      
      onBodySelect?.(null);
    };

    const canvas = gl.domElement;
    canvas.addEventListener('pointerdown', handlePointerDown);
    
    return () => {
      canvas.removeEventListener('pointerdown', handlePointerDown);
    };
  }, [mujocoContainer, scene, onBodySelect, camera, gl]);

  // Update the Three.js scene with information from the MuJoCo simulation.
  useFrame(({ clock }) => {
    if (!mujocoContainer || loadingSceneRef.current || errorRef.current) {
      return;
    }

    const simulation = mujocoContainer.getSimulation();
    const model = simulation.model();
    if (!model || !simulation) {
      return;
    }

    const timeMS = clock.getElapsedTime() * 1000;
    const timestep = model.getOptions().timestep;

    // If the real elapsed time (timeMS) has advanced more than 35 milliseconds
    // beyond the simulation's current time, the simulation time is reset to
    // match the real time. This prevents the simulation from falling too far
    // behind real time, which could happen if the rendering or simulation steps lag.
    if (timeMS - mujocoTimeRef.current > MAX_SIMULATION_LAG_MS) {
      mujocoTimeRef.current = timeMS;
    }
    // This while loop ensures that the simulation progresses in fixed timesteps
    // until it catches up with the real elapsed time.
    while (mujocoTimeRef.current < timeMS) {
      simulation.step();
      mujocoTimeRef.current += timestep * 1000;
    }

    if (!updatePropsRef.current) {
      return;
    }
    updateThreeScene(mujocoContainer, updatePropsRef.current, tmpVecRef.current);
  });

  return null; // This component doesn't render anything directly.
};

// Memoize the named component
export const Mujoco = memo(MujocoComponent);
