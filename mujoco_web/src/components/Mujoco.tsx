import { memo, useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { useFrame, useThree } from "@react-three/fiber";
import { Html } from "@react-three/drei";

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
	// Use state for module loading and errors.
	const [moduleLoading, setModuleLoading] = useState(true);
	const [moduleError, setModuleError] = useState(false);
	const [sceneLoading, setSceneLoading] = useState(false);
	const [sceneError, setSceneError] = useState(false);

	const { scene, gl, camera } = useThree();

	// Variables used to update the ThreeJS scene.
	const mujocoTimeRef = useRef(0);
	const updatePropsRef = useRef<UpdateProps>();
	const tmpVecRef = useRef<THREE.Vector3>(new THREE.Vector3(0, 0, 0));

	// The container of the MuJoCo module, model, state and simulation.
	const [mujocoContainer, setMujocoContainer] = useState<MujocoContainer | null>(null);

	// Combined module loading effect.
	useEffect(() => {
		const setupMujocoModule = async () => {
			try {
				const container = await loadMujocoModule();
				if (container) {
					setMujocoContainer(container);
					onContainerReady?.(container);
				}
			} catch (error: unknown) {
				setModuleError(true);
				console.error(error);
			} finally {
				setModuleLoading(false);
			}
		};
		setupMujocoModule();
	}, [onContainerReady]);

	// Load a scene each time the scene URL changes.
	useEffect(() => {
		const setupMujocoScene = async () => {
			try {
				if (mujocoContainer) {
					// Time-consuming operation that should be moved into a Worker.
					await loadMujocoScene(mujocoContainer, sceneUrl);
					updatePropsRef.current = await buildThreeScene(mujocoContainer, scene);
				}
			} catch (error: unknown) {
				setSceneError(true);
				console.error(error);
			} finally {
				setSceneLoading(false);
			}
		};
		if (mujocoContainer) {
			setSceneLoading(true);
			setupMujocoScene();
		}
	}, [mujocoContainer, scene, sceneUrl]);

	// Create a function to apply forces to bodies and expose it globally
	useEffect(() => {
		if (mujocoContainer) {
			(window as any).applyForceToBody = (
				bodyId: number,
				force: [number, number, number],
				point: [number, number, number]
			) => {
				const simulation = mujocoContainer.getSimulation();
				const model = simulation.model();
				if (!model) return;

				// Scale force by timestep to make it frame-rate independent
				const timestep = model.getOptions().timestep;
				const scaledForce = force.map(f => f * timestep) as [number, number, number];

				simulation.applyForce(
					scaledForce[0],
					scaledForce[1],
					scaledForce[2], // force vector
					0, 0, 0,      // torque vector (zero for now)
					point[0],
					point[1],
					point[2],     // application point
					bodyId         // body ID
				);
			};
		}
		return () => {
			delete (window as any).applyForceToBody;
		};
	}, [mujocoContainer]);

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
					obj.object.parent && "bodyID" in obj.object.parent
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
		canvas.addEventListener("pointerdown", handlePointerDown);
		return () => {
			canvas.removeEventListener("pointerdown", handlePointerDown);
		};
	}, [mujocoContainer, scene, onBodySelect, camera, gl]);

	// Update the Three.js scene with information from the MuJoCo simulation.
	useFrame(({ clock }) => {
		if (!mujocoContainer || sceneLoading || moduleError || sceneError) {
			return;
		}

		const simulation = mujocoContainer.getSimulation();
		const model = simulation.model();
		if (!model || !simulation) {
			return;
		}

		const timeMS = clock.getElapsedTime() * 1000;
		const timestep = model.getOptions().timestep;

		if (timeMS - mujocoTimeRef.current > 35.0) {
			mujocoTimeRef.current = timeMS;
		}
		while (mujocoTimeRef.current < timeMS) {
			simulation.step();
			mujocoTimeRef.current += timestep * 1000;
		}

		if (!updatePropsRef.current) {
			return;
		}
		updateThreeScene(mujocoContainer, updatePropsRef.current, tmpVecRef.current);
	});

	// Conditional UI overlay for error or loading.
	if (moduleError || sceneError) {
		return (
			<Html center>
				<div style={{ color: "red" }}>Error loading simulation module.</div>
			</Html>
		);
	}
	if (moduleLoading || sceneLoading) {
		return (
			<Html center>
				<div style={{ color: "white" }}>Loading simulation...</div>
			</Html>
		);
	}

	return null;
};

// Memoize the named component
export const Mujoco = memo(MujocoComponent);
