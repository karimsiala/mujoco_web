import React from "react";
import { Canvas } from "@react-three/fiber";
import {
  Environment,
  GizmoHelper,
  GizmoViewport,
  Grid,
  OrbitControls,
  PerspectiveCamera,
  StatsGl
} from "@react-three/drei";
import * as THREE from "three";
import { Mujoco } from "./Mujoco";
import { DepthOfField, EffectComposer } from "@react-three/postprocessing";

interface SceneProps {
  selectedScene: string;
}

const Scene: React.FC<SceneProps> = ({ selectedScene }) => {
  return (
    <Canvas
      shadows="soft"
      dpr={window.devicePixelRatio}
      onCreated={(state) => {
        state.scene.background = new THREE.Color(0x264059);
      }}
    >
      <StatsGl className="absolute top-4 right-4" />

      <GizmoHelper>
        <GizmoViewport />
      </GizmoHelper>
      <ambientLight color={0xffffff} intensity={0.1} />
      <Environment preset="night" ground={{ height: 10, radius: 40 }} />
      <mesh position={[5, 0, 0]} receiveShadow>
        <sphereGeometry args={[0.5, 32]} />
        <meshMatcapMaterial color="blue" />
      </mesh>
      <spotLight
        position={[0, 2, 2]}
        angle={0.15}
        penumbra={1}
        decay={1}
        intensity={3.14}
      />
      <PerspectiveCamera makeDefault position={[2.0, 1.7, 1.7]} fov={75} />
      <OrbitControls makeDefault />
      {/* Passing the scene URL to the Mujoco component */}
      <Mujoco sceneUrl={selectedScene} key={selectedScene} />
      <EffectComposer>
        <DepthOfField focusDistance={0} focalLength={0.02} bokehScale={2} height={480} />
      </EffectComposer>
    </Canvas>
  );
};

export default Scene;
