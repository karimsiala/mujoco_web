import React, { useState } from "react";
import { Canvas } from "@react-three/fiber";
import {
  Environment,
  GizmoHelper,
  GizmoViewport,
  Grid,
  OrbitControls,
  PerspectiveCamera,
  Stats,
  StatsGl
} from "@react-three/drei";
import * as THREE from "three";
import { Mujoco } from "./Mujoco";
import { DepthOfField, EffectComposer } from "@react-three/postprocessing";
import { DragInteraction } from "./DragInteraction";
import { CodeEditor } from "./CodeEditor";
import { DebugOverlay } from "./DebugOverlay";
import { BodyInfo } from "../types/BodyInfo";

interface SceneProps {
  selectedScene: string;
}

const Scene: React.FC<SceneProps> = ({ selectedScene }) => {
  const [container, setContainer] = React.useState<any>(null);
  const [selectedBody, setSelectedBody] = useState<BodyInfo | null>(null);

  const handleBodySelect = (bodyInfo: BodyInfo | null) => {
    setSelectedBody(bodyInfo);
  };

  return (
    <div className="flex h-screen">
      <div className="w-1/2 h-full">
        <Canvas
          shadows="soft"
          dpr={window.devicePixelRatio}
          onCreated={(state) => {
            state.scene.background = new THREE.Color(0x264059);
          }}
        >
          <Stats />
          <GizmoHelper>
            <GizmoViewport />
          </GizmoHelper>
          <ambientLight color={0xffffff} intensity={0.1} />
          
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
          <OrbitControls/>
          <Mujoco 
            sceneUrl={selectedScene} 
            key={selectedScene} 
            onContainerReady={setContainer}
            onBodySelect={handleBodySelect}
          />
          <DragInteraction />
          {selectedBody && (
            <DebugOverlay
              info={{
                bodyID: selectedBody.bodyID, // changed from bodyId
                name: selectedBody.name,       // changed from bodyName
                position: selectedBody.position,
                rotation: selectedBody.rotation,
                velocity: selectedBody.velocity
              }}
            />
          )}
          <EffectComposer>
            <DepthOfField focusDistance={0} focalLength={0.02} bokehScale={2} height={480} />
          </EffectComposer>
        </Canvas>
      </div>
      <div className="w-1/2 h-full">
        <CodeEditor 
          mujocoContainer={container}
          selectedBodyId={selectedBody?.id}
        />
      </div>
    </div>
  );
};

export default Scene;
