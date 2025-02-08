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
import { SimulationOverlay } from "./SimulationOverlay";

interface SceneProps {
  selectedScene: string;
}

const Scene: React.FC<SceneProps> = ({ selectedScene }) => {
  const [container, setContainer] = useState<any>(null);
  const [selectedBody, setSelectedBody] = useState<BodyInfo | null>(null);
  const [showEditor, setShowEditor] = useState<boolean>(false);
  const [editor, setEditor] = useState<any>(null); // hold the Monaco editor instance

  const handleBodySelect = (bodyInfo: BodyInfo | null) => {
    setSelectedBody(bodyInfo);
  };

  return (
    <div className="relative h-screen w-screen">
      {/* Canvas and 3D Scene */}
      <Canvas
        shadows="soft"
        dpr={window.devicePixelRatio}
        onCreated={(state) => {
          state.scene.background = new THREE.Color(0x264059);
        }}
      >
        <Stats />
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
        <OrbitControls />
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
              bodyID: selectedBody.bodyID,
              name: selectedBody.name,
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

      {/* Simulation controls and debug stats overlay */}
      <SimulationOverlay mujocoContainer={container} editor={editor} />

      {/* Overlayed CodeEditor */}
      {showEditor && (
        <div className="absolute z-60 w-[40%] h-[60%] bottom-20 right-5">
          <CodeEditor 
            mujocoContainer={container}
            selectedBodyId={selectedBody?.bodyID}
            onEditorMount={setEditor}
            onClose={() => setShowEditor(false)}
          />
        </div>
      )}

      {/* Toggle Button */}
      <button 
        onClick={() => setShowEditor(prev => !prev)}
        className="absolute bottom-4 right-4 z-50 px-4 py-2 bg-blue-500 text-white rounded shadow"
      >
        {showEditor ? "Hide Code Editor" : "Show Code Editor"}
      </button>
    </div>
  );
};

export default Scene;
