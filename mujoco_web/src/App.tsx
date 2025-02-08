import { useState } from "react";
import SceneSelector from "./components/SceneSelector";
import Scene from "./components/Scene";

import "./App.css";
import "./index.css";

const App = () => {
  const scenes = [
    "22_humanoids.xml",
    "adhesion.xml",
    "arm26.xml",
    "car.xml",
    "empty.xml",
    "hammock.xml",
    "humanoid.xml",
    "humanoid_body.xml",
    "model_with_tendon.xml",
    "scene.xml",
    "simple.xml",
    "slider_crank.xml",
    "balloons/balloons.xml",
    "agility_cassie/scene.xml",
    "shadow_hand/scene_left.xml"
  ];
  const [selectedScene, setSelectedScene] = useState("humanoid.xml");

  return (
    <div className="relative h-screen">
      {/* 3D Scene component */}
      <Scene selectedScene={selectedScene} />
      {/* Overlay SceneSelector on top of the scene */}
      <div className="absolute bottom-0 left-0 z-10 p-4">
        <SceneSelector
          scenes={scenes}
          selectedScene={selectedScene}
          onSceneChange={setSelectedScene}
        />
      </div>
    </div>
  );
};

export default App;
