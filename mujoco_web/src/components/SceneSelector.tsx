import React from "react";

interface SceneSelectorProps {
  scenes: string[];
  selectedScene: string;
  onSceneChange: (scene: string) => void;
}

const SceneSelector: React.FC<SceneSelectorProps> = ({
  scenes,
  selectedScene,
  onSceneChange
}) => {
  return (
    <div className="bg-white bg-opacity-75 p-2 items-center bottom- 10">
      <label htmlFor="scene-select">Select Scene: </label>
      <select
        id="scene-select"
        value={selectedScene}
        onChange={(e) => onSceneChange(e.target.value)}
      >
        {scenes.map((scene) => (
          <option key={scene} value={scene}>
            {scene}
          </option>
        ))}
      </select>
    </div>
  );
};

export default SceneSelector;
