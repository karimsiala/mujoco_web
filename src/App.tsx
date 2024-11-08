import { Canvas } from "@react-three/fiber";
import { Mujoco } from "./components/Mujoco";
import { OrbitControls, PerspectiveCamera } from "@react-three/drei";
import { Color, Fog } from "three";

import "./App.css";
import "./index.css";

// Define the initial scene
const INITIAL_SCENE = "humanoid.xml";
const BASE_URL = `${window.location.origin}${import.meta.env.BASE_URL}`;

const App = () => {
  return (
    <div className="w-full h-full border-4 border-blue-500">
      <Canvas
        shadows
        // only re-render when props changed or when requested.
        style={{
          backgroundColor: "#192635",
          borderRadius: "inherit",
          margin: "0 auto" // Center horizontally.
        }}
      >
        {/* <AdaptiveDpr /> */}
        <ambientLight color={0xffffff} intensity={0.1} />
        <spotLight
          position={[0, 2, 2]}
          angle={0.15}
          penumbra={1}
          decay={1}
          intensity={3.14}
        />
        <PerspectiveCamera makeDefault position={[2.0, 1.7, 1.7]} fov={45} />
        <OrbitControls makeDefault />
        <Mujoco sceneUrl={`${BASE_URL}/examples/scenes/${INITIAL_SCENE}`} />
      </Canvas>
    </div>
  );
};
export default App;
