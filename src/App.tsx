import { Canvas } from "@react-three/fiber";
import { Mujoco } from "./components/Mujoco";
import { AdaptiveDpr, OrbitControls } from "@react-three/drei";

import "./App.css";
import "./index.css";

// Define the initial scene
const INITIAL_SCENE = "humanoid.xml";
const BASE_URL = `${window.location.origin}${import.meta.env.BASE_URL}`;

const App = () => {
  return (
    <div className="w-screen h-screen border-4 border-blue-500">
      <Canvas
        className="bg-[#192635] rounded-none w-full h-full"
        shadows
        // only re-render when props changed or when requested.
        style={{
          backgroundColor: "#192635",
          borderRadius: "inherit",
          margin: "0 auto" // Center horizontally.
        }}
      >
        <AdaptiveDpr />
        <ambientLight intensity={1.0} />
        <spotLight
          position={[0, 2, 2]}
          angle={0.15}
          penumbra={1}
          decay={0}
          intensity={3.14}
        />
        <Mujoco sceneUrl={`${BASE_URL}/examples/scenes/${INITIAL_SCENE}`} />
        <OrbitControls />
      </Canvas>
    </div>
  );
};
export default App;
