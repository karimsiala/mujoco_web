import "./App.css";
import { Mujoco } from "./Mujoco";

// Define the initial scene
const INITIAL_SCENE = "humanoid.xml";
const BASE_URL = `${window.location.origin}${import.meta.env.BASE_URL}`;

// const AddBodies = () => {
//   const mujocoRef = useRef<mujoco | null>(null);

//   const demoRef = useRef<MuJoCoDemo>({
//     model: null,
//     state: null,
//     simulation: null,
//     scene: null,
//   });

//   const { scene } = useThree();

//   useEffect(() => {
//     const initialize = async () => {
//       // Load the MuJoCo Module
//       mujocoRef.current = await load_mujoco();

//       // Set up Emscripten's Virtual File System.
//       mujocoRef.current.FS.mkdir("/working");
//       mujocoRef.current.FS.mount(
//         mujocoRef.current.MEMFS,
//         { root: "." },
//         "/working"
//       );

//       // Fetch and write the initial scene file.
//       const sceneResponse = await fetch(
//         `${BASE_URL}/examples/scenes/${INITIAL_SCENE}`
//       );
//       const sceneText = await sceneResponse.text();
//       mujocoRef.current.FS.writeFile(`/working/${INITIAL_SCENE}`, sceneText);

//       // // Load in the state from XML
//       demoRef.current.model = new mujocoRef.current.Model(
//         `/working/${INITIAL_SCENE}`
//       );
//       demoRef.current.state = new mujocoRef.current.State(
//         demoRef.current.model
//       );
//       demoRef.current.simulation = new mujocoRef.current.Simulation(
//         demoRef.current.model,
//         demoRef.current.state
//       );
//       demoRef.current.scene = scene;

//       // Initialize MuJoCo Simulation
//       const [model, state, simulation, bodies, lights] = await loadSceneFromURL(
//         mujocoRef.current,
//         INITIAL_SCENE,
//         demoRef.current
//       );
//     };

//     initialize();

//     const geometry: THREE.BoxGeometry = new THREE.BoxGeometry(1, 1, 1);
//     const material: THREE.MeshStandardMaterial = new THREE.MeshStandardMaterial(
//       { color: 0x00ff00 }
//     );

//     // Create mesh and set position
//     const cube: THREE.Mesh = new THREE.Mesh(geometry, material);
//     cube.position.set(0, 0, 0);

//     // Add cube to the scene
//     scene.add(cube);

//     // Cleanup function to remove the cube when the component unmounts
//     return () => {
//       scene.remove(cube);
//       geometry.dispose();
//       material.dispose();
//     };
//   }, [scene]);

//   return null; // This component doesn't render anything visually
// };

const App = () => {
  return (
    <>
      <Mujoco sceneUrl={`${BASE_URL}/examples/scenes/${INITIAL_SCENE}`} />
      {/* <Canvas
        shadows
        // only re-render when props changed or when requested.
        // frameloop="demand"
        style={{
          backgroundColor: "#192635",
          borderRadius: "inherit",
          margin: "0 auto", // Center horizontally.
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
        <AddBodies />
      </Canvas> */}
    </>
  );
};
export default App;
