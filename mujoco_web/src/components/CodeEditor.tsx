import React, { useRef, useState, useEffect } from 'react';
import MonacoEditor from '@monaco-editor/react';
import { MujocoContainer } from './MujocoContainer';

interface CodeEditorProps {
  mujocoContainer: MujocoContainer | null;
  selectedBodyId?: number | null;
}

// Update the default script to use "await" for stepSimulation
const defaultScript = `// Example script to control the simulation
// Globals available:
//   simulation: MuJoCo simulation instance
//   model: MuJoCo model instance
//   stepSimulation(): advances simulation by one timestep (use "await" to run naturally)
//   applyForceToBody(bodyId, force, point): applies force to a body
//
// Example: Apply an upward force to body 1 and step simulation naturally
applyForceToBody(1, [0, 0, 10], [0, 0, 0]);
for (let i = 0; i < 100; i++) {
  await stepSimulation();
}
`;

interface DebugStats {
  elapsedTime: number;
  bodyCount: number;
  selectedBodyId: number | null;
  fps: number;
  timestep: number;
}

export const CodeEditor: React.FC<CodeEditorProps> = ({ mujocoContainer, selectedBodyId }) => {
  const editorRef = useRef<any>(null);
  const [error, setError] = useState<string>('');
  const [debugStats, setDebugStats] = useState<DebugStats>({
    elapsedTime: 0,
    bodyCount: 0,
    selectedBodyId: null,
    fps: 0,
    timestep: 0
  });
  const lastFrameTimeRef = useRef<number>(performance.now());
  const frameCountRef = useRef<number>(0);
  const fpsUpdateIntervalRef = useRef<number | null>(null);
  const [showHelp, setShowHelp] = useState(true);
  const [isScriptRunning, setIsScriptRunning] = useState(false);
  const abortScriptRef = useRef<boolean>(false);

  // Update FPS every second
  useEffect(() => {
    const updateFPS = () => {
      const currentTime = performance.now();
      const elapsed = currentTime - lastFrameTimeRef.current;
      const fps = Math.round((frameCountRef.current * 1000) / elapsed);
      setDebugStats(prev => ({ ...prev, fps }));
      frameCountRef.current = 0;
      lastFrameTimeRef.current = currentTime;
    };

    fpsUpdateIntervalRef.current = window.setInterval(updateFPS, 1000);

    return () => {
      if (fpsUpdateIntervalRef.current) {
        clearInterval(fpsUpdateIntervalRef.current);
      }
    };
  }, []);

  // Update debug stats when the simulation/model changes
  useEffect(() => {
    if (mujocoContainer) {
      const simulation = mujocoContainer.getSimulation();
      const model = simulation.model();
      if (model) {
        setDebugStats(prev => ({
          ...prev,
          bodyCount: model.nbody,
          timestep: model.getOptions().timestep
        }));
      }
    }
  }, [mujocoContainer]);

  // Update debug stats when selected body changes
  useEffect(() => {
    setDebugStats(prev => ({
      ...prev,
      selectedBodyId: selectedBodyId || null
    }));
  }, [selectedBodyId]);

  const handleEditorDidMount = (editor: any) => {
    editorRef.current = editor;
  };

  // Run the user script asynchronously while preventing concurrent execution
  const runScript = async () => {
    if (!mujocoContainer) {
      setError('Simulation not ready');
      return;
    }
    if (isScriptRunning) {
      setError('A script is already running.');
      return;
    }
    setIsScriptRunning(true);
    abortScriptRef.current = false;
    const simulation = mujocoContainer.getSimulation();
    const model = simulation.model();

    try {
      const code = editorRef.current.getValue();

      // Define asynchronous helper functions
      const stepSimulation = async () => {
        if (abortScriptRef.current) {
          throw new Error('Script aborted due to simulation reset.');
        }
        if (simulation && model) {
          simulation.step();
          // Increment frame count for FPS measurement
          frameCountRef.current++;
          const delay = model.getOptions().timestep * 1000;
          await new Promise(resolve => setTimeout(resolve, delay));
        }
      };

      const applyForceToBody = (
        bodyId: number, 
        force: [number, number, number], 
        point: [number, number, number]
      ) => {
        if (!model) return;
        const timestep = model.getOptions().timestep;
        const scaledForce = force.map(f => f * timestep) as [number, number, number];
        simulation.applyForce(
          scaledForce[0], scaledForce[1], scaledForce[2],
          0, 0, 0,
          point[0], point[1], point[2],
          bodyId
        );
      };

      // Create an async function from the user code
      const AsyncFunction = Object.getPrototypeOf(async function(){}).constructor;
      const scriptFn = new AsyncFunction(
        'simulation', 
        'model', 
        'stepSimulation', 
        'applyForceToBody', 
        code
      );

      await scriptFn(simulation, model, stepSimulation, applyForceToBody);
      setError('');
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Unknown error');
    } finally {
      setIsScriptRunning(false);
    }
  };

  // Reset the simulation and abort any running script
  const resetSimulation = () => {
    if (isScriptRunning) {
      abortScriptRef.current = true;
    }
    if (mujocoContainer) {
      const simulation = mujocoContainer.getSimulation();
      const model = simulation.model();
      if (model) {
        // Reset the simulation state
        simulation.resetData();
      }
    }
  };

  return (
    <div className="w-full h-full flex flex-col">
      <div className="flex justify-between items-center p-2 bg-gray-800">
        <h3 className="text-white">Simulation Script</h3>
        <div className="flex gap-2">
          <button
            onClick={runScript}
            disabled={isScriptRunning}
            className={`px-4 py-2 bg-blue-500 text-white rounded hover:bg-blue-600 ${
              isScriptRunning ? 'opacity-50 cursor-not-allowed' : ''
            }`}
          >
            {isScriptRunning ? 'Script Running...' : 'Run Script'}
          </button>
          <button
            onClick={resetSimulation}
            className="px-4 py-2 bg-red-500 text-white rounded hover:bg-red-600"
          >
            Reset Simulation
          </button>
          <button
            onClick={() => setShowHelp(prev => !prev)}
            className="px-4 py-2 bg-gray-700 text-white rounded hover:bg-gray-600"
          >
            {showHelp ? 'Hide Help' : 'Show Help'}
          </button>
        </div>
      </div>

      {/* Debug Stats Panel */}
      <div className="p-2 bg-gray-700 text-white text-sm flex gap-4">
        <p>FPS: {debugStats.fps}</p>
        <p>Body Count: {debugStats.bodyCount}</p>
        <p>Timestep: {debugStats.timestep}</p>
        <p>Selected Body: {debugStats.selectedBodyId !== null ? debugStats.selectedBodyId : 'None'}</p>
      </div>

      {showHelp && (
        <div className="p-3 bg-gray-900 text-white text-sm font-mono">
          <p><strong>Script Help:</strong> Write code to interact with the simulation.</p>
          <ul className="list-disc ml-5">
            <li><code>simulation</code>: simulation instance</li>
            <li><code>model</code>: simulation model</li>
            <li><code>stepSimulation()</code>: advance simulation by one timestep</li>
            <li>
              <code>applyForceToBody(bodyId, force, point)</code>: apply force on a body (force is scaled by timestep)
            </li>
          </ul>
        </div>
      )}
      
      <MonacoEditor
        height="calc(100% - 240px)"
        defaultLanguage="javascript"
        defaultValue={defaultScript}
        theme="vs-dark"
        options={{
          minimap: { enabled: false },
          fontSize: 14,
          scrollBeyondLastLine: false,
          automaticLayout: true
        }}
        onMount={handleEditorDidMount}
      />
      {error && (
        <div className="p-2 text-red-500 bg-red-100 border border-red-300">
          {error}
        </div>
      )}
    </div>
  );
};
