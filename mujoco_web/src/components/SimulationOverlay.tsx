import React, { useState, useEffect, useRef } from 'react';

interface SimulationOverlayProps {
  mujocoContainer: any;
  editor: any;
}

export const SimulationOverlay: React.FC<SimulationOverlayProps> = ({ mujocoContainer, editor }) => {
  const [error, setError] = useState<string>('');
  const [debugStats, setDebugStats] = useState({
    fps: 0,
    bodyCount: 0,
    timestep: 0,
    selectedBodyId: null as number | null
  });
  const [showHelp, setShowHelp] = useState(true);
  const [isScriptRunning, setIsScriptRunning] = useState(false);
  const lastFrameTimeRef = useRef<number>(performance.now());
  const frameCountRef = useRef<number>(0);
  const intervalRef = useRef<number | null>(null);
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
    intervalRef.current = window.setInterval(updateFPS, 1000);
    return () => {
      if (intervalRef.current) clearInterval(intervalRef.current);
    };
  }, []);

  // Optionally, update body count and timestep from the simulation model
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

  const runScript = async () => {
    if (!mujocoContainer || !editor) {
      setError('Simulation or Editor not ready');
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
      const code = editor.getValue();

      const stepSimulation = async () => {
        if (abortScriptRef.current) throw new Error('Script aborted due to simulation reset.');
        if (simulation && model) {
          simulation.step();
          frameCountRef.current++;
          const delay = model.getOptions().timestep * 1000;
          await new Promise(resolve => setTimeout(resolve, delay));
        }
      };

      const applyForceToBody = (bodyId: number, force: [number, number, number], point: [number, number, number]) => {
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

      const AsyncFunction = Object.getPrototypeOf(async function(){}).constructor;
      const scriptFn = new AsyncFunction('simulation', 'model', 'stepSimulation', 'applyForceToBody', code);
      await scriptFn(simulation, model, stepSimulation, applyForceToBody);
      setError('');
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Unknown error');
    } finally {
      setIsScriptRunning(false);
    }
  };

  const resetSimulation = () => {
    if (isScriptRunning) abortScriptRef.current = true;
    if (mujocoContainer) {
      const simulation = mujocoContainer.getSimulation();
      const model = simulation.model();
      if (model) simulation.resetData();
    }
  };

  return (
    <div className="absolute w-1/3 top-0 right-0 right-0 p-2 bg-black bg-opacity-50 text-white text-sm">
      <div className="flex justify-between items-center">
        <div className="flex gap-2">
          <button
            onClick={runScript}
            disabled={isScriptRunning}
            className={`px-4 py-2 bg-blue-500 rounded ${isScriptRunning ? 'opacity-50 cursor-not-allowed' : ''}`}
          >
            {isScriptRunning ? 'Script Running...' : 'Run Script'}
          </button>
          <button
            onClick={resetSimulation}
            className="px-4 py-2 bg-red-500 rounded"
          >
            Reset Simulation
          </button>
          <button
            onClick={() => setShowHelp(prev => !prev)}
            className="px-4 py-2 bg-gray-700 rounded"
          >
            {showHelp ? 'Hide Help' : 'Show Help'}
          </button>
        </div>
        {error && <div className="px-4 py-2 text-red-300">{error}</div>}
      </div>
      <div className="mt-2 flex gap-4">
        <p>FPS: {debugStats.fps}</p>
        <p>Body Count: {debugStats.bodyCount}</p>
        <p>Timestep: {debugStats.timestep}</p>
        <p>Selected Body: {debugStats.selectedBodyId !== null ? debugStats.selectedBodyId : 'None'}</p>
      </div>
      {showHelp && (
        <div className="mt-2 p-2 bg-gray-800 text-xs font-mono">
          <p><strong>Script Help:</strong> Access <code>simulation</code>, <code>model</code>, <code>stepSimulation()</code>, and <code>applyForceToBody()</code>.</p>
        </div>
      )}
    </div>
  );
};
