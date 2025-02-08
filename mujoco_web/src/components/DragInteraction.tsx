// DragInteraction.tsx
import { useThree } from '@react-three/fiber';
import { useEffect, useRef } from 'react';
import { DragStateManager } from './dragStateManager';
import { OrbitControls } from '@react-three/drei';

export const DragInteraction: React.FC = () => {
  const { scene, camera, gl: renderer, controls } = useThree();
  const dragManagerRef = useRef<DragStateManager | null>(null);

  useEffect(() => {
    // Get the canvas element which is the parent of the renderer's domElement
    const container = renderer.domElement.parentElement;
    
    if (container && scene && camera && renderer && controls) {
      dragManagerRef.current = new DragStateManager(
        scene,
        renderer,
        camera,
        container,
        controls
      );
    }

    return () => {
      dragManagerRef.current?.dispose();
    };
  }, [scene, camera, renderer, controls]);

  return null;
};
