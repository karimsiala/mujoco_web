import React from 'react';
import { Html } from '@react-three/drei';
import { BodyInfo } from '../types/BodyInfo';

interface DebugOverlayProps {
  info: Partial<BodyInfo>;
}

export const DebugOverlay: React.FC<DebugOverlayProps> = ({ info }) => {
  if (!info.bodyID) return null;

  const formatArray = (arr?: any[]) =>
    arr ? arr.map(v => typeof v === 'number' ? v.toFixed(2) : v).join(', ') : '';

  return (
    <Html position={info.position || [0, 0, 0]}>
      <div className="bg-black bg-opacity-75 text-white p-2 rounded text-xs whitespace-nowrap">
        {info.name && (
          <div>
            <span className="text-gray-400">Name:</span> {info.name}
          </div>
        )}
        <div>
          <span className="text-gray-400">ID:</span> {info.bodyID}
        </div>
        {info.position && (
          <div>
            <span className="text-gray-400">Pos:</span> {formatArray(info.position)}
          </div>
        )}
        {info.rotation && (
          <div>
            <span className="text-gray-400">Rot:</span> {formatArray(info.rotation)}
          </div>
        )}
        {info.velocity && (
          <div>
            <span className="text-gray-400">Vel:</span> {formatArray(info.velocity)}
          </div>
        )}
      </div>
    </Html>
  );
};