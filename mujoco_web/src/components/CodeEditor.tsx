import React from 'react';
import MonacoEditor from '@monaco-editor/react';

interface CodeEditorProps {
  mujocoContainer: any;
  selectedBodyId?: number | null;
  onEditorMount: (editor: any) => void;
  onClose: () => void; // new prop for closing editor
}

const defaultScript = `// Example script to control the simulation
applyForceToBody(1, [0, 0, 10], [0, 0, 0]);
for (let i = 0; i < 100; i++) {
  await stepSimulation();
}
`;

export const CodeEditor: React.FC<CodeEditorProps> = ({ onEditorMount, onClose }) => {  
  return (
    <div style={{ border: '1px solid #333', borderRadius: '4px', overflow: 'hidden', boxShadow: '0 2px 8px rgba(0,0,0,0.3)', height: '100%' }}>
      <div style={{ backgroundColor: '#444', color: '#fff', padding: '8px', display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
        <span>Code Editor</span>
        <button 
          onClick={onClose}
          style={{ background: 'none', border: 'none', color: '#fff', cursor: 'pointer', fontSize: '16px' }}
        >
          x
        </button>
      </div>
      {/* Set height to account for header */}
      <div style={{ height: 'calc(100% - 40px)' }} className="w-full">
        <MonacoEditor
          height="100%"
          defaultLanguage="javascript"
          defaultValue={defaultScript}
          theme="vs-dark"
          options={{
            minimap: { enabled: false },
            fontSize: 14,
            scrollBeyondLastLine: false,
            automaticLayout: true
          }}
          onMount={onEditorMount}
        />
      </div>
    </div>
  );
};
