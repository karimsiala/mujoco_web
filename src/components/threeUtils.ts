import { Reflector } from 'three/addons/objects/Reflector.js';
import * as THREE from "three";

const vertexShader = `
    varying float vFogDepth;
    varying vec2 vUv;

    void main() {
        vUv = uv;

        // Calculate model-view position
        vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
        gl_Position = projectionMatrix * mvPosition;

        // Calculate fog depth (distance from camera)
        vFogDepth = -mvPosition.z;
    }
`;

const fragmentShader = `
    uniform vec3 color1;
    uniform vec3 color2;
    uniform float scale;
    uniform float opacity;

    uniform vec3 fogColor;
    uniform float fogNear;
    uniform float fogFar;

    varying float vFogDepth;
    varying vec2 vUv;

    void main() {
        // Generate checkerboard pattern
        float checkSize = scale;
        float x = floor(vUv.x * checkSize);
        float y = floor(vUv.y * checkSize);
        float checker = mod(x + y, 2.0);
        vec3 checkerColor = mix(color1, color2, checker);
        
        // Set fragment color with opacity
        vec4 fragColor = vec4(checkerColor, opacity);

        // Compute fog factor using linear fog
        float fogFactor = smoothstep(fogNear, fogFar, vFogDepth);

        // Blend the fragment color with the fog color
        fragColor.rgb = mix(fogColor, fragColor.rgb, 1.0 - fogFactor);

        gl_FragColor = fragColor;
    }
`;

// Create the Shader Material with Fog Support
export const createCheckerMaterial = (
    color1: THREE.Color,
    color2: THREE.Color,
    scale: number,
    opacity: number,
    fogColor: THREE.Color,
    fogNear: number,
    fogFar: number
): THREE.ShaderMaterial => {
    const material = new THREE.ShaderMaterial({
        vertexShader,
        fragmentShader,
        uniforms: {
            color1: { value: color1 },
            color2: { value: color2 },
            scale: { value: scale },
            opacity: { value: opacity },
            fogColor: { value: fogColor },
            fogNear: { value: fogNear },
            fogFar: { value: fogFar }
        },
        transparent: true,
        side: THREE.DoubleSide
    });
    return material;
};

export const createGroundMirror = (): THREE.Object3D => {
    const geometry = new THREE.CircleGeometry(40, 64);
    const groundMirror = new Reflector(geometry, {
        clipBias: 0.003,
        textureWidth: window.innerWidth * window.devicePixelRatio,
        textureHeight: window.innerHeight * window.devicePixelRatio,
        color: 0xAAAAAA
    });
    return groundMirror;
}

export const createChekerboard = (color1: THREE.Color, color2: THREE.Color, scale: number, opacity: number, fogColor: THREE.Color, fogNear: number, fogFar: number): THREE.Object3D => {
    const geometry = new THREE.CircleGeometry(40, 64);
    const checkerMaterial = createCheckerMaterial(color1, color2, scale, opacity, fogColor, fogNear, fogFar);
    const checkerboard = new THREE.Mesh(geometry, checkerMaterial);
    return checkerboard;
}

export const createMirrotCheckerboard = (): THREE.Object3D => {
    const color1 = new THREE.Color(0x597BA1);
    const color2 = new THREE.Color(0x000000);
    const scale = 80;
    const opacity = 0.2;
    const fogColor = new THREE.Color(0x264059);
    const fogNear = 1;
    const fogFar = 30;
    const groundMirror = createGroundMirror();
    const checkerboard = createChekerboard(color1, color2, scale, opacity, fogColor, fogNear, fogFar);
    const group = new THREE.Group();
    group.add(groundMirror);
    group.add(checkerboard);
    return group;
}

