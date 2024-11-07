import { Model } from '../wasm/mujoco_wasm';
import * as THREE from 'three';

export const loadScene = (model: Model, scene: THREE.Scene): void => {


    // Decode the null-terminated string names.
    const textDecoder = new TextDecoder("utf-8");
    const fullString = textDecoder.decode(model.names);
    const names = fullString.split(textDecoder.decode(new ArrayBuffer(1)));
    console.log(names)

};