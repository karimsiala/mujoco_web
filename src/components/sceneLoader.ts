
import * as THREE from 'three';
import { Model, MujocoModule } from '../wasm/mujoco_wasm';

/**
 * Add a body ID to the Mesh object.
 */
class BodyMesh extends THREE.Mesh {
    bodyID?: number;
}

/**
 * Add a body ID to the Group object and a boolean to indicate if the mesh has
 * been customized.
 */
class Body extends THREE.Group {
    bodyID?: number;
    has_custom_mesh?: boolean;
}

/** 
 * Access the vector at index, swizzle for three.js, and apply to the target
 * THREE.Vector3
 */
export const getPosition = (
    buffer: Float32Array | Float64Array,
    index: number,
    target: THREE.Vector3,
    swizzle: boolean = true
) => {
    if (swizzle) {
        return target.set(
            buffer[index * 3 + 0],
            buffer[index * 3 + 2],
            -buffer[index * 3 + 1]
        );
    } else {
        return target.set(
            buffer[index * 3 + 0],
            buffer[index * 3 + 1],
            buffer[index * 3 + 2]
        );
    }
}

/** 
 * Access the quaternion at index, swizzle for three.js, and apply to the
 * target THREE.Quaternion.
 */
export const getQuaternion = (
    buffer: Float32Array | Float64Array,
    index: number,
    target: THREE.Quaternion,
    swizzle: boolean = true
) => {
    if (swizzle) {
        return target.set(
            -buffer[index * 4 + 1],
            -buffer[index * 4 + 3],
            buffer[index * 4 + 2],
            -buffer[index * 4 + 0]
        );
    } else {
        return target.set(
            buffer[index * 4 + 0],
            buffer[index * 4 + 1],
            buffer[index * 4 + 2],
            buffer[index * 4 + 3]
        );
    }
}

export const loadScene = (mujocoModule: MujocoModule, model: Model, scene: THREE.Scene): void => {

    // Decode the null-terminated string names.
    const textDecoder = new TextDecoder("utf-8");
    const fullString = textDecoder.decode(model.names);
    const names = fullString.split(textDecoder.decode(new ArrayBuffer(1)));

    // Create the root object.
    const mujocoRoot = new THREE.Group();
    mujocoRoot.name = "MuJoCo Root"
    scene.add(mujocoRoot);

    const bodies: { [key: number]: Body } = {};
    const meshes: { [key: number]: THREE.BufferGeometry } = {};
    const lights: THREE.Light[] = [];

    // Default material definition.
    let material = new THREE.MeshPhysicalMaterial();
    material.color = new THREE.Color(1, 1, 1);

    // Loop through the MuJoCo geoms and recreate them in three.js.
    for (let g = 0; g < model.ngeom; g++) {
        // Only visualize geom groups up to 2 (same default behavior as simulate).
        if (!(model.geom_group[g] < 3)) {
            continue;
        }

        // Get the body ID and type of the geom.
        const b = model.geom_bodyid[g];
        const type = model.geom_type[g];
        const size = [
            model.geom_size[g * 3 + 0],
            model.geom_size[g * 3 + 1],
            model.geom_size[g * 3 + 2],
        ];

        // Create the body if it doesn't exist.
        if (!(b in bodies)) {
            bodies[b] = new Body();
            bodies[b].name = names[model.name_bodyadr[b]];
            bodies[b].bodyID = b;
            bodies[b].has_custom_mesh = false;
        }

        // Set the default geometry. In MuJoCo, this is a sphere.
        let geometry: THREE.BufferGeometry = new THREE.SphereGeometry(
            size[0] * 0.5
        );

        if (type === mujocoModule.mjtGeom.mjGEOM_PLANE.value) {
            // Special handling for plane later.
        } else if (type === mujocoModule.mjtGeom.mjGEOM_HFIELD.value) {
            // TODO: Implement this.
        } else if (type === mujocoModule.mjtGeom.mjGEOM_SPHERE.value) {
            geometry = new THREE.SphereGeometry(size[0]);
        } else if (type === mujocoModule.mjtGeom.mjGEOM_CAPSULE.value) {
            geometry = new THREE.CapsuleGeometry(size[0], size[1] * 2.0, 20, 20);
        } else if (type === mujocoModule.mjtGeom.mjGEOM_ELLIPSOID.value) {
            geometry = new THREE.SphereGeometry(1); // Stretch this below
        } else if (type === mujocoModule.mjtGeom.mjGEOM_CYLINDER.value) {
            geometry = new THREE.CylinderGeometry(size[0], size[0], size[1] * 2.0);
        } else if (type === mujocoModule.mjtGeom.mjGEOM_BOX.value) {
            geometry = new THREE.BoxGeometry(
                size[0] * 2.0,
                size[2] * 2.0,
                size[1] * 2.0
            );
        } else if (type === mujocoModule.mjtGeom.mjGEOM_MESH.value) {
            const meshID = model.geom_dataid[g];

            if (!(meshID in meshes)) {
                geometry = new THREE.BufferGeometry(); // TODO: Populate the Buffer Geometry with Generic Mesh Data

                const vertex_buffer = model.mesh_vert.subarray(
                    model.mesh_vertadr[meshID] * 3,
                    (model.mesh_vertadr[meshID] + model.mesh_vertnum[meshID]) *
                    3
                );
                for (let v = 0; v < vertex_buffer.length; v += 3) {
                    //vertex_buffer[v + 0] =  vertex_buffer[v + 0];
                    const temp = vertex_buffer[v + 1];
                    vertex_buffer[v + 1] = vertex_buffer[v + 2];
                    vertex_buffer[v + 2] = -temp;
                }

                const normal_buffer = model.mesh_normal.subarray(
                    model.mesh_vertadr[meshID] * 3,
                    (model.mesh_vertadr[meshID] + model.mesh_vertnum[meshID]) *
                    3
                );
                for (let v = 0; v < normal_buffer.length; v += 3) {
                    //normal_buffer[v + 0] =  normal_buffer[v + 0];
                    const temp = normal_buffer[v + 1];
                    normal_buffer[v + 1] = normal_buffer[v + 2];
                    normal_buffer[v + 2] = -temp;
                }

                const uv_buffer = model.mesh_texcoord.subarray(
                    model.mesh_texcoordadr[meshID] * 2,
                    (model.mesh_texcoordadr[meshID] +
                        model.mesh_vertnum[meshID]) *
                    2
                );
                const triangle_buffer = model.mesh_face.subarray(
                    model.mesh_faceadr[meshID] * 3,
                    (model.mesh_faceadr[meshID] + model.mesh_facenum[meshID]) *
                    3
                );
                geometry.setAttribute(
                    "position",
                    new THREE.BufferAttribute(vertex_buffer, 3)
                );
                geometry.setAttribute(
                    "normal",
                    new THREE.BufferAttribute(normal_buffer, 3)
                );
                geometry.setAttribute("uv", new THREE.BufferAttribute(uv_buffer, 2));
                geometry.setIndex(Array.from(triangle_buffer));
                meshes[meshID] = geometry;
            } else {
                geometry = meshes[meshID];
            }

            bodies[b].has_custom_mesh = true;
        }
        // Done with geometry creation.

        // Set the Material Properties of incoming bodies
        let texture = undefined;
        let color = [
            model.geom_rgba[g * 4 + 0],
            model.geom_rgba[g * 4 + 1],
            model.geom_rgba[g * 4 + 2],
            model.geom_rgba[g * 4 + 3],
        ];
        if (model.geom_matid[g] != -1) {
            const matId = model.geom_matid[g];
            color = [
                model.mat_rgba[matId * 4 + 0],
                model.mat_rgba[matId * 4 + 1],
                model.mat_rgba[matId * 4 + 2],
                model.mat_rgba[matId * 4 + 3],
            ];

            // Construct Texture from model.tex_data
            texture = undefined;
            const texId = model.mat_texid[matId];
            if (texId != -1) {
                const width = model.tex_width[texId];
                const height = model.tex_height[texId];
                const offset = model.tex_adr[texId];
                const rgbArray = model.tex_data;
                const rgbaArray = new Uint8Array(width * height * 4);

                for (let p = 0; p < width * height; p++) {
                    rgbaArray[p * 4 + 0] = rgbArray[offset + (p * 3 + 0)];
                    rgbaArray[p * 4 + 1] = rgbArray[offset + (p * 3 + 1)];
                    rgbaArray[p * 4 + 2] = rgbArray[offset + (p * 3 + 2)];
                    rgbaArray[p * 4 + 3] = 1.0;
                }
                texture = new THREE.DataTexture(
                    rgbaArray,
                    width,
                    height,
                    THREE.RGBAFormat,
                    THREE.UnsignedByteType
                );
                if (texId == 2) {
                    texture.repeat = new THREE.Vector2(50, 50);
                    texture.wrapS = THREE.RepeatWrapping;
                    texture.wrapT = THREE.RepeatWrapping;
                } else {
                    texture.repeat = new THREE.Vector2(1, 1);
                    texture.wrapS = THREE.RepeatWrapping;
                    texture.wrapT = THREE.RepeatWrapping;
                }

                texture.needsUpdate = true;
            }
        }

        if (
            material.color.r != color[0] ||
            material.color.g != color[1] ||
            material.color.b != color[2] ||
            material.opacity != color[3] ||
            material.map != texture
        ) {
            console.log(`Texture: ${texture}`)

            material = new THREE.MeshPhysicalMaterial({
                color: new THREE.Color(color[0], color[1], color[2]),
                transparent: color[3] < 1.0,
                opacity: color[3],
                specularIntensity:
                    model.geom_matid[g] != -1
                        ? model.mat_specular[model.geom_matid[g]] * 0.5
                        : undefined,
                reflectivity:
                    model.geom_matid[g] != -1
                        ? model.mat_reflectance[model.geom_matid[g]]
                        : undefined,
                roughness:
                    model.geom_matid[g] != -1
                        ? 1.0 - model.mat_shininess[model.geom_matid[g]]
                        : undefined,
                metalness: model.geom_matid[g] != -1 ? 0.1 : undefined,
                map: texture,
            });
        }

        let mesh = new BodyMesh();
        if (type == 0) {
            // TODO: change to reflctor.
            mesh = new THREE.Mesh(new THREE.PlaneGeometry(100, 100));
            mesh.rotateX(-Math.PI / 2);
        } else {
            mesh = new THREE.Mesh(geometry, material);
        }

        mesh.castShadow = g == 0 ? false : true;
        mesh.receiveShadow = type != 7;
        mesh.bodyID = b;
        bodies[b].add(mesh);
        getPosition(model.geom_pos, g, mesh.position);
        if (type != 0) {
            getQuaternion(model.geom_quat, g, mesh.quaternion);
        }
        if (type == 4) {
            mesh.scale.set(size[0], size[2], size[1]);
        } // Stretch the Ellipsoid
    }


    // Parse tendons.
    const tendonMat = new THREE.MeshPhongMaterial();
    tendonMat.color = new THREE.Color(0.8, 0.3, 0.3);
    const cylinders = new THREE.InstancedMesh(
        new THREE.CylinderGeometry(1, 1, 1),
        tendonMat,
        1023
    );
    cylinders.receiveShadow = true;
    cylinders.castShadow = true;
    mujocoRoot.add(cylinders);

    const spheres = new THREE.InstancedMesh(
        new THREE.SphereGeometry(1, 10, 10),
        tendonMat,
        1023
    );
    spheres.receiveShadow = true;
    spheres.castShadow = true;
    mujocoRoot.add(spheres);

    // Parse lights.
    for (let l = 0; l < model.nlight; l++) {
        let light: THREE.SpotLight | THREE.DirectionalLight;
        if (model.light_directional[l]) {
            light = new THREE.DirectionalLight();
        } else {
            light = new THREE.SpotLight();
            light.decay = model.light_attenuation[l] * 100;
            light.penumbra = 0.5;
        }
        light.castShadow = true; // default false
        light.shadow.mapSize.width = 1024; // default
        light.shadow.mapSize.height = 1024; // default
        light.shadow.camera.near = 1; // default
        light.shadow.camera.far = 10; // default
        //bodies[model.light_bodyid()].add(light);
        if (bodies[0]) {
            bodies[0].add(light);
        } else {
            mujocoRoot.add(light);
        }
        lights.push(light);
    }
    if (model.nlight == 0) {
        const light = new THREE.DirectionalLight();
        mujocoRoot.add(light);
    }

    for (let b = 0; b < model.nbody; b++) {
        //let parent_body = model.body_parentid()[b];
        if (b == 0 || !bodies[0]) {
            mujocoRoot.add(bodies[b]);
        } else if (bodies[b]) {
            bodies[0].add(bodies[b]);
        } else {
            console.log(
                "Body without Geometry detected; adding to bodies",
                b,
                bodies[b]
            );
            bodies[b] = new THREE.Group();
            bodies[b].name = names[b + 1];
            bodies[b].bodyID = b;
            bodies[b].has_custom_mesh = false;
            bodies[0].add(bodies[b]);
        }
    }
}
