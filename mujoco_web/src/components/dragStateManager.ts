import * as THREE from 'three';
import { Vector3 } from 'three';

/**
 * DragStateManager
 *
 * Handles pointer interactions in a Three.js scene. It uses a raycaster to determine
 * which object (if any) is being "dragged" and displays an arrow helper to indicate
 * the drag direction and magnitude.
 */
export class DragStateManager {
  public scene: THREE.Scene;
  public renderer: THREE.WebGLRenderer;
  public camera: THREE.Camera;
  public container: HTMLElement;
  public controls: { enabled: boolean }; // Adjust type as needed (e.g. OrbitControls)
  
  public mousePos: THREE.Vector2;
  public raycaster: THREE.Raycaster;
  public grabDistance: number = 0;
  public active: boolean = false;
  public physicsObject: THREE.Object3D | null = null;
  public arrow: THREE.ArrowHelper;
  public previouslySelected: THREE.Object3D | null = null;
  public higlightColor: number = 0xff0000; // Highlight color for double-click selection
  public localHit: Vector3 = new Vector3();
  public worldHit: Vector3 = new Vector3();
  public currentWorld: Vector3 = new Vector3();
  public mouseDown: boolean = false;
  public doubleClick: boolean = false;

  // Save the bound version of onPointer for cleanup.
  private boundOnPointer: (evt: PointerEvent) => void;

  constructor(
    scene: THREE.Scene,
    renderer: THREE.WebGLRenderer,
    camera: THREE.Camera,
    container: HTMLElement,
    controls: { enabled: boolean }
  ) {
    this.scene = scene;
    this.renderer = renderer;
    this.camera = camera;
    this.container = container;
    this.controls = controls;

    this.mousePos = new THREE.Vector2();
    this.raycaster = new THREE.Raycaster();
    // Optionally, set raycaster parameters
    // this.raycaster.layers.set(1);
    // (this.raycaster.params as any).Mesh.threshold = 3;
    this.raycaster.params.Line.threshold = 0.1;
    this.raycaster.params.Points.threshold = 0.1;
    this.raycaster.params.Mesh.threshold = 0.1;

    // Create an arrow helper to show drag direction.
    this.arrow = new THREE.ArrowHelper(
      new THREE.Vector3(0, 1, 0),
      new THREE.Vector3(0, 0, 0),
      15,
      0x666666
    );
    this.arrow.setLength(15, 3, 1);
    this.scene.add(this.arrow);

    // Make arrow line and cone transparent.
    if (this.arrow.line?.material) {
      (this.arrow.line.material as THREE.Material).transparent = true;
      (this.arrow.line.material as THREE.Material).opacity = 0.5;
      (this.arrow.line.material as THREE.Material).depthTest = false;
    }
    if (this.arrow.cone?.material) {
      (this.arrow.cone.material as THREE.Material).transparent = true;
      (this.arrow.cone.material as THREE.Material).opacity = 0.5;
      (this.arrow.cone.material as THREE.Material).depthTest = false;
    }
    this.arrow.visible = false;

    // Bind pointer event handler.
    this.boundOnPointer = this.onPointer.bind(this);
    container.addEventListener('pointerdown', this.boundOnPointer, true);
    document.addEventListener('pointermove', this.boundOnPointer, true);
    document.addEventListener('pointerup', this.boundOnPointer, true);
    document.addEventListener('pointerout', this.boundOnPointer, true);
    container.addEventListener('dblclick', this.boundOnPointer as unknown as (evt: MouseEvent) => void, false);
  }

  /**
   * updateRaycaster
   * Converts screen coordinates (x, y) to normalized device coordinates and updates the raycaster.
   */
  public updateRaycaster(x: number, y: number): void {
    const rect = this.renderer.domElement.getBoundingClientRect();
    this.mousePos.x = ((x - rect.left) / rect.width) * 2 - 1;
    this.mousePos.y = -((y - rect.top) / rect.height) * 2 + 1;
    this.raycaster.setFromCamera(this.mousePos, this.camera);
  }

  /**
   * start
   * Initiates the drag by raycasting into the scene and selecting the first object
   * that has a valid `bodyID` property.
   */
  public start(x: number, y: number): void {
    this.updateRaycaster(x, y);
    const intersects = this.raycaster.intersectObjects(this.scene.children, true);
    
    // Reset any previous state
    this.physicsObject = null;
    this.active = false;
    this.arrow.visible = false;

    for (const intersect of intersects) {
      const obj = intersect.object;
      // Direct check for bodyID without walking up parent chain
      if ((obj as any).bodyID && (obj as any).bodyID > 0) {
        this.physicsObject = obj;
        this.grabDistance = intersect.distance;
        const hit = this.raycaster.ray.origin.clone()
          .addScaledVector(this.raycaster.ray.direction, this.grabDistance);
        
        this.arrow.position.copy(hit);
        this.active = true;
        if (this.controls) this.controls.enabled = false;
        
        // Store hit points
        this.localHit.copy(obj.worldToLocal(hit.clone()));
        this.worldHit.copy(hit);
        this.currentWorld.copy(hit);
        this.arrow.visible = true;
        return;
      }
    }
  }

  /**
   * move
   * Called when the pointer moves during an active drag.
   */
  public move(x: number, y: number): void {
    if (!this.active || !this.physicsObject) return;

    this.updateRaycaster(x, y);
    const hit = this.raycaster.ray.origin.clone()
      .addScaledVector(this.raycaster.ray.direction, this.grabDistance);
    
    this.currentWorld.copy(hit);
    this.update();
  }

  /**
   * update
   * Updates the arrow helper based on the object's original grab point and the current pointer position.
   */
  public update(): void {
    if (!this.active || !this.physicsObject) return;

    // Update world hit position based on object's current transform
    this.worldHit.copy(this.localHit);
    this.physicsObject.localToWorld(this.worldHit);
    
    // Calculate and apply force
    const force = this.currentWorld.clone().sub(this.worldHit);
    const bodyId = (this.physicsObject as any).bodyID;
    
    if (bodyId) {
      // Simple force calculation like the working implementation
      const forceMultiplier = 250;  // This matches the working implementation
      // Assume model.body_mass[bodyId] is available from the window global
      const mass = (window as any).getBodyMass ? (window as any).getBodyMass(bodyId) : 1;
      const scaledForce = force.clone().multiplyScalar(mass * forceMultiplier);
      
      // Apply force through physics API
      (window as any).applyForceToBody(bodyId, [
        scaledForce.x,
        scaledForce.y,
        scaledForce.z
      ], [
        this.worldHit.x,
        this.worldHit.y,
        this.worldHit.z
      ]);
    }
    
    // Update arrow visualization
    this.arrow.position.copy(this.worldHit);
    const arrowDir = force.clone().normalize();
    this.arrow.setDirection(arrowDir);
    this.arrow.setLength(force.length());
  }

  /**
   * end
   * Ends the dragging interaction.
   */
  public end(evt: PointerEvent): void {
    // Optionally, call endGrab on your physics object if needed.
    // (this.physicsObject as any).endGrab();
    this.physicsObject = null;
    this.active = false;
    this.controls.enabled = true;
    this.arrow.visible = false;
    this.mouseDown = false;
  }

  /**
   * onPointer
   * Unified pointer event handler. Dispatches based on event type.
   */
  public onPointer(evt: PointerEvent): void {
    switch (evt.type) {
      case "pointerdown":
        this.start(evt.clientX, evt.clientY);
        this.mouseDown = true;
        break;
      case "pointermove":
        if (this.mouseDown && this.active) {
          this.move(evt.clientX, evt.clientY);
        }
        break;
      case "pointerup":
      case "pointerout":
        this.end(evt);
        break;
      case "dblclick":
        this.start(evt.clientX, evt.clientY);
        this.doubleClick = true;
        if (this.physicsObject) {
          if (this.physicsObject === this.previouslySelected) {
            // Reset emissive color.
            (this.physicsObject as any).material.emissive.setHex(0x000000);
            this.previouslySelected = null;
          } else {
            if (this.previouslySelected) {
              (this.previouslySelected as any).material.emissive.setHex(0x000000);
            }
            (this.physicsObject as any).material.emissive.setHex(this.higlightColor);
            this.previouslySelected = this.physicsObject;
          }
        } else {
          if (this.previouslySelected) {
            (this.previouslySelected as any).material.emissive.setHex(0x000000);
            this.previouslySelected = null;
          }
        }
        break;
      default:
        break;
    }
  }

  /**
   * dispose
   * Removes event listeners. Call this method when the manager is no longer needed.
   */
  public dispose(): void {
    this.container.removeEventListener('pointerdown', this.boundOnPointer, true);
    document.removeEventListener('pointermove', this.boundOnPointer, true);
    document.removeEventListener('pointerup', this.boundOnPointer, true);
    document.removeEventListener('pointerout', this.boundOnPointer, true);
    this.container.removeEventListener('dblclick', this.boundOnPointer, false);
  }
}
