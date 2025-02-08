export interface BodyInfo {
  bodyID: number;
  name: string;
  position: [number, number, number];
  rotation?: [number, number, number];
  velocity?: [number, number, number];
}