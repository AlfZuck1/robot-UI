export interface TrajectoryPoint {
  positions: number[];
  time: number;
}

export interface Trajectory {
  name: string;
  points: TrajectoryPoint[];
}