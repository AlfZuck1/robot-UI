export interface TrajectoryPoint {
  angles: number[];
  time: number;
}

export interface Trajectory {
  id: number;
  name: string;
  points: TrajectoryPoint[];
}

export interface First_Pose {
  angle1: number;
  angle2: number;
  angle3: number;
  angle4: number;
  angle5: number;
  angle6: number;
}