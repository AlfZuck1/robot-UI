import { Injectable } from '@angular/core';
import { Trajectory } from '../components/models/trajectory';

@Injectable({
  providedIn: 'root'
})
export class TrajectoryService {
  private trajectories: Trajectory[] = [];

  addTrajectory(Trajectory: Trajectory) {
    this.trajectories.push(Trajectory);
  }

  getTrajectories(): Trajectory[] {
    return this.trajectories;
  }

  removeTrajectory(index: number) {
    this.trajectories.splice(index, 1);
  }

  clearTrajectories() {
    this.trajectories = [];
  }
}
