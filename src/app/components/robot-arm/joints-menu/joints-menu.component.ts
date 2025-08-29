import { Component, HostBinding, signal } from '@angular/core';
import { FormsModule } from '@angular/forms';
import { CommonModule } from '@angular/common';
import { RosService } from '../../../services/ros.service';
import { Trajectory } from '../../models/trajectory';
import { UiStateService } from '../../../services/ui-state.service';
import { TrajectoryService } from '../../../services/trajectory.service';
import { RobotStateService } from '../../../services/robot-state.service';


@Component({
  selector: 'app-joints-menu',
  imports: [CommonModule, FormsModule],
  templateUrl: './joints-menu.component.html',
  styleUrl: './joints-menu.component.scss'
})
export class JointsMenuComponent {

  constructor(private rosService: RosService,
    private uiStateService: UiStateService,
    private trajectoryService: TrajectoryService,
    public robot: RobotStateService) {
    this.uiStateService.menuOpen$.subscribe(open => this.menuOpen = open);
    this.uiStateService.isSmallScreen$.subscribe(small => this.isSmallScreen = small);
    this.uiStateService.manualMode$.subscribe(mode => this.manualMode = mode);
    this.trajectories = this.trajectoryService.getTrajectories();
  }

  menuOpen: boolean = false;
  isSmallScreen: boolean = false;
  errorMessage: string = '';
  manualMode: boolean = false;

  // local Angles
  pointAngles: number[] = [0, 0, 0, 0, 0, 0];

  trajectories: Trajectory[] = [];
  currentTrajectory: Trajectory | null = null;
  newTrajectory: Trajectory = {
    name: 'Trayectoria 1',
    points: []
  };
  newPoint: { positions: number[]; time: number } = { positions: [0, 0, 0, 0, 0, 0], time: 0 };

  moveJoint(joint: string, angle: number) {
    if (!this.manualMode) {
      this.robot.angle1.set(this.pointAngles[0]);
      this.robot.angle2.set(this.pointAngles[1]);
      this.robot.angle3.set(this.pointAngles[2]);
      this.robot.angle4.set(this.pointAngles[3]);
      this.robot.angle5.set(this.pointAngles[4]);
      this.robot.angle6.set(this.pointAngles[5]);
    }
    else {
      const jointMap: { [key: string]: number } = {
        'junta_0_1': this.pointAngles[0],
        'junta_1_2': this.pointAngles[1],
        'junta_2_3': this.pointAngles[2],
        'junta_3_4': this.pointAngles[3],
        'junta_4_5': this.pointAngles[4],
        'junta_5_6': this.pointAngles[5]
      };

      (this as any)[jointMap[joint]] = angle;

      const angles = [
        this.pointAngles[0], this.pointAngles[1], this.pointAngles[2],
        this.pointAngles[3], this.pointAngles[4], this.pointAngles[5]
      ].map(deg => deg * Math.PI / 180);

      this.rosService.publishJointState(Object.keys(jointMap), angles);
    }
  }

  rotate6Joint(angle: number) {
    const num = angle / 360;
    this.rosService.publishJoint6Command(num);
  }

  resetCameraPosition() {
    this.uiStateService.triggerResetCamera();
  }

  // --- Funciones de trayectorias ---
  addPoint() {
    const positions = [this.robot.angle1(), this.robot.angle2(), this.robot.angle3(), this.robot.angle4(), this.robot.angle5(), this.robot.angle6()];
    this.newPoint = { positions, time: this.robot.time() };
    this.newTrajectory.points.push({ positions: [...this.newPoint.positions], time: this.newPoint.time });
  }

  updatePoint(index: number) {
    const positions = [this.robot.angle1(), this.robot.angle2(), this.robot.angle3(), this.robot.angle4(), this.robot.angle5(), this.robot.angle6()];
    this.newTrajectory.points[index] = { positions: [...positions], time: this.robot.time() };
  }

  addTrajectory() {
    if (!this.newTrajectory.name || this.newTrajectory.points.length === 0) {
      this.errorMessage = 'Por favor, ingrese un nombre y al menos un punto para la trayectoria.';
      return;
    }
    this.trajectoryService.addTrajectory(this.newTrajectory);
    this.trajectories = this.trajectoryService.getTrajectories();
    this.newTrajectory = { name: '', points: [] };
    this.trajectories.push({ ...this.newTrajectory });
  }

  removeTrajectory() {
    if (!this.currentTrajectory) return;
    this.trajectoryService.removeTrajectory(this.trajectories.indexOf(this.currentTrajectory));
    this.trajectories = this.trajectoryService.getTrajectories();
    this.currentTrajectory = null;
  }

  onTrajectoryChange() {
    if (!this.newTrajectory) return;

    this.currentTrajectory = this.trajectories.find(t => t === this.newTrajectory) || null;

    if (this.currentTrajectory && this.currentTrajectory.points.length > 0) {
      const firstPoint = this.currentTrajectory.points[0];
      this.moveToPoint(firstPoint);
      this.robot.time.set(firstPoint.time);
    }
  }

  moveToPoint(point: { positions: number[], time: number }) {
    this.robot.angle1.set(point.positions[0]);
    this.robot.angle2.set(point.positions[1]);
    this.robot.angle3.set(point.positions[2]);
    this.robot.angle4.set(point.positions[3]);
    this.robot.angle5.set(point.positions[4]);
    this.robot.angle6.set(point.positions[5]);
    this.robot.time.set(point.time);

    // Llamamos a moveJoint para simular el movimiento
    this.moveJoint('junta_0_1', this.robot.angle1());
    this.moveJoint('junta_1_2', this.robot.angle2());
    this.moveJoint('junta_2_3', this.robot.angle3());
    this.moveJoint('junta_3_4', this.robot.angle4());
    this.moveJoint('junta_4_5', this.robot.angle5());
    this.moveJoint('junta_5_6', this.robot.angle6());
  }

  toggleMenu() {
    this.menuOpen = !this.menuOpen;
    this.uiStateService.setMenuOpen(this.menuOpen);
  }

  toggleManual(){
    this.manualMode = !this.manualMode;
    this.uiStateService.setManualMode(this.manualMode);
  }

}
