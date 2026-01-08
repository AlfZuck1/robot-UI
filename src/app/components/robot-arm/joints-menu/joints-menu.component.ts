import { CdkDragDrop, DragDropModule, moveItemInArray } from '@angular/cdk/drag-drop';
import { CommonModule } from '@angular/common';
import { Component, effect, OnInit, Signal, signal } from '@angular/core';
import { FormsModule } from '@angular/forms';
import { RouterOutlet } from '@angular/router';
import { AuthService } from '../../../services/auth.service';
import { ConfirmService } from '../../../services/confirm.service';
import { RobotStateService, RobotStateServiceSimulated } from '../../../services/robot-state.service';
import { RosService } from '../../../services/ros.service';
import { TrajectoryService } from '../../../services/trajectory.service';
import { UiStateService } from '../../../services/ui-state.service';
import { ConfirmDialogComponent } from '../../confirm-dialog-component/confirm-dialog.component';
import { LoginMenuComponent } from "../../login-menu/login-menu.component";
import { Trajectory } from '../../models/trajectory';


@Component({
  selector: 'app-joints-menu',
  imports: [CommonModule, FormsModule, DragDropModule, ConfirmDialogComponent, RouterOutlet, LoginMenuComponent],
  templateUrl: './joints-menu.component.html',
  styleUrl: './joints-menu.component.scss'
})
export class JointsMenuComponent implements OnInit {

  menuOpen: boolean = true;
  activeSubmenu: 'controls' | 'trajectories' | 'calibration' = 'controls';
  cartesianMode: boolean = false;
  errorMessage: string = '';
  isSmallScreen: boolean = false;
  velocity: Signal<number> = signal(50);

  // Articulaciones
  pointAngles: number[] = [0, 0, 0, 0, 0, 0];
  private trajectoryTimeouts: any[] = [];

  // Trayectorias
  trajectories: Trajectory[] = [];
  currentTrajectory: Trajectory | null = null;
  newTrajectory: Trajectory = {
    id: 0,
    name: 'Trayectoria 1',
    points: [],
  };

  // Calibración
  calibJoint: number = 0;
  calibBrake: number = 1;
  calibPosition: number = 0;

  // Pose actual para movimiento cartesiano
  currentPose = { x: 0.0, y: 0.0, z: 0.0, qx: 0.0, qy: 0.0, qz: 0.0, qw: 0.0 };
  currentSimulatedPose = { x: 0.0, y: 0.0, z: 0.0, qx: 0.0, qy: 0.0, qz: 0.0, qw: 0.0 };
  newPoint: { positions: number[]; time: number } = { positions: [0, 0, 0, 0, 0, 0], time: 0 };

  constructor(private rosService: RosService,
    private uiStateService: UiStateService,
    private trajectoryService: TrajectoryService,
    public authService: AuthService,
    public confirmService: ConfirmService,
    public robot: RobotStateService,
    public robotSimulated: RobotStateServiceSimulated) {
    this.uiStateService.menuOpen$.subscribe(open => this.menuOpen = open);
    this.uiStateService.isSmallScreen$.subscribe(small => this.isSmallScreen = small);
    this.trajectories = this.trajectoryService.getTrajectories();

    effect(() => {
      const currentPose = {
        x: this.robot.x(),
        y: this.robot.y(),
        z: this.robot.z(),
        qx: this.robot.qx(),
        qy: this.robot.qy(),
        qz: this.robot.qz(),
        qw: this.robot.qw()
      };

      const currentSimulatedPose = {
        x: this.robotSimulated.x(),
        y: this.robotSimulated.y(),
        z: this.robotSimulated.z(),
        qx: this.robotSimulated.qx(),
        qy: this.robotSimulated.qy(),
        qz: this.robotSimulated.qz(),
        qw: this.robotSimulated.qw()
      };

      this.currentPose = { ...currentPose };
      this.currentSimulatedPose = { ...currentSimulatedPose };
    });
  }

  ngOnInit(): void {
    this.loadTrajectories();
  }

  setActiveMenu(menu: 'controls' | 'trajectories' | 'calibration') {
    this.activeSubmenu = menu;
  }

  toggleCartesianMode() {
    this.cartesianMode = !this.cartesianMode;
  }

  formatAngles(angles: number[]): string {
    if (!angles) return '';
    return angles.map(a => a !== undefined ? a.toFixed(1) : '').join(', ');
  }

  moveSimulatedJoint(joint: string, angle: number) {
    const jointMap: { [key: string]: number } = {
      'junta_0_1': this.pointAngles[0],
      'junta_1_2': this.pointAngles[1],
      'junta_2_3': this.pointAngles[2],
      'junta_3_4': this.pointAngles[3],
      'junta_4_5': this.pointAngles[4],
      'junta_5_6': this.pointAngles[5]
    };

    this.robotSimulated.angle1.set(this.pointAngles[0]);
    this.robotSimulated.angle2.set(this.pointAngles[1]);
    this.robotSimulated.angle3.set(this.pointAngles[2]);
    this.robotSimulated.angle4.set(this.pointAngles[3]);
    this.robotSimulated.angle5.set(this.pointAngles[4]);
    this.robotSimulated.angle6.set(this.pointAngles[5]);
  }

  // --- Funciones de trayectorias ---
  loadTrajectories() {
    this.rosService.getTrajectories().subscribe({
      next: (trajectories) => {
        this.trajectories = trajectories;
        console.log("Trajectories:", this.trajectories);
      },
      error: (err) => {
        console.error("Error loading trajectories:", err);
      }
    });
  }

  addPoint() {
    const positions = [parseFloat(this.robotSimulated.angle1().toFixed(2)), parseFloat(this.robotSimulated.angle2().toFixed(2)), parseFloat(this.robotSimulated.angle3().toFixed(2)), parseFloat(this.robotSimulated.angle4().toFixed(2)), parseFloat(this.robotSimulated.angle5().toFixed(2)), parseFloat(this.robotSimulated.angle6().toFixed(2))];
    this.newPoint = { positions, time: parseFloat(this.robotSimulated.time().toFixed(2)) };
    this.newTrajectory.points.push({ angles: [...this.newPoint.positions], time: this.newPoint.time });
  }

  dropPoint(event: CdkDragDrop<any[]>) {
    moveItemInArray(
      this.newTrajectory.points,
      event.previousIndex,
      event.currentIndex
    );
  }

  deletePoint(index: number) {
    this.newTrajectory.points.splice(index, 1);
  }

  updatePoint(index: number) {
    const positions = [this.robot.angle1(), this.robot.angle2(), this.robot.angle3(), this.robot.angle4(), this.robot.angle5(), this.robot.angle6()];
    this.newTrajectory.points[index] = { angles: [...positions], time: this.robot.time() };
  }

  addTrajectory() {
    if (!this.newTrajectory.name || this.newTrajectory.points.length === 0) {
      this.errorMessage = 'Por favor, ingrese un nombre y al menos un punto para la trayectoria.';
      return;
    }

    this.rosService.createTrajectory(this.newTrajectory).subscribe({
      next: (data) => {
        this.loadTrajectories();
      },
      error: (err) => console.error('Error guardando trayectoria:', err)
    });
    this.loadTrajectories();
  }

  updateTrajectory() {
    if (!this.newTrajectory) return;

    this.rosService.updateTrajectory(this.newTrajectory.id, this.newTrajectory).subscribe({
      next: (data) => {
        this.loadTrajectories();
      },
      error: (err) => console.error('Error actualizando trayectoria:', err)
    });
  }

  async requestRemoveTrajectory() {
    try {
      await this.confirmService.confirm({
        title: 'Eliminar trayectoria',
        message: 'Esta acción no se puede deshacer.'
      });
      this.removeTrajectory();
    } catch {
      // El usuario canceló la acción
    }
  }

  removeTrajectory() {
    if (!this.currentTrajectory) return;

    this.rosService.deleteTrajectory(this.currentTrajectory.id).subscribe({
      next: () => this.loadTrajectories(),
      error: (err) => console.error('Error eliminando trayectoria:', err)
    });
    this.currentTrajectory = null;
  }

  onTrajectoryChange() {
    if (!this.newTrajectory) return;

    this.rosService.getTrajectory(this.newTrajectory.id).subscribe({
      next: (trajectory) => {
        this.currentTrajectory = {
          id: trajectory.id,
          name: trajectory.name,
          points: trajectory.points
        };

        if (this.currentTrajectory && this.currentTrajectory.points.length > 0) {
          this.newTrajectory = this.currentTrajectory;
        }
      },
      error: (err) => {
        console.error("Error loading trajectory:", err);
      }
    });
  }

  moveToPoint(point: { angles: number[], time: number }) {
    this.pointAngles = [...point.angles]
    this.robotSimulated.angle1.set(point.angles[0]);
    this.robotSimulated.angle2.set(point.angles[1]);
    this.robotSimulated.angle3.set(point.angles[2]);
    this.robotSimulated.angle4.set(point.angles[3]);
    this.robotSimulated.angle5.set(point.angles[4]);
    this.robotSimulated.angle6.set(point.angles[5]);
    this.robotSimulated.time.set(point.time);
  }

  toggleMenu() {
    this.menuOpen = !this.menuOpen;
    this.uiStateService.setMenuOpen(this.menuOpen);
  }

  planMultipleTrajectory() {
    if (!this.newTrajectory || this.newTrajectory.points.length === 0) return;

    this.clearTrajectoryTimeouts();

    const first_pose = {
      angle1: this.robot.angle1() * Math.PI / 180,
      angle2: this.robot.angle2() * Math.PI / 180,
      angle3: this.robot.angle3() * Math.PI / 180,
      angle4: this.robot.angle4() * Math.PI / 180,
      angle5: this.robot.angle5() * Math.PI / 180,
      angle6: this.robot.angle6() * Math.PI / 180,
    }

    const poses = [first_pose];
    poses.push(
      ...this.newTrajectory.points.map(pt => ({
        angle1: pt.angles[0] * Math.PI / 180 ,
        angle2: pt.angles[1] * Math.PI / 180,
        angle3: pt.angles[2] * Math.PI / 180,
        angle4: pt.angles[3] * Math.PI / 180,
        angle5: pt.angles[4] * Math.PI / 180,
        angle6: pt.angles[5] * Math.PI / 180
      }))
    );

    this.rosService.planTrajectory(first_pose, poses, false, this.velocity()).subscribe({
      next: (data: any) => {
        console.log('Planificación de trayectoria múltiple exitosa:', data);
        if (!data.trajectory || data.trajectory.length === 0) return;

        const allPoints: any[] = [];
        let totalTime = 0;
        data.trajectory.forEach((poseEntry: any) => {
          let prevTime = 0;
          poseEntry.points.forEach((pt: any) => {
            const delay = (pt.time_from_start - prevTime) * 1000;
            totalTime += delay;
            prevTime = pt.time_from_start;
            const point = {
              angles: pt.positions.map((angle: number) => angle * 180 / Math.PI),
              time: pt.time_from_start
            };
            const timeoutId = setTimeout(() => {
              this.moveToPoint(point);
            }, totalTime);
            this.trajectoryTimeouts.push(timeoutId);
            allPoints.push(point);
          });
        });
      },
      error: (err) => {
        console.error('Error planificando trayectoria múltiple:', err);
      }
    });
  }

  private clearTrajectoryTimeouts() {
    this.trajectoryTimeouts.forEach(timeoutId => clearTimeout(timeoutId));
    this.trajectoryTimeouts = [];
  }

  planTrajectory() {
    // Cancelar cualquier animación anterior antes de iniciar una nueva
    this.clearTrajectoryTimeouts();

    const first_pose = {
      angle1: this.robot.angle1() * Math.PI / 180,
      angle2: this.robot.angle2() * Math.PI / 180,
      angle3: this.robot.angle3() * Math.PI / 180,
      angle4: this.robot.angle4() * Math.PI / 180,
      angle5: this.robot.angle5() * Math.PI / 180,
      angle6: this.robot.angle6() * Math.PI / 180,
    }

    const targetPose = {
      angle1: this.robotSimulated.angle1() * Math.PI / 180,
      angle2: this.robotSimulated.angle2() * Math.PI / 180,
      angle3: this.robotSimulated.angle3() * Math.PI / 180,
      angle4: this.robotSimulated.angle4() * Math.PI / 180,
      angle5: this.robotSimulated.angle5() * Math.PI / 180,
      angle6: this.robotSimulated.angle6() * Math.PI / 180,
    }

    console.log(targetPose);

    const poses = [
      first_pose,
      targetPose
    ]

    this.rosService.planTrajectory(first_pose, poses, this.cartesianMode, this.velocity()).subscribe({
      next: (data: any) => {
        console.log('Planificación de trayectoria exitosa:', data);
        if (!data.trajectory || data.trajectory.length === 0) return;

        const allPoints: any[] = [];
        let totalTime = 0;
        data.trajectory.forEach((poseEntry: any) => {
          let prevTime = 0;
          poseEntry.points.forEach((pt: any) => {
            const delay = (pt.time_from_start - prevTime) * 1000;
            totalTime += delay;
            prevTime = pt.time_from_start;
            const point = {
              angles: pt.positions.map((angle: number) => angle * 180 / Math.PI),
              time: pt.time_from_start
            };
            const timeoutId = setTimeout(() => {
              this.moveToPoint(point);
            }, totalTime);
            this.trajectoryTimeouts.push(timeoutId);
            allPoints.push(point);
          });
        });
      },
      error: (err) => {
        console.error('Error planificando trayectoria:', err);
      }
    });
  }

  executePlannedTrajectory() {
    this.rosService.executeTrajectory().subscribe({
      next: (res) => {
        console.log('Ejecución de trayectoria iniciada:', res);
      },
      error: (err) => {
        console.error('Error ejecutando trayectoria:', err);
      }
    });
  }

  executeCommand(cmd: string) {
    this.rosService.executeCommand(cmd).subscribe({
      next: (res) => {
        console.log(`Comando ${cmd} ejecutado con éxito:`, res);
      },
      error: (err) => {
        console.error(`Error ejecutando comando ${cmd}:`, err);
      }
    });
  }

  sendCalibrate() {
    this.rosService.calibrate(
      this.calibJoint,
      this.calibBrake,
      this.calibPosition
    ).subscribe({
      next: () => console.log('Calibración enviada'),
      error: err => this.errorMessage = err.message
    });
  }

  resetRobotPosition() {
    this.robotSimulated.angle1.set(this.robot.angle1());
    this.robotSimulated.angle2.set(this.robot.angle2());
    this.robotSimulated.angle3.set(this.robot.angle3());
    this.robotSimulated.angle4.set(this.robot.angle4());
    this.robotSimulated.angle5.set(this.robot.angle5());
    this.robotSimulated.angle6.set(this.robot.angle6());

    this.pointAngles = [this.robot.angle1(), this.robot.angle2(), this.robot.angle3(), this.robot.angle4(), this.robot.angle5(), this.robot.angle6()];
  }
}

// Crear un cuaternión a partir de un eje y un ángulo en radianes
function quatFromAxisAngle(axis: [number, number, number], angleRad: number): [number, number, number, number] {
  const halfAngle = angleRad / 2;
  const s = Math.sin(halfAngle);
  return [axis[0] * s, axis[1] * s, axis[2] * s, Math.cos(halfAngle)];
}

// Multiplicar dos cuaterniones q1 * q2
function quatMultiply(q1: [number, number, number, number], q2: [number, number, number, number]): [number, number, number, number] {
  const [x1, y1, z1, w1] = q1;
  const [x2, y2, z2, w2] = q2;

  return [
    w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
    w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
    w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
  ];
}