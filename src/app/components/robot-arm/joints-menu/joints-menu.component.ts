import { Component, effect, HostBinding, OnInit, signal } from '@angular/core';
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
export class JointsMenuComponent implements OnInit {

  constructor(private rosService: RosService,
    private uiStateService: UiStateService,
    private trajectoryService: TrajectoryService,
    public robot: RobotStateService) {
    this.uiStateService.menuOpen$.subscribe(open => this.menuOpen = open);
    this.uiStateService.isSmallScreen$.subscribe(small => this.isSmallScreen = small);
    this.uiStateService.manualMode$.subscribe(mode => this.manualMode = mode);
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

      this.currentPose = { ...currentPose };
    });
  }

  ngOnInit(): void {
    this.loadTrajectories();
  }

  currentPose = { x: 0.0, y: 0.0, z: 0.0, qx: 0.0, qy: 0.0, qz: 0.0, qw: 0.0 };
  menuOpen: boolean = false;
  isSmallScreen: boolean = false;
  errorMessage: string = '';
  manualMode: boolean = false;

  // local Angles
  pointAngles: number[] = [0, 0, 0, 0, 0, 0];

  trajectories: Trajectory[] = [];
  currentTrajectory: Trajectory | null = null;
  newTrajectory: Trajectory = {
    id: 0,
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
    const positions = [parseFloat(this.robot.angle1().toFixed(2)), parseFloat(this.robot.angle2().toFixed(2)), parseFloat(this.robot.angle3().toFixed(2)), parseFloat(this.robot.angle4().toFixed(2)), parseFloat(this.robot.angle5().toFixed(2)), parseFloat(this.robot.angle6().toFixed(2))];
    this.newPoint = { positions, time: parseFloat(this.robot.time().toFixed(2)) };
    this.newTrajectory.points.push({ angles: [...this.newPoint.positions], time: this.newPoint.time });
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

    this.robot.angle1.set(point.angles[0]);
    this.robot.angle2.set(point.angles[1]);
    this.robot.angle3.set(point.angles[2]);
    this.robot.angle4.set(point.angles[3]);
    this.robot.angle5.set(point.angles[4]);
    this.robot.angle6.set(point.angles[5]);
    this.robot.time.set(point.time);
  }

  toggleMenu() {
    this.menuOpen = !this.menuOpen;
    this.uiStateService.setMenuOpen(this.menuOpen);
  }

  toggleManual() {
    this.manualMode = !this.manualMode;
    this.uiStateService.setManualMode(this.manualMode);
  }

  executeTrajectory() {
    if (!this.newTrajectory || this.newTrajectory.points.length === 0) return;
    const poses = this.newTrajectory.points.map(pt => {
      return {
        angle1: pt.angles[0] * Math.PI / 180,
        angle2: pt.angles[1] * Math.PI / 180,
        angle3: pt.angles[2] * Math.PI / 180,
        angle4: pt.angles[3] * Math.PI / 180,
        angle5: pt.angles[4] * Math.PI / 180,
        angle6: pt.angles[5] * Math.PI / 180
      };
    });

    this.rosService.planPath(poses[0], poses).subscribe({
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
            setTimeout(() => {
              this.moveToPoint(point);
            }, totalTime);
            allPoints.push(point);
          });
        }
        );
      },
      error: (err) => {
        console.error('Error planificando trayectoria:', err);
      }
    });
  }

  testPlanPathMultiple() {
    const poses = [
      { x: 0.3, y: 0.4, z: 0.8, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0 },
      { x: 0.29, y: 0.40, z: 0.8, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0 },
      { x: 0.23, y: 0.15, z: 0.76, qx: -0.77, qy: 0.0, qz: 0.0, qw: 0.63 },
    ];

    const first_pose = {
      angle1: -115 * Math.PI / 180,
      angle2: -30 * Math.PI / 180,
      angle3: 45 * Math.PI / 180,
      angle4: 180 * Math.PI / 180,
      angle5: 74 * Math.PI / 180,
      angle6: -65 * Math.PI / 180
    }

    this.rosService.planPath(first_pose, poses).subscribe({
      next: (data: any) => {
        console.log('Planificación de trayectoria exitosa:', data);

        // Recorremos la trayectoria recibida
        if (!data.trajectory || data.trajectory.length === 0) return;

        // Concatenamos todos los puntos de todas las poses
        const allPoints: any[] = [];
        let totalTime = 0;

        data.trajectory.forEach((poseEntry: any) => {
          let prevTime = 0; // time_from_start relativo a la pose
          poseEntry.points.forEach((pt: any) => {
            const delay = (pt.time_from_start - prevTime) * 1000; // ms
            totalTime += delay;
            prevTime = pt.time_from_start;

            const point = {
              angles: pt.positions.map((angle: number) => angle * 180 / Math.PI),
              time: pt.time_from_start
            };

            setTimeout(() => {
              this.moveToPoint(point);
            }, totalTime);
          });
        });
      },
      error: (err) => {
        console.error('Error planificando trayectoria:', err);
      }
    });
  }

  // Movimiento cartesiano incremental
  moveCartesian(axis: 'x' | 'y' | 'z', direction: number) {
    // console.log("Current Pose before move:", this.currentPose);
    // Asignamos la pose inicial de las articulaciones del robot (actuales)
    const first_pose = {
      angle1: this.robot.angle1() * Math.PI / 180,
      angle2: this.robot.angle2() * Math.PI / 180,
      angle3: this.robot.angle3() * Math.PI / 180,
      angle4: this.robot.angle4() * Math.PI / 180,
      angle5: this.robot.angle5() * Math.PI / 180,
      angle6: this.robot.angle6() * Math.PI / 180
    };
    // Creamos la mini-trayectoria: primero la pose actual que servirá como fallback
    const poses = [{ ...this.currentPose }];
    // Aumentamos/disminuimos distancia en el eje correspondiente
    this.currentPose = {
      ...this.currentPose,
      x: axis === 'x' ? this.currentPose.x + direction * 0.01 : this.currentPose.x,
      y: axis === 'y' ? this.currentPose.y + direction * 0.01 : this.currentPose.y,
      z: axis === 'z' ? this.currentPose.z + direction * 0.01 : this.currentPose.z,
    };
    poses.push({ ...this.currentPose });

    this.rosService.planPath(first_pose, poses).subscribe({
      next: (data: any) => {
        if (!data.trajectory || data.trajectory.length === 0) return;

        // Tomamos el último punto de la última pose
        const lastTrajectory = data.trajectory[data.trajectory.length - 1];
        const lastPoint = lastTrajectory.points[lastTrajectory.points.length - 1];
        const point = {
          angles: lastPoint.positions.map((angle: number) => angle * 180 / Math.PI),
          time: lastPoint.time_from_start
        };
        this.moveToPoint(point); // mueve al robot/UI al último punto de la trayectoria
        // console.log('Movimiento cartesiano incremental exitoso:', point);
      },
      error: (err) => {
        console.log("Reverting to previous pose due to error.", poses[0]);
        this.currentPose = { ...poses[0] }; // revertir a la pose original en caso de error
        console.error('Error planificando movimiento cartesiano:', err);
      }
    });
  }

  // Movimiento de rotación incremental
  moveRotation(axis: 'qx' | 'qy' | 'qz', direction: number) {
    // console.log("Current Pose before rotation:", this.currentPose);

    // Asignamos la pose inicial de las articulaciones del robot (actuales)
    const poses = [{ ...this.currentPose }];
    const first_pose = {
      angle1: this.robot.angle1() * Math.PI / 180,
      angle2: this.robot.angle2() * Math.PI / 180,
      angle3: this.robot.angle3() * Math.PI / 180,
      angle4: this.robot.angle4() * Math.PI / 180,
      angle5: this.robot.angle5() * Math.PI / 180,
      angle6: this.robot.angle6() * Math.PI / 180
    };

    // Ángulo incremental a rotar (por ejemplo, 5 grados convertido a radianes)
    const angleIncrementRad = direction * 5 * Math.PI / 180;
    // Normalizar el quaternion actual
    let q = [this.currentPose.qx, this.currentPose.qy, this.currentPose.qz, this.currentPose.qw] as [number, number, number, number];

    // Definir el eje de rotación
    let axisVec: [number, number, number];
    switch (axis) {
      case 'qx': axisVec = [1, 0, 0]; break;
      case 'qy': axisVec = [0, 1, 0]; break;
      case 'qz': axisVec = [0, 0, 1]; break;
      default: axisVec = [0, 0, 1];
    }

    // Crear cuaternión incremental
    const qIncrement = quatFromAxisAngle(axisVec, angleIncrementRad);

    // Multiplicar: nueva rotación = qIncrement * qActual
    const qNew = quatMultiply(qIncrement, q);

    // Actualizar la pose
    this.currentPose = {
      ...this.currentPose,
      qx: qNew[0],
      qy: qNew[1],
      qz: qNew[2],
      qw: qNew[3],
    };

    poses.push({ ...this.currentPose });

    this.rosService.planPath(first_pose, poses).subscribe({
      next: (data: any) => {
        if (!data.trajectory || data.trajectory.length === 0) return;

        const lastTrajectory = data.trajectory[data.trajectory.length - 1];
        const lastPoint = lastTrajectory.points[lastTrajectory.points.length - 1];

        const point = {
          angles: lastPoint.positions.map((angle: number) => angle * 180 / Math.PI),
          time: lastPoint.time_from_start
        };

        this.moveToPoint(point);
        console.log('Rotación incremental exitosa:', point);
      },
      error: (err) => {
        this.currentPose = { ...poses[0] }; // revertir a la pose original en caso de error
        console.error('Error planificando rotación:', err);
      }
    });
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